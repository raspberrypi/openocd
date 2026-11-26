// SPDX-License-Identifier: GPL-2.0-or-later AND MIT

/*
 * Copyright (C) 2021-2026 Raspberry Pi Ltd
 *
 * RP1 PIO hardware layer for the rp1pio SWD adapter. This is the only file
 * which includes piolib; see rp1pio_hw.h.
 *
 * Every piolib FIFO access is an RPC to RP1, so instead of feeding the state
 * machine word by word, whole batches are streamed through DMA. This relies
 * on the rp1-pio kernel driver's DMA write-behind: a TX transfer which fits
 * in the bounce buffers is queued and returns at once, so the caller can
 * then block collecting RX without the SM deadlocking on a full RX FIFO.
 * Kernels without write-behind would block on TX instead, so they are
 * detected (by their lack of the read-ahead support which arrived with it)
 * and rejected at init.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <helper/types.h>
#include <piolib.h>

#include "rp1pio_hw.h"

/*
 * The SWD bit engine from Debugprobe (src/probe.pio), with the addition of a
 * push_cmd entry point which pushes the empty ISR without clocking, used as
 * a completion marker at the end of each batch.
 *
 * The program is Copyright (c) 2021-2023 Raspberry Pi (Trading) Ltd., and
 * is used under the MIT licence; see LICENSES/preferred/MIT.
 *
 * Every TX FIFO entry is either a command, or up to 32 bits of data.
 * Command format:
 *
 * | 13:9 |  8  |  7:0  |
 * | Cmd  | Dir | Count |
 *
 * Count is the number of bits to be transferred by this command, minus 1.
 * Dir is the output enable for the SWDIO pin.
 * Cmd is the absolute address of the command routine.
 * The SWCLK period is 4 PIO SM execution cycles.
 *
 * .program swd
 * .side_set 1 opt
 *
 * public write_cmd:
 * public turnaround_cmd:                      ; Alias of write, with Dir = 0
 *     pull
 * write_bitloop:
 *     out pins, 1             [1]  side 0x0   ; Data is output by host on negedge
 *     jmp x-- write_bitloop   [1]  side 0x1   ; ...and captured by target on posedge
 *                                             ; Fall through to next command
 * .wrap_target
 * public get_next_cmd:
 *     pull                         side 0x0   ; SWCLK is initially low
 *     out x, 8                                ; Get bit count
 *     out pindirs, 1                          ; Set SWDIO direction
 *     out pc, 5                               ; Go to command routine
 *
 * read_bitloop:
 *     nop                                     ; Additional delay on taken loop branch
 * public read_cmd:
 *     in pins, 1              [1]  side 0x1   ; Data is captured by host on posedge
 *     jmp x-- read_bitloop         side 0x0
 * public push_cmd:
 *     push
 * .wrap                                       ; Wrap to next command
 */

/* Assembled with pioasm */
static const uint16_t swd_program_instructions[] = {
	0x80a0, /*  0: pull   block                   */
	0x7101, /*  1: out    pins, 1         side 0 [1] */
	0x1941, /*  2: jmp    x--, 1          side 1 [1] */
		/*     .wrap_target                   */
	0x90a0, /*  3: pull   block           side 0  */
	0x6028, /*  4: out    x, 8                    */
	0x6081, /*  5: out    pindirs, 1              */
	0x60a5, /*  6: out    pc, 5                   */
	0xa042, /*  7: nop                            */
	0x5901, /*  8: in     pins, 1         side 1 [1] */
	0x1047, /*  9: jmp    x--, 7          side 0  */
	0x8020, /* 10: push   block                   */
		/*     .wrap                          */
};

#define SWD_WRAP_TARGET			3
#define SWD_WRAP			10
#define SWD_OFFSET_WRITE_CMD		0
#define SWD_OFFSET_TURNAROUND_CMD	0
#define SWD_OFFSET_GET_NEXT_CMD		3
#define SWD_OFFSET_READ_CMD		8
#define SWD_OFFSET_PUSH_CMD		10

static const struct pio_program swd_program = {
	.instructions = swd_program_instructions,
	.length = ARRAY_SIZE(swd_program_instructions),
	.origin = -1,
};

#define SM_CYCLES_PER_SWCLK	4
/*
 * The hardware can generate 50 MHz (200 MHz / 4), but reads sample SWDIO on
 * the SWCLK rising edge (as Debugprobe does), so the target's response must
 * make the full round trip - including RP1's 2-cycle input synchroniser -
 * within one period. With typical jumper wiring to an RP2040 that fails
 * above ~25 MHz (corrupt read data at 30 MHz); going faster would need a
 * configurable sample delay in the PIO read routine.
 */
#define MAX_SWCLK_KHZ		25000

/*
 * TX: the kernel allows at most 4 bounce buffers; a whole batch must fit, so
 * that the TX ioctl never blocks. RX: a batch's results fit in one buffer.
 */
#define TX_BUF_SIZE		(RP1PIO_MAX_TX_WORDS * sizeof(uint32_t) / TX_BUF_COUNT)
#define TX_BUF_COUNT		4
#define RX_BUF_SIZE		(RP1PIO_MAX_RX_WORDS * sizeof(uint32_t))
#define RX_BUF_COUNT		2

static struct {
	PIO pio;
	int sm;
	unsigned int offset;
	unsigned int swclk, swdio;
	bool program_loaded;
	bool pins_claimed;
} hw = {
	.pio = NULL,
	.sm = -1,
};

unsigned int rp1pio_hw_max_khz(void)
{
	return MAX_SWCLK_KHZ;
}

/*
 * Use an integer divider, rounded up: a fractional divider would stretch
 * some SM cycles and not others, making some SWCLK periods shorter than
 * requested, and rounding down would make the clock faster than requested.
 */
static unsigned int khz_to_div(unsigned int khz, uint32_t sys_hz)
{
	uint32_t swclk_hz;
	uint32_t div;

	if (khz == 0)
		khz = 1;
	if (khz > MAX_SWCLK_KHZ)
		khz = MAX_SWCLK_KHZ;

	swclk_hz = khz * 1000;
	div = DIV_ROUND_UP(sys_hz, SM_CYCLES_PER_SWCLK * swclk_hz);
	if (div < 1)
		div = 1;
	if (div > 65535)
		div = 65535;
	return div;
}

static unsigned int div_to_khz(unsigned int div, uint32_t sys_hz)
{
	return sys_hz / (SM_CYCLES_PER_SWCLK * div) / 1000;
}

unsigned int rp1pio_hw_get_khz(unsigned int khz)
{
	uint32_t sys_hz = clock_get_hz(clk_sys);

	return div_to_khz(khz_to_div(khz, sys_hz), sys_hz);
}

unsigned int rp1pio_hw_set_khz(unsigned int khz)
{
	uint32_t sys_hz = clock_get_hz(clk_sys);
	unsigned int div = khz_to_div(khz, sys_hz);

	pio_sm_set_clkdiv_int_frac(hw.pio, hw.sm, div, 0);
	return div_to_khz(div, sys_hz);
}

uint32_t rp1pio_hw_cmd(enum rp1pio_op op, unsigned int bits)
{
	unsigned int label;
	bool out_en = false;

	switch (op) {
	case RP1PIO_OP_WRITE:
		label = SWD_OFFSET_WRITE_CMD;
		out_en = true;
		break;
	case RP1PIO_OP_HIZ:
		label = SWD_OFFSET_TURNAROUND_CMD;
		break;
	case RP1PIO_OP_READ:
		label = SWD_OFFSET_READ_CMD;
		break;
	case RP1PIO_OP_PUSH:
	default:
		label = SWD_OFFSET_PUSH_CMD;
		bits = 1;
		break;
	}

	return ((bits - 1) & 0xff) | ((uint32_t)out_en << 8) |
		((hw.offset + label) << 9);
}

static int config_dma(void)
{
	if (pio_sm_config_xfer(hw.pio, hw.sm, PIO_DIR_TO_SM, TX_BUF_SIZE, TX_BUF_COUNT) ||
		pio_sm_config_xfer(hw.pio, hw.sm, PIO_DIR_FROM_SM, RX_BUF_SIZE, RX_BUF_COUNT))
		return -EIO;
	return 0;
}

/* Get the engine back to a known state, discarding any queued DMA */
static void reset_engine(void)
{
	pio_sm_set_enabled(hw.pio, hw.sm, false);
	/* Reconfiguring the channels terminates any outstanding transfers */
	config_dma();
	pio_sm_clear_fifos(hw.pio, hw.sm);
	pio_sm_restart(hw.pio, hw.sm);
	pio_sm_exec(hw.pio, hw.sm,
		pio_encode_jmp(hw.offset + SWD_OFFSET_GET_NEXT_CMD));
	pio_sm_set_enabled(hw.pio, hw.sm, true);
}

/*
 * Check for DMA write-behind, via the RX read-ahead support (a NULL buffer)
 * which arrived with it, then consume the read-ahead with a PUSH command.
 */
static int check_write_behind(void)
{
	uint32_t cmd = rp1pio_hw_cmd(RP1PIO_OP_PUSH, 0);
	uint32_t result;

	if (pio_sm_xfer_data(hw.pio, hw.sm, PIO_DIR_FROM_SM, sizeof(result), NULL))
		return -ENOTSUP;

	if (pio_sm_xfer_data(hw.pio, hw.sm, PIO_DIR_TO_SM, sizeof(cmd), &cmd) ||
		pio_sm_xfer_data(hw.pio, hw.sm, PIO_DIR_FROM_SM, sizeof(result), &result)) {
		reset_engine();
		return -EIO;
	}

	return 0;
}

void rp1pio_hw_quit(void)
{
	if (!hw.pio)
		return;

	if (hw.sm >= 0) {
		pio_sm_set_enabled(hw.pio, hw.sm, false);
		if (hw.pins_claimed) {
			/* DAP says all pins must be high-Z when disabled */
			gpio_disable_pulls(hw.swclk);
			gpio_set_function(hw.swclk, GPIO_FUNC_NULL);
			gpio_disable_pulls(hw.swdio);
			gpio_set_function(hw.swdio, GPIO_FUNC_NULL);
			hw.pins_claimed = false;
		}
		if (hw.program_loaded) {
			pio_remove_program(hw.pio, &swd_program, hw.offset);
			hw.program_loaded = false;
		}
		pio_sm_unclaim(hw.pio, hw.sm);
		hw.sm = -1;
	}

	pio_close(hw.pio);
	hw.pio = NULL;
}

int rp1pio_hw_init(unsigned int swclk_gpio, unsigned int swdio_gpio, const char **err)
{
	pio_sm_config c;
	PIO pio;
	int ret;

	if (hw.pio) {
		*err = "already initialised";
		return -EBUSY;
	}

	hw.swclk = swclk_gpio;
	hw.swdio = swdio_gpio;

	pio = pio_open(0);
	if (PIO_IS_ERR(pio)) {
		*err = "failed to open PIO device (is /dev/pio0 present and accessible?)";
		return PIO_ERR_VAL(pio);
	}
	hw.pio = pio;

	hw.sm = pio_claim_unused_sm(pio, false);
	if (hw.sm < 0) {
		*err = "no free PIO state machine";
		ret = -EBUSY;
		goto fail;
	}

	if (!pio_can_add_program(pio, &swd_program)) {
		*err = "insufficient PIO instruction memory";
		ret = -ENOSPC;
		goto fail;
	}
	hw.offset = pio_add_program(pio, &swd_program);
	hw.program_loaded = true;

	/* SM: as Debugprobe's probe_sm_init() */
	c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, hw.offset + SWD_WRAP_TARGET, hw.offset + SWD_WRAP);
	sm_config_set_sideset(&c, 2, true, false);
	sm_config_set_sideset_pins(&c, swclk_gpio);
	sm_config_set_out_pins(&c, swdio_gpio, 1);
	sm_config_set_set_pins(&c, swdio_gpio, 1);
	sm_config_set_in_pins(&c, swdio_gpio);
	sm_config_set_out_shift(&c, true, false, 0);	/* LSB first, no autopull */
	sm_config_set_in_shift(&c, true, false, 0);	/* LSB first, no autopush */
	pio_sm_init(pio, hw.sm, hw.offset + SWD_OFFSET_GET_NEXT_CMD, &c);
	rp1pio_hw_set_khz(1000);

	/* DMA must be configured after pio_sm_init(), which resets it */
	if (config_dma()) {
		*err = "failed to configure PIO DMA";
		ret = -EIO;
		goto fail;
	}

	/* Run the engine before the pins are switched to PIO, for the check */
	pio_sm_set_enabled(pio, hw.sm, true);

	ret = check_write_behind();
	if (ret) {
		*err = ret == -ENOTSUP ?
			"the rp1-pio kernel driver lacks DMA write-behind (a newer kernel is needed)" :
			"PIO DMA check failed";
		goto fail;
	}

	/* Pins: as Debugprobe's probe_gpio_init(), with the SM stopped */
	pio_sm_set_enabled(pio, hw.sm, false);
	hw.pins_claimed = true;
	pio_gpio_init(pio, swclk_gpio);
	pio_gpio_init(pio, swdio_gpio);
	gpio_pull_up(swdio_gpio);	/* Idle state is high */

	/* SWCLK and SWDIO start as outputs; SWDIO direction is set per command */
	pio_sm_set_consecutive_pindirs(pio, hw.sm, swclk_gpio, 1, true);
	pio_sm_set_consecutive_pindirs(pio, hw.sm, swdio_gpio, 1, true);
	pio_sm_set_enabled(pio, hw.sm, true);

	return 0;

fail:
	rp1pio_hw_quit();
	return ret;
}

int rp1pio_hw_xfer(const uint32_t *tx, size_t ntx, uint32_t *rx, size_t nrx)
{
	int ret;

	if (!hw.pio || !ntx || !nrx)
		return -EINVAL;

	/* A larger TX would block until the SM drains it, deadlocking on RX */
	if (ntx > RP1PIO_MAX_TX_WORDS || nrx > RP1PIO_MAX_RX_WORDS)
		return -E2BIG;

	/* Queued by write-behind; returns without waiting for the SM */
	ret = pio_sm_xfer_data(hw.pio, hw.sm, PIO_DIR_TO_SM,
		ntx * sizeof(uint32_t), (void *)tx);
	if (!ret)
		ret = pio_sm_xfer_data(hw.pio, hw.sm, PIO_DIR_FROM_SM,
			nrx * sizeof(uint32_t), rx);

	if (ret)
		reset_engine();

	return ret;
}
