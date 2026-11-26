// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2026 Raspberry Pi Ltd
 *
 * SWD adapter driver for the Raspberry Pi 5, using a PIO state machine in
 * RP1 (via piolib) to generate SWD on any pair of header GPIOs. The PIO
 * program is Debugprobe's SWD engine; see rp1pio_hw.c.
 *
 * Because each piolib FIFO access is an RPC to RP1, transfers are queued and
 * the whole queue is streamed through DMA when it is run. Like linuxspidev,
 * this is a "blind" adapter: every transfer has a fixed shape regardless of
 * ACK (OpenOCD sets CTRL/STAT.ORUNDETECT for such adapters), and ACKs and
 * parity are checked once the queue has been executed.
 *
 * OpenOCD expects the adapter to retry WAIT responses itself. With overrun
 * detection enabled, a WAIT sets STICKYORUN and the DP ignores the rest of
 * the queue, so on a WAIT the overrun is cleared and the queue is re-run
 * from the transfer which got the WAIT. That transfer was not accepted, so
 * re-issuing it (even a posted AP read) is safe.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <string.h>

#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <target/arm_adi_v5.h>

#if BUILD_LINUXGPIOD
#include "linuxgpiod.h"
#endif

#include "rp1pio_hw.h"

/* RP1 GPIOs available on the 40-pin header */
#define RP1PIO_NUM_GPIOS	28

/* Queue limits; the stream is rebuilt from the queued operations on each run */
#define MAX_QUEUE_ENTRIES	256
#define TX_QUEUE_WORDS		RP1PIO_MAX_TX_WORDS
#define RX_QUEUE_WORDS		(MAX_QUEUE_ENTRIES * 3 + 8)

/* A transfer needs at most 9 TX words (plus idle cycles) and 3 RX words */
#define XFER_TX_WORDS		9
#define XFER_RX_WORDS		3

/*
 * Room reserved for the overrun-clearing ABORT write which prefixes a retry,
 * and the end-of-queue idle cycles and completion marker
 */
#define RESERVED_TX_WORDS	(XFER_TX_WORDS + 3)
#define RESERVED_RX_WORDS	(XFER_RX_WORDS + 1)

/* A transfer must be followed by another, or at least 8 idle cycles */
#define END_IDLE_CLOCKS		8

/* How long to keep retrying a transfer which gets WAIT, as for bitbang */
#define SWD_WAIT_TIMEOUT	500

#define TRN_BITS		1
#define ACK_BITS		3
#define DATA_BITS		32

enum op_type {
	OP_XFER,
	OP_SEQ,
};

struct pending_op {
	enum op_type type;

	/* OP_SEQ */
	const uint8_t *seq;
	unsigned int seq_bits;

	/* OP_XFER */
	uint8_t cmd;
	uint32_t *dst;		/* reads only */
	uint32_t wdata;		/* writes only */
	uint32_t ap_delay_clk;
	unsigned int ack_idx;
	unsigned int data_idx;	/* reads only */
	unsigned int parity_idx;	/* reads only */
};

static uint32_t *tx_buf;
static unsigned int tx_fill;
static uint32_t *rx_buf;
static unsigned int rx_fill;
static struct pending_op *pending;
static unsigned int pending_fill;
static unsigned int queued_tx_words;
static unsigned int queued_rx_words;
static int queue_retval = ERROR_OK;

/* Prefixed to a retry, to clear STICKYORUN set by the WAIT */
static struct pending_op clear_overrun_op = {
	.type = OP_XFER,
	.cmd = SWD_CMD_START | SWD_CMD_PARK,	/* | swd_cmd(false, false, DP_ABORT) == 0 */
	.wdata = ORUNERRCLR,
};

static bool hw_ready;
static int requested_khz = 1000;

/* ------------------------------------------------------------------------ */
/* Command stream builders, after Debugprobe's probe.c                       */
/* ------------------------------------------------------------------------ */

static void q_word(uint32_t word)
{
	tx_buf[tx_fill++] = word;
}

/* Drive 1..256 bits; only the first 32 come from data, the rest are 0 */
static void q_write(unsigned int bits, uint32_t data)
{
	q_word(rp1pio_hw_cmd(RP1PIO_OP_WRITE, bits));
	q_word(data);
}

/* Release SWDIO for 1..256 clocks */
static void q_hiz(unsigned int bits)
{
	q_word(rp1pio_hw_cmd(RP1PIO_OP_HIZ, bits));
	q_word(0);
}

/* Sample 1..32 bits; returns the RX slot, in which the bits are left-aligned */
static unsigned int q_read(unsigned int bits)
{
	q_word(rp1pio_hw_cmd(RP1PIO_OP_READ, bits));
	return rx_fill++;
}

static unsigned int idle_words(unsigned int clocks)
{
	return 2 * DIV_ROUND_UP(clocks, RP1PIO_MAX_CLOCKS);
}

static void q_idle(unsigned int clocks)
{
	while (clocks) {
		unsigned int n = MIN(clocks, RP1PIO_MAX_CLOCKS);

		q_write(n, 0);
		clocks -= n;
	}
}

/* LSB-first bit sequence */
static void q_seq(const uint8_t *seq, unsigned int bits)
{
	for (unsigned int pos = 0; pos < bits; pos += RP1PIO_MAX_DATA_BITS) {
		unsigned int n = MIN(bits - pos, RP1PIO_MAX_DATA_BITS);

		q_write(n, buf_get_u32(seq, pos, n));
	}
}

/* As Debugprobe's SWD_Transfer(), with the data phase always performed */
static void q_xfer(struct pending_op *op)
{
	bool read = op->cmd & SWD_CMD_RNW;

	q_write(8, op->cmd);
	op->ack_idx = q_read(TRN_BITS + ACK_BITS);

	if (read) {
		op->data_idx = q_read(DATA_BITS);
		op->parity_idx = q_read(1);
		q_hiz(TRN_BITS);
	} else {
		q_hiz(TRN_BITS);
		q_write(DATA_BITS, op->wdata);
		q_write(1, parity_u32(op->wdata));
	}

	/* Idle cycles after AP accesses to avoid WAIT */
	q_idle(op->ap_delay_clk);
}

static void q_op(struct pending_op *op)
{
	if (op->type == OP_SEQ)
		q_seq(op->seq, op->seq_bits);
	else
		q_xfer(op);
}

/* Build the stream for pending[start..], optionally clearing an overrun first */
static void build_stream(unsigned int start, bool clear_overrun)
{
	tx_fill = 0;
	rx_fill = 0;

	if (clear_overrun)
		q_xfer(&clear_overrun_op);

	for (unsigned int i = start; i < pending_fill; i++)
		q_op(&pending[i]);

	q_idle(END_IDLE_CLOCKS);
	q_word(rp1pio_hw_cmd(RP1PIO_OP_PUSH, 0));
	rx_fill++;
}

static void clear_queue(void)
{
	tx_fill = 0;
	rx_fill = 0;
	pending_fill = 0;
	queued_tx_words = 0;
	queued_rx_words = 0;
}

static bool queue_has_room(unsigned int tx_words, unsigned int rx_words)
{
	return pending_fill < MAX_QUEUE_ENTRIES &&
		queued_tx_words + tx_words + RESERVED_TX_WORDS <= TX_QUEUE_WORDS &&
		queued_rx_words + rx_words + RESERVED_RX_WORDS <= RX_QUEUE_WORDS;
}

/* ------------------------------------------------------------------------ */
/* SWD operations                                                            */
/* ------------------------------------------------------------------------ */

static int xfer_ack(const struct pending_op *op)
{
	/* 4-bit read, left-aligned: turnaround in bit 28, ACK in 31:29 */
	return (rx_buf[op->ack_idx] >> (32 - ACK_BITS)) & 7;
}

/*
 * Check the result of a transfer, storing read data. Sets *wait (and returns
 * ERROR_OK) if the transfer needs to be retried.
 */
static int complete_xfer(const struct pending_op *op, bool retrying, bool *wait)
{
	bool read = op->cmd & SWD_CMD_RNW;
	int ack = xfer_ack(op);
	uint32_t data = read ? rx_buf[op->data_idx] : op->wdata;
	/* Devices do not reply to DP_TARGETSEL write cmd, ignore received ack */
	bool check_ack = swd_cmd_returns_ack(op->cmd);

	LOG_CUSTOM_LEVEL((check_ack && ack != SWD_ACK_OK && (!retrying || ack != SWD_ACK_WAIT))
			? LOG_LVL_DEBUG : LOG_LVL_DEBUG_IO,
			"%s%s %s %s reg %X = %08" PRIx32,
			check_ack ? "" : "ack ignored ",
			ack == SWD_ACK_OK ? "OK" :
			ack == SWD_ACK_WAIT ? "WAIT" :
			ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			op->cmd & SWD_CMD_APNDP ? "AP" : "DP",
			read ? "read" : "write",
			(op->cmd & SWD_CMD_A32) >> 1,
			data);

	if (check_ack && ack == SWD_ACK_WAIT) {
		*wait = true;
		return ERROR_OK;
	}

	if (check_ack && ack != SWD_ACK_OK)
		return swd_ack_to_error_code(ack);

	if (read) {
		int parity = rx_buf[op->parity_idx] >> 31;

		if (parity != parity_u32(data)) {
			LOG_ERROR("SWD Read data parity mismatch");
			return ERROR_FAIL;
		}

		if (op->dst)
			*op->dst = data;
	}

	return ERROR_OK;
}

static int rp1pio_swd_execute_queue(void)
{
	unsigned int start = 0;
	unsigned int retry = 0;
	int64_t timeout = 0;
	int ret;

	LOG_DEBUG_IO("Executing %u queued operations", pending_fill);

	if (queue_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", queue_retval);
		goto skip;
	}

	if (!pending_fill)
		goto skip;

	for (;;) {
		bool wait = false;
		unsigned int i;

		build_stream(start, retry > 0);

		ret = rp1pio_hw_xfer(tx_buf, tx_fill, rx_buf, rx_fill);
		if (ret) {
			LOG_ERROR("RP1 PIO transfer failed: %s", strerror(-ret));
			queue_retval = ERROR_FAIL;
			goto skip;
		}

		if (retry > 0) {
			int ack = xfer_ack(&clear_overrun_op);

			if (ack != SWD_ACK_OK) {
				LOG_DEBUG("SWD overrun clear failed, ack %d", ack);
				queue_retval = swd_ack_to_error_code(ack);
				goto skip;
			}
		}

		ret = ERROR_OK;
		for (i = start; i < pending_fill; i++) {
			if (pending[i].type != OP_XFER)
				continue;
			ret = complete_xfer(&pending[i], retry > 0, &wait);
			if (wait || ret != ERROR_OK)
				break;
		}

		if (!wait) {
			if (retry > 1)
				LOG_DEBUG("SWD WAIT: retried %u times", retry);
			queue_retval = ret;
			break;
		}

		if (retry == 0)
			timeout = timeval_ms() + SWD_WAIT_TIMEOUT;
		if (timeval_ms() > timeout) {
			LOG_DEBUG("SWD WAIT: timed out after %u retries", retry);
			queue_retval = ERROR_WAIT;
			break;
		}

		if (++retry > 20)
			alive_sleep(1);
		start = i;
	}

skip:
	clear_queue();

	ret = queue_retval;
	queue_retval = ERROR_OK;

	return ret;
}

static int rp1pio_swd_run_queue(void)
{
	return rp1pio_swd_execute_queue();
}

static struct pending_op *queue_op(unsigned int tx_words, unsigned int rx_words)
{
	struct pending_op *op;

	if (!queue_has_room(tx_words, rx_words))
		queue_retval = rp1pio_swd_execute_queue();

	if (queue_retval != ERROR_OK)
		return NULL;

	op = &pending[pending_fill++];
	memset(op, 0, sizeof(*op));
	queued_tx_words += tx_words;
	queued_rx_words += rx_words;
	return op;
}

static void rp1pio_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	struct pending_op *op = queue_op(XFER_TX_WORDS + idle_words(ap_delay_clk), XFER_RX_WORDS);

	if (!op)
		return;

	op->type = OP_XFER;
	op->cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;
	op->dst = dst;
	op->wdata = data;
	op->ap_delay_clk = ap_delay_clk;
}

static void rp1pio_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	rp1pio_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void rp1pio_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	rp1pio_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int rp1pio_swd_switch_seq(enum swd_special_seq seq)
{
	struct pending_op *op;
	const uint8_t *bits;
	unsigned int len;

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		bits = swd_seq_line_reset;
		len = swd_seq_line_reset_len;
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		bits = swd_seq_jtag_to_swd;
		len = swd_seq_jtag_to_swd_len;
		break;
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		bits = swd_seq_jtag_to_dormant;
		len = swd_seq_jtag_to_dormant_len;
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		bits = swd_seq_swd_to_jtag;
		len = swd_seq_swd_to_jtag_len;
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		bits = swd_seq_swd_to_dormant;
		len = swd_seq_swd_to_dormant_len;
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		bits = swd_seq_dormant_to_swd;
		len = swd_seq_dormant_to_swd_len;
		break;
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		bits = swd_seq_dormant_to_jtag;
		len = swd_seq_dormant_to_jtag_len;
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	op = queue_op(2 * DIV_ROUND_UP(len, RP1PIO_MAX_DATA_BITS), 0);
	if (op) {
		op->type = OP_SEQ;
		op->seq = bits;
		op->seq_bits = len;
	}

	return ERROR_OK;
}

static int rp1pio_swd_init(void)
{
	return ERROR_OK;
}

static const struct swd_driver rp1pio_swd = {
	.init = rp1pio_swd_init,
	.switch_seq = rp1pio_swd_switch_seq,
	.read_reg = rp1pio_swd_read_reg,
	.write_reg = rp1pio_swd_write_reg,
	.run = rp1pio_swd_run_queue,
};

/* ------------------------------------------------------------------------ */
/* Adapter operations                                                        */
/* ------------------------------------------------------------------------ */

static int rp1pio_speed(int speed)
{
	requested_khz = speed;
	if (hw_ready) {
		unsigned int actual = rp1pio_hw_set_khz(speed);

		LOG_DEBUG("SWCLK %d kHz requested, %u kHz actual", speed, actual);
	}
	return ERROR_OK;
}

static int rp1pio_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = MIN((unsigned int)khz, rp1pio_hw_max_khz());
	return ERROR_OK;
}

static int rp1pio_speed_div(int speed, int *khz)
{
	/* Report the frequency the integer divider actually achieves */
	*khz = hw_ready ? (int)rp1pio_hw_get_khz(speed) : speed;
	return ERROR_OK;
}

static bool gpio_valid(enum adapter_gpio_config_index idx)
{
	const struct adapter_gpio_config *cfg = &adapter_gpio_get_config()[idx];

	if (cfg->gpio_num == ADAPTER_GPIO_NOT_SET) {
		LOG_ERROR("rp1pio: %s GPIO not set (use 'adapter gpio %s <num>')",
			  adapter_gpio_get_name(idx), adapter_gpio_get_name(idx));
		return false;
	}
	if (cfg->gpio_num >= RP1PIO_NUM_GPIOS) {
		LOG_ERROR("rp1pio: %s GPIO %u out of range (0-%d)",
			  adapter_gpio_get_name(idx), cfg->gpio_num, RP1PIO_NUM_GPIOS - 1);
		return false;
	}
	if (cfg->active_low)
		LOG_WARNING("rp1pio: -active-low ignored for %s", adapter_gpio_get_name(idx));
	return true;
}

static int rp1pio_quit(void);

static int rp1pio_init(void)
{
	const struct adapter_gpio_config *cfg = adapter_gpio_get_config();
	unsigned int swclk, swdio;
	const char *err = "";
	int ret;

	LOG_INFO("RP1 PIO SWD driver");

	if (!gpio_valid(ADAPTER_GPIO_IDX_SWCLK) || !gpio_valid(ADAPTER_GPIO_IDX_SWDIO))
		return ERROR_JTAG_INIT_FAILED;

	swclk = cfg[ADAPTER_GPIO_IDX_SWCLK].gpio_num;
	swdio = cfg[ADAPTER_GPIO_IDX_SWDIO].gpio_num;
	if (swclk == swdio) {
		LOG_ERROR("rp1pio: SWCLK and SWDIO must be different GPIOs");
		return ERROR_JTAG_INIT_FAILED;
	}

	tx_buf = calloc(TX_QUEUE_WORDS, sizeof(*tx_buf));
	rx_buf = calloc(RX_QUEUE_WORDS, sizeof(*rx_buf));
	pending = calloc(MAX_QUEUE_ENTRIES, sizeof(*pending));
	if (!tx_buf || !rx_buf || !pending) {
		LOG_ERROR("Out of memory");
		goto fail;
	}
	clear_queue();
	queue_retval = ERROR_OK;

#if BUILD_LINUXGPIOD
	if (linuxgpiod_helper_get_line(ADAPTER_GPIO_IDX_SRST) != ERROR_OK)
		goto fail;
#endif

	ret = rp1pio_hw_init(swclk, swdio, &err);
	if (ret) {
		LOG_ERROR("rp1pio: %s (%s)", err, strerror(-ret));
		goto fail;
	}
	hw_ready = true;
	rp1pio_speed(requested_khz);

	LOG_INFO("rp1pio: SWCLK on GPIO%u, SWDIO on GPIO%u", swclk, swdio);
	return ERROR_OK;

fail:
	rp1pio_quit();
	return ERROR_JTAG_INIT_FAILED;
}

static int rp1pio_quit(void)
{
	if (hw_ready) {
		rp1pio_hw_quit();
		hw_ready = false;
	}

#if BUILD_LINUXGPIOD
	linuxgpiod_quit();
#endif

	free(tx_buf);
	tx_buf = NULL;
	free(rx_buf);
	rx_buf = NULL;
	free(pending);
	pending = NULL;

	return ERROR_OK;
}

struct adapter_driver rp1pio_adapter_driver = {
	.name = "rp1pio",
	.transport_ids = TRANSPORT_SWD,
	.transport_preferred_id = TRANSPORT_SWD,

	.init = rp1pio_init,
	.quit = rp1pio_quit,
	.speed = rp1pio_speed,
	.khz = rp1pio_khz,
	.speed_div = rp1pio_speed_div,
#if BUILD_LINUXGPIOD
	.reset = linuxgpiod_reset,
#endif

	.swd_ops = &rp1pio_swd,
};
