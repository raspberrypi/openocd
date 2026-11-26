/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2026 Raspberry Pi Ltd
 *
 * Private interface between the rp1pio adapter driver and the RP1 PIO
 * hardware layer. This header deliberately uses only standard C types, so
 * that piolib's Pico-SDK-style headers never meet OpenOCD's in one
 * translation unit.
 */

#ifndef OPENOCD_JTAG_DRIVERS_RP1PIO_HW_H
#define OPENOCD_JTAG_DRIVERS_RP1PIO_HW_H

#include <stddef.h>
#include <stdint.h>

/* Operations understood by the PIO SWD engine */
enum rp1pio_op {
	RP1PIO_OP_WRITE,	/* drive 1..256 bits, consumes one data word */
	RP1PIO_OP_HIZ,		/* release SWDIO for 1..256 clocks, consumes one data word */
	RP1PIO_OP_READ,		/* sample 1..32 bits, produces one RX word */
	RP1PIO_OP_PUSH,		/* no clocks, produces one RX word */
};

/* Maximum bits for a single read, or the data-carrying part of a write */
#define RP1PIO_MAX_DATA_BITS	32
/* Maximum clocks for a single write/hiz command */
#define RP1PIO_MAX_CLOCKS	256

/* Maximum sizes of a single rp1pio_hw_xfer() batch, in 32-bit words */
#define RP1PIO_MAX_TX_WORDS	8192
#define RP1PIO_MAX_RX_WORDS	1024

/*
 * Claim a PIO state machine, load the program and set up SWCLK/SWDIO.
 * Returns 0 on success; on failure returns a negative errno value and sets
 * *err to a human-readable description.
 */
int rp1pio_hw_init(unsigned int swclk_gpio, unsigned int swdio_gpio, const char **err);
void rp1pio_hw_quit(void);

/*
 * Set SWCLK to the fastest achievable frequency not above khz, and return
 * that frequency in kHz. rp1pio_hw_get_khz() returns the same value without
 * changing the clock. Both require rp1pio_hw_init() to have succeeded.
 */
unsigned int rp1pio_hw_set_khz(unsigned int khz);
unsigned int rp1pio_hw_get_khz(unsigned int khz);
/* Highest SWCLK frequency the engine will generate, in kHz */
unsigned int rp1pio_hw_max_khz(void);

/* Build a command word; bits is 1..256 (ignored for RP1PIO_OP_PUSH) */
uint32_t rp1pio_hw_cmd(enum rp1pio_op op, unsigned int bits);

/*
 * Stream ntx command/data words to the engine, and collect exactly nrx
 * result words (one per READ or PUSH command). Read results are left-aligned:
 * an n-bit read occupies bits [31:32-n]. ntx and nrx must not exceed
 * RP1PIO_MAX_TX_WORDS and RP1PIO_MAX_RX_WORDS. Returns 0 or a negative errno.
 */
int rp1pio_hw_xfer(const uint32_t *tx, size_t ntx, uint32_t *rx, size_t nrx);

#endif /* OPENOCD_JTAG_DRIVERS_RP1PIO_HW_H */
