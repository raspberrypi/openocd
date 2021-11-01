/***************************************************************************
 *   Copyright (C) 2020 by Liam Fraser                                     *
 *   liam@raspberrypi.com                                                  *
 *                                                                         *
 *   Based on: kitprog.c, ftdi.c, mpsse.c                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/command.h>

#include "libusb_helper.h"

#define VID 0x2E8A /* Raspberry Pi */
#define PID 0x0004 /* Picoprobe */

#define BULK_EP_OUT 4
#define BULK_EP_IN  5
#define PICOPROBE_INTERFACE 2

#define PICOPROBE_MAX_PACKET_LENGTH 512
#define LIBUSB_TIMEOUT 10000

struct picoprobe {
	struct libusb_device_handle *usb_handle;
	uint8_t *packet_buffer;
	int freq;
};

static struct swd_cmd_queue_entry {
	uint8_t cmd;
	uint32_t *dst;
	uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} *swd_cmd_queue;
static size_t swd_cmd_queue_length;
static size_t swd_cmd_queue_alloced;
static int queued_retval;

static struct picoprobe *picoprobe_handle;

static int picoprobe_init(void);
static int picoprobe_quit(void);

enum PROBE_CMDS {
	PROBE_INVALID = 0,   /* Invalid */
	PROBE_WRITE_BITS = 1, /* Host wants us to write bits */
	PROBE_READ_BITS = 2,  /* Host wants us to read bits */
	PROBE_SET_FREQ = 3, /* Set TCK freq */
	PROBE_RESET = 4
};

struct __attribute__((__packed__)) probe_cmd_hdr {
	uint8_t id;
	uint8_t cmd;
	uint32_t bits;
};

struct __attribute__((__packed__)) probe_pkt_hdr {
	uint32_t total_packet_length;
};

/* Separate queue to swd_cmd_queue because we sometimes insert idle cycles not described
 * there */
#define PICOPROBE_QUEUE_SIZE 64
static struct picoprobe_queue_entry {
	uint8_t id;
	uint8_t cmd; /* PROBE_CMDS */
	unsigned bits;
	unsigned offset;
	const uint8_t *buf;
} *picoprobe_queue;
static size_t picoprobe_queue_length;
static size_t picoprobe_queue_alloced;

static inline unsigned packet_length(uint8_t *pkt)
{
	return pkt - picoprobe_handle->packet_buffer;
}

static int picoprobe_bulk_write(struct probe_pkt_hdr *pkt_hdr, uint8_t *pkt)
{
	pkt_hdr->total_packet_length = packet_length(pkt);
	assert(pkt_hdr->total_packet_length <= PICOPROBE_MAX_PACKET_LENGTH);
	int ret = 0;
	jtag_libusb_bulk_write(picoprobe_handle->usb_handle,
			  BULK_EP_OUT, (char *)picoprobe_handle->packet_buffer, packet_length(pkt), LIBUSB_TIMEOUT,
			  &ret);
	if (ret < 0)
		return ERROR_JTAG_DEVICE_ERROR;

	return ERROR_OK;
}

static int picoprobe_flush(void)
{
	LOG_DEBUG_IO("Flush %d transactions", (int)picoprobe_queue_length);
	int ret = ERROR_OK;

	struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr *)picoprobe_handle->packet_buffer;

	/* Chain pending write and read commands together */
	uint8_t *pkt = picoprobe_handle->packet_buffer + sizeof(struct probe_pkt_hdr);

	unsigned total_reads = 0;
	unsigned total_read_bytes = 0;

	for (unsigned i = 0; i < picoprobe_queue_length; i++) {
		/* Copy header regardless of read or write */
		struct picoprobe_queue_entry *q = &picoprobe_queue[i];
		struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr *)pkt;
		if (q->id != i) {
			LOG_ERROR("Wrong queue id. q->id %d != %d", q->id, i);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		hdr->id = q->id;
		hdr->cmd = q->cmd;
		hdr->bits = q->bits;
		pkt += sizeof(struct probe_cmd_hdr);
		unsigned length_bytes = DIV_ROUND_UP(q->bits, 8);

		if (q->cmd == PROBE_WRITE_BITS) {
			/* Copy the data to write into the packet buffer */
			if (q->buf) {
				bit_copy(pkt, 0, q->buf, q->offset, q->bits);
			} else {
				/* Make sure the packet buffer is zerod to clock zeros */
				assert(q->offset == 0);
				memset(pkt, 0, length_bytes);
			}
			pkt += length_bytes;
		} else if (q->cmd == PROBE_READ_BITS) {
			/* Nothing to do for a read as we have already copied the header
			 * Will process the data later in one go */
			total_reads++;
			total_read_bytes += length_bytes;
		} else {
			/* Unexpected cmd to flush */
			return ERROR_FAIL;
		}
	}

	/* Send all read/write commands + write data */
	ret = picoprobe_bulk_write(pkt_hdr, pkt);
	if (ret < 0)
		return ERROR_JTAG_DEVICE_ERROR;

	/* If no reads we can bail */
	if (total_reads == 0) {
		picoprobe_queue_length = 0;
		return ret;
	}

	/* Now get any read responses */
	unsigned rx_pkt_len = sizeof(struct probe_pkt_hdr) +
					     (sizeof(struct probe_cmd_hdr) * total_reads) +
						 total_read_bytes;
	jtag_libusb_bulk_read(picoprobe_handle->usb_handle,
		BULK_EP_IN | LIBUSB_ENDPOINT_IN, (char *)picoprobe_handle->packet_buffer,
		rx_pkt_len, LIBUSB_TIMEOUT, &ret);

	if (ret < 0)
		return ERROR_JTAG_DEVICE_ERROR;

	/* Now time to process the rx data */
	LOG_DEBUG_IO("Read %d bytes from probe", ret);

	/* If we didn't get length we expected */
	if ((int)rx_pkt_len != ret)
		return ERROR_JTAG_DEVICE_ERROR;

	struct probe_pkt_hdr *response_hdr = (struct probe_pkt_hdr *)picoprobe_handle->packet_buffer;
	if (rx_pkt_len != response_hdr->total_packet_length)
		return ERROR_JTAG_DEVICE_ERROR;

	pkt = picoprobe_handle->packet_buffer + sizeof(struct probe_pkt_hdr);

	/* Now go through read responses */
	for (unsigned i = 0; i < total_reads; i++) {
		struct probe_cmd_hdr *read_hdr = (struct probe_cmd_hdr *)pkt;
		pkt += sizeof(struct probe_cmd_hdr);
		unsigned read_bytes = DIV_ROUND_UP(read_hdr->bits, 8);

		if (read_hdr->cmd != PROBE_READ_BITS)
			return ERROR_JTAG_DEVICE_ERROR;

		uint8_t id = read_hdr->id;
		struct picoprobe_queue_entry *q = &picoprobe_queue[id];
		assert(read_hdr->cmd  == q->cmd);
		assert(read_hdr->id   == q->id);
		assert(read_hdr->bits == q->bits);
		LOG_DEBUG_IO("Processing read of %d bits", read_hdr->bits);

		/* Copy data back to swd cmd queue */
		memcpy((void *)q->buf, pkt, read_bytes);
		pkt += read_bytes;
	}

	unsigned processed_len = (pkt - picoprobe_handle->packet_buffer);
	if (processed_len != rx_pkt_len)
		return ERROR_JTAG_DEVICE_ERROR;

	picoprobe_queue_length = 0;

	/* Keep gdb alive */
	keep_alive();

	return ERROR_OK;
}

static int picoprobe_read_write_bits(const uint8_t *buf, unsigned offset, unsigned length, uint8_t cmd)
{
	if (picoprobe_queue_length == picoprobe_queue_alloced) {
		LOG_ERROR("Picoprobe queue full");
		return ERROR_BUF_TOO_SMALL;
	} else {
		LOG_DEBUG_IO("Picoprobe queue len %d -> %d", (int)picoprobe_queue_length,
					 (int)picoprobe_queue_length + 1);
	}

	struct picoprobe_queue_entry *q = &picoprobe_queue[picoprobe_queue_length];
	q->id = picoprobe_queue_length++;
	q->cmd = cmd;
	q->bits = length;
	q->offset = offset;
	q->buf = buf;

	return ERROR_OK;
}

static int picoprobe_write_bits(const uint8_t *buf, unsigned offset, unsigned length)
{
	LOG_DEBUG_IO("Write %d bits @ offset %d", length, offset);
	return picoprobe_read_write_bits(buf, offset, length, PROBE_WRITE_BITS);
}

static int picoprobe_read_bits(const uint8_t *buf, unsigned offset, unsigned length)
{
	LOG_DEBUG_IO("Read %d bits @ offset %d", length, offset);

	if (picoprobe_queue_length == picoprobe_queue_alloced)
		return ERROR_BUF_TOO_SMALL;

	return picoprobe_read_write_bits(buf, offset, length, PROBE_READ_BITS);
}

static int picoprobe_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %zu queued transactions", swd_cmd_queue_length);
	int retval;

	queued_retval = picoprobe_flush();

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	for (size_t i = 0; i < swd_cmd_queue_length; i++) {
		if (0 == ((swd_cmd_queue[i].cmd ^ swd_cmd(false, false, DP_TARGETSEL)) &
				(SWD_CMD_APnDP|SWD_CMD_RnW|SWD_CMD_A32))) {
			/* Targetsel has no ack so force it */
			buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3, SWD_ACK_OK);
		}


		LOG_DEBUG_IO("trn_ack_data_parity_trn:");
		for (size_t y = 0; y < sizeof(swd_cmd_queue[i].trn_ack_data_parity_trn); y++)
			LOG_DEBUG_IO("BYTE %d 0x%x", (int)y, swd_cmd_queue[i].trn_ack_data_parity_trn[y]);

		int ack = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3);

		LOG_DEBUG_IO("%s %s %s reg %X = %08"PRIx32,
				ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
				swd_cmd_queue[i].cmd & SWD_CMD_APnDP ? "AP" : "DP",
				swd_cmd_queue[i].cmd & SWD_CMD_RnW ? "read" : "write",
				(swd_cmd_queue[i].cmd & SWD_CMD_A32) >> 1,
				buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn,
						1 + 3 + (swd_cmd_queue[i].cmd & SWD_CMD_RnW ? 0 : 1), 32));

		if (ack != SWD_ACK_OK) {
			queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
			goto skip;

		} else if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
			uint32_t data = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3, 32);
			int parity = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 32, 1);

			if (parity != parity_u32(data)) {
				LOG_ERROR("SWD Read data parity mismatch");
				queued_retval = ERROR_FAIL;
				goto skip;
			}

			if (swd_cmd_queue[i].dst != NULL)
				*swd_cmd_queue[i].dst = data;
		}
	}

skip:
	/* Defensive cleanup - seems like a bad idea to have potentially stale pointers sticking around */
	for (size_t i = 0; i < swd_cmd_queue_length; i++)
		swd_cmd_queue[i].dst = NULL;

	swd_cmd_queue_length = 0;
	retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void picoprobe_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	if (swd_cmd_queue_length == swd_cmd_queue_alloced)
		queued_retval = picoprobe_swd_run_queue();

	if (queued_retval != ERROR_OK)
		return;

	size_t i = swd_cmd_queue_length++;
	swd_cmd_queue[i].cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	picoprobe_write_bits(&swd_cmd_queue[i].cmd, 0, 8);

	if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		swd_cmd_queue[i].dst = dst;

		picoprobe_read_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
				0, 1 + 3 + 32 + 1 + 1);
	} else {
		/* Queue a write transaction */
		picoprobe_read_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
				0, 1 + 3 + 1);

		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1, 32, data);
		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(data));

		picoprobe_write_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
				1 + 3 + 1, 32 + 1);
	}

	/* Insert idle cycles after AP accesses to avoid WAIT */
	if (cmd & SWD_CMD_APnDP) {
		if (ap_delay_clk == 0)
			return;
		LOG_DEBUG("Add %d idle cycles", ap_delay_clk);
		picoprobe_write_bits(NULL, 0, ap_delay_clk);
	}

}

static void picoprobe_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	picoprobe_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void picoprobe_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	picoprobe_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int_least32_t picoprobe_set_frequency(int_least32_t hz)
{
	int ret;
	struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr *)picoprobe_handle->packet_buffer;

	/* Assert this is a standalone command with nothing else queued */
	assert(picoprobe_queue_length == 0);

	/* Chain writes and read commands together */
	uint8_t *pkt = picoprobe_handle->packet_buffer + sizeof(struct probe_pkt_hdr);
	struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr *)pkt;
	hdr->id = 0;
	hdr->cmd = PROBE_SET_FREQ;
	hdr->bits = hz / 1000;
	pkt += sizeof(struct probe_cmd_hdr);

	/* Send all read/write commands + write data */
	ret = picoprobe_bulk_write(pkt_hdr, pkt);
	if (ret < 0)
		return ERROR_JTAG_DEVICE_ERROR;

	return hz;
}

static int_least32_t picoprobe_speed(int_least32_t hz)
{
	int ret = picoprobe_set_frequency(hz);

	if (ret < 0)
		LOG_ERROR("Couldn't set picoprobe speed");
	else
		picoprobe_handle->freq = ret;

	return ERROR_OK;
}

static int picoprobe_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static int picoprobe_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int picoprobe_swd_init(void)
{
	return ERROR_OK;
}

static int picoprobe_swd_switch_seq(enum swd_special_seq seq)
{
	int ret = ERROR_OK;

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		ret = picoprobe_write_bits(swd_seq_line_reset, 0, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		ret = picoprobe_write_bits(swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		ret = picoprobe_write_bits(swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		ret = picoprobe_write_bits(swd_seq_dormant_to_swd, 0, swd_seq_dormant_to_swd_len);
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		ret = picoprobe_write_bits(swd_seq_swd_to_dormant, 0, swd_seq_swd_to_dormant_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ret;
}

static int picoprobe_reset(int trst, int srst)
{
	return ERROR_OK;
}

static const struct swd_driver picoprobe_swd = {
	.init = picoprobe_swd_init,
	.switch_seq = picoprobe_swd_switch_seq,
	.read_reg = picoprobe_swd_read_reg,
	.write_reg = picoprobe_swd_write_reg,
	.run = picoprobe_swd_run_queue,
};

const char *probe_serial_number = NULL;

static COMMAND_HELPER(handle_serialnum_args, const char **serialNumber)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("%s: need single argument with serial number", CMD_NAME);
		*serialNumber = NULL;
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		*serialNumber = CMD_ARGV[0];
		return ERROR_OK;
	}
}

COMMAND_HANDLER(handle_serialnum_command)
{
	const char *serialNumber = NULL;
	int retval = CALL_COMMAND_HANDLER(handle_serialnum_args, &serialNumber);
	if (ERROR_OK == retval) {
		probe_serial_number = malloc(strlen(serialNumber) + 1);
		if (probe_serial_number) {
			strcpy((char *) probe_serial_number, (char *) serialNumber);
			command_print(CMD, "Using serial number : %s", serialNumber);
		}
	}
	return retval;
}

static const struct command_registration serialnum_command_handlers[] = {
	{
		.name = "pico_serialnum",
		.mode = COMMAND_ANY,
		.handler = handle_serialnum_command,
		.help = "use picoprobe with this serial number",
		.usage = "'serial number'",
	},
	COMMAND_REGISTRATION_DONE
};
static const char * const picoprobe_transports[] = { "swd", NULL };

struct adapter_driver picoprobe_adapter_driver = {
	.name = "picoprobe",
	.commands = serialnum_command_handlers,
	.transports = picoprobe_transports,
	.swd_ops = &picoprobe_swd,
	.init = picoprobe_init,
	.quit = picoprobe_quit,
	.reset = picoprobe_reset,
	.speed = picoprobe_speed,
	.speed_div = picoprobe_speed_div,
	.khz = picoprobe_khz,
};

static int picoprobe_usb_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };

	if (jtag_libusb_open(vids, pids, probe_serial_number,
			&picoprobe_handle->usb_handle, NULL) != ERROR_OK) {
		LOG_ERROR("Failed to open or find the device");
		return ERROR_FAIL;
	}

	if (libusb_claim_interface(picoprobe_handle->usb_handle, PICOPROBE_INTERFACE) != ERROR_OK) {
		LOG_ERROR("Failed to claim picoprobe interface");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void picoprobe_usb_close(void)
{
	jtag_libusb_close(picoprobe_handle->usb_handle);
}

static int picoprobe_init(void)
{
	picoprobe_handle = malloc(sizeof(struct picoprobe));
	if (picoprobe_handle == NULL) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}

	if (picoprobe_usb_open() != ERROR_OK) {
		LOG_ERROR("Can't find a picoprobe device! Please check device connections and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* Allocate packet buffers and queues */
	picoprobe_handle->packet_buffer = malloc(PICOPROBE_MAX_PACKET_LENGTH);
	if (picoprobe_handle->packet_buffer == NULL) {
		LOG_ERROR("Failed to allocate memory for the packet buffer");
		return ERROR_FAIL;
	}

	picoprobe_queue_alloced = PICOPROBE_QUEUE_SIZE;
	picoprobe_queue_length = 0;
	picoprobe_queue = malloc(picoprobe_queue_alloced * sizeof(*picoprobe_queue));
	if (picoprobe_queue == NULL)
		return ERROR_FAIL;

	swd_cmd_queue_alloced = 10;
	swd_cmd_queue = malloc(swd_cmd_queue_alloced * sizeof(*swd_cmd_queue));

	return swd_cmd_queue != NULL ? ERROR_OK : ERROR_FAIL;
}


static int picoprobe_quit(void)
{
	picoprobe_usb_close();
	return ERROR_OK;
}
