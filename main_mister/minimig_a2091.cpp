// SPDX-License-Identifier: GPL-3.0-or-later
//
// Commodore A2091 SCSI host adapter (DMAC rev 2 + WD33C93A) - HPS side.
//
// The FPGA (rtl/a2091.v) owns everything the 68k touches at bus speed: the
// boot ROM, the DMAC registers and the WD33C93 register file with its
// SASR / AUX STATUS / SCSI STATUS side effects. It raises events through the
// hps_ext mailbox when the 68k writes the WD COMMAND register, touches the
// DATA register in PIO mode, or starts the DMAC (ST_DMA).
//
// This file runs the WD33C93 command engine and the SCSI disk targets. The
// command engine is a port of amiberry/WinUAE a2091.cpp (Toni Wilen); the
// disk target follows the A4091 MiSTer port's C1 SCSI-2 target. DMA goes
// through a 4 KB FPGA buffer and the core's chipdma_arb, so chip RAM, slow
// RAM and Zorro II RAM are all reachable.
//
// Mailbox (hps_ext):
//   0x70 READ  : word 0 = status; then WD regs 0..31, {sasr,cntr}, ACR hi/lo,
//                WTC hi/lo, ISTR, FIFO count
//   0x71 WRITE : word 0 = status; word 1 = opcode; operands
//   0x72 BUFRD : word 0 = status; then DMA buffer words

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "../../spi.h"
#include "../../fpga_io.h"
#include "../../user_io.h"
#include "../../file_io.h"
#include "../../hardware.h"
#include "minimig_config.h"
#include "minimig_a2091.h"

// ---------------------------------------------------------------- logging
// Silent unless <root>/a2091_debug exists (touch it, reload the core).

static FILE *a2_logf(void)
{
	static FILE *lf = NULL;
	static int tried = 0;
	if (!lf && !tried)
	{
		tried = 1;
		char path[1024];
		snprintf(path, sizeof(path), "%s/a2091_debug", getRootDir());
		if (!access(path, F_OK) || !access("/media/fat/a2091_debug", F_OK) || getenv("A2091_DEBUG"))
		{
			snprintf(path, sizeof(path), "%s/logs", getRootDir());
			mkdir(path, 0755);
			strcat(path, "/a2091.log");
			lf = fopen(path, "a");
			if (lf) fprintf(lf, "---- a2091 log start ----\n");
			if (lf) setvbuf(lf, NULL, _IOLBF, 0);
		}
	}
	return lf;
}

#include <time.h>
static uint64_t now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000;
}

static void a2log(const char *fmt, ...)
{
	FILE *lf = a2_logf();
	if (!lf) return;
	static uint64_t t0 = 0;
	uint64_t t = now_us();
	if (!t0) t0 = t;
	if (fmt[0] != ' ') fprintf(lf, "%9.3f ", (t - t0) / 1000.0);
	va_list ap;
	va_start(ap, fmt);
	vfprintf(lf, fmt, ap);
	va_end(ap);
}

static int      g_prof = -1;
static uint64_t g_prof_t0, g_prof_busy, g_prof_ev[4];

// ---------------------------------------------------------------- mailbox

#define A2_CMD_READ   0x70
#define A2_CMD_WRITE  0x71
#define A2_CMD_BUFRD  0x72

#define ST_EV_CMD     0x0001
#define ST_EV_STDMA   0x0002
#define ST_EV_PIO     0x0004
#define ST_EV_RESET   0x0008
#define ST_DMA_BUSY   0x0010
#define ST_INT        0x0020
#define ST_BUSY       0x0040
#define ST_DMAC_DMA   0x0080
#define ST_ENABLED    0x1000
#define ST_MARKER_M   0xE000
#define ST_MARKER     0xA000

#define OP_WDREGS     0x0000
#define OP_CTRL       0x0100
#define OP_STATUS     0x0200
#define OP_ACR        0x0300
#define OP_BUFWR      0x0400
#define OP_DMA        0x0500
#define OP_ROM        0x0600
#define OP_CFG        0x0700
#define OP_PIO        0x0800
#define PIO_MAX       4094    // bytes per FPGA stream (buffer is 4 KB)

#define CTL_CLR_EV_CMD   (1 << 0)
#define CTL_CLR_EV_STDMA (1 << 1)
#define CTL_CLR_EV_PIO   (1 << 2)
#define CTL_CLR_EV_RESET (1 << 3)
#define CTL_SET_BUSY     (1 << 4)
#define CTL_CLR_BUSY     (1 << 5)
#define CTL_SET_DBR      (1 << 6)
#define CTL_CLR_DBR      (1 << 7)
#define CTL_SET_BSYX     (1 << 8)
#define CTL_CLR_BSYX     (1 << 9)
#define CTL_WD_RESET     (1 << 10)
#define CTL_DMA_DONE     (1 << 11)

#define DMA_BUF_WORDS 2048

static uint16_t mb_status(void)
{
	EnableIO();
	uint16_t st = spi_w(A2_CMD_READ);
	DisableIO();
	return st;
}

// Bulk data rides the unhandshaked fast path IDE uses
// (fpga_spi_fast_block_*_be: big-endian words straight from/to bytes). Only
// the command word goes through the handshaked spi_w(), which returns the
// status word.
static uint16_t g_bounce[DMA_BUF_WORDS + 2];

static void fast_write_bytes(const uint8_t *p, int n)
{
	int words = (n + 1) / 2;
	if (((uintptr_t)p & 1) || (n & 1))
	{
		memcpy(g_bounce, p, n);
		if (n & 1) ((uint8_t *)g_bounce)[n] = 0;
		p = (const uint8_t *)g_bounce;
	}
	fpga_spi_fast_block_write_be((const uint16_t *)p, words);
}

static void fast_read_bytes(uint8_t *p, int n)
{
	int words = (n + 1) / 2;
	if (((uintptr_t)p & 1) || (n & 1))
	{
		fpga_spi_fast_block_read_be(g_bounce, words);
		memcpy(p, g_bounce, n);
	}
	else fpga_spi_fast_block_read_be((uint16_t *)p, words);
}

struct mb_regs
{
	uint8_t  wd[32];
	uint8_t  sasr;
	uint8_t  cntr;
	uint32_t acr;
};

static uint16_t mb_read(mb_regs *r)
{
	EnableIO();
	uint16_t st = spi_w(A2_CMD_READ);
	uint16_t v[35];
	fpga_spi_fast_block_read(v, 35);
	for (int i = 0; i < 32; i++) r->wd[i] = (uint8_t)v[i];
	r->sasr = (v[32] >> 8) & 0x1f;
	r->cntr = v[32] & 0xff;
	r->acr = ((uint32_t)(v[33] & 0xff) << 16) | v[34];
	DisableIO();
	return st;
}

static void mb_write(uint16_t op, const uint16_t *w, int n)
{
	EnableIO();
	spi_w(A2_CMD_WRITE);
	spi_w(op);
	for (int i = 0; i < n; i++) spi_w(w[i]);
	DisableIO();
}

static void mb_ctrl_now(uint16_t bits)
{
	mb_write(OP_CTRL, &bits, 1);
}

// Everything the 68k can see (statuses, DBR/BSY, busy) is held back while a
// command runs and released by mb_flush() *after* the register write-back.
// Otherwise the 68k can react to a status and write e.g. a new transfer count
// before the ARM's stale copy of the registers lands on top of it.
static int      g_defer = 0;
static uint16_t g_ctrl_pending = 0;
static uint16_t g_status_pending[8];
static int      g_status_n = 0;

static void mb_ctrl(uint16_t bits)
{
	if (g_defer) g_ctrl_pending |= bits;
	else mb_ctrl_now(bits);
}

static void mb_wdregs(int first, const uint8_t *v, int n)
{
	uint16_t w[32];
	for (int i = 0; i < n; i++) w[i] = v[i];
	mb_write(OP_WDREGS | first, w, n);
}

static void mb_acr(uint32_t acr)
{
	uint16_t w[2] = { (uint16_t)((acr >> 16) & 0xff), (uint16_t)(acr & 0xfffe) };
	mb_write(OP_ACR, w, 2);
}

static void mb_cfg(int bits)
{
	uint16_t w = (uint16_t)bits;
	mb_write(OP_CFG, &w, 1);
}

static int mb_dma_wait(void)
{
	for (int i = 0; i < 200000; i++)
	{
		if (!(mb_status() & ST_DMA_BUSY)) return 1;
	}
	a2log("DMA timeout\n");
	return 0;
}

// Amiga memory <- data (big-endian words)
static void mb_dma_to_amiga(uint32_t addr, const uint8_t *data, int words)
{
	while (words > 0)
	{
		int n = words > DMA_BUF_WORDS ? DMA_BUF_WORDS : words;

		EnableIO();
		spi_w(A2_CMD_WRITE);
		spi_w(OP_BUFWR);
		fast_write_bytes(data, n * 2);
		DisableIO();

		uint16_t w[3] = { (uint16_t)((addr >> 16) & 0xff), (uint16_t)(addr & 0xfffe), (uint16_t)n };
		mb_write(OP_DMA, w, 3);
		mb_dma_wait();

		addr += n * 2;
		data += n * 2;
		words -= n;
	}
}

// Amiga memory -> data
static void mb_dma_from_amiga(uint32_t addr, uint8_t *data, int words)
{
	while (words > 0)
	{
		int n = words > DMA_BUF_WORDS ? DMA_BUF_WORDS : words;

		uint16_t w[3] = { (uint16_t)((addr >> 16) & 0xff), (uint16_t)(addr & 0xfffe), (uint16_t)(0x8000 | n) };
		mb_write(OP_DMA, w, 3);
		mb_dma_wait();

		EnableIO();
		spi_w(A2_CMD_BUFRD);
		fast_read_bytes(data, n * 2);
		DisableIO();

		addr += n * 2;
		data += n * 2;
		words -= n;
	}
}

// ---------------------------------------------------------------- SCSI target

#define BLOCK_SIZE 512
#define SCSI_MAX_XFER (16 * 1024 * 1024)

struct scsi_data
{
	int       id;
	fileTYPE  file;
	uint64_t  blocks;
	int       present;
	int       readonly;

	uint8_t   cmd[16];
	int       cmd_len;
	int       data_len;
	int       status;
	int       direction;      // >0 to device, <0 from device, 2 = command bytes
	int       offset;
	uint8_t   message[1];
	uint8_t   sense[32];
	int       sense_len;
	uint32_t  unit_attention;

	uint64_t  rw_lba;
	uint32_t  rw_blocks;
};

static scsi_data g_scsi[A2091_NDRV];

static uint8_t *g_buf = NULL;     // one buffer: one command at a time per HBA
static int      g_bufsz = 0;

static uint8_t *scsi_buf(int need)
{
	if (need > g_bufsz)
	{
		int n = need + 64 * 1024;
		uint8_t *p = (uint8_t *)realloc(g_buf, n);
		if (!p) return g_buf;
		g_buf = p;
		g_bufsz = n;
	}
	return g_buf;
}

static const int scsicmdsizes[] = { 6, 10, 10, 12, 16, 12, 10, 10 };

static uint32_t be16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static uint32_t be32(const uint8_t *p) { return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static void wbe32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

static void scsi_set_sense(scsi_data *sd, uint8_t key, uint8_t asc, uint8_t ascq)
{
	memset(sd->sense, 0, sizeof(sd->sense));
	sd->sense[0] = 0x70;
	sd->sense[2] = key;
	sd->sense[7] = 0x0a;
	sd->sense[12] = asc;
	sd->sense[13] = ascq;
	sd->sense_len = 18;
	sd->status = 2;    // CHECK CONDITION
	a2log("  ID%d CHECK key=%02x asc=%02x/%02x op=%02x\n", sd->id, key, asc, ascq, sd->cmd[0]);
}

static void scsi_start_transfer(scsi_data *sd)
{
	sd->offset = 0;
}

static int scsi_send_data(scsi_data *sd, uint8_t b)
{
	if (sd->direction == 1)
	{
		if (sd->offset >= g_bufsz) return 0;
		g_buf[sd->offset++] = b;
	}
	else if (sd->direction == 2)
	{
		if (sd->offset >= 16) return 0;
		sd->cmd[sd->offset++] = b;
		if (sd->offset == sd->cmd_len) return 1;
	}
	else return 0;
	if (sd->offset == sd->data_len) return 1;
	return 0;
}

static int scsi_receive_data(scsi_data *sd, uint8_t *b, bool next)
{
	if (!sd->data_len) return -1;
	*b = g_buf[sd->offset];
	if (next)
	{
		sd->offset++;
		if (sd->offset == sd->data_len) return 1;
	}
	return 0;
}

static int scsi_lun(scsi_data *sd)
{
	if ((sd->message[0] & 0xc0) == 0x80) return sd->message[0] & 7;
	return (sd->cmd[1] >> 5) & 7;
}

// amiberry scsi_emulate_analyze(): command length, direction and, for
// commands that send data to the target, the data length.
static bool scsi_emulate_analyze(scsi_data *sd)
{
	const uint8_t *c = sd->cmd;
	sd->cmd_len = scsicmdsizes[c[0] >> 5];
	int data_len = -1;

	switch (c[0])
	{
	case 0x04: // FORMAT UNIT
		if (c[1] & 0x10) data_len = 4;
		else { sd->direction = 0; sd->data_len = 0; return true; }
		break;
	case 0x0a: // WRITE(6)
		data_len = (c[4] ? c[4] : 256) * BLOCK_SIZE;
		break;
	case 0x2a: // WRITE(10)
	case 0x2e: // WRITE AND VERIFY(10)
		data_len = be16(c + 7) * BLOCK_SIZE;
		break;
	case 0xaa: // WRITE(12)
		data_len = be32(c + 6) * BLOCK_SIZE;
		break;
	case 0x15: // MODE SELECT(6)
		data_len = c[4];
		break;
	case 0x55: // MODE SELECT(10)
		data_len = be16(c + 7);
		break;
	case 0x3b: // WRITE BUFFER
		data_len = (c[6] << 16) | (c[7] << 8) | c[8];
		break;
	case 0x2f: // VERIFY
		if (c[1] & 2) { sd->data_len = be16(c + 7) * BLOCK_SIZE; sd->direction = 1; }
		else          { sd->data_len = 0; sd->direction = 0; }
		if (sd->data_len > SCSI_MAX_XFER) sd->data_len = SCSI_MAX_XFER;
		scsi_buf(sd->data_len);
		return true;
	}

	if (data_len >= 0)
	{
		if (data_len > SCSI_MAX_XFER) data_len = SCSI_MAX_XFER;
		sd->data_len = data_len;
		scsi_buf(data_len);
		sd->direction = data_len ? 1 : 0;
		return true;
	}

	// data from target, or no data
	switch (c[0])
	{
	case 0x00: case 0x01: case 0x0b: case 0x1b: case 0x2b: case 0x35:
	case 0x16: case 0x17: case 0x1e: case 0x07: case 0x06:
		sd->direction = 0;
		break;
	default:
		sd->direction = -1;
		break;
	}
	return true;
}

static void scsi_rw_params(scsi_data *sd)
{
	const uint8_t *c = sd->cmd;
	switch (c[0])
	{
	case 0x08: case 0x0a:
		sd->rw_lba = ((c[1] & 0x1f) << 16) | (c[2] << 8) | c[3];
		sd->rw_blocks = c[4] ? c[4] : 256;
		break;
	case 0x28: case 0x2a: case 0x2e: case 0x2f:
		sd->rw_lba = be32(c + 2);
		sd->rw_blocks = be16(c + 7);
		break;
	case 0xa8: case 0xaa:
		sd->rw_lba = be32(c + 2);
		sd->rw_blocks = be32(c + 6);
		break;
	}
}

// amiberry scsi_emulate_cmd() + scsi_hd_emulate(): run the command. For
// commands to the target the data is already in g_buf.
static void scsi_emulate_cmd(scsi_data *sd)
{
	const uint8_t *c = sd->cmd;
	int alloc;
	uint8_t *b;
	int lun = scsi_lun(sd);

	sd->status = 0;

	if (c[0] == 0x03) // REQUEST SENSE
	{
		alloc = c[4] ? c[4] : 4;
		b = scsi_buf(64);
		memset(b, 0, 32);
		if (sd->sense_len) memcpy(b, sd->sense, sd->sense_len);
		else { b[0] = 0x70; b[7] = 0x0a; }
		if (lun) { b[0] = 0x70; b[2] = 5; b[7] = 0x0a; b[12] = 0x25; }
		sd->data_len = alloc > 18 ? 18 : alloc;
		sd->sense_len = 0;
		memset(sd->sense, 0, sizeof(sd->sense));
		sd->offset = 0;
		return;
	}

	if (c[0] == 0x12) // INQUIRY
	{
		alloc = c[4];
		b = scsi_buf(64);
		memset(b, 0, 36);
		if (lun) b[0] = 0x7f;      // no device on this LUN
		b[2] = 2;                  // SCSI-2
		b[3] = 2;
		b[4] = 31;
		memcpy(b + 8,  "MiSTer  ", 8);
		memcpy(b + 16, "A2091 HD        ", 16);
		memcpy(b + 32, "0001", 4);
		sd->data_len = alloc > 36 ? 36 : alloc;
		sd->offset = 0;
		return;
	}

	if (lun)
	{
		scsi_set_sense(sd, 5, 0x25, 0); // LOGICAL UNIT NOT SUPPORTED
		sd->data_len = 0;
		sd->offset = 0;
		return;
	}

	memset(sd->sense, 0, sizeof(sd->sense));
	sd->sense_len = 0;

	switch (c[0])
	{
	case 0x00: // TEST UNIT READY
	case 0x01: // REZERO
	case 0x04: // FORMAT UNIT
	case 0x0b: case 0x2b: // SEEK
	case 0x1b: // START/STOP
	case 0x35: // SYNCHRONIZE CACHE
	case 0x15: case 0x55: // MODE SELECT
	case 0x16: case 0x17: // RESERVE / RELEASE
	case 0x1e: // PREVENT/ALLOW
	case 0x3b: // WRITE BUFFER
		sd->data_len = 0;
		break;

	case 0x2f: // VERIFY
		sd->data_len = 0;
		break;

	case 0x25: // READ CAPACITY(10)
		b = scsi_buf(8);
		wbe32(b, (uint32_t)(sd->blocks - 1));
		wbe32(b + 4, BLOCK_SIZE);
		sd->data_len = 8;
		break;

	case 0x1a: // MODE SENSE(6)
	case 0x5a: // MODE SENSE(10)
	{
		int ten = c[0] == 0x5a;
		uint8_t pc = c[2] & 0x3f;
		int dbd = (c[1] >> 3) & 1;
		uint32_t heads = 16, spt = 63;
		uint32_t cyls = (uint32_t)(sd->blocks / ((uint64_t)heads * spt));
		if (!cyls) cyls = 1;
		if (pc != 0x03 && pc != 0x04 && pc != 0x3f && pc != 0x00)
		{
			scsi_set_sense(sd, 5, 0x24, 0);
			sd->data_len = 0;
			break;
		}
		b = scsi_buf(128);
		memset(b, 0, 128);
		int hdr = ten ? 8 : 4;
		int p = hdr;
		if (!dbd)
		{
			uint8_t *bd = b + p;
			uint32_t nb = sd->blocks > 0xffffff ? 0xffffff : (uint32_t)sd->blocks;
			bd[1] = nb >> 16; bd[2] = nb >> 8; bd[3] = nb;
			bd[6] = BLOCK_SIZE >> 8; bd[7] = BLOCK_SIZE & 0xff;
			if (ten) b[7] = 8; else b[3] = 8;
			p += 8;
		}
		if (pc == 0x03 || pc == 0x3f)
		{
			uint8_t *pg = b + p;
			pg[0] = 0x03; pg[1] = 0x16;
			pg[10] = spt >> 8; pg[11] = spt;
			pg[12] = BLOCK_SIZE >> 8; pg[13] = BLOCK_SIZE & 0xff;
			pg[20] = 0x40;
			p += 24;
		}
		if (pc == 0x04 || pc == 0x3f)
		{
			uint8_t *pg = b + p;
			pg[0] = 0x04; pg[1] = 0x16;
			pg[2] = cyls >> 16; pg[3] = cyls >> 8; pg[4] = cyls;
			pg[5] = heads;
			pg[20] = 0x1c; pg[21] = 0x20;
			p += 24;
		}
		if (ten) { b[0] = (p - 2) >> 8; b[1] = p - 2; }
		else b[0] = p - 1;
		if (sd->readonly) b[ten ? 3 : 2] |= 0x80;
		alloc = ten ? be16(c + 7) : c[4];
		sd->data_len = alloc < p ? alloc : p;
		break;
	}

	case 0x37: // READ DEFECT DATA(10)
		b = scsi_buf(4);
		memset(b, 0, 4);
		b[1] = c[2] & 0x1f;
		alloc = be16(c + 7);
		sd->data_len = alloc < 4 ? alloc : 4;
		break;

	case 0x08: case 0x28: case 0xa8: // READ
	{
		scsi_rw_params(sd);
		uint64_t bytes = (uint64_t)sd->rw_blocks * BLOCK_SIZE;
		if (sd->rw_lba + sd->rw_blocks > sd->blocks) { scsi_set_sense(sd, 5, 0x21, 0); sd->data_len = 0; break; }
		if (bytes > SCSI_MAX_XFER) { scsi_set_sense(sd, 5, 0x24, 0); sd->data_len = 0; break; }
		b = scsi_buf((int)bytes);
		if (!FileSeek(&sd->file, (__off64_t)sd->rw_lba * BLOCK_SIZE, SEEK_SET))
		{
			scsi_set_sense(sd, 3, 0x11, 0);
			sd->data_len = 0;
			break;
		}
		diskled_on();
		int got = FileReadAdv(&sd->file, b, (int)bytes);
		if (got < (int)bytes) memset(b + (got > 0 ? got : 0), 0, bytes - (got > 0 ? got : 0));
		sd->data_len = (int)bytes;
		break;
	}

	case 0x0a: case 0x2a: case 0x2e: case 0xaa: // WRITE
	{
		scsi_rw_params(sd);
		uint64_t bytes = (uint64_t)sd->rw_blocks * BLOCK_SIZE;
		if (sd->rw_lba + sd->rw_blocks > sd->blocks) { scsi_set_sense(sd, 5, 0x21, 0); sd->data_len = 0; break; }
		if (sd->readonly) { scsi_set_sense(sd, 7, 0x27, 0); sd->data_len = 0; break; }
		if (bytes > SCSI_MAX_XFER) bytes = SCSI_MAX_XFER;
		if (!FileSeek(&sd->file, (__off64_t)sd->rw_lba * BLOCK_SIZE, SEEK_SET))
		{
			scsi_set_sense(sd, 3, 0x0c, 0);
			sd->data_len = 0;
			break;
		}
		diskled_on();
		FileWriteAdv(&sd->file, g_buf, (int)bytes);
		sd->data_len = 0;
		break;
	}

	default:
		scsi_set_sense(sd, 5, 0x20, 0); // INVALID COMMAND OPERATION CODE
		sd->data_len = 0;
		break;
	}

	sd->offset = 0;
}

// ---------------------------------------------------------------- WD33C93
// Port of amiberry a2091.cpp (COMMODORE_DMAC), single board.

#define WD_OWN_ID               0x00
#define WD_CONTROL              0x01
#define WD_CDB_1                0x03
#define WD_T_SECTORS            0x03
#define WD_T_HEADS              0x04
#define WD_T_CYLS_0             0x05
#define WD_T_CYLS_1             0x06
#define WD_L_ADDR_0             0x07
#define WD_L_ADDR_1             0x08
#define WD_L_ADDR_2             0x09
#define WD_L_ADDR_3             0x0a
#define WD_SECTOR               0x0b
#define WD_HEAD                 0x0c
#define WD_CYL_0                0x0d
#define WD_CYL_1                0x0e
#define WD_TARGET_LUN           0x0f
#define WD_COMMAND_PHASE        0x10
#define WD_TRANSFER_COUNT_MSB   0x12
#define WD_TRANSFER_COUNT       0x13
#define WD_TRANSFER_COUNT_LSB   0x14
#define WD_DESTINATION_ID       0x15
#define WD_SCSI_STATUS          0x17
#define WD_COMMAND              0x18
#define WD_DATA                 0x19

#define WD_CMD_RESET            0x00
#define WD_CMD_ABORT            0x01
#define WD_CMD_ASSERT_ATN       0x02
#define WD_CMD_NEGATE_ACK       0x03
#define WD_CMD_SEL_ATN          0x06
#define WD_CMD_SEL              0x07
#define WD_CMD_SEL_ATN_XFER     0x08
#define WD_CMD_SEL_XFER         0x09
#define WD_CMD_TRANS_ADDR       0x18
#define WD_CMD_TRANS_INFO       0x20
#define WD_CMD_TRANSFER_PAD     0x21

#define CSR_MSGIN               0x20
#define CSR_SELECT              0x11
#define CSR_TRANS_ADDR          0x15
#define CSR_SEL_XFER_DONE       0x16
#define CSR_XFER_DONE           0x18
#define CSR_INVALID             0x40
#define CSR_TIMEOUT             0x42
#define CSR_BAD_STATUS          0x45
#define CSR_UNEXP               0x48
#define CSR_DISC                0x85
#define CSR_SRV_REQ             0x88

#define PHS_DATA_OUT            0x00
#define PHS_DATA_IN             0x01
#define PHS_COMMAND             0x02
#define PHS_STATUS              0x03
#define PHS_MESS_OUT            0x06
#define PHS_MESS_IN             0x07

#define CTL_EDI                 0x08

struct wd_chip_state
{
	uint8_t    wdregs[32];
	uint32_t   acr;
	int        wd_data_avail;     // 0 none, 1 PIO (DBR), -1 DMA pending (BSY)
	bool       wd_selected;
	bool       wd_busy;
	uint8_t    wd_phase;
	uint8_t    wd_data[32];
	int        wd_dataoffset;
	scsi_data *scsi;
	bool       pio_write;         // direction of the pending PIO transfer
};

static wd_chip_state wd;
static int g_enabled = 0;

static void wd_sync_avail(void)
{
	// mirror wd_data_avail into AUX STATUS (DBR / BSY) in the FPGA
	if (g_defer) return;      // computed once in mb_flush()
	uint16_t c = 0;
	if (wd.wd_data_avail > 0)      c = CTL_SET_DBR | CTL_CLR_BSYX;
	else if (wd.wd_data_avail < 0) c = CTL_SET_DBR | CTL_SET_BSYX;
	else                           c = CTL_CLR_DBR | CTL_CLR_BSYX;
	mb_ctrl(c);
}

// amiberry set_status(): queue a status; delay in amiberry irq units
// (delay 0 -> next hsync). The FPGA presents queued statuses one at a time.
static void set_status(uint8_t status, int delay)
{
	// amiberry counts hsyncs (64 us). The FIFO already guarantees one status at
	// a time, after the previous one was read, so short delays are dropped and
	// long ones (reset, selection timeout) scaled down by 8.
	int irq = delay == 0 ? 1 : (delay <= 2 ? 2 : delay);
	int ticks = (irq - 1) / 8;
	if (ticks > 255) ticks = 255;
	uint16_t w = (uint16_t)((ticks << 8) | status);
	if (g_defer && g_status_n < 8) g_status_pending[g_status_n++] = w;
	else mb_write(OP_STATUS, &w, 1);
	a2log("  status %02x (delay %d)\n", status, ticks);
}

static void set_status(uint8_t status) { set_status(status, 0); }

static uint32_t gettc(void)
{
	return wd.wdregs[WD_TRANSFER_COUNT_LSB] | (wd.wdregs[WD_TRANSFER_COUNT] << 8) | (wd.wdregs[WD_TRANSFER_COUNT_MSB] << 16);
}

static void settc(uint32_t tc)
{
	wd.wdregs[WD_TRANSFER_COUNT_LSB] = tc & 0xff;
	wd.wdregs[WD_TRANSFER_COUNT] = (tc >> 8) & 0xff;
	wd.wdregs[WD_TRANSFER_COUNT_MSB] = (tc >> 16) & 0xff;
}

static bool decreasetc(void)
{
	uint32_t tc = gettc();
	if (!tc) return true;
	tc--;
	settc(tc);
	return tc == 0;
}

static void setphase(uint8_t phase)
{
	wd.wdregs[WD_COMMAND_PHASE] = phase;
}

static int canwddma(void)
{
	uint8_t mode = wd.wdregs[WD_CONTROL] >> 5;
	if (mode != 0 && mode != 4 && mode != 1) a2log("WD weird DMA mode %d\n", mode);
	return mode == 4 || mode == 1;
}

// amiberry do_dma_commodore(), DMAC rev 2: word transfers, ACR += 2 per word
static bool do_dma(void)
{
	scsi_data *scsi = wd.scsi;
	wd.wd_data_avail = 0;
	if (!scsi) return false;
	if (scsi->direction == 0) a2log("WD DMA but no data!?\n");

	uint32_t tc = gettc();
	uint32_t acr = wd.acr & 0xffffff;

	if (scsi->direction < 0)
	{
		int avail = scsi->data_len - scsi->offset;
		if (avail <= 0 || !scsi->data_len) return true;
		int n = tc ? ((int)tc < avail ? (int)tc : avail) : 1;
		uint8_t *src = g_buf + scsi->offset;
		int words = n / 2;
		if (words) mb_dma_to_amiga(acr, src, words);
		if (n > 0) memcpy(wd.wd_data, src, n > (int)sizeof(wd.wd_data) ? sizeof(wd.wd_data) : n);
		scsi->offset += n;
		settc(tc ? tc - n : 0);
		wd.acr = (acr + ((n + 1) / 2) * 2) & 0xffffff;
		a2log("  DMA from WD %d bytes -> %06x, %d/%d\n", n, acr, scsi->offset, scsi->data_len);
		return true;
	}
	else if (scsi->direction > 0)
	{
		int room = scsi->data_len - scsi->offset;
		int n = tc ? (int)tc : 1;
		if (room > 0 && n > room) n = room;
		if (room <= 0) n = tc ? (int)tc : 1;
		int words = (n + 1) / 2;
		uint8_t *tmp = (uint8_t *)malloc(words * 2 + 2);
		if (!tmp) return false;
		mb_dma_from_amiga(acr, tmp, words);
		for (int i = 0; i < n; i++) scsi_send_data(scsi, tmp[i]);
		if (n > 0) memcpy(wd.wd_data, tmp, n > (int)sizeof(wd.wd_data) ? sizeof(wd.wd_data) : n);
		free(tmp);
		settc(tc ? tc - n : 0);
		wd.acr = (acr + words * 2) & 0xffffff;
		a2log("  DMA to WD %d bytes <- %06x, %d/%d\n", n, acr, scsi->offset, scsi->data_len);
		return true;
	}
	return false;
}

static void wd_cmd_sel_xfer(bool atn);

static bool wd_do_transfer_out(void)
{
	scsi_data *scsi = wd.scsi;
	if (wd.wdregs[WD_COMMAND_PHASE] < 0x20)
	{
		int msg = wd.wd_data[0];
		setphase(0x20);
		wd.wd_phase = CSR_XFER_DONE | PHS_COMMAND;
		scsi->status = 0;
		scsi_start_transfer(scsi);
		scsi->message[0] = msg;
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x30)
	{
		if (scsi->offset < scsi->data_len)
		{
			wd.wd_phase = CSR_XFER_DONE | PHS_COMMAND;
			setphase(0x30 + scsi->offset);
			set_status(wd.wd_phase, 1);
			return false;
		}
		settc(0);
		scsi_start_transfer(scsi);
		scsi_emulate_analyze(scsi);
		a2log("  CDB %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			scsi->cmd[0], scsi->cmd[1], scsi->cmd[2], scsi->cmd[3], scsi->cmd[4],
			scsi->cmd[5], scsi->cmd[6], scsi->cmd[7], scsi->cmd[8], scsi->cmd[9]);
		if (scsi->direction > 0)
		{
			if (scsi->data_len <= 0 || scsi->direction == 0)
			{
				wd.wd_phase = CSR_XFER_DONE | PHS_STATUS;
				setphase(0x46);
			}
			else
			{
				wd.wd_phase = CSR_XFER_DONE | PHS_DATA_OUT;
				setphase(0x45);
			}
		}
		else
		{
			scsi_emulate_cmd(scsi);
			if (scsi->data_len <= 0 || scsi->direction == 0)
			{
				wd.wd_phase = CSR_XFER_DONE | PHS_STATUS;
				setphase(0x46);
			}
			else
			{
				wd.wd_phase = CSR_XFER_DONE | PHS_DATA_IN;
				setphase(0x45);
			}
		}
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x46 || wd.wdregs[WD_COMMAND_PHASE] == 0x45)
	{
		if (scsi->offset < scsi->data_len)
		{
			wd.wd_phase = CSR_XFER_DONE | (scsi->direction < 0 ? PHS_DATA_IN : PHS_DATA_OUT);
			set_status(wd.wd_phase, 10);
			return false;
		}
		settc(0);
		if (scsi->direction > 0)
		{
			scsi_emulate_cmd(scsi);
			scsi->data_len = 0;
			wd.wd_phase = CSR_XFER_DONE | PHS_STATUS;
		}
		scsi_start_transfer(scsi);
		setphase(0x47);
	}
	wd.wd_dataoffset = 0;
	if (wd.wdregs[WD_COMMAND] == WD_CMD_SEL_ATN_XFER || wd.wdregs[WD_COMMAND] == WD_CMD_SEL_XFER)
	{
		wd_cmd_sel_xfer(false);
		return true;
	}
	set_status(wd.wd_phase, scsi->direction <= 0 ? 0 : 1);
	wd.wd_busy = false;
	return true;
}

static bool wd_do_transfer_in(bool message_in_transfer_info)
{
	scsi_data *scsi = wd.scsi;
	wd.wd_dataoffset = 0;
	if (wd.wdregs[WD_COMMAND_PHASE] >= 0x36 && wd.wdregs[WD_COMMAND_PHASE] < 0x46)
	{
		if (scsi->offset < scsi->data_len)
		{
			wd.wd_phase = CSR_XFER_DONE | (scsi->direction < 0 ? PHS_DATA_IN : PHS_DATA_OUT);
			set_status(wd.wd_phase, 1);
			return false;
		}
		if (gettc() != 0)
		{
			wd.wd_phase = CSR_UNEXP | PHS_STATUS;
			setphase(0x46);
		}
		else
		{
			wd.wd_phase = CSR_XFER_DONE | PHS_STATUS;
			setphase(0x46);
			if (wd.wdregs[WD_COMMAND] == WD_CMD_SEL_ATN_XFER || wd.wdregs[WD_COMMAND] == WD_CMD_SEL_XFER)
			{
				wd_cmd_sel_xfer(false);
				return true;
			}
		}
		scsi_start_transfer(scsi);
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x46 || wd.wdregs[WD_COMMAND_PHASE] == 0x47)
	{
		if (wd.wdregs[WD_COMMAND] == WD_CMD_SEL_ATN_XFER || wd.wdregs[WD_COMMAND] == WD_CMD_SEL_XFER)
		{
			wd_cmd_sel_xfer(false);
			return true;
		}
		setphase(0x50);
		wd.wd_phase = CSR_XFER_DONE | PHS_MESS_IN;
		scsi_start_transfer(scsi);
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x50)
	{
		if (!message_in_transfer_info)
		{
			wd.wd_phase = CSR_DISC;
			wd.wd_selected = false;
			scsi_start_transfer(scsi);
			setphase(0x60);
		}
		else
		{
			wd.wd_phase = CSR_MSGIN;
		}
	}
	set_status(wd.wd_phase, 1);
	scsi->direction = 0;
	return true;
}

static void wd_cmd_sel_xfer(bool atn)
{
	int i, tmp_tc;
	scsi_data *scsi;

	(void)atn;
	wd.wd_data_avail = 0;
	tmp_tc = gettc();
	int id = wd.wdregs[WD_DESTINATION_ID] & 7;
	scsi = wd.scsi = (id < A2091_NDRV && g_scsi[id].present) ? &g_scsi[id] : NULL;
	if (!scsi)
	{
		set_status(CSR_TIMEOUT, 0);
		wd.wdregs[WD_COMMAND_PHASE] = 0x00;
		a2log("* select and transfer ID=%d: no device\n", id);
		return;
	}
	if (!wd.wd_selected)
	{
		scsi->message[0] = 0x80;
		wd.wd_selected = true;
		wd.wdregs[WD_COMMAND_PHASE] = 0x10;
	}
	a2log("* select and transfer ID=%d PHASE=%02X TC=%d ctrl=%02x\n", id, wd.wdregs[WD_COMMAND_PHASE], tmp_tc, wd.wdregs[WD_CONTROL]);

	if (wd.wdregs[WD_COMMAND_PHASE] <= 0x30)
	{
		scsi->status = 0;
		memcpy(scsi->cmd, &wd.wdregs[3], 12);
		memset(scsi->cmd + 12, 0, 4);
		scsi->data_len = tmp_tc;
		scsi_emulate_analyze(scsi);
		settc(scsi->cmd_len);
		wd.wd_dataoffset = 0;
		scsi_start_transfer(scsi);
		scsi->direction = 2;
		scsi->data_len = scsi->cmd_len;
		int tc = gettc();
		for (i = 0; i < tc; i++)
		{
			uint8_t b = scsi->cmd[i];
			if (i < (int)sizeof(wd.wd_data)) wd.wd_data[i] = b;
			scsi_send_data(scsi, b);
			wd.wd_dataoffset++;
		}
		scsi->data_len = tmp_tc;
		a2log("  CDB %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x len=%d\n",
			scsi->cmd[0], scsi->cmd[1], scsi->cmd[2], scsi->cmd[3], scsi->cmd[4],
			scsi->cmd[5], scsi->cmd[6], scsi->cmd[7], scsi->cmd[8], scsi->cmd[9], scsi->data_len);
		if (!scsi_emulate_analyze(scsi))
		{
			wd.wdregs[WD_COMMAND_PHASE] = 0x46;
			goto end;
		}
		wd.wdregs[WD_COMMAND_PHASE] = 0x30 + gettc();
		settc(0);
		if (scsi->direction <= 0) scsi_emulate_cmd(scsi);
		scsi_start_transfer(scsi);
	}

	if (wd.wdregs[WD_COMMAND_PHASE] <= 0x41) wd.wdregs[WD_COMMAND_PHASE] = 0x44;
	if (wd.wdregs[WD_COMMAND_PHASE] == 0x44) wd.wdregs[WD_COMMAND_PHASE] = 0x45;

	if (wd.wdregs[WD_COMMAND_PHASE] == 0x45)
	{
		settc(tmp_tc);
		wd.wd_dataoffset = 0;
		setphase(0x45);

		if (gettc() == 0)
		{
			if (scsi->direction != 0)
			{
				if (scsi->direction < 0)
				{
					if (scsi->data_len == 0 || scsi->offset >= scsi->data_len)
					{
						setphase(0x46);
						goto end;
					}
				}
				wd.wd_phase = CSR_UNEXP | (scsi->direction < 0 ? PHS_DATA_IN : PHS_DATA_OUT);
				set_status(wd.wd_phase, 1);
				return;
			}
		}

		if (scsi->direction)
		{
			if (scsi->direction < 0 && scsi->data_len == 0 && gettc())
			{
				setphase(0x46);
				wd.wd_phase = CSR_UNEXP | PHS_STATUS;
				set_status(wd.wd_phase, 1);
				return;
			}
			if (canwddma() > 0)
			{
				if (scsi->direction <= 0)
				{
					do_dma();
					if (scsi->offset < scsi->data_len)
					{
						wd.wd_phase = CSR_UNEXP | PHS_DATA_IN;
						set_status(wd.wd_phase, 1);
						return;
					}
					if (gettc() > 0)
					{
						setphase(0x46);
						wd.wd_phase = CSR_UNEXP | PHS_STATUS;
						set_status(wd.wd_phase, 1);
						return;
					}
					setphase(0x46);
				}
				else
				{
					if (do_dma())
					{
						setphase(0x46);
						if (scsi->offset < scsi->data_len)
						{
							wd.wd_phase = CSR_UNEXP | PHS_DATA_OUT;
							set_status(wd.wd_phase, 1);
							return;
						}
						scsi_emulate_cmd(scsi);
					}
				}
			}
			else
			{
				// no DMA = Service Request
				wd.wd_phase = CSR_SRV_REQ | (scsi->direction < 0 ? PHS_DATA_IN : PHS_DATA_OUT);
				set_status(wd.wd_phase, 1);
				return;
			}
		}
		else
		{
			if (gettc())
			{
				wd.wd_phase = CSR_UNEXP | PHS_STATUS;
				set_status(wd.wd_phase, 1);
				return;
			}
			setphase(0x46);
		}
	}

end:
	if (wd.wdregs[WD_COMMAND_PHASE] == 0x46)
	{
		wd.wdregs[WD_COMMAND_PHASE] = 0x50;
		wd.wdregs[WD_TARGET_LUN] = scsi->status;
	}

	wd.wdregs[WD_COMMAND_PHASE] = 0x60;
	if (!(wd.wdregs[WD_CONTROL] & CTL_EDI))
	{
		wd.wd_phase = CSR_SEL_XFER_DONE;
		set_status(wd.wd_phase, 2);
		wd.wd_phase = CSR_DISC;
		set_status(wd.wd_phase, 0);
	}
	else
	{
		wd.wd_phase = CSR_SEL_XFER_DONE;
		set_status(wd.wd_phase, 2);
	}
	wd.wd_selected = false;
	a2log("  done status=%02x\n", scsi->status);
}

// PIO: the FPGA streams a whole block through the WD DATA register from/into
// its 4 KB buffer, so the 68k never waits on the HPS between bytes. The
// stream is armed in mb_flush(), after the register write-back.
static int      g_pio_n = 0;          // bytes in the stream being moved
static uint16_t g_pio_start = 0;      // OP_PIO operand waiting for mb_flush()

static void mb_buf_write(const uint8_t *p, int n)
{
	EnableIO();
	spi_w(A2_CMD_WRITE);
	spi_w(OP_BUFWR);
	fast_write_bytes(p, n);
	DisableIO();
}

static void mb_buf_read(uint8_t *p, int n)
{
	EnableIO();
	spi_w(A2_CMD_BUFRD);
	fast_read_bytes(p, n);
	DisableIO();
}

// amiberry moves one byte per DATA access until the transfer count runs
// out or the target has no more data (reads) / has all it wants (writes);
// work out how many that is and hand the whole block to the FPGA.
static void wd_pio_start(void)
{
	scsi_data *scsi = wd.scsi;
	if (!scsi || wd.wd_data_avail <= 0) return;
	uint32_t tc = gettc();
	int n;
	if (wd.pio_write)
	{
		int lim = (scsi->direction == 2 ? scsi->cmd_len : scsi->data_len) - scsi->offset;
		n = tc ? (int)tc : 1;
		if (lim > 0 && n > lim) n = lim;
	}
	else
	{
		int avail = scsi->data_len - scsi->offset;
		n = tc ? (int)tc : 1;
		if (avail > 0 && n > avail) n = avail;
		if (avail <= 0) n = 1;
	}
	if (n > PIO_MAX) n = PIO_MAX;
	if (n < 1) n = 1;

	if (!wd.pio_write)
	{
		static uint8_t z[2];
		int avail = scsi->data_len - scsi->offset;
		mb_buf_write(avail > 0 ? g_buf + scsi->offset : z, avail > 0 ? n : 1);
	}
	g_pio_n = n;
	g_pio_start = (uint16_t)((wd.pio_write ? 0x8000 : 0) | n);
	a2log("  PIO %s %d bytes\n", wd.pio_write ? "out" : "in", n);
}

static void wd_cmd_trans_info(bool pad)
{
	scsi_data *scsi = wd.scsi;
	if (!scsi) { set_status(CSR_INVALID, 10); return; }

	if (wd.wdregs[WD_COMMAND_PHASE] == 0x20)
	{
		wd.wdregs[WD_COMMAND_PHASE] = 0x30;
		scsi->status = 0;
	}
	wd.wd_busy = true;
	if (wd.wdregs[WD_COMMAND] & 0x80) settc(1);
	if (gettc() == 0) settc(1);
	wd.wd_dataoffset = 0;

	if (wd.wdregs[WD_COMMAND_PHASE] == 0x30)
	{
		scsi->direction = 2;
		scsi->cmd_len = scsi->data_len = gettc();
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x10)
	{
		scsi->direction = 1;
		scsi->data_len = gettc();
		scsi_buf(scsi->data_len);
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x45)
	{
		scsi_emulate_analyze(scsi);
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x46 || wd.wdregs[WD_COMMAND_PHASE] == 0x47)
	{
		scsi_buf(16)[0] = scsi->status;
		wd.wdregs[WD_TARGET_LUN] = scsi->status;
		scsi->direction = -1;
		scsi->data_len = 1;
	}
	else if (wd.wdregs[WD_COMMAND_PHASE] == 0x50)
	{
		scsi->direction = -1;
		scsi->data_len = gettc();
		scsi_buf(scsi->data_len + 1)[0] = 0; // COMMAND COMPLETE
	}

	if (pad)
	{
		settc(0);
		if (scsi->direction < 0) wd_do_transfer_in(false);
		else if (scsi->direction > 0) wd_do_transfer_out();
		scsi->direction = 0;
		wd.wd_data_avail = 0;
	}
	else
	{
		wd.pio_write = scsi->direction > 0;
		wd.wd_data_avail = canwddma() > 0 ? -1 : 1;
		wd_pio_start();
	}
	a2log("* transfer info phase=%02x TC=%d dir=%d data=%d/%d\n",
		wd.wdregs[WD_COMMAND_PHASE], gettc(), scsi->direction, scsi->offset, scsi->data_len);
}

static void wd_cmd_trans_addr(void)
{
	uint32_t tcyls = (wd.wdregs[WD_T_CYLS_0] << 8) | wd.wdregs[WD_T_CYLS_1];
	uint32_t theads = wd.wdregs[WD_T_HEADS];
	uint32_t tsectors = wd.wdregs[WD_T_SECTORS];
	uint32_t lba = (wd.wdregs[WD_L_ADDR_0] << 24) | (wd.wdregs[WD_L_ADDR_1] << 16) |
		(wd.wdregs[WD_L_ADDR_2] << 8) | wd.wdregs[WD_L_ADDR_3];
	if (!theads || !tsectors) { set_status(CSR_BAD_STATUS); return; }
	uint32_t cyls = lba / (theads * tsectors);
	uint32_t heads = (lba - cyls * theads * tsectors) / tsectors;
	uint32_t sectors = (lba - cyls * theads * tsectors) % tsectors;
	wd.wdregs[WD_CYL_0] = cyls >> 8;
	wd.wdregs[WD_CYL_1] = cyls;
	wd.wdregs[WD_HEAD] = heads;
	wd.wdregs[WD_SECTOR] = sectors;
	set_status(cyls >= tcyls ? CSR_BAD_STATUS : CSR_TRANS_ADDR);
}

static void wd_cmd_sel(bool atn)
{
	wd.wd_phase = 0;
	wd.wdregs[WD_COMMAND_PHASE] = 0;
	int id = wd.wdregs[WD_DESTINATION_ID] & 7;
	scsi_data *scsi = wd.scsi = (id < A2091_NDRV && g_scsi[id].present) ? &g_scsi[id] : NULL;
	a2log("* select%s ID=%d %s\n", atn ? " with atn" : "", id, scsi ? "" : "(no device)");
	if (!scsi)
	{
		set_status(CSR_TIMEOUT, 1000);
		return;
	}
	scsi_start_transfer(scsi);
	wd.wd_selected = true;
	scsi->message[0] = 0x80;
	set_status(CSR_SELECT, 2);
	if (atn)
	{
		wd.wdregs[WD_COMMAND_PHASE] = 0x10;
		set_status(CSR_SRV_REQ | PHS_MESS_OUT, 4);
	}
	else
	{
		wd.wdregs[WD_COMMAND_PHASE] = 0x20;
		set_status(CSR_SRV_REQ | PHS_COMMAND, 4);
	}
}

static void wd_cmd_reset(bool irq)
{
	for (int i = 1; i <= 0x16; i++) wd.wdregs[i] = 0;
	wd.wdregs[0x18] = 0;
	wd.wd_selected = false;
	wd.scsi = NULL;
	wd.wd_data_avail = 0;
	g_pio_n = 0;
	g_pio_start = 0;
	mb_ctrl(CTL_WD_RESET | CTL_CLR_DBR | CTL_CLR_BSYX);
	if (irq)
	{
		uint8_t status = (wd.wdregs[0] & 0x08) ? 1 : 0;
		set_status(status, 50);
	}
	else
	{
		wd.wd_busy = false;
		mb_ctrl(CTL_CLR_BUSY);
	}
}

static void wd_execute_cmd(int cmd)
{
	// amiberry wd_execute_cmd(): the target is whatever DESTINATION ID names
	int id = wd.wdregs[WD_DESTINATION_ID] & 7;
	wd.scsi = (id < A2091_NDRV && g_scsi[id].present) ? &g_scsi[id] : NULL;
	a2log("WD command %02x\n", cmd);

	switch (cmd & 0x7f)
	{
	case WD_CMD_RESET:
		wd_cmd_reset(true);
		break;
	case WD_CMD_ABORT:
		mb_ctrl(CTL_CLR_BUSY);
		break;
	case WD_CMD_ASSERT_ATN:
		wd.wdregs[WD_COMMAND_PHASE] = 0x10;
		mb_ctrl(CTL_CLR_BUSY);
		break;
	case WD_CMD_SEL:
		wd_cmd_sel(false);
		break;
	case WD_CMD_SEL_ATN:
		wd_cmd_sel(true);
		break;
	case WD_CMD_SEL_ATN_XFER:
		wd_cmd_sel_xfer(true);
		break;
	case WD_CMD_SEL_XFER:
		wd_cmd_sel_xfer(false);
		break;
	case WD_CMD_TRANS_INFO:
		wd_cmd_trans_info(false);
		break;
	case WD_CMD_TRANS_ADDR:
		wd_cmd_trans_addr();
		break;
	case WD_CMD_NEGATE_ACK:
		if (wd.wd_phase == CSR_MSGIN && wd.wd_selected && wd.scsi) wd_do_transfer_in(false);
		else mb_ctrl(CTL_CLR_BUSY);
		break;
	case WD_CMD_TRANSFER_PAD:
		wd_cmd_trans_info(true);
		break;
	default:
		a2log("WD unimplemented/unknown command %02X\n", cmd);
		set_status(CSR_INVALID, 10);
		break;
	}
}

// amiberry scsi_hsync_check_dma(): a TRANSFER INFO in DMA mode waits for ST_DMA
static void wd_check_dma(int dmac_dma)
{
	if (wd.wd_data_avail < 0 && dmac_dma && wd.scsi)
	{
		bool v;
		do_dma();
		if (wd.scsi->direction < 0) v = wd_do_transfer_in(false);
		else if (wd.scsi->direction > 0) v = wd_do_transfer_out();
		else { a2log("WD data transfer attempt without data!\n"); v = true; }
		if (v)
		{
			wd.scsi->direction = 0;
			wd.wd_data_avail = 0;
		}
		else mb_ctrl(CTL_DMA_DONE);   // set_dma_done(): wait for the next ST_DMA
		wd_sync_avail();
	}
}

// The FPGA has moved a whole PIO block (g_pio_n bytes)
static void wd_pio_event(void)
{
	if (!wd.scsi || !wd.wd_data_avail || !g_pio_n) return;
	int n = g_pio_n;
	g_pio_n = 0;
	bool done = false;

	if (wd.pio_write)
	{
		static uint8_t tmp[PIO_MAX + 2];
		mb_buf_read(tmp, n);
		for (int i = 0; i < n && !done; i++)
		{
			if (wd.wd_dataoffset < (int)sizeof(wd.wd_data)) wd.wd_data[wd.wd_dataoffset] = tmp[i];
			wd.wd_dataoffset++;
			decreasetc();
			if (scsi_send_data(wd.scsi, tmp[i]) || gettc() == 0) done = true;
		}
		if (done)
		{
			wd.wd_data_avail = 0;
			wd_do_transfer_out();
		}
	}
	else
	{
		for (int i = 0; i < n && !done; i++)
		{
			uint8_t v = 0;
			int status = scsi_receive_data(wd.scsi, &v, true);
			if (wd.wd_dataoffset < (int)sizeof(wd.wd_data)) wd.wd_data[wd.wd_dataoffset] = v;
			wd.wd_dataoffset++;
			decreasetc();
			if (status || gettc() == 0) done = true;
		}
		if (done)
		{
			wd.wd_data_avail = 0;
			wd_do_transfer_in(true);
		}
	}
	if (!done) wd_pio_start();     // longer than one buffer: next block
	a2log("  PIO done %d bytes%s\n", n, done ? "" : ", more");
}

// ---------------------------------------------------------------- config

struct a2091_settings
{
	int  enable;
	char rom[1024];
	char hdd[A2091_NDRV][1024];
};

static a2091_settings g_set;

static int load_rom(void)
{
	static const char *names[] = {
		"a2091.rom", "A2091.ROM", "a590.rom", "A590.ROM",
		"roms/a2091.rom", "roms/a590.rom", "roms/amiga-boot-a590.rom",
		"roms/amiga-boot-a2091.rom", NULL
	};

	char path[1024], keypath[1024];
	static uint8_t raw[16384 + 11];
	static uint8_t rom[16384] __attribute__((aligned(4)));
	int size = 0;

	for (int i = -1; i < 0 || names[i]; i++)
	{
		if (i < 0)
		{
			if (!g_set.rom[0]) continue;
			snprintf(path, sizeof(path), "%s", g_set.rom);
		}
		else snprintf(path, sizeof(path), "%s/%s", HomeDir(), names[i]);
		if (!FileExists(path, 0)) continue;
		size = FileLoad(path, 0, 0);
		if (size == 16384 || size == 16384 + 11)
		{
			FileLoad(path, raw, size);
			break;
		}
		printf("A2091: %s has unsupported size %d\n", path, size);
		size = 0;
	}
	if (!size)
	{
		printf("A2091: no boot ROM found (put a 16 KB A590/A2091 ROM at %s/a2091.rom)\n", HomeDir());
		return 0;
	}

	if (size == 16384 + 11)
	{
		// Cloanto "AMIROMTYPE1" image, XOR with rom.key from the same folder
		if (memcmp(raw, "AMIROMTYPE1", 11))
		{
			printf("A2091: %s: unknown ROM header\n", path);
			return 0;
		}
		strcpy(keypath, path);
		char *slash = strrchr(keypath, '/');
		strcpy(slash ? slash + 1 : keypath, "rom.key");
		static uint8_t key[4096];
		int klen = FileLoad(keypath, 0, 0);
		if (klen <= 0 || klen > (int)sizeof(key) || !FileLoad(keypath, key, klen))
		{
			printf("A2091: %s is encrypted and %s is missing\n", path, keypath);
			return 0;
		}
		for (int i = 0; i < 16384; i++) rom[i] = raw[11 + i] ^ key[i % klen];
	}
	else memcpy(rom, raw, 16384);

	// soft-dumped ROM without the $2000 offset (amiberry a2091_init)
	int i;
	for (i = 1; i <= 256; i++) if (rom[16384 - i] != 0x00 && rom[16384 - i] != 0xff) break;
	if (i > 256)
	{
		uint8_t tmp[0x2000];
		memcpy(tmp, rom + 0x2000, 0x2000);
		memcpy(rom + 0x2000, rom, 0x2000);
		memcpy(rom, tmp, 0x2000);
	}

	EnableIO();
	spi_w(A2_CMD_WRITE);
	spi_w(OP_ROM);
	fpga_spi_fast_block_write_be((const uint16_t *)rom, 8192);
	DisableIO();

	printf("A2091: boot ROM %s loaded\n", path);
	return 1;
}

static void wd_state_reset(void)
{
	memset(&wd, 0, sizeof(wd));
	g_pio_n = 0;
	g_pio_start = 0;
}

// ---------------------------------------------------------------- settings
// A2091 settings live in their own file so the Minimig config struct (and
// config/Minimig.cfg) are untouched:
//
//   config/minimig_a2091.ini
//     enable=1                     # 0 = remove the board from autoconfig
//     rom=games/Amiga/a2091.rom    # optional, default: search (see load_rom)
//     id0=../usb0/games/Amiga/scsi0.hdf
//     ...
//     id6=...
//
// Without the file the board stays present (it autoconfigs from power-up)
// and the boot ROM is loaded if one is found, with no drives attached.

static void trim(char *s)
{
	char *e = s + strlen(s);
	while (e > s && (e[-1] == '\r' || e[-1] == '\n' || e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
	char *b = s;
	while (*b == ' ' || *b == '\t') b++;
	if (b != s) memmove(s, b, strlen(b) + 1);
}

static void load_settings(void)
{
	memset(&g_set, 0, sizeof(g_set));
	g_set.enable = 1;

	char path[1024];
	snprintf(path, sizeof(path), "%s/config/minimig_a2091.ini", getRootDir());
	FILE *f = fopen(path, "r");
	if (!f) f = fopen("/media/fat/config/minimig_a2091.ini", "r");
	if (!f)
	{
		// No ini: the OSD's SCSI page decides (Minimig.cfg scsi_cfg + scsi[i]
		// = SCSI ID i+1, shared with the A4091). Only the selected controller
		// gets the drives.
		g_set.enable = minimig_scsi_controller() == SCSI_CTRL_A2091;
		if (g_set.enable)
			for (int i = 0; i < 6; i++)
				if (minimig_config.scsi[i].cfg && minimig_config.scsi[i].filename[0])
					snprintf(g_set.hdd[i + 1], sizeof(g_set.hdd[0]), "%s", minimig_config.scsi[i].filename);
		return;
	}

	char line[1100];
	while (fgets(line, sizeof(line), f))
	{
		char *h = strchr(line, '#');
		if (h) *h = 0;
		char *eq = strchr(line, '=');
		if (!eq) continue;
		*eq = 0;
		char *key = line, *val = eq + 1;
		trim(key);
		trim(val);
		if (!strcasecmp(key, "enable")) g_set.enable = atoi(val);
		else if (!strcasecmp(key, "rom")) snprintf(g_set.rom, sizeof(g_set.rom), "%s", val);
		else if (!strncasecmp(key, "id", 2) && key[2] >= '0' && key[2] <= '6' && !key[3])
			snprintf(g_set.hdd[key[2] - '0'], sizeof(g_set.hdd[0]), "%s", val);
	}
	fclose(f);
}

void a2091_apply_config(void)
{
	uint16_t st = mb_status();
	if ((st & ST_MARKER_M) != ST_MARKER)
	{
		g_enabled = 0;
		return;     // core without the A2091
	}

	load_settings();
	int enable = g_set.enable;

	for (int i = 0; i < A2091_NDRV; i++)
	{
		scsi_data *sd = &g_scsi[i];
		if (sd->file.opened()) FileClose(&sd->file);
		memset(sd, 0, sizeof(*sd));
		sd->id = i;

		const char *name = g_set.hdd[i];
		if (!enable || !name[0]) continue;

		int rw = FileCanWrite(name);
		if (FileOpenEx(&sd->file, name, rw ? O_RDWR : O_RDONLY))
		{
			sd->blocks = sd->file.size / BLOCK_SIZE;
			sd->present = sd->blocks > 0;
			sd->readonly = !rw;
			printf("A2091: ID%d = %s (%llu MB%s)\n", i, name,
				(unsigned long long)(sd->file.size >> 20), rw ? "" : ", read only");
		}
		else printf("A2091: ID%d: cannot open %s\n", i, name);
	}

	if (enable) load_rom();     // without a ROM the board is still present, just not bootable

	wd_state_reset();
	mb_cfg(enable | (minimig_scsi_controller() == SCSI_CTRL_A4091 ? 2 : 0));
	mb_ctrl(CTL_CLR_EV_CMD | CTL_CLR_EV_STDMA | CTL_CLR_EV_PIO | CTL_CLR_EV_RESET | CTL_WD_RESET | CTL_CLR_BUSY | CTL_CLR_DBR | CTL_CLR_BSYX);
	g_enabled = enable;
	printf("A2091 SCSI %s\n", enable ? "enabled" : "disabled");
}

// ---------------------------------------------------------------- poll

static void mb_flush(const uint8_t *snap)
{
	// 1. registers the command engine changed (SCSI STATUS / COMMAND are
	//    FPGA owned, DATA streams through the buffer)
	for (int i = 0; i < 0x17; i++)
	{
		if (wd.wdregs[i] == snap[i]) continue;
		int j = i;
		while (j + 1 < 0x17 && wd.wdregs[j + 1] != snap[j + 1]) j++;
		mb_wdregs(i, &wd.wdregs[i], j - i + 1);
		i = j;
	}

	// 2. control bits, with DBR/BSY reflecting the final data_avail state
	g_defer = 0;
	uint16_t c = g_ctrl_pending;
	g_ctrl_pending = 0;
	if (g_pio_start)               c |= CTL_CLR_BSYX;      // OP_PIO raises DBR itself
	else if (wd.wd_data_avail > 0) c |= CTL_SET_DBR | CTL_CLR_BSYX;
	else if (wd.wd_data_avail < 0) c |= CTL_SET_DBR | CTL_SET_BSYX;
	else                           c |= CTL_CLR_DBR | CTL_CLR_BSYX;
	mb_ctrl_now(c);

	// 3. arm the PIO stream (sets DBR in the FPGA)
	if (g_pio_start) mb_write(OP_PIO, &g_pio_start, 1);
	g_pio_start = 0;

	// 4. interrupts last
	if (g_status_n) mb_write(OP_STATUS, g_status_pending, g_status_n);
	g_status_n = 0;
}

static int a2091_service(void)
{
	mb_regs r;
	uint8_t snap[32];
	uint16_t st = mb_status();
	if ((st & ST_MARKER_M) != ST_MARKER) return 0;

	if (!(st & (ST_EV_CMD | ST_EV_PIO | ST_EV_RESET | ST_EV_STDMA))) return 0;

	if (st & ST_EV_RESET)
	{
		a2log("bus reset\n");
		mb_ctrl_now(CTL_CLR_EV_RESET);
		wd_state_reset();
		return 1;
	}

	// acknowledge first: an event raised while we work must survive
	if (g_prof > 0) g_prof_ev[(st & ST_EV_CMD) ? 0 : (st & ST_EV_PIO) ? 1 : 2]++;
	if (st & ST_EV_CMD)        mb_ctrl_now(CTL_CLR_EV_CMD);
	else if (st & ST_EV_PIO)   mb_ctrl_now(CTL_CLR_EV_PIO);
	else if (st & ST_EV_STDMA) mb_ctrl_now(CTL_CLR_EV_STDMA);

	uint16_t st2 = mb_read(&r);
	st = (st & (ST_EV_CMD | ST_EV_PIO | ST_EV_STDMA)) | (st2 & ~(ST_EV_CMD | ST_EV_PIO | ST_EV_STDMA));
	memcpy(wd.wdregs, r.wd, sizeof(wd.wdregs));
	memcpy(snap, r.wd, sizeof(snap));
	wd.acr = r.acr;
	uint32_t acr0 = wd.acr;

	g_defer = 1;
	g_ctrl_pending = 0;
	g_status_n = 0;

	if (st & ST_EV_CMD)
	{
		wd.wd_busy = true;
		wd_execute_cmd(wd.wdregs[WD_COMMAND]);
		if (wd.wd_data_avail < 0) wd_check_dma(st & ST_DMAC_DMA);
	}
	else if (st & ST_EV_PIO)
	{
		wd_pio_event();
	}
	else if (st & ST_EV_STDMA)
	{
		wd_check_dma(1);
	}

	if (wd.acr != acr0) mb_acr(wd.acr);
	mb_flush(snap);
	return 1;
}

#ifndef A2091_IDLE_US
#define A2091_IDLE_US 50000
#endif

// Profiling (A2091_PROFILE file in the storage root): event counts and time
// spent in the handler vs. waiting for the 68k, summarised every 2 s.

static void prof_tick(uint64_t now)
{
	if (now - g_prof_t0 < 2000000) return;
	if (g_prof_ev[0] | g_prof_ev[1] | g_prof_ev[2])
	{
		char path[1024];
		snprintf(path, sizeof(path), "%s/logs/a2091_prof.log", getRootDir());
		FILE *f = fopen(path, "a");
		if (f)
		{
			fprintf(f, "2s: cmd %llu pio %llu stdma %llu, handler %.1f ms (%.1f us/event)\n",
				(unsigned long long)g_prof_ev[0], (unsigned long long)g_prof_ev[1], (unsigned long long)g_prof_ev[2],
				g_prof_busy / 1000.0,
				(double)g_prof_busy / (double)(g_prof_ev[0] + g_prof_ev[1] + g_prof_ev[2]));
			fclose(f);
		}
	}
	memset(g_prof_ev, 0, sizeof(g_prof_ev));
	g_prof_busy = 0;
	g_prof_t0 = now;
}

void a2091_poll(void)
{
	if (g_prof < 0)
	{
		char path[1024];
		snprintf(path, sizeof(path), "%s/a2091_profile", getRootDir());
		g_prof = !access(path, F_OK) || !access("/media/fat/a2091_profile", F_OK);
	}
	if (!g_enabled) return;
	if (!a2091_service()) return;

	// a command sequence is in flight: stay attached for a while so the next
	// WD command is picked up at SPI speed instead of once per main loop
	// Stay attached until the board has been quiet for 50 ms (PIO phases move
	// one byte per event), but never hold the main loop for more than 2 s.
	uint64_t start = now_us(), last = start;
	while (1)
	{
		uint64_t t = now_us();
		int did = a2091_service();
		if (did)
		{
			last = t;
			if (g_prof > 0) g_prof_busy += now_us() - t;
		}
		if (g_prof > 0) prof_tick(t);
		else if (t - last > A2091_IDLE_US) break;
		if (t - start > 2000000) break;
	}
}

int a2091_core_supported(void)
{
	return (mb_status() & ST_MARKER_M) == ST_MARKER;
}

// Does the running core carry an A4091? Cores with the A2091 mailbox say so
// in capability word 39; a core without the mailbox is an A4091-only build.
int a4091_core_supported(void)
{
	if (!a2091_core_supported()) return 1;
	EnableIO();
	spi_w(A2_CMD_READ);
	uint16_t v[40];
	fpga_spi_fast_block_read(v, 40);
	DisableIO();
	return v[39] & 1;
}

// SCSI controller selection (OSD Drives page, Minimig.cfg scsi_cfg):
//   bit0 = SCSI on, bit1 = A2091 chosen (else A4091; old configs = A4091).
// The A4091 is Zorro III: a 68000/68010 or a core without it gets the A2091.
int minimig_scsi_controller(void)
{
	if (!(minimig_config.scsi_cfg & 1)) return SCSI_CTRL_NONE;
	int has2091 = a2091_core_supported();
	int has4091 = a4091_core_supported() && (minimig_config.cpu & 2);
	if (has4091 && !(minimig_config.scsi_cfg & 2)) return SCSI_CTRL_A4091;
	if (has2091) return SCSI_CTRL_A2091;
	return has4091 ? SCSI_CTRL_A4091 : SCSI_CTRL_NONE;
}

// Byte-exact DMA to/from 24-bit Amiga RAM (chip, slow, Zorro II) through the
// A2091's DMA engine and chipdma_arb, for other HPS-side bus masters in the
// same core (the A4091 software SIOP can only reach Zorro III RAM itself).
// Returns 1 if handled.
int a2091_dma_rw(uint32_t addr, uint8_t *buf, uint32_t len, int to_amiga)
{
	if (!len) return 1;
	if (addr >= 0x1000000 || addr + len > 0x1000000) return 0;
	if (!a2091_core_supported()) return 0;

	static uint8_t span[DMA_BUF_WORDS * 2];
	while (len)
	{
		uint32_t a0 = addr & ~1u;
		uint32_t n = len;
		if ((addr - a0) + n > sizeof(span)) n = sizeof(span) - (addr - a0);
		uint32_t a1 = (addr + n + 1) & ~1u;
		int words = (a1 - a0) / 2;

		if (to_amiga)
		{
			// keep the neighbouring bytes of a partial first / last word
			if (addr & 1) mb_dma_from_amiga(a0, span, 1);
			if ((addr + n) & 1) mb_dma_from_amiga(a1 - 2, span + (a1 - 2 - a0), 1);
			memcpy(span + (addr - a0), buf, n);
			mb_dma_to_amiga(a0, span, words);
		}
		else
		{
			mb_dma_from_amiga(a0, span, words);
			memcpy(buf, span + (addr - a0), n);
		}
		addr += n;
		buf += n;
		len -= n;
	}
	return 1;
}
