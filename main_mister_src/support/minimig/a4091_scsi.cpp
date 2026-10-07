// A4091 software SIOP - C1 SCSI-2 direct-access target.
//
// Provides the scsi710_req_* / scsi710_device_find seam that a4091_lsi.cpp
// (the ported 53C710 SCRIPTS VM) calls, backed by a port of a4091_target.v's
// command decode and FileReadAdv/FileWriteAdv on the mounted .hdf images.
//
// Up to 6 targets (SCSI IDs 1..6). a4091_scsi_set_image() is called from
// a4091_apply_config().
//
// One HBA, one command at a time -> a single data buffer, grown on demand to
// the exact transfer size (WinUAE scsi.cpp does the same). No per-command
// slicing: the whole DATA phase is one FileRead/FileWriteAdv.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "a4091_lsi_glue.h"
#include "a4091_scsi_defs.h"
#include "minimig_a4091.h"

#define NDEV        7          // index by SCSI ID; [0] unused
#define BLOCK_SIZE  512
// Largest single READ/WRITE we serve. The 53C710 SCRIPTS `datain`/`dataout`
// loop has 9 S/G segments; past ~128 KB in one command the a4091.device
// SCRIPTS re-run the segment list and desync (observed: 128 KB ok, 256 KB
// wedges). Real AmigaOS FS I/O is <=64 KB. An over-cap request gets
// CHECK CONDITION / INVALID FIELD IN CDB, never a hang. (Lifting this
// needs multi-pass DATA-phase handling in a4091_lsi.cpp - a follow-up.)
#define RW_MAX_BYTES (1024u * 1024)

extern "C" void a4091_lsi_log(const char *fmt, ...);
#define slog a4091_lsi_log

// ---- per-target state -------------------------------------------------
struct a4091_target {
	SCSIDevice sdev;          // what scsi710_device_find returns
	fileTYPE  *f;
	uint64_t   blocks;        // capacity in 512 B blocks
	int        present;
	// current request working state (one command at a time per HBA)
	uint8_t    cmd[16];
	int        cmd_len;
	int        data_len;      // >0 bytes ready / expected; <0 = done-no-data
	int        direction;     // >0 to device (write), <0 from device (read), 0 none
	int        status;        // SCSI status byte
	uint8_t    sense_key, sense_asc, sense_ascq;
	uint64_t   rw_lba;
	uint32_t   rw_blocks;
};

static a4091_target g_tgt[NDEV];

// ---- the one shared data buffer (grows, never shrinks) ---------------
static uint8_t *g_buf   = NULL;
static size_t   g_bufsz = 0;

static uint8_t *xbuf(size_t need)
{
	if (need > g_bufsz) {
		size_t n = need + 64 * 1024;
		uint8_t *p = (uint8_t *)realloc(g_buf, n);
		if (!p) return NULL;
		g_buf = p; g_bufsz = n;
	}
	return g_buf;
}

// ---------------------------------------------------------------------
void a4091_scsi_set_image(int id, fileTYPE *f, uint64_t bytes)
{
	if (id < 1 || id >= NDEV) return;
	a4091_target *t = &g_tgt[id];
	t->f       = (f && f->opened()) ? f : 0;
	t->blocks  = bytes >> 9;
	t->present = t->f && t->blocks;
	t->sdev.id = id;
	t->sdev.handle = t;
	slog("scsi: ID%d %s (%llu blocks)\n", id, t->present ? "present" : "absent",
	     (unsigned long long)t->blocks);
}

void a4091_scsi_reset(void)
{
	for (int i = 0; i < NDEV; i++) {
		g_tgt[i].data_len = 0; g_tgt[i].direction = 0; g_tgt[i].status = 0;
		g_tgt[i].sense_key = g_tgt[i].sense_asc = g_tgt[i].sense_ascq = 0;
	}
}

// ---- RDBFF_LAST fixup on the read path ------------------------------
// AmigaOS mounts A4091 drives at boot via the ROM driver's mounter, which
// walks SCSI targets in order and stops at the first drive whose
// RigidDiskBlock claims to be the last one:
//
//     3rdparty/mounter/mounter.c
//         md->wasLastDev = (flags & RDBFF_LAST) != 0;   // rdb_Flags bit 0
//         if (md->wasLastDev && !ms->ignoreLast) break; // no further targets
//
// .hdf images are partitioned independently, each as "the only drive", so
// they all carry rdb_Flags bit 0 set - and only the first configured SCSI ID
// ever gets mounted (HDToolBox still lists them all: it opens each unit
// directly). Rather than edit the user's images, present the flag that
// matches the CURRENT chain: the highest configured ID keeps RDBFF_LAST,
// every other drive gets it cleared, with the RDB checksum recomputed.
//
// Read path only - the bytes on disk are never modified. If AmigaOS ever
// writes the block back (HDToolBox "Save Changes"), what lands on disk is
// the corrected flag, which is what the image should have said anyway.
#define RDB_LOCATION_LIMIT 16          // RDB lives in the first 16 blocks
#define RDBFF_LAST         0x01

static int rdb_highest_present_id(void)
{
	int last = 0;
	for (int i = 1; i < NDEV; i++)
		if (g_tgt[i].present) last = i;
	return last;
}

static uint32_t rdb_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static void rdb_wbe32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

// Patch one 512-byte block in place if it is an RDSK block. Returns 1 if the
// flags changed.
static int rdb_patch_block(uint8_t *blk, int want_last)
{
	if (memcmp(blk, "RDSK", 4)) return 0;

	uint32_t summed = rdb_be32(blk + 4);
	if (summed < 0x10 || summed > BLOCK_SIZE / 4) summed = 0x40;  // sane default

	uint32_t flags = rdb_be32(blk + 0x14);
	uint32_t want  = want_last ? (flags | RDBFF_LAST) : (flags & ~RDBFF_LAST);
	if (want == flags) return 0;

	rdb_wbe32(blk + 0x14, want);
	rdb_wbe32(blk + 8, 0);                       // zero rdb_ChkSum first
	uint32_t sum = 0;
	for (uint32_t i = 0; i < summed; i++) sum += rdb_be32(blk + i * 4);
	rdb_wbe32(blk + 8, (uint32_t)(-(int32_t)sum));
	return 1;
}

// Called on every served READ. Cheap: only touches reads that reach into the
// first 16 blocks, and only rewrites an actual RDSK block.
static void rdb_fixup_last(a4091_target *t, uint8_t *b, uint32_t bytes)
{
	if (t->rw_lba >= RDB_LOCATION_LIMIT) return;

	int last_id  = rdb_highest_present_id();
	int want_last = (last_id == 0) || ((int)t->sdev.id == last_id);
	uint32_t nblk = bytes / BLOCK_SIZE;

	for (uint32_t i = 0; i < nblk; i++) {
		uint64_t lba = t->rw_lba + i;
		if (lba >= RDB_LOCATION_LIMIT) break;
		if (rdb_patch_block(b + i * BLOCK_SIZE, want_last))
			slog("scsi: ID%d RDB block %llu: RDBFF_LAST %s (last ID is %d)\n",
			     t->sdev.id, (unsigned long long)lba,
			     want_last ? "set" : "cleared", last_id);
	}
}

// ---- SCSIDevice lookup ---------------------------------------------
SCSIDevice *scsi710_device_find(SCSIBus *bus, int channel, int target, int lun)
{
	(void)bus; (void)channel;
	if (lun != 0 || target < 1 || target >= NDEV) return 0;
	a4091_target *t = &g_tgt[target];
	return t->present ? &t->sdev : 0;
}

// ---- CDB helpers -------------------------------------------------
static uint32_t be16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static uint32_t be32(const uint8_t *p) { return (p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3]; }
static void wbe32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }

static void set_check(a4091_target *t, uint8_t key, uint8_t asc, uint8_t ascq)
{
	t->status = 0x02;   // CHECK CONDITION
	t->sense_key = key; t->sense_asc = asc; t->sense_ascq = ascq;
	slog("  a4091 tgt CHECK key=%02x asc=%02x/%02x (op %02x)\n",
	     key, asc, ascq, t->cmd[0]);
}

static inline int is_rw(const a4091_target *t)
{
	uint8_t o = t->cmd[0];
	return o == 0x08 || o == 0x28 || o == 0x0a || o == 0x2a;
}

// ---- analyze: direction + total transfer length (a4091_target.v T_DECODE)
static void analyze(a4091_target *t)
{
	const uint8_t *c = t->cmd;
	t->direction = 0;
	t->data_len  = 0;
	t->status    = 0x00;
	t->rw_lba    = 0;
	t->rw_blocks = 0;

	switch (c[0]) {
	case 0x00: case 0x1b: case 0x01:      // TUR / START-STOP / REZERO
	case 0x35:                            // SYNCHRONIZE CACHE
	case 0x0b: case 0x2b:                 // SEEK(6/10) - no mechanism
		t->direction = 0; break;

	case 0x04:                            // FORMAT UNIT - virtual .hdf, no-op
		// FmtData=1 (byte1 bit4): initiator sends a defect-list parameter
		// list. We must still consume that DATA-OUT phase or the SCRIPTS VM
		// desyncs (HDToolBox "low-level format" hang). HDToolBox sends a
		// bare 4-byte header (empty list); accept and discard it.
		if (c[1] & 0x10) { t->direction = 1; t->data_len = 4; }
		else             { t->direction = 0; }
		break;

	case 0x15:                            // MODE SELECT(6) - accept + discard
		t->direction = c[4] ? 1 : 0;
		t->data_len  = c[4];
		break;
	case 0x55:                            // MODE SELECT(10) - accept + discard
		t->data_len  = be16(c + 7);
		t->direction = t->data_len ? 1 : 0;
		break;

	case 0x03:                            // REQUEST SENSE
		t->direction = -1;
		t->data_len  = (c[4] && c[4] < 18) ? c[4] : 18;
		break;
	case 0x12:                            // INQUIRY
		t->direction = -1;
		t->data_len  = (c[4] && c[4] < 36) ? c[4] : 36;
		break;
	case 0x25:                            // READ CAPACITY(10)
		t->direction = -1; t->data_len = 8; break;
	case 0x9e:                            // SERVICE ACTION IN(16): READ CAPACITY(16) = SA 0x10
		t->direction = -1;
		t->data_len  = ((c[1] & 0x1f) == 0x10) ? 32 : 8;
		break;
	case 0x1a:                            // MODE SENSE(6)
		t->direction = -1;
		t->data_len  = c[4] ? c[4] : 4;   // allocation length; emulate() clamps
		break;
	case 0x37:                            // READ DEFECT DATA(10)
		t->direction = -1;
		t->data_len  = 4;                 // header only, empty defect list
		break;

	case 0x08:                            // READ(6)
		t->rw_lba = ((c[1] & 0x1f) << 16) | (c[2] << 8) | c[3];
		t->rw_blocks = c[4] ? c[4] : 256;
		t->direction = -1;
		break;
	case 0x28:                            // READ(10)
		t->rw_lba = be32(c + 2); t->rw_blocks = be16(c + 7);
		t->direction = -1;
		break;
	case 0x0a:                            // WRITE(6)
		t->rw_lba = ((c[1] & 0x1f) << 16) | (c[2] << 8) | c[3];
		t->rw_blocks = c[4] ? c[4] : 256;
		t->direction = 1;
		break;
	case 0x2a:                            // WRITE(10)
		t->rw_lba = be32(c + 2); t->rw_blocks = be16(c + 7);
		t->direction = 1;
		break;

	default:
		set_check(t, 0x05, 0x20, 0x00);   // ILLEGAL REQUEST / invalid opcode
		break;
	}

	if (is_rw(t)) {
		uint64_t bytes = (uint64_t)t->rw_blocks * BLOCK_SIZE;
		if (t->rw_lba + t->rw_blocks > t->blocks)
			set_check(t, 0x05, 0x21, 0x00);   // LBA out of range
		else if (bytes > RW_MAX_BYTES)
			set_check(t, 0x05, 0x24, 0x00);   // INVALID FIELD IN CDB (xfer too big)
		if (t->status) {                      // reject before any DATA phase
			t->direction = 0;
			t->data_len  = 0;
		} else {
			t->data_len = (int)bytes;
		}
	}
}

// ---- execute: fill buffer for reads / commit writes / no-data cmds --
static void emulate(a4091_target *t)
{
	const uint8_t *c = t->cmd;
	uint8_t *b = xbuf(t->data_len > 64 ? (size_t)t->data_len : 64);
	if (!b) { set_check(t, 0x04, 0x00, 0x00); t->data_len = 0; return; }

	switch (c[0]) {
	case 0x00: case 0x1b: case 0x01: case 0x04: case 0x35:
	case 0x0b: case 0x2b:
	case 0x15: case 0x55:                  // MODE SELECT(6/10) - discard params
		t->status = 0x00; t->data_len = 0;
		break;

	case 0x03: {                           // REQUEST SENSE
		int n = (c[4] && c[4] < 18) ? c[4] : 18;
		memset(b, 0, 18);
		b[0] = 0x70; b[2] = t->sense_key; b[7] = 0x0a;
		b[12] = t->sense_asc; b[13] = t->sense_ascq;
		t->sense_key = t->sense_asc = t->sense_ascq = 0;
		t->data_len = n; t->status = 0x00;
		break;
	}
	case 0x12: {                           // INQUIRY
		int n = (c[4] && c[4] < 36) ? c[4] : 36;
		memset(b, 0, 36);
		b[0] = 0x00; b[1] = 0x00;
		b[2] = 0x02; b[3] = 0x02;          // SCSI-2
		b[4] = 31;
		memcpy(b + 8,  "MiSTer  ", 8);
		memcpy(b + 16, "A4091 HD        ", 16);
		memcpy(b + 32, "0001", 4);
		t->data_len = n; t->status = 0x00;
		break;
	}
	case 0x25:                             // READ CAPACITY(10)
		wbe32(b + 0, (uint32_t)(t->blocks - 1));
		wbe32(b + 4, BLOCK_SIZE);
		t->data_len = 8; t->status = 0x00;
		break;

	case 0x9e:                             // READ CAPACITY(16)
		if ((c[1] & 0x1f) != 0x10) { set_check(t, 0x05, 0x20, 0x00); t->data_len = 0; break; }
		memset(b, 0, 32);
		wbe32(b + 0, 0);
		wbe32(b + 4, (uint32_t)(t->blocks - 1));
		wbe32(b + 8, BLOCK_SIZE);
		t->data_len = 32; t->status = 0x00;
		break;

	case 0x1a: {                           // MODE SENSE(6)
		uint8_t  pc   = c[2] & 0x3f;       // page code (0x3f = all)
		int      dbd  = (c[1] >> 3) & 1;   // disable block descriptor
		uint32_t heads = 16, spt = 63;
		uint32_t cyls = (uint32_t)(t->blocks / ((uint64_t)heads * spt));
		if (!cyls) cyls = 1;

		if (pc != 0x03 && pc != 0x04 && pc != 0x3f) {
			set_check(t, 0x05, 0x24, 0x00);   // INVALID FIELD IN CDB
			t->data_len = 0; break;
		}

		memset(b, 0, 64);
		int p = 4;                        // 4-byte MODE SENSE(6) header
		if (!dbd) {
			b[3] = 8;                     // block descriptor length
			b[5] = (uint8_t)(t->blocks >> 16);
			b[6] = (uint8_t)(t->blocks >> 8);
			b[7] = (uint8_t)(t->blocks);
			b[9]  = (BLOCK_SIZE >> 8) & 0xff;
			b[10] =  BLOCK_SIZE       & 0xff;
			p = 12;
		}
		if (pc == 0x03 || pc == 0x3f) {   // Format Device Parameters
			uint8_t *pg = b + p;
			pg[0] = 0x03; pg[1] = 0x16;
			pg[10] = (spt >> 8) & 0xff; pg[11] = spt & 0xff;
			pg[12] = (BLOCK_SIZE >> 8) & 0xff; pg[13] = BLOCK_SIZE & 0xff;
			pg[20] = 0x40;               // HSEC
			p += 24;
		}
		if (pc == 0x04 || pc == 0x3f) {   // Rigid Disk Drive Geometry
			uint8_t *pg = b + p;
			pg[0] = 0x04; pg[1] = 0x16;
			pg[2] = (cyls >> 16) & 0xff; pg[3] = (cyls >> 8) & 0xff; pg[4] = cyls & 0xff;
			pg[5] = (uint8_t)heads;
			pg[20] = 0x1c; pg[21] = 0x20; // 7200 rpm
			p += 24;
		}
		b[0] = (uint8_t)(p - 1);          // mode data length (excludes this byte)

		int n = c[4] ? c[4] : 4;
		if (n > p) n = p;
		t->data_len = n; t->status = 0x00;
		break;
	}

	case 0x37: {                           // READ DEFECT DATA(10) - empty list
		memset(b, 0, 4);
		b[1] = c[2] & 0x1f;               // echo P/G/format bits
		t->data_len = 4; t->status = 0x00;
		break;
	}

	case 0x08: case 0x28: {                // READ(6/10)
		if (t->status != 0) { t->data_len = 0; break; }   // range error from analyze
		uint32_t bytes = t->rw_blocks * BLOCK_SIZE;
		if (!FileSeek(t->f, (__off64_t)t->rw_lba << 9, SEEK_SET)) {
			set_check(t, 0x04, 0x00, 0x00); t->data_len = 0; break;
		}
		diskled_on();
		int got = FileReadAdv(t->f, b, bytes);
		if (got != (int)bytes) memset(b + (got > 0 ? got : 0), 0, bytes - (got > 0 ? got : 0));
		rdb_fixup_last(t, b, bytes);
		t->data_len = bytes; t->status = 0x00;
		break;
	}
	case 0x0a: case 0x2a: {                // WRITE(6/10) - b[] already DMA-filled
		if (t->status != 0) { t->data_len = -1; break; }
		uint32_t bytes = t->rw_blocks * BLOCK_SIZE;
		if (!FileSeek(t->f, (__off64_t)t->rw_lba << 9, SEEK_SET)) {
			set_check(t, 0x04, 0x00, 0x00); t->data_len = -1; break;
		}
		diskled_on();
		FileWriteAdv(t->f, b, bytes);
		t->data_len = -1; t->status = 0x00;
		break;
	}

	default:
		set_check(t, 0x05, 0x20, 0x00);
		t->data_len = 0;
		break;
	}
}

// ---- scsi710_req_* seam (mirrors WinUAE ncr_scsi.cpp) ---------------
static SCSIRequest g_req;          // one HBA, one command at a time

SCSIRequest *scsi710_req_new(SCSIDevice *d, uint32_t tag, uint32_t lun,
                             uint8_t *buf, int len, void *hba_private)
{
	a4091_target *t = (a4091_target *)d->handle;
	if (len > 16) len = 16;
	memcpy(t->cmd, buf, len);
	t->cmd_len = len;

	memset(&g_req, 0, sizeof(g_req));
	g_req.dev = d;
	g_req.tag = tag;
	g_req.lun = lun;
	g_req.hba_private = hba_private;
	return &g_req;
}

int32_t scsi710_req_enqueue(SCSIRequest *req)
{
	a4091_target *t = (a4091_target *)req->dev->handle;
	t->data_len = 0;
	analyze(t);
	slog("a4091 tgt CDB %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
	     "  dir=%d len=%d st=%02x\n",
	     t->cmd[0], t->cmd[1], t->cmd[2], t->cmd[3], t->cmd[4], t->cmd[5],
	     t->cmd[6], t->cmd[7], t->cmd[8], t->cmd[9],
	     t->direction, t->data_len, t->status);
	if (t->direction <= 0) emulate(t);        // reads + no-data run now
	if (t->direction == 0) return 1;
	return -t->direction;                     // >0 = DATA IN, <0 = DATA OUT
}

void scsi710_req_continue(SCSIRequest *req)
{
	a4091_target *t = (a4091_target *)req->dev->handle;
	if (t->data_len < 0) {
		lsi710_command_complete(req, t->status, 0);
	} else if (t->data_len > 0) {
		lsi710_transfer_data(req, t->data_len);
	} else {
		if (t->direction > 0) emulate(t);     // write: buffer is now DMA-filled
		lsi710_command_complete(req, t->status, 0);
	}
}

uint8_t *scsi710_req_get_buf(SCSIRequest *req)
{
	a4091_target *t = (a4091_target *)req->dev->handle;
	t->data_len = 0;
	size_t n = t->rw_blocks ? (size_t)t->rw_blocks * BLOCK_SIZE : g_bufsz ? g_bufsz : 64;
	uint8_t *b = xbuf(n);
	// WRITE: pre-zero so a short / no-data DATA-OUT lands zeros, not stale
	// bytes. HDToolBox low-level format issues WRITE(6) with no data buffer
	// and expects the "drive" to zero the blocks.
	if (b && (t->cmd[0] == 0x0a || t->cmd[0] == 0x2a)) memset(b, 0, n);
	return b;
}

void scsi710_req_unref(SCSIRequest *req)      { (void)req; }
void scsi710_req_cancel(SCSIRequest *req)     { (void)req; }
