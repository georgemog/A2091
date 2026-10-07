// A2091 co-simulation: rtl/a2091.v (Verilator) + the real HPS code
// (main_mister/minimig_a2091.cpp) with spi_w() routed to the mailbox, and a
// scripted 68k that drives the board the way the Commodore driver does.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "Va2091.h"
#include "verilated.h"
#include "file_io.h"
#include "fpga_io.h"
#include "support/minimig/minimig_config.h"
#include "support/minimig/minimig_a2091.h"

static Va2091 *top;
static uint64_t cycles;
static uint16_t mem[0x100000];   // 2 MB chip RAM, word addressed
static int dma_lat;
static int z2 = 0;

static void tick()
{
	top->clk = 0; top->eval();
	top->clk = 1; top->eval();
	cycles++;
	// memory model for the DMA port: ack after 4 clocks
	top->dma_ack = 0;
	if (top->dma_req)
	{
		if (++dma_lat == 4)
		{
			dma_lat = 0;
			uint32_t a = top->dma_addr & 0xfffff;
			if (top->dma_we) mem[a] = top->dma_wdata;
			else top->dma_rdata = mem[a];
			top->dma_ack = 1;
		}
	}
}

static void ticks(int n) { while (n--) tick(); }

// ---------------------------------------------------------------- hps_ext
static int io_active, io_first;

void EnableIO() { io_active = 1; io_first = 1; ticks(2); }
void DisableIO() { io_active = 0; ticks(2); }

uint16_t spi_w(uint16_t w)
{
	uint16_t r;
	if (io_first)
	{
		io_first = 0;
		top->hp_cmd = w & 3;
		top->hp_start = 1;
		r = top->hp_status;
		tick();
		top->hp_start = 0;
		ticks(6);
		return r;
	}
	top->hp_wdata = w;
	top->hp_strobe = 1;
	r = top->hp_rdata;
	tick();
	top->hp_strobe = 0;
	ticks(6);
	return r;
}

// fast block transfers: no handshake on hardware; here just more strobes
// with the minimum spacing hps_ext sees (a few clocks)
#ifndef SPI_GAP
#define SPI_GAP 3
#endif
static uint16_t spi_fast(uint16_t w)
{
	top->hp_wdata = w; top->hp_strobe = 1;
	uint16_t r = top->hp_rdata;
	tick(); top->hp_strobe = 0; ticks(SPI_GAP);
	return r;
}
void fpga_spi_fast_block_write(const uint16_t *b, uint32_t n) { while (n--) spi_fast(*b++); }
void fpga_spi_fast_block_read(uint16_t *b, uint32_t n) { while (n--) *b++ = spi_fast(0); }
void fpga_spi_fast_block_write_be(const uint16_t *b, uint32_t n) { while (n--) { uint16_t v = *b++; spi_fast((uint16_t)((v << 8) | (v >> 8))); } }
void fpga_spi_fast_block_read_be(uint16_t *b, uint32_t n) { while (n--) { uint16_t v = spi_fast(0); *b++ = (uint16_t)((v << 8) | (v >> 8)); } }

// ---------------------------------------------------------------- stubs
mm_configTYPE minimig_config;
void diskled_on() {}
const char *user_io_get_core_path(const char *, int) { return "."; }
const char *getRootDir() { return "."; }

int FileOpenEx(fileTYPE *file, const char *name, int mode, char, int)
{
	file->f = fopen(name, (mode & O_RDWR) ? "r+b" : "rb");
	if (!file->f) return 0;
	fseeko(file->f, 0, SEEK_END);
	file->size = ftello(file->f);
	fseeko(file->f, 0, SEEK_SET);
	strncpy(file->name, name, sizeof(file->name) - 1);
	return 1;
}
int FileClose(fileTYPE *file) { if (file->f) fclose(file->f); file->f = 0; return 1; }
int FileSeek(fileTYPE *file, __off64_t off, int origin) { return fseeko(file->f, off, origin) == 0; }
int FileReadAdv(fileTYPE *file, void *b, int len, int) { return (int)fread(b, 1, len, file->f); }
int FileWriteAdv(fileTYPE *file, void *b, int len, int) { return (int)fwrite(b, 1, len, file->f); }
int FileExists(const char *name, int) { return access(name, F_OK) == 0; }
int FileCanWrite(const char *name) { return access(name, W_OK) == 0; }
int FileLoad(const char *name, void *b, int size)
{
	FILE *f = fopen(name, "rb");
	if (!f) return 0;
	fseek(f, 0, SEEK_END);
	int n = (int)ftell(f);
	if (b) { fseek(f, 0, SEEK_SET); n = (int)fread(b, 1, size, f); }
	fclose(f);
	return n;
}

// ---------------------------------------------------------------- 68k bus
static uint16_t bus(uint16_t off, int rd, uint16_t d, int hw, int lw)
{
	tick();
	top->sel = 1; top->addr = off >> 1; top->din = d;
	top->rd = rd; top->hwr = hw; top->lwr = lw;
	ticks(6);
	uint16_t v = top->dout;
	top->sel = 0; top->rd = 0; top->hwr = 0; top->lwr = 0;
	ticks(6);
	a2091_poll();     // the ARM runs concurrently
	return v;
}
static uint16_t rw(uint16_t off) { return bus(off, 1, 0, 0, 0); }
static uint8_t  rb(uint16_t off) { uint16_t v = bus(off & ~1, 1, 0, 0, 0); return (off & 1) ? v & 0xff : v >> 8; }
static void ww(uint16_t off, uint16_t v) { bus(off, 0, v, 1, 1); }
static void wb(uint16_t off, uint8_t v) { if (off & 1) bus(off & ~1, 0, v, 0, 1); else bus(off, 0, v << 8, 1, 0); }

static void wd_w(int reg, uint8_t v) { wb(0x91, reg); wb(0x93, v); }
static uint8_t wd_r(int reg) { wb(0x91, reg); return rb(0x93); }
static uint8_t aux() { return rb(0x91); }

static int errors;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); errors++; } } while (0)

// wait for WD INT, return SCSI STATUS
static int wait_int(int maxloops = 20000)
{
	for (int i = 0; i < maxloops; i++)
	{
		uint8_t a = aux();
		if (a & 0x80) return wd_r(0x17);
		ticks(200);
	}
	return -1;
}

// Commodore-style select-and-transfer with DMA
static int sel_xfer(int id, const uint8_t *cdb, int cdblen, uint32_t buf, uint32_t len, int write, uint8_t *phase)
{
	ww(0x84, buf >> 16);           // ACR
	ww(0x86, buf & 0xffff);
	ww(0x42, 0x10 | (write ? 0x08 : 0));  // CNTR: INTEN, DDIR
	ww(0xe0, 0);                   // ST_DMA
	wd_w(0x01, 0x80);              // CONTROL: DMA mode (bus master), no EDI
	wd_w(0x15, id);                // destination ID
	wd_w(0x0f, 0);                 // target LUN
	wb(0x91, 0x03);
	for (int i = 0; i < cdblen; i++) wb(0x93, cdb[i]);
	wd_w(0x12, len >> 16);
	wd_w(0x13, len >> 8);
	wd_w(0x14, len);
	wd_w(0x10, 0);                 // command phase
	wd_w(0x18, 0x09);              // SELECT AND TRANSFER
	int st = wait_int();
	*phase = wd_r(0x10);
	ww(0xe2, 0);                   // SP_DMA
	return st;
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	top = new Va2091;
	top->configured = 1;
	top->z2ram_ena = z2;
	top->reset = 1; ticks(10); top->reset = 0; ticks(10);

	// HDF: 2 MB with a pattern
	const char *hdf = "sim_test.hdf";
	{
		FILE *f = fopen(hdf, "wb");
		static uint8_t blk[512];
		for (int b = 0; b < 4096; b++)
		{
			for (int i = 0; i < 512; i++) blk[i] = (uint8_t)(b * 7 + i * 13 + (i >> 8));
			fwrite(blk, 1, 512, f);
		}
		fclose(f);
	}
	// boot ROM (any 16 KB file) so apply_config enables the board
	{
		FILE *f = fopen("a2091.rom", "wb");
		static uint8_t rom[16384];
		for (int i = 0; i < 16384; i++) rom[i] = i & 0xff;
		fwrite(rom, 1, sizeof(rom), f);
		fclose(f);
	}

	FILE *ini = (mkdir("config",0755), fopen("config/minimig_a2091.ini","w")); fprintf(ini,"enable=1\nid0=%s\nrom=a2091.rom\n",hdf); fclose(ini);
	a2091_apply_config();
	CHECK(top->enabled, "board not enabled");
	CHECK(rw(0x2000) == 0x0001, "rom $2000 = %04x", rw(0x2000));

	// ---- WD reset
	wd_w(0x00, 0x07);                         // own ID 7
	wd_w(0x18, 0x00);                         // RESET
	CHECK(aux() & 0x20, "busy after RESET command");
	int st = wait_int();
	CHECK(st == 0x00, "reset status %02x", st);
	CHECK(!(aux() & 0xa0), "aux after reset %02x", aux());

	uint8_t phase;

	// ---- INQUIRY to ID 0 into $1000
	uint8_t inq[6] = { 0x12, 0, 0, 0, 36, 0 };
	st = sel_xfer(0, inq, 6, 0x1000, 36, 0, &phase);
	CHECK(st == 0x16, "inquiry status %02x", st);
	CHECK(phase == 0x60, "inquiry phase %02x", phase);
	CHECK(wd_r(0x0f) == 0, "inquiry target status %02x", wd_r(0x0f));
	char vend[9] = {};
	for (int i = 0; i < 4; i++) { vend[i * 2] = mem[0x800 + 4 + i] >> 8; vend[i * 2 + 1] = mem[0x800 + 4 + i] & 0xff; }
	CHECK(!strcmp(vend, "MiSTer  "), "inquiry vendor '%s'", vend);
	st = wait_int();
	CHECK(st == 0x85, "disconnect status %02x", st);

	// ---- INQUIRY to absent ID 3: selection timeout
	st = sel_xfer(3, inq, 6, 0x1000, 36, 0, &phase);
	CHECK(st == 0x42, "timeout status %02x", st);

	// ---- READ(10) 8 blocks from LBA 100 into $20000
	uint8_t rd10[10] = { 0x28, 0, 0, 0, 0, 100, 0, 0, 8, 0 };
	st = sel_xfer(0, rd10, 10, 0x20000, 8 * 512, 0, &phase);
	CHECK(st == 0x16, "read status %02x", st);
	wait_int();
	int bad = 0;
	for (int i = 0; i < 8 * 512 && !bad; i++)
	{
		int b = 100 + i / 512, o = i % 512;
		uint8_t exp = (uint8_t)(b * 7 + o * 13 + (o >> 8));
		uint16_t w = mem[(0x20000 + i) >> 1];
		uint8_t got = (i & 1) ? (w & 0xff) : (w >> 8);
		if (got != exp) { CHECK(0, "read data at %d: %02x != %02x", i, got, exp); bad = 1; }
	}

	// ---- READ(10) 300 blocks (150 KB, > DMA buffer) into $40000
	uint8_t rd10b[10] = { 0x28, 0, 0, 0, 0x01, 0x00, 0, 0x01, 0x2c, 0 };
	st = sel_xfer(0, rd10b, 10, 0x40000, 300 * 512, 0, &phase);
	CHECK(st == 0x16, "big read status %02x", st);
	wait_int();
	bad = 0;
	for (int i = 0; i < 300 * 512 && !bad; i++)
	{
		int b = 256 + i / 512, o = i % 512;
		uint8_t exp = (uint8_t)(b * 7 + o * 13 + (o >> 8));
		uint16_t w = mem[(0x40000 + i) >> 1];
		uint8_t got = (i & 1) ? (w & 0xff) : (w >> 8);
		if (got != exp) { CHECK(0, "big read data at %d: %02x != %02x", i, got, exp); bad = 1; }
	}

	// ---- WRITE(10) 4 blocks to LBA 2000 from $60000, then read back
	for (int i = 0; i < 4 * 256; i++) mem[(0x60000 >> 1) + i] = (uint16_t)(0xC0DE ^ (i * 0x101));
	uint8_t wr10[10] = { 0x2a, 0, 0, 0, 0x07, 0xd0, 0, 0, 4, 0 };
	st = sel_xfer(0, wr10, 10, 0x60000, 4 * 512, 1, &phase);
	CHECK(st == 0x16, "write status %02x", st);
	wait_int();
	uint8_t rdb[10] = { 0x28, 0, 0, 0, 0x07, 0xd0, 0, 0, 4, 0 };
	st = sel_xfer(0, rdb, 10, 0x70000, 4 * 512, 0, &phase);
	CHECK(st == 0x16, "readback status %02x", st);
	wait_int();
	CHECK(!memcmp(&mem[0x60000 >> 1], &mem[0x70000 >> 1], 4 * 512), "write/readback mismatch");

	// ---- READ beyond the end: CHECK CONDITION, then REQUEST SENSE
	uint8_t rdx[10] = { 0x28, 0, 0, 0, 0x20, 0x00, 0, 0, 1, 0 };
	st = sel_xfer(0, rdx, 10, 0x1000, 512, 0, &phase);
	CHECK(st == 0x4b && phase == 0x46, "out-of-range read: status %02x phase %02x", st, phase);
	// driver resumes the select-and-transfer from the status phase
	wd_w(0x18, 0x09);
	st = wait_int();
	CHECK(st == 0x16, "resumed status %02x", st);
	wait_int();
	CHECK(wd_r(0x0f) == 2, "check condition expected, got %02x", wd_r(0x0f));
	uint8_t rs[6] = { 0x03, 0, 0, 0, 18, 0 };
	st = sel_xfer(0, rs, 6, 0x1000, 18, 0, &phase);
	CHECK(st == 0x16, "sense status %02x", st);
	wait_int();
	CHECK((mem[0x801] >> 8) == 5, "sense key %02x", mem[0x801] >> 8);

	// ---- A590 ROM style: select with ATN, then PIO message/command/data
	{
		wd_w(0x01, 0x00);                    // CONTROL: polled I/O
		wd_w(0x15, 0);
		wd_w(0x18, 0x06);                    // SELECT WITH ATN
		st = wait_int();
		CHECK(st == 0x11, "sel atn status %02x", st);
		st = wait_int();
		CHECK(st == 0x8e, "msg out request %02x", st);
		wd_w(0x18, 0xa0);                    // TRANSFER INFO, single byte
		for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100);
		wd_w(0x19, 0x80);                    // IDENTIFY
		st = wait_int();
		CHECK(st == 0x1a, "after identify %02x", st);
		uint8_t rd6[6] = { 0x08, 0, 0, 5, 1, 0 };   // READ(6) LBA 5, 1 block
		wd_w(0x12, 0); wd_w(0x13, 0); wd_w(0x14, 6);
		wd_w(0x18, 0x20);
		for (int k = 0; k < 6; k++)
		{
			for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100);
			wd_w(0x19, rd6[k]);
		}
		st = wait_int();
		CHECK(st == 0x19, "after command %02x", st);
		wd_w(0x12, 0); wd_w(0x13, 2); wd_w(0x14, 0);
		wd_w(0x18, 0x20);
		int pbad = 0;
		for (int k = 0; k < 512; k++)
		{
			for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100);
			wb(0x91, 0x19);
			uint8_t v = rb(0x93);
			uint8_t exp = (uint8_t)(5 * 7 + k * 13 + (k >> 8));
			if (v != exp && !pbad) { CHECK(0, "PIO byte %d: %02x != %02x", k, v, exp); pbad = 1; }
		}
		st = wait_int();
		printf("  PIO read done, status %02x\n", st);
	}

	// PIO helper: select w/ ATN, identify, command, then data in/out by polling
	auto pio_cmd = [&](const uint8_t *cdb, int len, uint8_t *data, int dlen, bool out) -> int {
		wd_w(0x01, 0x00);
		wd_w(0x15, 0);
		wd_w(0x18, 0x06);
		if (wait_int() != 0x11) return -1;
		if (wait_int() != 0x8e) return -2;
		wd_w(0x18, 0xa0);
		for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100);
		wd_w(0x19, 0x80);
		if (wait_int() != 0x1a) return -3;
		wd_w(0x12, 0); wd_w(0x13, 0); wd_w(0x14, len);
		wd_w(0x18, 0x20);
		for (int k = 0; k < len; k++) { for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100); wd_w(0x19, cdb[k]); }
		int st2 = wait_int();
		if (st2 != (out ? 0x18 : 0x19)) return 0x100 | st2;
		wd_w(0x12, dlen >> 16); wd_w(0x13, dlen >> 8); wd_w(0x14, dlen);
		wd_w(0x18, 0x20);
		wb(0x91, 0x19);
		for (int k = 0; k < dlen; k++)
		{
			for (int i = 0; i < 2000 && !(aux() & 1); i++) ticks(100);
			wb(0x91, 0x19);
			if (out) wb(0x93, data[k]); else data[k] = rb(0x93);
		}
		return wait_int();
	};
	{
		static uint8_t wbuf[512], rbuf[5120];
		for (int k = 0; k < 512; k++) wbuf[k] = (uint8_t)(k * 31 + 7);
		uint8_t w6[6] = { 0x0a, 0, 0, 6, 1, 0 };
		st = pio_cmd(w6, 6, wbuf, 512, true);
		printf("  PIO write status %02x\n", st);
		CHECK(st == 0x1b || st == 0x4b || st == 0x1b, "PIO write status %02x", st);
		wd_w(0x18, 0x09); wait_int(); wait_int();       // finish (status + msg via sel_xfer resume)
		uint8_t r6[6] = { 0x08, 0, 0, 6, 1, 0 };
		st = sel_xfer(0, r6, 6, 0x10000, 512, 0, &phase);
		wait_int();
		int wbad = 0;
		for (int k = 0; k < 512; k++) { uint16_t w = mem[(0x10000 + k) >> 1]; uint8_t g = (k & 1) ? (w & 0xff) : (w >> 8); if (g != wbuf[k]) wbad++; }
		CHECK(!wbad, "PIO write readback: %d bad bytes", wbad);

		uint8_t r10[6] = { 0x08, 0, 0, 20, 10, 0 };     // 10 blocks = 5120 bytes, two streams
		st = pio_cmd(r10, 6, rbuf, 5120, false);
		printf("  PIO 5K read status %02x\n", st);
		int rbad = 0;
		for (int k = 0; k < 5120; k++) { int b = 20 + k / 512, o = k % 512; if (rbuf[k] != (uint8_t)(b * 7 + o * 13 + (o >> 8))) rbad++; }
		CHECK(!rbad, "PIO 5K read: %d bad bytes", rbad);
		wd_w(0x18, 0x09); wait_int(); wait_int();
	}

	// ---- a2091_dma_rw (used by the A4091 in a combined core): byte exact
	{
		static uint8_t src[9001], chk[9001];
		for (int k = 0; k < 9001; k++) src[k] = (uint8_t)(k * 7 + 3);
		for (int k = 0; k < 0x8000; k++) mem[(0x30000 >> 1) + k] = 0xA5A5;
		CHECK(a2091_dma_rw(0x30001, src, 9001, 1) == 1, "dma_rw write");
		uint16_t before = mem[0x30000 >> 1], after = mem[(0x30001 + 9001) >> 1];
		CHECK((before >> 8) == 0xA5, "dma_rw clobbered byte before (%04x)", before);
		CHECK((after & 0xff) == 0xA5, "dma_rw clobbered byte after (%04x)", after);
		CHECK(a2091_dma_rw(0x30001, chk, 9001, 0) == 1, "dma_rw read");
		CHECK(!memcmp(src, chk, 9001), "dma_rw readback mismatch");
		CHECK(a2091_dma_rw(0x40000000, chk, 16, 0) == 0, "dma_rw must refuse Z3 addresses");
	}

	// ---- DMAC interrupt line
	wd_w(0x18, 0x00);
	for (int i = 0; i < 2000 && !(aux() & 0x80); i++) ticks(200);
	CHECK(top->int2, "INT2 with INTEN and WD INT");
	CHECK((rb(0x41) & 0x50) == 0x50, "ISTR INTS|INT_P");
	wd_r(0x17);
	tick();
	CHECK(!top->int2, "INT2 cleared after status read");

	printf("%s: %d errors, %llu cycles\n", errors ? "FAILED" : "PASSED", errors, (unsigned long long)cycles);
	delete top;
	return errors ? 1 : 0;
}
