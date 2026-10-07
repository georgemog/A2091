// A4091 SCSI host adapter - SOFTWARE SIOP (Phase B/C1/D).
//
// FPGA side = a4091_bridge.v: a 256-byte shadow 53C710 register RAM + a kick
// flag + an IRQ line. This module runs the ported 53C710 + SCRIPTS VM
// (a4091_lsi.cpp) and the C1 SCSI-2 target (a4091_scsi.cpp) on the ARM.
//
// Kick poll: read 0x64 status; on kick -> burst-read the shadow RAM
// (0x65), load it into the model, lsi_execute_script(), store it back
// (0x66), raise/lower a4091_int2 (0x67).
//
// pci710_dma_rw() is the one memory seam - Amiga guest RAM <-> ARM. Z3
// fast RAM is physically HPS DDR (mmap /dev/mem); the 68k<->ARM byte
// swizzle is contained here.

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "../../spi.h"
#include "../../user_io.h"
#include "../../file_io.h"
#include "../../fpga_io.h"
#include "../../shmem.h"
#include "minimig_config.h"
#include "minimig_a4091.h"
#include "minimig_a2091.h"
#include "a4091_lsi_glue.h"

#define A4091_NDRV  6

// Logging is runtime-gated: the log file only opens when
//   /media/usb0/a4091_debug   exists (or $A4091_DEBUG is set).
// A release build is then silent; `touch /media/usb0/a4091_debug` +
// core reload turns tracing back on. Per-SCRIPTS-instruction tracing
// additionally needs a `-DA4091_SWSIOP_TRACE` build (see a4091_lsi.cpp).
#define A4091_LOGFILE "/media/usb0/a4091_sd.log"

// status word bits (a4091_bridge mbx_status / hps_ext 0x64)
#define ST_KICK   0x0001
#define ST_INT    0x0002
#define ST_SRST   0x0004
#define ST_DMAREQ 0x0008
#define ST_SIGP   0x0010

// ---- ported VM + target (a4091_lsi.cpp / a4091_scsi.cpp) --------------
void lsi710_scsi_init (DeviceState *dev);
void lsi710_scsi_reset(DeviceState *dev, void *privdata);
extern "C" {
void a4091_lsi_load (DeviceState *dev, const uint8_t r[256]);
void a4091_lsi_store(DeviceState *dev, uint8_t r[256]);
int  a4091_lsi_run  (DeviceState *dev);
}
void a4091_scsi_set_image(int id, fileTYPE *f, uint64_t bytes);
void a4091_scsi_reset(void);

static DeviceState g_dev;
static int         g_dev_ready = 0;
static int         g_lsi_irq   = 0;

static fileTYPE a4091_img[A4091_NDRV];
static uint64_t a4091_size[A4091_NDRV];

// ---------------------------------------------------------------------
#include <unistd.h>
#include <stdlib.h>

static FILE *a4lf(void)
{
	static FILE *lf = NULL;
	static int tried = 0;
	if (!lf && !tried) {
		tried = 1;
		if (access("/media/usb0/a4091_debug", F_OK) == 0 || getenv("A4091_DEBUG")) {
			lf = fopen(A4091_LOGFILE, "w");
			if (lf) { setvbuf(lf, NULL, _IOLBF, 0); fprintf(lf, "--- a4091 sw-siop log ---\n"); }
		}
	}
	return lf;
}
static void a4log(const char *fmt, ...)
{
	FILE *lf = a4lf(); if (!lf) return;
	va_list ap; va_start(ap, fmt); vfprintf(lf, fmt, ap); va_end(ap); fflush(lf);
}
extern "C" void a4091_lsi_log(const char *fmt, ...)
{
	FILE *lf = a4lf(); if (!lf) return;
	va_list ap; va_start(ap, fmt); vfprintf(lf, fmt, ap); va_end(ap); fflush(lf);
}

// ---- shadow-register burst (hps_ext 0x65 / 0x66 / 0x67) -------------
// Only the low 64 registers matter - the 53C710 file is 0x00-0x3F and the
// VM (a4091_lsi_load/store) + this poll loop never touch anything above
// 0x3B. Shuffling 64 B instead of 256 is 4x fewer GPIO-handshake `spi_w`
// per SCSI command, which is the throughput bottleneck (the 16 KB DATA
// itself is a direct mmap-DDR memcpy). hps_ext streams whatever length
// the ARM asks for - no RTL change.
#define A4091_REGS_N 64

static void regs_read(uint8_t regs[256])
{
	EnableIO();
	spi_w(0x0065);
	for (int i = 0; i < A4091_REGS_N; i++) regs[i] = (uint8_t)spi_w(0);
	DisableIO();
}
static void regs_write(const uint8_t regs[256], int n)
{
	if (n > A4091_REGS_N) n = A4091_REGS_N;
	EnableIO();
	spi_w(0x0066);
	for (int i = 0; i < n; i++) spi_w(regs[i]);
	DisableIO();
}
static void ctrl(uint8_t bits)
{
	EnableIO();
	spi_w(0x0067);
	spi_w(bits);
	DisableIO();
}

// ---- the one memory seam ------------------------------------------
// Amiga Z3 fast RAM (autoconfig base 0x40000000) is physically HPS DDR.
// cpu_wrapper remaps a Z3 access to ddram word addr {1'b1, cpu_addr[27:1]};
// ddram_ctrl then places it at {3'b001, addr[28:3]} 8-byte DDR words, i.e.
// HPS phys 0x30000000 + (A - 0x40000000). The 16-bit Amiga word lands
// byte-swapped inside the little-endian 64-bit DDR word, so every byte
// access XORs offset bit 0. Verified on HW: the driver's scripts[]
// (Amiga 0x40000090) reads back at phys 0x30000090 with each halfword
// swapped; DSA 0x4001b0f0 -> a real acb->ds at 0x3001b0f0.
#define A4091_AMIGA_BASE 0x40000000u
#define A4091_DDR_BASE   0x30000000u
#define A4091_Z3_WINDOW  0x10000000u        // 256 MB - Minimig Z3 fast max

static int      g_memfd   = -1;
static uint8_t *g_ddr     = NULL;   // 64 MB /dev/mem window
static uint32_t g_ddr_phys = 0;
static uint32_t g_ddr_len  = 0;

static uint8_t *ddr_ptr(uint32_t amiga_addr)
{
	// Only Z3 fast RAM (>= 0x40000000, first 256 MB) is HPS DDR. Chip /
	// Z2-fast / bogus targets have no path here (the chip-DMA mailbox is
	// tied off) - drop them, never fault the kernel on a wild /dev/mem
	// offset (a bad SoC bus access reboots the box, not just MiSTer).
	if (amiga_addr < A4091_AMIGA_BASE) return NULL;
	uint32_t aoff = amiga_addr - A4091_AMIGA_BASE;
	if (aoff >= A4091_Z3_WINDOW) return NULL;
	uint32_t phys = (A4091_DDR_BASE + aoff) ^ 1u;
	if (g_memfd < 0) g_memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (g_memfd < 0) return NULL;
	uint32_t win = phys & ~0x00FFFFFFu;                 // 16 MB-aligned window
	if (!g_ddr || win != g_ddr_phys || (phys - win) >= g_ddr_len) {
		if (g_ddr) munmap(g_ddr, g_ddr_len);
		g_ddr_len  = 0x04000000;
		g_ddr_phys = win;
		g_ddr = (uint8_t *)mmap(0, g_ddr_len, PROT_READ | PROT_WRITE, MAP_SHARED, g_memfd, win);
		if (g_ddr == MAP_FAILED) { g_ddr = NULL; return NULL; }
	}
	uint32_t off = phys - g_ddr_phys;
	return (off < g_ddr_len) ? (g_ddr + off) : NULL;
}

// Map a whole [amiga_addr, amiga_addr+len) range in one shot and return the
// base of the mmap window plus the un-XORed byte offset of the first byte.
// Byte k of the range then lives at base[(off + k) ^ 1]. Returns NULL if the
// range is outside Z3 fast RAM or straddles a window boundary (caller falls
// back to the safe per-byte path).
static uint8_t *ddr_map_range(uint32_t amiga_addr, uint32_t len, uint32_t *off)
{
	if (len == 0) return NULL;
	if (amiga_addr < A4091_AMIGA_BASE) return NULL;
	uint32_t aoff = amiga_addr - A4091_AMIGA_BASE;
	if (aoff >= A4091_Z3_WINDOW || len > A4091_Z3_WINDOW - aoff) return NULL;

	uint32_t q   = A4091_DDR_BASE + aoff;          // un-XORed phys of byte 0
	uint32_t win = q & ~0x00FFFFFFu;               // 16 MB-aligned window base
	if (g_memfd < 0) g_memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (g_memfd < 0) return NULL;
	if (!g_ddr || win != g_ddr_phys) {
		if (g_ddr) munmap(g_ddr, g_ddr_len);
		g_ddr_len  = 0x04000000;
		g_ddr_phys = win;
		g_ddr = (uint8_t *)mmap(0, g_ddr_len, PROT_READ | PROT_WRITE, MAP_SHARED, g_memfd, win);
		if (g_ddr == MAP_FAILED) { g_ddr = NULL; return NULL; }
	}
	uint32_t o = q - g_ddr_phys;
	if (o + len + 1 > g_ddr_len) return NULL;      // range (incl. ^1 slop) fits
	*off = o;
	return g_ddr;
}

// Set whenever a DATA-IN transfer writes Amiga RAM behind the FPGA's back;
// the kick loop then pulses the RAM-controller cache flush (0x67 bit4).
static int g_dma_wrote_ram = 0;

int pci710_dma_rw(PCIDevice *dev, dma_addr_t addr, void *buf, dma_addr_t len, DMADirection dir)
{
	(void)dev;
	uint8_t *p = (uint8_t *)buf;
	uint32_t o;
	uint8_t *m = ddr_map_range((uint32_t)addr, (uint32_t)len, &o);

	if (dir != DMA_DIRECTION_TO_DEVICE && len) g_dma_wrote_ram = 1;

	if (m) {
		// Fast path: one mmap, halfword-swapped copy. 16-bit accesses on the
		// aligned interior halve the DDR transactions vs the byte loop.
		uint8_t *d = m + o;
		dma_addr_t i = 0;
		if ((o & 1) && i < len) {                  // odd head byte
			if (dir == DMA_DIRECTION_TO_DEVICE) p[i] = d[-1]; else d[-1] = p[i];
			i++;
		}
		for (; i + 1 < len; i += 2) {              // aligned body, 2 bytes
			uint8_t *w = d + i;
			if (dir == DMA_DIRECTION_TO_DEVICE) { p[i] = w[1]; p[i+1] = w[0]; }
			else                                { w[1] = p[i]; w[0] = p[i+1]; }
		}
		if (i < len) {                             // trailing byte
			uint8_t *w = d + i;
			if (dir == DMA_DIRECTION_TO_DEVICE) p[i] = w[1]; else w[1] = p[i];
		}
		return 0;
	}

	// Chip / slow / Zorro II RAM: not HPS memory. A combined core lends the
	// A2091's DMA engine (chipdma_arb) for it.
	if (a2091_dma_rw((uint32_t)addr, p, (uint32_t)len, dir != DMA_DIRECTION_TO_DEVICE))
		return 0;

	// Safe path: unmapped / window-straddling range.
	for (dma_addr_t i = 0; i < len; i++) {
		uint8_t *d = ddr_ptr((uint32_t)(addr + i));
		if (!d) { if (dir == DMA_DIRECTION_TO_DEVICE) p[i] = 0xff; continue; }
		if (dir == DMA_DIRECTION_TO_DEVICE) p[i] = *d;
		else                               *d = p[i];
	}
	return 0;
}

void pci710_set_irq(PCIDevice *pci_dev, int level)
{
	(void)pci_dev;
	g_lsi_irq = level ? 1 : 0;
}

// ---------------------------------------------------------------------
void a4091_apply_config(void)
{
	if (!g_dev_ready) { lsi710_scsi_init(&g_dev); g_dev_ready = 1; }

	int enabled = (minimig_scsi_controller() == SCSI_CTRL_A4091);

	for (int i = 0; i < A4091_NDRV; i++) {
		if (a4091_img[i].opened()) FileClose(&a4091_img[i]);
		a4091_size[i] = 0;

		const mm_hardfileTYPE *hf = &minimig_config.scsi[i];
		int want = enabled && hf->cfg == 1 && hf->filename[0];

		if (want && FileOpenEx(&a4091_img[i], hf->filename,
		                       FileCanWrite(hf->filename) ? O_RDWR : O_RDONLY)) {
			a4091_size[i] = a4091_img[i].size;
			a4log("ID%d = \"%s\"  %llu MB\n", i + 1, a4091_img[i].name,
			      (unsigned long long)(a4091_size[i] >> 20));
			a4091_scsi_set_image(i + 1, &a4091_img[i], a4091_size[i]);
		} else {
			a4091_scsi_set_image(i + 1, 0, 0);
			if (want) a4log("ID%d open FAILED: \"%s\"\n", i + 1, hf->filename);
		}
	}

	lsi710_scsi_reset(&g_dev, NULL);
	a4091_scsi_reset();
	a4log("apply: A4091 SCSI %s (software SIOP)\n", enabled ? "enabled" : "disabled");
}

// ---------------------------------------------------------------------
// One 0x64 poll + whatever it asks for. Returns 1 if it did real work
// (a kick or an int-ack), 0 if the bridge was idle.
static int a4091_service_once(void);

void a4091_sd_poll(void)
{
	if (!g_dev_ready) return;

	// When a command is in flight, stay attached and spin-poll at SPI
	// speed rather than servicing one SCSI phase per Main_MiSTer main
	// loop. (Measured: this is NOT the throughput bottleneck - that is
	// the driver's mm_ramctrl_cache_evict() sweep, ~11 ms per DATA-IN
	// command. But it costs nothing and removes the loop-period jitter.)
	// Bail on a short idle streak or a hard cap - a hung driver must
	// never freeze the core.
	if (!a4091_service_once()) return;
	int idle = 0;
	for (int i = 0; i < 20000 && idle < 200; i++)
		idle = a4091_service_once() ? 0 : (idle + 1);
}

static int a4091_service_once(void)
{
	static uint32_t polls = 0, kicks = 0;
	static int int_raised = 0;
	polls++;
	int did_work = 0;

	EnableIO();
	uint16_t st = spi_w(0x0064);
	DisableIO();

	// The driver acked the interrupt (read ISTAT -> bridge cleared int_pending).
	// On the real 53C710 that also clears DIP/SIP once DSTAT/SSTAT0 are read
	// and leaves DSTAT.DFE. Model the residual-status clear here.
	if (int_raised && !(st & ST_INT)) {
		int_raised = 0;
		uint8_t r[256];
		regs_read(r);
		r[0x21] &= ~0x03;       // ISTAT: clear DIP | SIP
		r[0x0c]  = 0x80;        // DSTAT: DFE only
		r[0x0d]  = 0;           // SSTAT0
		regs_write(r, 256);
		did_work = 1;
	}

	if (st & ST_SRST) {
		lsi710_scsi_reset(&g_dev, NULL);
		a4091_scsi_reset();
		g_lsi_irq = 0;
		// 53C710 soft reset clears the control / status / interrupt regs but
		// NOT DSP/DSA/DBC/DNAD/SCRATCH/TEMP/SCID/SDID. Model that on the
		// shadow RAM directly - do NOT store the (all-zero) fresh model over
		// the driver's in-progress register setup.
		uint8_t r[256];
		regs_read(r);
		r[0x00] = 0; r[0x01] = 0; r[0x03] = 0;      // SCNTL0/1, SIEN
		r[0x0c] = 0x80;                             // DSTAT = DFE
		r[0x0d] = 0; r[0x0e] = 0; r[0x0f] = 0;      // SSTAT0/1/2
		r[0x21] = 0;                                // ISTAT
		r[0x38] = 0; r[0x39] = 0; r[0x3b] = 0;      // DMODE, DIEN, DCNTL
		regs_write(r, 256);
		ctrl(0x02 | 0x08);                          // clr_int | clr_srst
		did_work = 1;
	}

	if (st & ST_KICK) {
		kicks++;
		did_work = 1;
		uint8_t r[256];
		regs_read(r);
		a4091_lsi_load(&g_dev, r);

		uint32_t dsp = r[0x2c] | (r[0x2d]<<8) | (r[0x2e]<<16) | ((uint32_t)r[0x2f]<<24);
		uint32_t dsa = r[0x10] | (r[0x11]<<8) | (r[0x12]<<16) | ((uint32_t)r[0x13]<<24);

		// Spurious kick: the driver zeroes DSP during chip init.
		if (dsp == 0) { ctrl(0x02 | 0x04); return 1; }   // clr_int | clr_kick

#ifdef A4091_SWSIOP_TRACE
		if (kicks <= 8) {
			uint8_t d[32], da[64];
			pci710_dma_read((PCIDevice*)0, dsp, d, 32);
			pci710_dma_read((PCIDevice*)0, dsa, da, 64);
			a4log("kick#%u ddr=%08x DSP=%08x DSA=%08x DCMD=%02x DMODE=%02x\n"
			      "  [DSP]: %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n"
			      "  [DSA]: %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
			      kicks, A4091_DDR_BASE, dsp, dsa, r[0x27], r[0x38],
			      d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11],d[12],d[13],d[14],d[15],
			      da[0],da[1],da[2],da[3],da[4],da[5],da[6],da[7],da[8],da[9],da[10],da[11],da[12],da[13],da[14],da[15],
			      da[16],da[17],da[18],da[19],da[20],da[21],da[22],da[23],da[24],da[25],da[26],da[27],da[28],da[29],da[30],da[31]);
		}
#else
		(void)dsp; (void)dsa;
#endif

		g_lsi_irq = 0;
		g_dma_wrote_ram = 0;
		int irq = a4091_lsi_run(&g_dev);

		a4091_lsi_store(&g_dev, r);
		regs_write(r, 256);

#ifdef A4091_SWSIOP_TRACE
		if (kicks <= 20 || (kicks & 0x3ff) == 0)
			a4log("kick #%u  DSP=%02x%02x%02x%02x DSPS=%02x%02x%02x%02x DSTAT=%02x ISTAT=%02x irq=%d/%d\n",
			      kicks, r[0x2f],r[0x2e],r[0x2d],r[0x2c],
			      r[0x33],r[0x32],r[0x31],r[0x30], r[0x0c], r[0x21], irq, g_lsi_irq);
#endif

		// bit4 = flush the Minimig RAM-controller read cache: the DATA-IN
		// memcpy wrote HPS DDR directly, which cpu_cache_new can't snoop.
		uint8_t cc = g_dma_wrote_ram ? 0x10 : 0x00;
		if (irq || g_lsi_irq) { ctrl(0x01 | 0x04 | cc); int_raised = 1; }  // set_int | clr_kick
		else                    ctrl(0x02 | 0x04 | cc);                    // clr_int | clr_kick
	}

	return did_work;
}
