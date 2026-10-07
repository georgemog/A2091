// Minimal QEMU/UAE glue for the A4091 port of lsi53c710.cpp.
// Replaces WinUAE's qemuuaeglue.h - only what a4091_lsi.cpp actually needs.

#ifndef A4091_LSI_GLUE_H
#define A4091_LSI_GLUE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ---- opaque device handles -----------------------------------------------
typedef struct DeviceState {
	void *lsistate;          // -> LSIState710 (calloc'd by lsi710_scsi_init)
} DeviceState;

typedef struct PCIDevice PCIDevice;   // opaque - the a4091 has no PCI

// lsi53c710.cpp calls pci710_dma_rw(PCI_DEVICE(s), ...) / pci710_set_irq.
// There is no PCI here - the LSIState710 itself is the cookie; the hw hooks
// use file-scope state, not this pointer.
#define PCI_DEVICE(s) ((PCIDevice *)(s))

typedef uint32_t hwaddr;

// ---- DMA ---------------------------------------------------------------
typedef uint32_t dma_addr_t;
typedef enum {
	DMA_DIRECTION_TO_DEVICE   = 0,   // guest RAM -> SIOP (read)
	DMA_DIRECTION_FROM_DEVICE = 1,   // SIOP -> guest RAM (write)
} DMADirection;

// implemented in a4091_lsi_hw.cpp: routes Z3-fast <-> HPS DDR mmap (memcpy),
// chip / Z2-fast <-> the 0x68/0x69 sdram_ctrl mailbox. Handles the 68k<->ARM
// byte swizzle in one place.
int pci710_dma_rw(PCIDevice *dev, dma_addr_t addr, void *buf, dma_addr_t len, DMADirection dir);

static inline int pci710_dma_read(PCIDevice *dev, dma_addr_t addr, void *buf, dma_addr_t len)
{
	return pci710_dma_rw(dev, addr, buf, len, DMA_DIRECTION_TO_DEVICE);
}
static inline int pci710_dma_write(PCIDevice *dev, dma_addr_t addr, const void *buf, dma_addr_t len)
{
	return pci710_dma_rw(dev, addr, (void *)buf, len, DMA_DIRECTION_FROM_DEVICE);
}

// implemented in a4091_lsi_hw.cpp: pushes the IRQ level to the bridge (0x67).
void pci710_set_irq(PCIDevice *pci_dev, int level);

// ---- misc shims ------------------------------------------------------
#define g_free free
#define DMA_ADDR_FMT "%08x"

#ifdef __cplusplus
extern "C" {
#endif
void a4091_lsi_log(const char *fmt, ...);
#ifdef __cplusplus
}
#endif
#define write_log a4091_lsi_log

static inline uint32_t cpu_to_le32(uint32_t v)
{
	return ((v & 0x000000ffu) << 24) | ((v & 0x0000ff00u) << 8) |
	       ((v & 0x00ff0000u) >> 8)  | ((v & 0xff000000u) >> 24);
}

// QEMU bit-extract helpers
static inline int32_t sextract32(uint32_t value, int start, int length)
{
	return ((int32_t)(value << (32 - length - start))) >> (32 - length);
}
static inline uint32_t extract32(uint32_t value, int start, int length)
{
	return (value >> start) & (~0u >> (32 - length));
}

#endif
