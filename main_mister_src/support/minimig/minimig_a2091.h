// SPDX-License-Identifier: GPL-3.0-or-later
// A2091 SCSI host adapter (DMAC + WD33C93A) - HPS side.
#ifndef MINIMIG_A2091_H
#define MINIMIG_A2091_H

#include <stdint.h>

#define A2091_NDRV 7            // SCSI IDs 0..6 (7 is the host adapter)

// (re)load ROM, open the images and enable/disable the board in the FPGA.
// Call with the CPU halted, before the Amiga reset that runs autoconfig.
void a2091_apply_config(void);

// service the FPGA mailbox; called from the Minimig main poll loop
void a2091_poll(void);

// the running core carries the A2091 / A4091 bridge
int a2091_core_supported(void);
int a4091_core_supported(void);

// effective SCSI controller from Minimig.cfg scsi_cfg + CPU type
#define SCSI_CTRL_NONE  0
#define SCSI_CTRL_A2091 1
#define SCSI_CTRL_A4091 2
int minimig_scsi_controller(void);

// DMA for other HPS-side bus masters into 24-bit Amiga RAM (returns 1 if done)
int a2091_dma_rw(uint32_t addr, uint8_t *buf, uint32_t len, int to_amiga);

#endif
