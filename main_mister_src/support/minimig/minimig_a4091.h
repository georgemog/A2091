#ifndef MINIMIG_A4091_H
#define MINIMIG_A4091_H

// A4091 SCSI host adapter support for the Minimig core - SOFTWARE SIOP.
//
// The FPGA side (rtl/a4091/a4091.v + a4091_bridge.v) is a thin Zorro-III
// bridge: autoconfig ROM + a 256-byte shadow 53C710 register RAM + a kick
// flag + an IRQ line. The 53C710 model, SCRIPTS VM and SCSI-2 target
// emulation run here on the ARM.
//
// Phase A (current): a4091_sd_poll() polls the kick flag and, since the
// SCRIPTS interpreter is not ported yet, fakes an illegal-instruction abort
// so a4091.device loads with zero units. a4091_apply_config() opens the up
// to 6 .hdf images (SCSI IDs 1..6) selected in the OSD "A4091 SCSI" section.
//
// Debug logging (a4log) is compiled in only when A4091_DEBUG is defined.

void a4091_sd_poll(void);        // bridge poll - called from user_io poll loop
void a4091_apply_config(void);   // (re)open images - ApplyConfiguration + OSD change

#endif
