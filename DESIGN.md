# A2091 SCSI for the Minimig-AGA MiSTer core — design notes

Commodore A2091 Zorro II SCSI host adapter — DMAC (rev 2) + WD33C93A — for the
Minimig-AGA MiSTer core, using amiberry/WinUAE `a2091.cpp` as the behavioural
reference. Base: upstream Minimig-AGA_MiSTer `8325735` and Main_MiSTer
`d5f3844`.

The board autoconfigs as Commodore 514/3 (64 KB, DiagArea at `$2000`), runs the
original A590/A2091 boot ROM (supplied by the user, loaded at core start) and
autoboots/mounts RDB-partitioned `.hdf` images on SCSI IDs 0–6.

## Architecture

```
 68k (fx68k / TG68)            FPGA  rtl/a2091.v                      HPS  minimig_a2091.cpp
 ──────────────────            ──────────────────────────────         ─────────────────────────
 ROM fetches       ──────────► 16 KB boot ROM (BRAM, HPS-loaded)
 DMAC regs         ──────────► CNTR ISTR ACR WTC ST/SP_DMA CINT FLUSH
 WD33C93 SASR/SCMD ──────────► 32-byte register file                   WD33C93 command engine
                               SASR auto-increment, AUX STATUS         (port of amiberry:
                               SCSI STATUS read clears INT              select-and-transfer,
                               status FIFO (ARM fills, FPGA presents)   select, transfer info,
 INT2 ◄──────────────────────  INTEN & ISTR.INTS                        reset, trans addr, PIO)
                               events ─── hps_ext 0x70/71/72 ────────► SCSI-2 disk targets
                               (WD command, ST_DMA, PIO byte, reset)    (.hdf via FileReadAdv)
 Amiga RAM ◄─────────────────  word DMA engine + 4 KB buffer ◄──────── data, 4 KB chunks
            chipdma_arb        (chip, slow $C00000, Zorro II RAM)
```

Design choices, and why:

* **Everything the 68k touches at bus speed is local to the FPGA.** The
  driver polls AUX STATUS in tight loops and reads SCSI STATUS to acknowledge
  interrupts; doing these over SPI would stall the CPU. The side effects that
  matter (SASR auto-increment, INT clear on status read, status queue
  sequencing, ISTR bits) are implemented in RTL, mirroring amiberry.
* **The WD33C93 command logic runs on the ARM.** It is the complex, stateful
  part (phases, select-and-transfer resume, PIO) and amiberry's code ports
  almost line for line. The FPGA raises `ev_cmd` when the 68k writes the
  COMMAND register; the ARM reads the register file, runs the command, writes
  results back and queues the interrupt statuses (with amiberry's hsync
  delays, in ~64 µs ticks).
* **DMA reuses upstream `chipdma_arb`.** The A2091 is a 24-bit Zorro II master:
  its targets are chip RAM, `$C00000` slow RAM and Zorro II RAM, which live in
  SDRAM and DDR3 respectively and are not reachable from the HPS. Upstream's
  arbiter (written for CDTV/Akiko) already steals idle chip-bus slots and
  covers both RAMs with cache snooping; it gained a word-wide A2091 port. The
  A4091 port's hard-won lesson — DMA writes the CPU cache does not see —
  does not recur here because both RAM controllers snoop this path.
* **No ROM in the bitstream.** The A590/A2091 ROM is Commodore's; Main_MiSTer
  loads it at core start (plain 16 KB image or Cloanto `AMIROMTYPE1` +
  `rom.key`).

## Files

| Path | What |
|---|---|
| `rtl/a2091.v` | the board (drop into `rtl/` of the core) |
| `integration/minimig_core.patch` | core changes: `chipdma_arb` A2091 port, autoconfig link in `cpu_wrapper`, `gary` decode, `minimig.v` instance + INT2 + data mux, `hps_ext` 0x70–0x72, `Minimig.sv` wiring, `files.qip` |
| `integration/chipdma_arb.v` | the patched arbiter, for reference |
| `main_mister/minimig_a2091.{cpp,h}` | HPS side → `support/minimig/` |
| `integration/main_mister.patch` | Main_MiSTer hooks: config fields, `ApplyConfiguration`, poll, OSD rows |
| `tb/a2091_tb.v` | RTL unit testbench (`iverilog -g2012`) |
| `sim/` | Verilator co-simulation: `a2091.v` + the real HPS code + a scripted 68k (`make`) |
| `tools/a2091cfg.py` | show/edit the A2091 fields of `config/Minimig.cfg` |
| `tools/a2091test.c` | Amiga file write/read/verify + throughput test |

## Using it

1. Put the A590/A2091 boot ROM (16 KB) in the Amiga games folder as
   `a2091.rom`, or keep the Cloanto `roms/amiga-boot-a590.rom` + `roms/rom.key`.
2. OSD → Drives → **A2091 SCSI: Enabled**, pick images for ID0…ID6, then reset.
   (Or `tools/a2091cfg.py Minimig.cfg --enable --id 0 ../usb0/games/Amiga/x.hdf`.)
3. Images need a Rigid Disk Block; the boot ROM mounts every partition it finds
   and can boot from them.

Config: `mm_configTYPE` gains `a2091_cfg` + `a2091_hdd[7]` at the end, so older
config files still load (board disabled). Debug log: create `a2091_debug` in
the storage root, reload; the log is appended to `<storage>/logs/a2091.log`.

## Mailbox protocol (hps_ext)

| Cmd | Word 0 | Then |
|---|---|---|
| `0x70` READ | status | WD regs 0–31, `{sasr,cntr}`, ACR[23:16], ACR[15:0], WTC hi/lo, ISTR, FIFO count |
| `0x71` WRITE | status | opcode word, operands: `00rr` regs from rr · `0100` control bits · `0200` push `{delay,status}` · `0300` ACR · `0400` buffer words · `0500` DMA `{addr hi, addr lo, dir|count}` · `0600` ROM words · `0700` enable |
| `0x72` BUFRD | status | DMA buffer words |

Status word: `[15:13]=101` marker, `12` enabled, `11` BSY(x), `10` DBR,
`9:8` FIFO count, `7` DMAC DMA active, `6` WD busy, `5` WD INT, `4` DMA busy,
`3` ev_reset, `2` ev_pio, `1` ev_stdma, `0` ev_cmd.

## Hardware results (2026-10-07)

`Minimig_A2091_20261007d.rbf` + `MiSTer_A2091`, 68020, A590/A2091 ROM 390721/2-07,
ID1 = `Test200MB.hdf` → mounts as `RegrTest:` at boot (drives from the OSD SCSI page).

`a2091test RegrTest: 256 2` — **PASSED** (debug log off)

| Buffers | Path | Write | Read |
|---|---|---:|---:|
| chip RAM | DMAC DMA (single ~220 KB transfers) | 1280 KB/s | 1600–1828 KB/s |
| Z3 fast (`$40xxxxxx`, beyond 24-bit DMA) | polled I/O, FPGA-streamed blocks | 112–130 KB/s | 131–134 KB/s |

| Step | DMA (r/w) | polled (r/w) |
|---|---:|---:|
| first working build (per-byte mailbox) | 365 / 441 KB/s* | 16 / 17 KB/s |
| OP_PIO block streaming | 984–1066 / 984 | 129 / 126 |
| IDE-style fast SPI blocks + shorter status delays | 1600–1828 / 1280 | 131–134 / 112–130 |

\* with the debug log on.

**Where the polled time goes** (`a2091_profile` counters, 2 s window under load):
648 WD commands + 386 PIO blocks, HPS handler busy 358 ms = **18 %**. The other
82 % is the A590 driver itself on the 68k (chip-bus register polling, one 512-byte
block per SCSI command in polled mode). Making the HPS side faster no longer moves
this number.

**Why fast-RAM transfers are polled at all:** `Test200MB.hdf`'s partition has
`Mask = 0x7FFFFFFE`, so FFS hands the driver Zorro III buffers the 24-bit DMAC
can't reach and the driver falls back to polled I/O. A2091 partitions normally
use `Mask = 0x00FFFFFE` (FFS then bounces through 24-bit buffers and the driver
DMAs) — expected to bring fast-RAM transfers close to the DMA figures.

**Mask experiment on `Test200MB.hdf`** (RDB blocks 0–15 backed up beside it as
`Test200MB.rdb_blocks0-15.bak`):

| PART settings | fast-RAM write | fast-RAM read | chip (DMA) |
|---|---:|---:|---:|
| Mask 0x7FFFFFFE, BufMemType 0 (original) | 112–130 KB/s | 131–134 KB/s | 1.28 / 1.83 MB/s |
| Mask 0x00FFFFFE, BufMemType 0 | 75 KB/s | 80–83 KB/s | unchanged |
| Mask 0x00FFFFFE, BufMemType 0x201 (PUBLIC\|24BITDMA) | 147–148 KB/s | 172–175 KB/s | unchanged |

With only `Mask` changed, FFS bounces through buffers it allocates with
`BufMemType 0` — which can be Zorro III RAM again, so the driver still polls and
FFS adds a copy. With `BufMemType` asking for 24-bit DMA memory the bounce
buffers land in chip RAM and the transfers go by DMA, but FFS moves them one
512-byte block per command, so per-command overhead (~3 ms) caps the rate.
Application buffers in chip/24-bit RAM get the full DMA rate.

**Zorro II fast RAM (8 MB at `$200000`, DDR3-backed) — DMA through chipdma_arb's
DDR port, first hardware run, `a2091test RegrTest: 256 2` PASSED:**

| Buffers | Write | Read |
|---|---:|---:|
| chip RAM | 1422 KB/s | 1600–1828 KB/s |
| Zorro II fast RAM | 1163–1280 KB/s | 1600 KB/s |
| Zorro II fast RAM, 1 MB file | 1248–1383 KB/s | 1651–1765 KB/s |

With only 24-bit RAM in the machine every transfer is DMA; the polled path is
not used at all.

**Autoboot (2026-10-07, user test):** AmigaOS boots from a drive on the A2091
(A590/A2091 ROM 390721/2-07 autoboot via the DiagArea at `$2000`).

**68000 (fx68k) autoboot from the A2091** confirmed by user on RBF d.

## Combined A2091 + A4091 core

`Minimig_A2091_A4091_20261007.rbf` + combined `MiSTer_A2091` (live A4091 tree +
`integration/main_mister_combo_patch*.py`). Core diff: `integration/combined_a4091.patch`
(A4091 software-SIOP bridge from the verified `1d93256` combined core, no RTG).

* OSD Drives → **SCSI controller: Disabled / A2091 / A4091**. The A4091 is only
  offered on a 68020+ (and only if the core has it); a 68000/010 always gets the
  A2091. `Minimig.cfg` `scsi_cfg`: bit0 = on, bit1 = A2091 (old configs = A4091).
  Both controllers use the same ID1–ID6 drive list. Changes take effect on reset.
* One board at a time: the HPS enables the A2091 (OP_CFG bit0) or the A4091
  (OP_CFG bit1, still gated by `cpucfg[1]` in RTL) — the A4091 no longer uses O[57].
* Autoconfig order: … A2065, A2091, Z3 RAM, A4091 (last; the A4091's HPS side
  assumes Z3 RAM at `$40000000`).
* **A4091 DMA into chip / Zorro II RAM:** the software SIOP can only `mmap` Zorro III
  RAM. In the combined core it borrows the A2091's DMA engine (`a2091_dma_rw()`,
  byte-exact via read-modify-write of edge words) for 24-bit addresses — before
  this, buffers in chip RAM read back as `0xFF` (garbage volume name).

HW (68020, `RegrTest:` via A4091, image Mask 0x00FFFFFE / BufMemType 0x201):
`a2091test RegrTest: 256 2` PASSED — chip 1163 / 1422 KB/s, Z3 60–66 KB/s (FFS
bounce through chip RAM because of the 24-bit Mask; use Mask 0x7FFFFFFE for an
A4091-only image).

### Main_MiSTer on official master

`integration/main_mister_d5f3844_scsi.patch` = A4091 + A2091 SCSI rebased onto
official Main_MiSTer `d5f3844` (2026-10-07). Upstream added `userport` /
`externalfloppy` at 7272–7279, where the old A4091 build kept `scsi_cfg` /
`scsi[]`; the SCSI fields now follow `externalfloppy` (`sizeof` 13440) and
configs saved by the old A4091 build (13426 bytes) are converted on load.
Note: a config *saved* by this build (13440 bytes) is rejected by the old
A4091 binary.
