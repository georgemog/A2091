# A2091 / A4091 SCSI for the Minimig-AGA MiSTer core

An emulation of the Commodore **A2091** Zorro II SCSI host adapter (DMAC +
WD33C93A, the same design as the A590) for the Minimig-AGA core on MiSTer,
combined with the existing **A4091** Zorro III SCSI adapter in one core. The
original Commodore A590/A2091 boot ROM runs unmodified: drives are found at
boot, partitions mount automatically and AmigaOS can boot from them.

* Works with **every CPU setting**, including the 68000 — the A2091 is a
  Zorro II board with 24-bit DMA into chip, slow and Zorro II fast RAM.
* With a 68020 or better you can choose the **A4091** instead (Zorro III).
* Up to six hard disk images (`.hdf`, RDB partitioned) on SCSI IDs 1–6.
* Behaviour modelled on amiberry/WinUAE (`a2091.cpp`).

Design details, mailbox protocol and the bring-up history are in
[DESIGN.md](DESIGN.md); the file index is in [CONTENTS.md](CONTENTS.md).

---

## How it works (short version)

| Part | Where | Does |
|---|---|---|
| Board, boot ROM, DMAC registers, WD33C93 register file | FPGA (`rtl/a2091.v`) | everything the 68k touches at bus speed |
| WD33C93 command engine + SCSI disks | ARM (`minimig_a2091.cpp` in Main_MiSTer) | runs the SCSI commands against the `.hdf` files |
| DMA | FPGA, through the core's `chipdma_arb` | moves data into chip / slow / Zorro II RAM |
| Polled transfers (buffers above 16 MB) | FPGA streams whole blocks through a 4 KB buffer | the 68k never waits on the ARM per byte |
| A4091 | FPGA bridge + ARM 53C710 model (existing A4091 port) | in the combined core it borrows the A2091 DMA engine for chip/Zorro II RAM |

---

## Installing

Files (all in this repository):

| File | Copy to |
|---|---|
| `releases/Minimig_A2091_A4091_20261007.rbf` (combined, latest) — or `build/Minimig_A2091_20261007d.rbf` (A2091 only) | `/media/fat/_Computer/` |
| `releases/MiSTer_A2091` (Main_MiSTer, official `d5f3844` + SCSI) | `/media/fat/MiSTer_A2091` |
| your A590/A2091 boot ROM (16 KB, not included — see [rom/README.md](rom/README.md)) | the Amiga games folder as **`a2091.rom`** (e.g. `/media/usb0/games/Amiga/a2091.rom`) |
| `build/a2091test` (optional test tool) | the shared folder (`SHARE:`) |

1. **Main_MiSTer:** add to `MiSTer.ini`

   ```ini
   [minimig]
   main=MiSTer_A2091
   ```

   Loading a Minimig core then starts `MiSTer_A2091` instead of the stock
   `MiSTer`. Remove the line to go back.

   ⚠️ Never overwrite `MiSTer_A2091` while it is running: load the Menu core
   first, replace the file, then load the Minimig core again (a core load
   re-executes the running binary; if its file has been replaced the MiSTer
   reboots).

2. **Boot ROM:** the A590/A2091 ROM is Commodore's and is not part of the
   core. Main_MiSTer loads it at core start, searching the Amiga games folder
   for, in order: `a2091.rom`, `A2091.ROM`, `a590.rom`, `A590.ROM`,
   `roms/a2091.rom`, `roms/a590.rom`, `roms/amiga-boot-a590.rom`,
   `roms/amiga-boot-a2091.rom`. Plain 16 KB images and Cloanto encrypted
   images (`AMIROMTYPE1` + `rom.key` in the same folder) are accepted. A ROM
   dumped from the two EPROMs is built by interleaving them, U13 = even bytes
   (`390721-xx.u13`) and U12 = odd bytes (`390722-xx.u12`). 32 KB ROMs (e.g.
   GuruROM) are not supported yet. Without a ROM the board still appears in
   `showconfig` but cannot boot or mount drives.

3. **Config compatibility:** this Main_MiSTer stores the SCSI settings after
   upstream's newest fields. Configs from the older A4091 build (13426 bytes)
   are converted automatically when loaded; once saved by this build (13440
   bytes) the old A4091 `MiSTer` will no longer accept them.

---

## Configuring

OSD → **Drives**:

```
 SCSI controller  : A2091          <- Disabled / A2091 / A4091
 ·ID1             : Fixed/HDD
   AmigaOS3.2/hdf/Test200MB.hdf
 ·ID2             : Disabled
 ...
```

* **SCSI controller** cycles through what the core and CPU allow:
  * 68000 / 68010: `Disabled` / `A2091`
  * 68020 and up: `Disabled` / `A2091` / `A4091`
  If the A4091 is selected and the CPU is later set to a 68000/010, the
  A2091 is used instead.
* **ID1–ID6**: set `Fixed/HDD` and pick an `.hdf`. Both controllers use the
  same list. Images must have a Rigid Disk Block (partition with HDToolBox or
  similar).
* Changes take effect on the next Amiga **reset**.

Optional override: `config/minimig_a2091.ini` (if present, the A2091 uses it
instead of the OSD drive list):

```ini
enable=1
rom=/media/usb0/games/Amiga/a2091.rom
id0=../usb0/games/Amiga/scsi0.hdf
id1=...
```

### Partition settings (Mask / BufMemType)

The A2091 can only DMA into the first 16 MB (chip, slow `$C00000`, Zorro II
RAM). What happens with buffers above that depends on the partition's RDB
settings:

| Partition setting | Effect with Zorro III fast RAM |
|---|---|
| `Mask 0x7FFFFFFE` (default of most tools) | driver uses polled I/O into Z3 RAM: ~130 KB/s |
| `Mask 0x00FFFFFE`, `BufMemType 0` | FFS bounces, but may put its buffers in Z3 RAM again: ~80 KB/s |
| `Mask 0x00FFFFFE`, `BufMemType 0x201` | FFS bounces through chip RAM, transfers by DMA: ~150–175 KB/s |

With **Zorro II fast RAM** (or chip RAM only) everything is DMA and runs at
1.2–1.8 MB/s regardless of these settings. For the A4091 keep
`Mask 0x7FFFFFFE`.

### Debugging

* Create `/media/fat/a2091_debug` and reload the core: command log in
  `/media/fat/logs/a2091.log` (halves throughput while enabled).
* Create `/media/fat/a2091_profile`: per-2-second activity summary in
  `/media/fat/logs/a2091_prof.log`.

---

## Tests

### Before hardware

| Test | Result |
|---|---|
| `tb/a2091_tb.v` — RTL unit test (iverilog): autoconfig, ROM, WD register auto-increment, status FIFO, INT2/ISTR, DMA both ways, unreachable-address DMA, PIO handshake, reset | pass |
| `sim/` — Verilator co-simulation of `a2091.v` with the real ARM code and a scripted 68k: WD reset, INQUIRY, selection timeout, 150 KB READ (multi-chunk DMA), WRITE + readback, CHECK CONDITION + REQUEST SENSE, A590-style select-with-ATN + PIO message/command/data, PIO WRITE, 5 KB PIO read across two streams, byte-exact helper DMA for the A4091, INT2; mailbox stressed at a strobe every 2 clocks | pass |

### On a MiSTer (68020 / 68000, A590/A2091 ROM 390721/2-07)

| Test | Result |
|---|---|
| `showconfig` | `Commodore (West Chester) A 590/2091 SCSI: Prod=514/3 (@$EB0000 64KB)`; in the combined core with the A4091 selected only the A4091 appears (`Prod=514/84 @$50000000`) |
| Drive scan at boot | IDs 0–7 probed, present drive answers, absent IDs time out |
| Mount | `Test200MB.hdf` (ID1) mounts as `RegrTest:` |
| **Autoboot from the A2091** | AmigaOS boots from the A2091 drive — **68020 and 68000** |
| `a2091test RegrTest: 256 2` (write / read / byte compare, plus 16 odd-sized reads at odd offsets, chip and fast buffers) | **PASSED** in every configuration below |
| OSD controller selector | works (user tested) |
| A4091 in the combined core, FFS buffers in chip RAM | PASSED (chip 1163 / 1422 KB/s) |
| Main_MiSTer on official `d5f3844` | old 13426-byte config converted, IDE boots, `RegrTest:` mounts |

Throughput (`a2091test`, 256 KB file, KB/s, write / read):

| Memory | A2091 | Path |
|---|---:|---|
| Chip RAM | 1422 / 1600–1828 | DMA |
| Zorro II fast RAM (8 MB) | 1163–1280 / 1600 | DMA |
| Zorro II fast RAM, 1 MB file | 1248–1383 / 1651–1765 | DMA |
| Zorro III fast RAM, Mask 0x7FFFFFFE | 112–130 / 131–134 | polled I/O |
| Zorro III fast RAM, Mask 0x00FFFFFE + BufMemType 0x201 | 147 / 175 | FFS bounce + DMA |

How the polled path got there: 16 KB/s (one ARM round trip per byte) →
128 KB/s (whole blocks streamed by the FPGA) → 130 KB/s with IDE-style fast
SPI (which mainly lifted DMA from ~1.0 to ~1.8 MB/s). The remaining polled-I/O
limit is the A590 driver itself (one 512-byte block per SCSI command; the ARM
is busy only 18 % of the time).

### Not yet tested

* Selecting **A2091** in the OSD on a 68020 in the combined core, and a 68000
  on the combined core.
* Several drives on the A2091 at once.
* GuruROM / other 32 KB ROMs (not supported).

### Known issues

* The combined core misses timing by 0.40 ns on a stock path (floppy/CIA →
  Agnus → DMA arbiter → SDRAM address), not the SCSI logic; it has run
  without problems. Other fitter seeds are being tried.
* An image partitioned with `Mask 0x00FFFFFE` + `BufMemType 0x201` (best for
  the A2091 with Zorro III RAM) is slow on the A4091 (~60 KB/s with fast-RAM
  buffers); use `Mask 0x7FFFFFFE` for A4091 images.
