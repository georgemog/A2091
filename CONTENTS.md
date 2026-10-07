# Repository contents

| Path | What |
|---|---|
| `README.md` | project overview, installing, configuring, tests |
| `DESIGN.md` | design, mailbox protocol, bring-up history |
| `releases/` | **latest release**: `Minimig_A2091_A4091_20261007.rbf` (combined core) + `MiSTer_A2091` (Main_MiSTer, official `d5f3844` + A4091 + A2091 SCSI; install as `/media/fat/MiSTer_A2091` with `MiSTer.ini [minimig] main=MiSTer_A2091`) |
| `build/` | `Minimig_A2091_20261007d.rbf` (A2091-only core), `a2091test` (Amiga test tool) |
| `rtl/` | `a2091.v` (the board) |
| `core/a2091_only/`, `core/a2091_a4091/` | every core file changed vs upstream Minimig `8325735`, full contents; `*.patch` = the same as diffs |
| `main_mister/` | `minimig_a2091.{cpp,h}` |
| `integration/` | patches: core (`minimig_core.patch`, `combined_a4091.patch`), Main_MiSTer (`main_mister_d5f3844_scsi.patch` = current; older variants kept) |
| `tb/` | RTL unit testbench (iverilog) |
| `sim/` | Verilator co-simulation (a2091.v + real HPS code), `make` |
| `rom/` | boot ROM not included (Commodore copyright) — see `rom/README.md` |
| `tools/` | `a2091cfg.py`, `a2091test.c`, `deploy.sh` |

