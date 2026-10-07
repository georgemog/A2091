#!/usr/bin/env python3
"""
a2091cfg.py - show / edit the A2091 SCSI fields of a MiSTer Minimig config
(config/Minimig*.cfg = raw dump of Main_MiSTer mm_configTYPE).

Offsets are for Main_MiSTer upstream + the A2091 patch (sizeof = 14468):

  1038 memory   1039 chipset   1043 ide_cfg   1046 hardfile[4] (1026 each)
  5150 cpu      7280 a2091_cfg 7284 a2091_hdd[7] (1026 each: cfg, res, name[1024])

Older (shorter) files are zero-extended; writing produces a full-size file.

Examples:
  a2091cfg.py Minimig.cfg                          # show
  a2091cfg.py Minimig.cfg --enable --id 0 games/Amiga/scsi0.hdf
  a2091cfg.py Minimig.cfg --disable
  a2091cfg.py Minimig.cfg --id 0 ""                # clear ID0
  a2091cfg.py Minimig.cfg --ide off                # IDE controller off
"""
import argparse
import struct
import sys

SIZE = 14468
OFF_MEMORY = 1038
OFF_IDE = 1043
OFF_HF = 1046
OFF_CPU = 5150
OFF_A2091 = 7280
OFF_A2091_HDD = 7284
HF = 1026
LEGACY = 7280      # size of an upstream config without the A2091 fields


def hf_get(buf, off):
    cfg = buf[off]
    name = buf[off + 2:off + HF].split(b"\0", 1)[0].decode("latin-1")
    return cfg, name


def hf_set(buf, off, name):
    raw = name.encode("latin-1")[:1023]
    buf[off] = 1 if raw else 0
    buf[off + 1] = 0
    buf[off + 2:off + HF] = raw + b"\0" * (1024 - len(raw))


def show(path, buf, orig_len):
    print(f"file      {path} ({orig_len} bytes{', legacy' if orig_len < SIZE else ''})")
    print(f"id        {bytes(buf[0:8])!r}")
    print(f"cpu       0x{buf[OFF_CPU]:02x}   memory 0x{buf[OFF_MEMORY]:02x}")
    print(f"IDE       {'on' if buf[OFF_IDE] & 1 else 'off'} (ide_cfg 0x{buf[OFF_IDE]:02x})")
    for i in range(4):
        cfg, name = hf_get(buf, OFF_HF + i * HF)
        print(f"  hdd{i}    cfg={cfg} {name}")
    print(f"A2091     {'ENABLED' if buf[OFF_A2091] & 1 else 'disabled'}")
    for i in range(7):
        cfg, name = hf_get(buf, OFF_A2091_HDD + i * HF)
        if cfg or name:
            print(f"  ID{i}     cfg={cfg} {name}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--enable", action="store_true")
    ap.add_argument("--disable", action="store_true")
    ap.add_argument("--id", nargs=2, action="append", metavar=("N", "PATH"),
                    help="set SCSI ID N (0-6) image, empty PATH clears it")
    ap.add_argument("--ide", choices=["on", "off"])
    ap.add_argument("--out", help="write here instead of in place")
    a = ap.parse_args(argv)

    data = open(a.file, "rb").read()
    if data[:8] != b"MNMGCFG0":
        sys.exit("not a Minimig config")
    if len(data) > SIZE:
        sys.exit(f"config is {len(data)} bytes, larger than this layout ({SIZE}) - "
                 "written by a different Main_MiSTer build?")
    buf = bytearray(data) + bytearray(SIZE - len(data))
    if len(data) > LEGACY and len(data) != SIZE:
        print(f"warning: {len(data)} bytes is neither upstream ({LEGACY}) nor A2091 ({SIZE}); "
              "fields past 7270 belong to another build and are reset", file=sys.stderr)
        # A4091 builds put their SCSI fields at 7270, over userport/externalfloppy
        buf[7270:] = bytearray(SIZE - 7270)

    changed = False
    if a.enable:
        buf[OFF_A2091] |= 1; changed = True
    if a.disable:
        buf[OFF_A2091] &= ~1; changed = True
    for n, path in a.id or []:
        n = int(n)
        if not 0 <= n <= 6:
            sys.exit("SCSI ID must be 0..6")
        hf_set(buf, OFF_A2091_HDD + n * HF, path); changed = True
    if a.ide:
        buf[OFF_IDE] = (buf[OFF_IDE] | 1) if a.ide == "on" else (buf[OFF_IDE] & ~1)
        changed = True

    show(a.file, buf, len(data))
    if changed:
        out = a.out or a.file
        open(out, "wb").write(bytes(buf))
        print(f"written   {out}")


if __name__ == "__main__":
    main()
