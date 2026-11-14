#!/usr/bin/env python3
"""Check and cut out RAM snapshots captured from pin 48 (OSD Debug / Snapshot = S1+S2, S1 + S2 pressed together).

Capture: stty -F /dev/ttyUSB1 921600 raw; cat /dev/ttyUSB1 > cap.bin   (baud = OSD Debug / UART: 115200, 460800 or
         921600, default 921600; or any terminal program that logs raw bytes)
Use:     python3 -I tools/snap_check.py cap.bin [ram.bin] [-n N]

A capture may contain several snapshots one after the other (e.g. cat appending); every one is checked and listed,
`-n N` selects the N-th (default 1) for ram.bin (the 64 KB RAM, written when ram.bin is given).

Layout, version 2 (65792 bytes; the header is 256 bytes, the RAM starts at offset 0x100, 16-bit values little endian):
    0..3    'SNAP'
    4       version 2
    5..6    PC (address of the last opcode fetch)
    7       last ROM select (&DF00)
    8       last RAM configuration (&7Fxx, 11xxxxxx)
    9       last Gate Array mode / ROM enable (&7Fxx, 10xxxxxx)
    10..11  16-bit sum of the 65536 RAM bytes
    12      number of 64 KB RAM pages that follow (1: the base RAM the video reads)
    13      CRTC type (0 / 1)
    14..15  0
    16..33  CRTC registers R0..R17 as last written by the Z80 (R16 / R17 are read only: 0)
    34..50  Gate Array palette as last written, hardware colour numbers 0..31: pens 0..15, then the border
    51..255 0
    256..   65536 bytes RAM (offset 0x100 .. 0x100FF)
Layout, version 1 (older builds, 65554 bytes): 'SNAP', 1, PC (2), ROM select, RAM configuration, GA mode, 6 x 0,
    65536 bytes RAM, 16-bit sum (low byte first). Neither CRTC registers nor palette are in it.
"""
import sys

HDR2 = 256
RAM = 65536


def parse(d, i):
    """Parse the snapshot whose magic is at offset i. Returns (header dict, ram bytes, end offset) or (None, msg, i + 5)."""
    ver = d[i + 4]
    if ver == 2:
        need = HDR2 + RAM
        if len(d) < i + need:
            return None, f"capture too short: {len(d) - i} bytes after the magic, {need} needed", i + 5
        h = d[i:i + HDR2]
        ram = d[i + HDR2:i + HDR2 + RAM]
        return dict(ver=2, pc=h[5] | h[6] << 8, rsel=h[7], mmr=h[8], gam=h[9], sum=h[10] | h[11] << 8, pages=h[12],
                    crtc_type=h[13], crtc=list(h[16:34]), pal=list(h[34:51])), ram, i + need
    if ver == 1:
        need = 16 + RAM + 2
        if len(d) < i + need:
            return None, f"capture too short: {len(d) - i} bytes after the magic, {need} needed", i + 5
        h = d[i:i + 16]
        ram = d[i + 16:i + 16 + RAM]
        return dict(ver=1, pc=h[5] | h[6] << 8, rsel=h[7], mmr=h[8], gam=h[9],
                    sum=d[i + 16 + RAM] | d[i + 17 + RAM] << 8), ram, i + need
    return None, f"unknown version {ver}", i + 5


def find_all(d):
    """List of (offset, header or None, ram or message). After a complete snapshot the search continues behind it,
    so a "SNAP" inside the RAM data is not taken for a new snapshot."""
    out = []
    i = 0
    while True:
        i = d.find(b"SNAP", i)
        if i < 0 or i + 5 > len(d):
            return out
        h, ram, end = parse(d, i)
        out.append((i, h, ram))
        i = end


def describe(h, ram):
    s = sum(ram) & 0xFFFF
    print(f"  version {h['ver']}  PC=&{h['pc']:04X}  ROM select=&{h['rsel']:02X}  RAM config=&{h['mmr']:02X}  GA mode=&{h['gam']:02X}"
          f" (screen mode {h['gam'] & 3})")
    print(f"  checksum {'OK' if s == h['sum'] else 'MISMATCH'} (sent &{h['sum']:04X}, computed &{s:04X})")
    if h["ver"] == 2:
        r = h["crtc"]
        start = (r[12] & 0x3F) << 8 | r[13]
        print(f"  CRTC type {h['crtc_type']}  R0={r[0]} R1={r[1]} R2={r[2]} R3=&{r[3]:02X} R4={r[4]} R5={r[5]} R6={r[6]} R7={r[7]}"
              f" R8=&{r[8]:02X} R9={r[9]}  start R12/R13=&{start:04X} -> screen block &{(start & 0x3000) << 2:04X}, offset &{(start & 0x3FF) * 2:03X}")
        print("  palette (hardware colour numbers) pens 0..15: " + " ".join(f"{p:2d}" for p in h["pal"][:16]) + f"   border: {h['pal'][16]}")
        if h["pages"] != 1:
            print(f"  note: header says {h['pages']} RAM pages, this tool handles 1")


def main():
    args = sys.argv[1:]
    n = 1
    if "-n" in args:
        k = args.index("-n")
        try:
            n = int(args[k + 1])
        except (IndexError, ValueError):
            sys.exit(__doc__)
        del args[k:k + 2]
    if not args:
        sys.exit(__doc__)
    d = open(args[0], "rb").read()
    found = find_all(d)
    if not found:
        sys.exit("no SNAP header found")
    good = []
    for k, (off, h, ram) in enumerate(found, 1):
        print(f"snapshot {k} at offset {off}:")
        if h is None:
            print(f"  {ram}")
            continue
        describe(h, ram)
        good.append((k, h, ram))
    if len(args) > 1:
        if not 1 <= n <= len(found) or found[n - 1][1] is None:
            sys.exit(f"snapshot {n} is not complete / not present")
        open(args[1], "wb").write(found[n - 1][2])
        print(f"{RAM} bytes of snapshot {n} written to {args[1]}")


main()
