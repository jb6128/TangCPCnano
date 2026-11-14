#!/usr/bin/env python3
"""Turn the CPC screen memory of a RAM snapshot into a PNG (standard library only).

Use:  python3 -I snap_to_png.py INPUT OUTPUT.png [--mode 0|1|2] [--base 0xC000] [--pens 1,24,20,6] [--scale 2] [-n N]

INPUT   a capture from pin 48 (SNAP header, version 2 or 1; several snapshots in one file: -n N selects the N-th),
        or a 64 KB RAM image (ram.bin from tools/snap_check.py)
--mode  screen mode 0 (160x200, 16 pens), 1 (320x200, 4 pens), 2 (640x200, 2 pens); default: from the header
        (Gate Array mode byte, bits 1:0), else 1
--base  start of the 16 KB screen block; with this option the CRTC registers of the header are ignored and the
        standard raster is drawn (40 characters x 25 rows x 8 lines, one of 0x0000, 0x4000, 0x8000, 0xC000)
--pens  ink numbers (0..26) of pen 0, 1, 2 ...; default: the palette of the header (version 2), else the firmware
        inks 1,24,20,6,26,0,2,8,10,12,14,16,18,22,1,16
--scale integer enlargement of the picture, default 2

Version 2 header: the picture is drawn the way the CRTC does it: screen start from R12/R13 (hardware scrolling, 16 KB
block), width R1 characters (2 bytes each), height R6 rows of R9 + 1 lines; the pens come from the Gate Array palette
(hardware colour numbers, decoded with the same gate logic as video.sv). The standard screen (R1 = 40, R6 = 25,
R9 = 7, R12/R13 = &3000) is 640 x 400 (times scale): CPC pixels keep their shape (mode 2 pixel 1x2, mode 1 pixel 2x2,
mode 0 pixel 4x2). Not used: the border colour, flashing of pens 14 / 15, a mode or palette change inside the
picture (one snapshot shows one state), the screen is taken from the 64 KB base RAM only.
"""
import argparse
import struct
import sys
import zlib

# the 27 hardware colours by ink number (0, 50 %, 100 % per channel)
LEVEL = (0, 128, 255)
INKS = [
    (0, 0, 0), (0, 0, 1), (0, 0, 2), (1, 0, 0), (1, 0, 1), (1, 0, 2), (2, 0, 0), (2, 0, 1), (2, 0, 2),
    (0, 1, 0), (0, 1, 1), (0, 1, 2), (1, 1, 0), (1, 1, 1), (1, 1, 2), (2, 1, 0), (2, 1, 1), (2, 1, 2),
    (0, 2, 0), (0, 2, 1), (0, 2, 2), (1, 2, 0), (1, 2, 1), (1, 2, 2), (2, 2, 0), (2, 2, 1), (2, 2, 2),
]
DEFAULT_PENS = [1, 24, 20, 6, 26, 0, 2, 8, 10, 12, 14, 16, 18, 22, 1, 16]


def hw_rgb(c):
    """RGB bytes of a Gate Array hardware colour number (0..31). Same gate logic as the RGB decoder in
    src/rtl/GA40010/video.sv: per channel the output is driven low / high (0 / 100 %) or switched off (50 %)."""
    b = [(c >> k) & 1 for k in range(5)]
    blue_oe_n = not ((b[1] or b[2]) and (b[3] or b[4]))
    green_oe_n = (b[1] and b[2]) or not (b[1] or b[2] or b[3] or b[4])
    red_oe_n = not (b[1] or b[2] or b[3] or b[4]) or (b[3] and b[4])
    blue = b[0]
    green = (not b[2] and b[0]) or b[1]
    red = (b[0] and not b[4]) or b[3]

    def lvl(oe_n, v):
        return 1 if oe_n else (2 if v else 0)

    return bytes(LEVEL[lvl(oe_n, v)] for oe_n, v in ((red_oe_n, red), (green_oe_n, green), (blue_oe_n, blue)))


def png(path, width, height, rows):
    """rows: list of bytes objects, 3 bytes (RGB) per pixel."""
    raw = b"".join(b"\x00" + r for r in rows)

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def pixels(byte, mode):
    """Pen numbers of one screen byte, left to right."""
    b = byte
    if mode == 2:
        return [(b >> (7 - i)) & 1 for i in range(8)]
    if mode == 1:
        return [((b >> (7 - i)) & 1) | (((b >> (3 - i)) & 1) << 1) for i in range(4)]
    p0 = ((b >> 7) & 1) | (((b >> 3) & 1) << 1) | (((b >> 5) & 1) << 2) | (((b >> 1) & 1) << 3)
    p1 = ((b >> 6) & 1) | (((b >> 2) & 1) << 1) | (((b >> 4) & 1) << 2) | ((b & 1) << 3)
    return [p0, p1]


def find_snapshots(d):
    """[(header dict, 64 KB RAM)] of all complete snapshots in a capture (the same scan as tools/snap_check.py)."""
    out = []
    i = 0
    while True:
        i = d.find(b"SNAP", i)
        if i < 0 or i + 5 > len(d):
            return out
        ver = d[i + 4]
        if ver == 2 and len(d) >= i + 256 + 65536:
            h = d[i:i + 256]
            out.append((dict(ver=2, gam=h[9], crtc=list(h[16:34]), pal=list(h[34:51])), d[i + 256:i + 256 + 65536]))
            i += 256 + 65536
        elif ver == 1 and len(d) >= i + 16 + 65536 + 2:
            out.append((dict(ver=1, gam=d[i + 9]), d[i + 16:i + 16 + 65536]))
            i += 16 + 65536 + 2
        else:
            i += 5


def render_legacy(ram, mode, base, rgb, scale):
    """Standard raster: 200 lines from one 16 KB block."""
    xs = {0: 4, 1: 2, 2: 1}[mode] * scale     # horizontal size of one CPC pixel
    ys = 2 * scale
    out = []
    for y in range(200):
        addr = base + (y & 7) * 0x800 + (y >> 3) * 80
        line = b"".join(rgb[p] * xs for byte in ram[addr:addr + 80] for p in pixels(byte, mode))
        out.extend([line] * ys)
    return out


def render_crtc(ram, crtc, mode, rgb, scale):
    """The picture as the CRTC / Gate Array fetch it: MA = start + row * R1 + column (14 bit), RAM address =
    MA13..12, raster line (3 bit), MA9..0, byte select; MA11..10 are not used (the screen wraps in 2 KB)."""
    r1, r6, r9 = crtc[1], crtc[6], crtc[9] & 0x1F
    start = (crtc[12] & 0x3F) << 8 | crtc[13]
    xs = {0: 4, 1: 2, 2: 1}[mode] * scale
    ys = 2 * scale
    out = []
    for row in range(r6):
        for ra in range(r9 + 1):
            line = bytearray()
            for col in range(r1):
                ma = (start + row * r1 + col) & 0x3FFF
                a = ((ma & 0x3000) << 2) | ((ra & 7) << 11) | ((ma & 0x3FF) << 1)
                for byte in (ram[a], ram[a + 1]):
                    for p in pixels(byte, mode):
                        line += rgb[p] * xs
            out.extend([bytes(line)] * ys)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--mode", type=int, choices=(0, 1, 2))
    ap.add_argument("--base", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--pens", default=None)
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("-n", type=int, default=1, help="which snapshot of a capture (default 1)")
    a = ap.parse_args()

    d = open(a.input, "rb").read()
    mode = a.mode
    hdr = None
    snaps = find_snapshots(d)
    if snaps:
        if not 1 <= a.n <= len(snaps):
            sys.exit(f"snapshot {a.n} not found ({len(snaps)} complete snapshots in the capture)")
        hdr, ram = snaps[a.n - 1]
        if mode is None:
            mode = hdr["gam"] & 3
            if mode == 3:
                mode = 1
            print(f"mode {mode} from the Gate Array mode byte &{hdr['gam']:02X}")
    else:
        ram = d
    if len(ram) != 65536:
        sys.exit(f"expected a 64 KB RAM image or a capture with a SNAP header, got {len(ram)} bytes")
    if mode is None:
        mode = 1
        print("no header: mode 1 assumed (--mode to change)")
    if a.base is not None and (a.base < 0 or a.base > 0xC000 or a.base & 0x3FFF):
        sys.exit("--base must be 0x0000, 0x4000, 0x8000 or 0xC000")

    npens = {0: 16, 1: 4, 2: 2}[mode]
    if a.pens:
        pens = [int(x) for x in a.pens.split(",")]
        pens = (pens + DEFAULT_PENS)[:npens] if len(pens) < npens else pens[:npens]
        if any(p < 0 or p > 26 for p in pens):
            sys.exit("ink numbers must be 0..26")
        rgb = [bytes(LEVEL[c] for c in INKS[p]) for p in pens]
        pal_src = "--pens"
    elif hdr is not None and hdr["ver"] == 2:
        rgb = [hw_rgb(p & 31) for p in hdr["pal"][:npens]]
        pal_src = "the header palette"
    else:
        rgb = [bytes(LEVEL[c] for c in INKS[p]) for p in DEFAULT_PENS[:npens]]
        pal_src = "the firmware inks"

    if hdr is not None and hdr["ver"] == 2 and a.base is None:
        c = hdr["crtc"]
        if c[1] == 0 or c[6] == 0:
            print("CRTC R1 / R6 are 0 (never written): standard raster at &C000")
            out = render_legacy(ram, mode, 0xC000, rgb, a.scale)
            what = "standard raster"
        else:
            out = render_crtc(ram, c, mode, rgb, a.scale)
            what = (f"CRTC raster {c[1] * 2} bytes x {c[6]} rows x {(c[9] & 0x1F) + 1} lines, screen block "
                    f"&{((c[12] & 0x30) << 10):04X}, start offset &{((c[12] & 3) << 8 | c[13]) * 2:03X}")
    else:
        base = 0xC000 if a.base is None else a.base
        out = render_legacy(ram, mode, base, rgb, a.scale)
        what = f"standard raster, screen at &{base:04X}"
    width = len(out[0]) // 3
    png(a.output, width, len(out), out)
    print(f"{a.output}: {width} x {len(out)}, mode {mode}, {what}, colours from {pal_src}")


main()
