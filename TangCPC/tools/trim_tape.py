#!/usr/bin/env python3
"""Cut the unused zero fill off a recorded CDT (everything after the end marker).

usage: trim_tape.py IN.cdt OUT.cdt
The end marker written by the core is block 0x2A (stop the tape if in 48K mode)
followed by the terminator byte 0x00; the output ends after the 0x2A block.
"""
import sys

def main():
    if len(sys.argv) != 3:
        print(__doc__); return 1
    d = open(sys.argv[1], "rb").read()
    i = 10
    sizes = {0x20: 2, 0x21: None, 0x2A: 4}
    while i < len(d):
        b = d[i]
        if b == 0x00:
            break
        if b == 0x10:   n = 4 + (d[i+3] | d[i+4] << 8)
        elif b == 0x11: n = 19 + (d[i+16] | d[i+17] << 8 | d[i+18] << 16)
        elif b == 0x12: n = 4
        elif b == 0x13: n = 1 + 2 * d[i+1]
        elif b == 0x14: n = 10 + (d[i+8] | d[i+9] << 8 | d[i+10] << 16)
        elif b == 0x20: n = 2
        elif b == 0x2A: n = 4
        else:
            print(f"unsupported block 0x{b:02X} at {i}, copying the rest unchanged"); i = len(d); break
        i += 1 + n
    open(sys.argv[2], "wb").write(d[:i])
    print(f"{sys.argv[2]}: {i} bytes (from {len(d)})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
