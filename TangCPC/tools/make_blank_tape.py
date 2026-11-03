#!/usr/bin/env python3
"""Create a blank CDT/TZX tape image for CPCTang tape recording (SAVE).

The image cannot grow while the core records, so it is created at its final size:
a TZX header followed by zero bytes. Block ID 0x00 is not a valid block, the core
treats it as "end of tape" and starts recording there.

usage: make_blank_tape.py OUT.cdt [size_in_KiB]      (default 4096 KiB)

Rule of thumb: one byte saved at 1000 baud takes ~32 bytes of CDT, at 2000 baud
~16..20 bytes (pilot tone and gaps come on top), so 4 MiB hold about 128..200 KiB.
"""
import sys

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    kib = int(sys.argv[2]) if len(sys.argv) > 2 else 4096
    size = kib * 1024
    with open(sys.argv[1], "wb") as f:
        f.write(b"ZXTape!\x1a" + bytes([1, 20]))
        f.write(bytes(size - 10))
    print(f"{sys.argv[1]}: blank tape, {kib} KiB")
    return 0

if __name__ == "__main__":
    sys.exit(main())
