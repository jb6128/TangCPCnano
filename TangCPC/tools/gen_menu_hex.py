#!/usr/bin/env python3
"""Turn src/mcu/cpc.xml into src/mcu/cpc_xml.hex, the gzip'ed OSD menu that
sysctrl.v serves to the FPGA Companion (CMD 8).

Equivalent to:  gzip -n -c cpc.xml | xxd -c1 -p > cpc_xml.hex
(gzip -n = no name/timestamp, so the output is reproducible). The menu ROM in
sysctrl.v holds 2048 bytes; this script fails if the gzip stream is larger.

Run from TangCPC/:  python3 -I tools/gen_menu_hex.py
"""
import gzip
import io
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src/mcu/cpc.xml"
DST = ROOT / "src/mcu/cpc_xml.hex"
LIMIT = 2048

raw = SRC.read_bytes()
buf = io.BytesIO()
with gzip.GzipFile(fileobj=buf, mode="wb", mtime=0, filename="", compresslevel=9) as g:
    g.write(raw)
z = buf.getvalue()

if len(z) > LIMIT:
    sys.exit(f"menu too large: {len(z)} bytes gzip'ed, limit {LIMIT}")

DST.write_text("".join(f"{b:02x}\n" for b in z))
print(f"{SRC.name}: {len(raw)} bytes -> {len(z)} bytes gzip'ed (limit {LIMIT}) -> {DST.name}")
