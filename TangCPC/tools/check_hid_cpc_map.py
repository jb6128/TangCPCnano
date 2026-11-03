#!/usr/bin/env python3
"""Cross-check src/mcu/hid_cpc_matrix.v (USB HID -> CPC matrix) against the
Amstrad_MiSTer PS/2 table in src/rtl/hid.sv.

hid.sv maps PS/2 set-2 scancodes to key[row][bit]. We translate each of those
scancodes to the USB HID usage code of the same physical key and require that
hid_cpc_matrix.v lists that HID code in the same cell. Entries in hid.sv that
are multi-line / keypad_mod ternaries are not machine-compared; they are
checked against a short explicit list below.

Run from TangCPC/:  python3 -I tools/check_hid_cpc_map.py
Exit code 0 = all checks pass.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
V = (ROOT / "src/mcu/hid_cpc_matrix.v").read_text()
S = (ROOT / "src/rtl/hid.sv").read_text()

# --- PS/2 set-2 scancode -> USB HID usage (physical US key), (ext, code) -----
PS2 = {}
def add(ext, ps2, hid): PS2[(ext, ps2)] = hid
for ps2, hid in {
    0x1C:0x04,0x32:0x05,0x21:0x06,0x23:0x07,0x24:0x08,0x2B:0x09,0x34:0x0A,0x33:0x0B,
    0x43:0x0C,0x3B:0x0D,0x42:0x0E,0x4B:0x0F,0x3A:0x10,0x31:0x11,0x44:0x12,0x4D:0x13,
    0x15:0x14,0x2D:0x15,0x1B:0x16,0x2C:0x17,0x3C:0x18,0x2A:0x19,0x1D:0x1A,0x22:0x1B,
    0x35:0x1C,0x1A:0x1D,
    0x16:0x1E,0x1E:0x1F,0x26:0x20,0x25:0x21,0x2E:0x22,0x36:0x23,0x3D:0x24,0x3E:0x25,
    0x46:0x26,0x45:0x27,
    0x5A:0x28,0x76:0x29,0x66:0x2A,0x0D:0x2B,0x29:0x2C,0x4E:0x2D,0x55:0x2E,0x54:0x2F,
    0x5B:0x30,0x5D:0x31,0x4C:0x33,0x52:0x34,0x41:0x36,0x49:0x37,0x4A:0x38,0x58:0x39,
    0x61:0x64,
    0x05:0x3A,0x06:0x3B,0x04:0x3C,0x0C:0x3D,0x03:0x3E,0x0B:0x3F,0x83:0x40,0x0A:0x41,
    0x01:0x42,0x09:0x43,
    0x7D:0x61, 0x6C:0x5F, 0x7B:0x56,   # KP9, KP7, KP- (plain lines in hid.sv)
    0x12:0x69,   # LShift (companion remap E1 -> 0x69)
    0x14:0x68,   # LCtrl  (E0 -> 0x68)
}.items():
    add(False, ps2, hid)
for ps2, hid in {   # E0-prefixed
    0x70:0x49,0x71:0x4C,0x69:0x4D,0x7A:0x4E,0x74:0x4F,0x6B:0x50,0x72:0x51,0x75:0x52,
    0x4A:0x54,0x5A:0x58,0x14:0x6C,
}.items():
    add(True, ps2, hid)

# --- parse our Verilog table ---------------------------------------------------
ours = {}   # (row, bit) -> set(hid)
for m in re.finditer(r"assign\s+key\[(\d+)\*8\+(\d+)\]\s*=\s*([^;]+);", V):
    row, bit, rhs = int(m[1]), int(m[2]), m[3]
    codes = {int(c, 16) for c in re.findall(r"kb\[7'h([0-9A-Fa-f]+)\]", rhs)}
    ours[(row, bit)] = codes

errors = []
def err(msg): errors.append(msg); print("FAIL:", msg)

# structure checks
expected_cells = {(r, b) for r in range(9) for b in range(8)} | {(9, 7)}
if set(ours) != expected_cells:
    err(f"cell set mismatch: missing={sorted(expected_cells-set(ours))} "
        f"extra={sorted(set(ours)-expected_cells)}")
for cell, codes in ours.items():
    if not codes: err(f"cell {cell} has no HID code")
    for c in codes:
        if c > 0x7F: err(f"cell {cell}: code {c:#x} does not fit 7 bits")
if "assign key[9*8+6:9*8+0] = 7'd0;" not in V:
    err("row 9 joystick bits 0..6 are not tied to 0")

# --- parse hid.sv simple entries ------------------------------------------------
mode = None
simple = []
pat = re.compile(r"8'h([0-9A-Fa-f]{2}):\s*key\[(\d+)\]\[(\d)\]\s*<=\s*press(?:\s*&\s*~alt)?\s*;")
for line in S.splitlines():
    st = line.strip()
    if "if (ps2_key[8])" in st: mode = True; continue
    if st == "else" and mode is True: mode = False; continue
    if mode is False and st.startswith("case(ps2_key[7:0])") and simple:
        # third case block (Fn) starts after this one's endcase; stop at it
        pass
    if st.startswith("8'h78: begin"): break          # F11 NMI/reset, not a matrix key
    m = pat.match(st)
    if m and mode is not None:
        simple.append((mode, int(m[1], 16), int(m[2]), int(m[3])))

checked = 0
for ext, ps2, row, bit in simple:
    if (ext, ps2) not in PS2:
        err(f"PS/2 {'E0 ' if ext else ''}{ps2:02X} -> key[{row}][{bit}]: no HID translation in checker")
        continue
    hid = PS2[(ext, ps2)]
    if hid not in ours.get((row, bit), set()):
        err(f"PS/2 {'E0 ' if ext else ''}{ps2:02X} (HID {hid:#04x}) -> key[{row}][{bit}] "
            f"missing in hid_cpc_matrix.v (has {sorted(hex(c) for c in ours.get((row, bit), []))})")
    checked += 1

# --- keypad / ternary entries of hid.sv (keypad_mod = 0 branch), explicit ------
manual = {
    (0x59, (1, 5)), (0x5A, (1, 6)), (0x5B, (0, 5)), (0x5C, (2, 4)), (0x5D, (1, 4)),
    (0x5E, (0, 4)), (0x5F, (1, 2)), (0x60, (1, 3)), (0x61, (0, 3)), (0x62, (1, 7)),
    (0x63, (3, 7)), (0x56, (3, 1)),                       # KP1..9,0 -> F1..F9,F0; KP. KP-
    (0x57, (3, 4)), (0x57, (2, 5)), (0x55, (3, 5)), (0x55, (2, 5)),   # KP+ KP* (+Shift)
    (0x69, (2, 5)), (0x6D, (2, 5)), (0x6C, (2, 7)),       # shifts / right ctrl
    (0x58, (2, 2)), (0x54, (3, 6)),                       # KP Enter, KP /
}
for hid, cell in sorted(manual):
    if hid not in ours.get(cell, set()):
        err(f"manual check: HID {hid:#04x} expected in key[{cell[0]}][{cell[1]}]")

# --- report extras (human review) ----------------------------------------------
derived = {(PS2[(e, p)], (r, b)) for e, p, r, b in simple if (e, p) in PS2} | manual
extras = sorted((hid, cell) for cell, codes in ours.items() for hid in codes
                if (hid, cell) not in derived)
print(f"compared {checked} hid.sv entries + {len(manual)} manual keypad/modifier entries")
print(f"cells covered: {len(ours)}/73  distinct HID codes used: {len({c for s in ours.values() for c in s})}")
if extras:
    print("entries in hid_cpc_matrix.v NOT derived from hid.sv (review):")
    for hid, cell in extras:
        print(f"   HID {hid:#04x} -> key[{cell[0]}][{cell[1]}]")

print("RESULT:", "FAIL" if errors else "PASS")
sys.exit(1 if errors else 0)
