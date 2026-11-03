#!/usr/bin/env python3
"""Generate a diagnostic CPC background ROM for testing upper-ROM slots.

  python3 -I tools/make_test_rom.py [out.rom]

Load it into an unused slot (1..6, 8..15) with the OSD, reset, then type:
  |HELLO0   code in sector 0  of the ROM  (offset 0x0080)
  |HELLO1   code in sector 1               (offset 0x0240)
  |HELLO8   code in sector 8               (offset 0x1040)
  |HELLO31  code in sector 31              (offset 0x3E40)
  |CHK      sums all 16384 bytes of the ROM and prints the 16 bit sum in hex;
            the expected value is printed by this script.

Reading the results:
  - all HELLO* print           : code execution from this upper ROM slot works
  - |CHK differs from expected : ROM bytes are corrupted on the way (SD read, SDRAM write/read)
  - HELLO0 ok, others hang     : sectors > 0 were not loaded correctly
  - nothing works / hangs      : executing upper ROM code != BASIC is broken

CPC header: &C000 type(1=background) mark version mod, &C004 name table address,
&C006 JP init, &C009 JP cmd1 ...  Names: last char has bit 7 set, table ends with 0.
"""
import sys

out = sys.argv[1] if len(sys.argv) > 1 else "hello.rom"
B = 0xC000
rom = bytearray(16384)

class Asm:
    def __init__(self, org):
        self.org, self.b, self.lab, self.fix = org, bytearray(), {}, []
    def here(self):          return self.org + len(self.b)
    def label(self, n):      self.lab[n] = self.here()
    def db(self, *v):        self.b += bytes(v)
    def jp(self, n):         self.db(0xC3); self.fix.append((len(self.b), n, 'abs')); self.db(0, 0)
    def call(self, a):
        self.db(0xCD);       self.b += a.to_bytes(2, "little")
    def jr(self, op, n):     self.db(op); self.fix.append((len(self.b), n, 'rel')); self.db(0)
    def ldhl(self, n):       self.db(0x21); self.fix.append((len(self.b), n, 'abs')); self.db(0, 0)
    def done(self):
        for pos, n, kind in self.fix:
            t = self.lab[n]
            if kind == 'abs': self.b[pos:pos+2] = t.to_bytes(2, "little")
            else:
                rel = t - (self.org + pos + 1)
                assert -128 <= rel <= 127, n
                self.b[pos] = rel & 0xFF
        return bytes(self.b)

TXT_OUTPUT = 0xBB5A

def hello_block(off, tag, text):
    a = Asm(B + off)
    a.ldhl(tag + "msg")
    a.label(tag + "loop")
    a.db(0x7E, 0xB7, 0xC8)               # LD A,(HL) ; OR A ; RET Z
    a.call(TXT_OUTPUT)
    a.db(0x23)                           # INC HL
    a.jr(0x18, tag + "loop")             # JR loop
    a.label(tag + "msg")
    a.b += text.encode("ascii") + b"\r\n\x00"
    return B + off, a.done()

cmds = [("HELLO0", 0x0080, "HELLO0: code in sector 0 ok"),
        ("HELLO1", 0x0240, "HELLO1: code in sector 1 ok"),
        ("HELLO8", 0x1040, "HELLO8: code in sector 8 ok"),
        ("HELLO31", 0x3E40, "HELLO31: code in sector 31 ok")]

# |CHK : sum of all 16 KB of this ROM, printed as 4 hex digits
CHK_OFF = 0x0100
a = Asm(B + CHK_OFF)
a.db(0x21, 0x00, 0xC0)                   # LD HL,&C000
a.db(0x01, 0x00, 0x40)                   # LD BC,&4000
a.db(0x11, 0x00, 0x00)                   # LD DE,0
a.label("lp")
a.db(0x7E)                               # LD A,(HL)
a.db(0x83)                               # ADD A,E
a.db(0x5F)                               # LD E,A
a.jr(0x30, "nc")                         # JR NC,nc
a.db(0x14)                               # INC D
a.label("nc")
a.db(0x23)                               # INC HL
a.db(0x0B)                               # DEC BC
a.db(0x78, 0xB1)                         # LD A,B ; OR C
a.jr(0x20, "lp")                         # JR NZ,lp
a.db(0x7A); a.call(0)                    # LD A,D ; CALL hex8   (patched below)
call1 = len(a.b) - 2
a.db(0x7B); a.call(0)                    # LD A,E ; CALL hex8
call2 = len(a.b) - 2
a.db(0x3E, 0x0D); a.call(TXT_OUTPUT)     # CR
a.db(0x3E, 0x0A); a.call(TXT_OUTPUT)     # LF
a.db(0xC9)                               # RET
a.label("hex8")
a.db(0xF5)                               # PUSH AF
a.db(0x0F, 0x0F, 0x0F, 0x0F)             # RRCA x4
a.jr(0x18, "hexn_call")                  # (skip over nothing, keeps layout simple)
a.label("hexn_call")
a.db(0xE6, 0x0F)                         # AND &0F
a.db(0xC6, 0x30)                         # ADD A,'0'
a.db(0xFE, 0x3A)                         # CP '9'+1
a.jr(0x38, "o1")                         # JR C,o1
a.db(0xC6, 0x07)                         # ADD A,7
a.label("o1")
a.call(TXT_OUTPUT)
a.db(0xF1)                               # POP AF
a.db(0xE6, 0x0F)                         # AND &0F
a.db(0xC6, 0x30)
a.db(0xFE, 0x3A)
a.jr(0x38, "o2")
a.db(0xC6, 0x07)
a.label("o2")
a.jp_target = None
a.db(0xC3); a.b += TXT_OUTPUT.to_bytes(2, "little")   # JP TXT_OUTPUT (returns to caller)
hex8_addr = a.lab["hex8"]
chk_code = bytearray(a.done())
chk_code[call1:call1+2] = hex8_addr.to_bytes(2, "little")
chk_code[call2:call2+2] = hex8_addr.to_bytes(2, "little")
rom[CHK_OFF:CHK_OFF + len(chk_code)] = chk_code

names = b""
def name(s):
    b = s.encode("ascii"); return b[:-1] + bytes([b[-1] | 0x80])
names += name("TESTROM")
jps = [0x0050]                           # init at offset 0x50
for nm, off, text in cmds:
    names += name(nm)
    addr, code = hello_block(off, nm + "_", text)
    rom[off:off + len(code)] = code
    jps.append(off)
names += name("CHK"); jps.append(CHK_OFF)
names += b"\x00"

rom[0:4] = bytes([0x01, 0x00, 0x01, 0x00])
NAMES = 0x20
rom[4:6] = (B + NAMES).to_bytes(2, "little")
for i, off in enumerate(jps):
    rom[6 + 3*i: 9 + 3*i] = bytes([0xC3]) + (B + off).to_bytes(2, "little")
assert 6 + 3*len(jps) <= NAMES, "jump block collides with name table"
rom[NAMES:NAMES + len(names)] = names
assert NAMES + len(names) <= 0x50
rom[0x50:0x52] = bytes([0x37, 0xC9])     # init: SCF ; RET

open(out, "wb").write(rom)
print(f"{out}: 16384 bytes; commands |HELLO0 |HELLO1 |HELLO8 |HELLO31 |CHK")
print(f"expected |CHK output: {sum(rom) & 0xFFFF:04X}")
