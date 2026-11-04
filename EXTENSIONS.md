# CPCTang – Extensions beyond the original port

In collaboration with Claude (Sonnet 5.5 Medium, free plan).

This file describes the features added on top of the original Tang Nano 20K port
(see `README.md` and `TangCPC/PLAN.md` for the original work).

Added from other projects in this fork:
- OSD menu and FPGA Companion from [MiSTle-Dev](https://github.com/MiSTle-Dev).
- The TZX tape player is from NextNano (GPL).

## Setup assumed

- Tang Nano 20K with the **FPGA Companion** on the SPI link: it provides the USB keyboard/joystick, the SD card access and the on-screen menu (OSD) file selectors.
- Settings and mounted images are stored in `cpc.ini` on the SD card (OSD: Settings → Save settings).
- ROMs for the OS, BASIC and AMSDOS come from the flash (`TangCPC/boot.rom`, offset `0x200000`).

## Features

| Area | What it does | Status |
|---|---|---|
| Keyboard | USB keyboard through the FPGA Companion. Optional `Num Joystick` (OSD, default Off): the numeric keypad acts as CPC joystick 1 (see the table below). | works |
| DB9 joystick | CPC joystick 1 on the DB9 pins, optional fire-button swap. | works |
| Floppy | `.dsk` images for drive A/B through the OSD, read and write, AMSDOS On/Off, disk write protection, FDC speed. | works |
| Tape | `.cdt`/`.tzx` playback from SD, optional tape sound, polarity option. To rewind, reopen the file. | works |
| Tape recording | `SAVE` is recorded into the mounted CDT (OSD `Tape prot.` = Off). The image must be a pre-made blank tape of at least 1 MB (`tools/make_blank_tape.py tape.cdt 1024`). | works |
| Upper ROM slots | Up to 16 slots in the OSD: single ROM files (16k), 16-slot ROM sets (256k), clear slot, restore of the flash AMSDOS/BASIC, loading without a reset. A progress bar is shown while ROMs load. | works |
| Models | CPC 6128 (default) and 664. | experimental |
| HDMI video | 720×576p50 through a scandoubler, H pos / V pos, H lock, V lock (with selectable correction rate), 800 or 720 px width, green monitor mode, volume. OSD is fixed to the HDMI raster. | works (V lock: demos with changing video modes can still show dropouts) |
| System clock | Exact 32.000 MHz system clock (PLL cascade 27 → 96 → 160 MHz), so the CPC runs at its real speed and pixel clock. | works |
| 15 kHz RGB + CSYNC | Original CPC timing on header pins (2 bits per colour, Hi-Z unless `RGB out` is On) for RGB-to-HDMI converters or a CRT; fixed pixel phase offset. | works |
| Start-up | ROM set / disk / tape mounts from `cpc.ini` are deferred until the cold reset is released, so the Companion and the core do not use the SD card at the same time. | works |
| Debug LEDs | The six on-board LEDs show activity (AMSDOS, disk, tape, ROM load, tape recording, alive); holding S1/S2 shows static diagnostic pages. | works |

## OSD menu

Press **F12** to open the on-screen menu. Settings are stored with Settings → Save settings (`cpc.ini`). **Cold boot** behaves like a power-on (`cpc.ini` is loaded again), **Factory boot** starts with all settings at their defaults and without external ROMs (`cpc.ini` is neither read nor changed).

## Upper ROMs

A ROM set is a single .rom file: you can concatenate up to 16 ROM images (16 KB each) into one file of at most 256 KB. Block n is loaded into upper ROM slot n. Example: `copy /b a.rom+b.rom+c.rom set.rom` (Windows) or `cat a.rom b.rom c.rom > set.rom` (Linux).

Note: block 0 replaces the BASIC ROM (slot 0) and block 7 the AMSDOS ROM (slot 7). `Remove ROM-set` or `Clear slot` brings back the original ROMs from the flash.

## Pin assignments (FPGA pin numbers, Tang Nano 20K, `constraint/TangCPC.cst`)

All pins are 3.3 V LVCMOS. **Do not apply 5 V to any of them.**

### DB9 joystick (active low, internal pull-up)

| Signal | Core bus | FPGA pin | DB9 pin | Num key |
|---|---|---|---|---|
| UP | `io[2]` | 25 | 1 | 8 |
| RIGHT | `io[3]` | 26 | 4 | 6 |
| BTN0 = Fire 1 | `io[0]` | 27 | - | 0 |
| DOWN | `io[1]` | 28 | 2 | 2 |
| LEFT | `io[4]` | 29 | 3 | 4 |
| BTN1 = Fire 2 | `io[5]` | 30 | 6 | 5 |
| GND | – | any GND | 8 | – |

"Num key" applies when the OSD option `Num Joystick` is On: the keypad keys 8/2/4/6 are up/down/left/right, 7/9/1/3 are the diagonals (two directions at once), 5 and 0 are the fire buttons. With the option On, keypad 0–9 no longer type the CPC F-keys (the PC keys F1–F10 still do). Num Lock has no effect. The DB9 port keeps working in parallel. `DB9-Fire` (swap) only exchanges the two DB9 fire buttons, not the keypad keys.

Note: Do not connect DB9 pin 5/7 (+5 V).

### 15 kHz RGB + CSYNC

| Signal | FPGA pin (bit 0 = "full", bit 1 = "≥ half") |
|---|---|
| Red | `rgb_r[0]` 72, `rgb_r[1]` 71 |
| Green | `rgb_g[0]` 55, `rgb_g[1]` 49 |
| Blue | `rgb_b[0]` 73, `rgb_b[1]` 74 |
| CSYNC (active low) | 77 |

Right header: 72, 71 | 55, 49 (GND next to them); left header: 73, 74 and 77. Drive 8 mA. Notes: pin 49 is also `LCD_BL` (do not attach an LCD to the FPC socket); pin 55 is also `I2S_LRCK` of the on-board audio amplifier (harmless while `PA_EN` is not driven). The pins are high impedance until the OSD option `RGB out` is set to On.

Pins are 3V3. Connect CSYNC through a 220 Ω resistor and the colour pins through a 2-bit DAC:
```
FPGA bit 1 pin ──[ R ]──┐
                        ├──●── colour output ──► monitor input (75 or 100 Ω to GND)
FPGA bit 0 pin ──[ R ]──┘
```
Two equal resistors per colour (thermometer coding).
Full-scale level, calculated for 3.3 V pins:

| R     | 75 Ω input | 100 Ω input |
|-------|------------|-------------|
| 470 Ω | 0.80 V     | 0.99 V      |
| 499 Ω | 0.76 V     | 0.94 V      |
| 560 Ω | 0.70 V     | 0.88 V      |

The mid level is exactly half of full-scale.
According to the CPC6128 Service Manual, the Amstrad CTM644 colour monitor
has 100 Ω input termination (DC coupled, 0 V = black). A real CPC delivers
roughly 1 V full-scale, so R = 470 Ω is a good choice for the original
monitor, and also fine for 75 Ω RGB-to-HDMI converters (about 0.8 V).

## Known limitations

- The OSD menu cannot show, which ROM slots are loaded.
- Some demos that change the CRTC timing (e.g. Batman Forever) show brief picture dropouts or a rolling picture on HDMI; a frame buffer would be needed to retime them.
- A phantom "cursor right" can appear when the monitor is switched to the HDMI input (floating DB9 RIGHT line); it is not fully explained.
- The Pico UART keyboard path is still in the RTL but is not used by the current setup.
- The OSS toolchain build (`tools/build_oss.sh`) is parked (placement fails); use the Gowin flow.

## Tools

`tools/gen_menu_hex.py` (OSD menu → `cpc_xml.hex`), `tools/make_blank_tape.py`, `tools/trim_tape.py` (cut the zero fill of a recorded tape), `tools/make_test_rom.py` (test ROM), `tools/check_hid_cpc_map.py` (key table check), `tools/build_oss.sh` (parked).
