# CPCTang — Amstrad CPC 6128 on Tang Nano 20K

An Amstrad CPC 6128 FPGA core for the [Sipeed Tang Nano 20K](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html) board, with HDMI output.

## Origins

This project is a port of the [Amstrad_MiSTer](https://github.com/MiSTer-devel/Amstrad_MiSTer) core to the Tang Nano 20K FPGA (Gowin GW2AR-18). The video pipeline (HDMI/TMDS, scandoubler) and infrastructure (flash loader, PLL, SPI) are adapted from [MiSTeryNano](https://github.com/harbaum/MiSTeryNano) (Atari ST on Tang Nano 20K).

**From Amstrad_MiSTer** (kept mostly unchanged):
- Z80 CPU (T80pa, VHDL)
- Gate Array (GA40010)
- CRTC (UM6845R)
- PSG (YM2149)
- FDC (u765)
- PPI (i8255)
- MMU, color mixer, motherboard glue logic

**From MiSTeryNano** (adapted for CPC):
- HDMI/TMDS encoder and PAL timing
- Scandoubler (15 kHz → 31 kHz)
- Flash SPI loader (boot ROM from onboard flash)
- PLL configuration (27 MHz → 160 MHz TMDS + 32 MHz system)
- MCU SPI interface stubs (for future BL616 integration)

**Written new for this project:**
- `top.sv` — top-level integration and pin mapping
- `cpc_core.sv` — CPC core wrapper (replaces MiSTer HPS interface)
- `cpc_sdram.v` — multi-port SDRAM controller (3 ports, 8-bit, with boot/VRAM/CPU arbitrage)
- `cpc_flash.v` — ROM loader from flash SPI
- Pin constraints and timing constraints for Tang Nano 20K

## Current status

The CPC boots from flash ROM and displays the Amstrad 128K prompt (yellow text on blue background) over HDMI. The Z80 executes the OS ROM, programs the CRTC, and shows the command prompt.

**Working:**
- Z80 CPU execution from SDRAM
- ROM loading from flash (OS + BASIC + AMSDOS)
- CRTC video output via HDMI (scandoubled PAL)
- Gate Array colour palette

**Not yet implemented:**
- Keyboard input (BL616 MCU / HID)
- Floppy disk controller (directly from SD card)
- Audio (YM2149 PSG output)
- OSD menu

## How to build

### Prerequisites

- **Gowin Education IDE** v1.9.11 or later — [download here](https://www.gowinsemi.com/en/support/download_eda/)
- **openFPGALoader** — for flashing the bitstream
- A **Tang Nano 20K** board

### Build the bitstream

```bash
cd TangCPC
"C:/Gowin/Gowin_V1.9.11.03_Education_x64/IDE/bin/gw_sh.exe" build.tcl
```

Adjust the Gowin path to match your installation. The build takes about 25 seconds and produces `impl/pnr/TangCPC.fs`.

### Flash the FPGA

```bash
openFPGALoader -b tangnano20k impl/pnr/TangCPC.fs
```

### Flash the CPC ROMs

The CPC ROMs must be written to flash at offset `0x200000`. A `boot.rom` is included in `TangCPC/`:

```bash
openFPGALoader -b tangnano20k --write-flash --offset 0x200000 TangCPC/boot.rom
```

## Architecture

```
27 MHz oscillator (Tang Nano 20K)
    │
    ├── rPLL → 160 MHz (HDMI TMDS clock)
    │     └── CLKDIV /5 → 32 MHz (system clock, pixel clock)
    ├── rPLL → 100 MHz (flash SPI clock)
    │
    └── cpc_core
         │
         ├── Amstrad_motherboard (from MiSTer)
         │    ├── GA40010  — timing, video, memory arbitration
         │    ├── T80pa    — Z80 CPU (VHDL)
         │    ├── UM6845R  — CRTC 6845
         │    ├── YM2149   — PSG sound
         │    ├── u765     — floppy disk controller
         │    └── i8255    — PPI (keyboard, tape, vsync)
         │
         ├── cpc_sdram  — 3-port SDRAM controller
         ├── cpc_flash  — ROM loader from SPI flash
         │
         └── Video pipeline:
              GA40010 (2-bit RGB)
               → color_mix (palette → 8-bit RGB)
                → scandoubler (15 kHz → 31 kHz)
                 → video2hdmi (TMDS encoding)
                  → HDMI output
```

## Developed with Claude

This project was designed and coded in collaboration with [Claude Code](https://claude.ai/), Anthropic's AI coding assistant. The entire RTL integration — `top.sv`, `cpc_core.sv`, `cpc_sdram.v`, the video pipeline adaptation, pin constraints, build scripts, and iterative debugging — was done through conversation with Claude, starting from the MiSTer and MiSTeryNano source code as references.

The development process followed an incremental approach:
1. **Phase 0** — Project scaffolding: PLLs, HDMI, pin mapping → first bitstream
2. **Phase 1** — Multi-port SDRAM controller for Boot/VRAM/CPU access
3. **Phase 2** — Flash ROM loader (OS + BASIC + AMSDOS from SPI flash)
4. **Phase 3** — RTL integration: connecting the Amstrad motherboard, fixing bus timing, debugging with LEDs until the CPC booted and displayed the prompt

Each phase was designed, implemented, synthesized, and debugged interactively — Claude writing the Verilog/SystemVerilog, the human flashing the board and reporting LED states and screen output.

## License

The Amstrad CPC core RTL originates from [Amstrad_MiSTer](https://github.com/MiSTer-devel/Amstrad_MiSTer) (GPL-2.0).
The video/HDMI pipeline originates from [MiSTeryNano](https://github.com/harbaum/MiSTeryNano) (GPL-3.0).
