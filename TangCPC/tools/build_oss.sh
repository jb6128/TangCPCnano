#!/usr/bin/env bash
# build_oss.sh -- TangCPC bitstream with the OSS CAD Suite (yosys + nextpnr-himbaechel + gowin_pack)
#
#   Usage (from anywhere):
#     OSS_CAD=~/oss-cad-suite tools/build_oss.sh          # build -> build_oss/TangCPC.fs
#     tools/build_oss.sh load                              # build and load into RAM
#     tools/build_oss.sh flash                             # build and write to flash
#
# The file list is taken from build.tcl (add_file lines), so it stays in sync with
# the Gowin flow: .v/.sv -> yosys, .vhd -> ghdl (yosys-ghdl plugin, for the T80 Z80),
# .hex -> copied next to the build so $readmemh finds it.
#
# All sources are read as SystemVerilog regardless of the .v/.sv extension (slang default);
# FRONTEND=yosys selects the built-in parser (does not work for this design: unpacked-array
# ports in src/video). Check the options with:  yosys -p 'plugin -i slang; help read_slang'
#
# NOT TESTED: written without access to the OSS CAD Suite. Expect to tune options.
# Differences to the Gowin flow: no SDC is used (nextpnr gets --freq only), and
# the vendor primitives rPLL, CLKDIV and OSER10 must be supported by Apicula.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OUT:-$ROOT/build_oss}"
TOP=top
DEVICE='GW2AR-LV18QN88C8/I7'
FAMILY='GW2A-18C'
FREQ="${FREQ:-32}"            # MHz, nextpnr timing target for the slowest-critical clock (clk32)
ACTION="${1:-build}"
FRONTEND="${FRONTEND:-slang}"      # slang | yosys   (yosys = built-in read_verilog, fails on the HDMI stack)
# --compat vcs: allows use before declaration (Gowin accepts it, slang does not by default)
# --ignore-unknown-modules is not known by the installed yosys-slang; removed (tested by the user)
# -D OSS_FLOW: see src/video/serializer.sv (yosys defines SYNTHESIS, the Gowin IDE does not)
SLANG_OPTS="${SLANG_OPTS:---extern-modules --compat vcs -D OSS_FLOW}"

# environment of the OSS CAD Suite (adds yosys, nextpnr-himbaechel, gowin_pack, ghdl, openFPGALoader)
if [ -n "${OSS_CAD:-}" ]; then
    # shellcheck disable=SC1091
    source "$OSS_CAD/environment"
fi
for t in yosys nextpnr-himbaechel gowin_pack; do
    command -v "$t" >/dev/null || { echo "missing tool: $t (set OSS_CAD=<path of oss-cad-suite>)"; exit 1; }
done

mkdir -p "$OUT"
cd "$ROOT"

# --- collect sources from build.tcl ----------------------------------------
VLOG=(); VHDL=(); HEX=()
while read -r f; do
    case "$f" in
        *.vhd) VHDL+=("$f") ;;
        *.v|*.sv) VLOG+=("$f") ;;
        *.hex) HEX+=("$f") ;;
    esac
done < <(sed -n 's/^add_file[[:space:]]\+\([^[:space:]]\+\).*/\1/p' build.tcl | grep -v '\.cst$\|\.sdc$')
for h in "${HEX[@]}"; do cp "$h" "$OUT/"; done

# --- yosys script ----------------------------------------------------------
YS="$OUT/synth.ys"
{
    if [ ${#VHDL[@]} -gt 0 ]; then
        echo "plugin -i ghdl"
        # T80 is VHDL-2008; elaborate its entity T80pa as a yosys module
        T80=(); PLAYER=()
        for v in "${VHDL[@]}"; do case "$v" in *tzxplayer*) PLAYER+=("$ROOT/$v");; *) T80+=("$ROOT/$v");; esac; done
        echo "ghdl --std=08 -fsynopsys ${T80[*]} -e T80pa"
        [ ${#PLAYER[@]} -gt 0 ] && echo "ghdl --std=08 -fsynopsys ${PLAYER[*]} -e tzxplayer"
    fi
    if [ "$FRONTEND" = slang ]; then
        # yosys-slang: full SystemVerilog (the HDMI stack uses unpacked-array ports,
        # which the built-in read_verilog -sv cannot parse).
        # --extern-modules : the VHDL modules imported by ghdl above stay visible
        # --compat vcs : use before declaration, as the Gowin flow accepts it
        echo "plugin -i slang"
        # Gowin primitives (rPLL, CLKDIV, OSER10, ELVDS_OBUF) are declared with their parameters in
        # tools/oss_gowin_prims.v ((* blackbox *)); the Gowin flow does not need that file.
        echo "read_slang --top $TOP $SLANG_OPTS -I$OUT -I$ROOT/src/mcu $ROOT/tools/oss_gowin_prims.v ${VLOG[*]/#/$ROOT/}"
        # make sure the primitives are real blackboxes whatever slang did with (* blackbox *)
        # (otherwise they would be empty modules that get flattened away)
        echo "blackbox rPLL CLKDIV OSER10 ELVDS_OBUF"
    else
        for f in "${VLOG[@]}"; do
            echo "read_verilog -sv -I$OUT -I$ROOT/src/mcu $ROOT/$f"
        done
    fi
    echo "hierarchy -top $TOP"
    echo "synth_gowin -top $TOP -json $OUT/$TOP.json"
} > "$YS"

echo "== yosys"
( cd "$OUT" && yosys -l "$OUT/yosys.log" -s "$YS" )

# --- constraints for nextpnr ------------------------------------------------
# The Gowin tool takes a differential pair as  IO_LOC "x_p" a,b;  (b = complementary pin).
# nextpnr reports the *_n port as unconstrained, so split each pair into two explicit IO_LOC
# lines and give the _n port the same IO_PORT attributes. The Gowin .cst itself stays untouched.
CST="$OUT/TangCPC_oss.cst"
sed -E \
    -e 's/^IO_LOC "(tmds_[a-z]+_)p(\[[0-9]\])?" ([0-9]+),([0-9]+);/IO_LOC "\1p\2" \3;\nIO_LOC "\1n\2" \4;/' \
    -e '/^IO_PORT "tmds_[a-z]+_p/{p;s/^(IO_PORT "tmds_[a-z]+_)p/\1n/}' \
    "$ROOT/constraint/TangCPC.cst" > "$CST"

# --- place & route ---------------------------------------------------------
echo "== nextpnr-himbaechel"
nextpnr-himbaechel --json "$OUT/$TOP.json" \
    --write "$OUT/${TOP}_pnr.json" \
    --device "$DEVICE" \
    --vopt family="$FAMILY" \
    --vopt cst="$CST" \
    --freq "$FREQ" \
    --log "$OUT/nextpnr.log"

# --- bitstream (MSPI/SSPI/JTAG pins are used as GPIO, like in build.tcl) ----
echo "== gowin_pack"
gowin_pack -d "$FAMILY" -o "$OUT/TangCPC.fs" \
    --mspi_as_gpio --sspi_as_gpio --jtag_as_gpio \
    "$OUT/${TOP}_pnr.json"
echo "bitstream: $OUT/TangCPC.fs"

case "$ACTION" in
    load)  openFPGALoader -b tangnano20k "$OUT/TangCPC.fs" ;;
    flash) openFPGALoader -b tangnano20k -f "$OUT/TangCPC.fs" ;;
esac
