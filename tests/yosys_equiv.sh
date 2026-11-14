#!/usr/bin/env bash
# Formal cross-check against Yosys: writes our optimized NAND/INV netlist for a design and
# asks Yosys to prove it computes the same function Yosys reads from the RTL: a SAT miter for
# combinational designs, sequential equivalence by induction (registers matched by name) when
# the design has always blocks.
# Unlike our own random simulation this is a proof, and it uses an independent Verilog frontend,
# so it also catches mistakes in how we interpret the language (widths, precedence, ...).
#
# usage: yosys_equiv.sh <output_test binary> <design.v>
set -euo pipefail

BIN=$1
SRC=$2
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

"$BIN" --quiet -o "$TMP/netlist.v" "$SRC" > /dev/null

# Hard macros (IHP SRAMs): both sides get the same stand-in model, sensitive to every macro
# input, so the proof also covers how the design is wired to the macro
VDIR=${PDK_ROOT:-$HOME/ttsetup/pdk}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog
STUBS=""
for m in $(grep -oE '\bRM_IHPSG13_[A-Za-z0-9_]+' "$SRC" | sort -u); do
    python3 "$(dirname "$0")/macro_stub.py" "$m" "$VDIR" > "$TMP/$m.stub.v"
    STUBS="$STUBS read_verilog $TMP/$m.stub.v;"
done

if grep -q "always" "$SRC"; then
    # Sequential: memories lowered to flip-flops, registers matched by name and proven by
    # induction (async resets made synchronous for the proof)
    yosys -q -p "
        $STUBS read_verilog $SRC; hierarchy -auto-top; proc; flatten; memory; opt_clean; rename -top gold; design -stash gold;
        $STUBS read_verilog $TMP/netlist.v; hierarchy -auto-top; proc; flatten; memory; opt_clean; rename -top gate; design -stash gate;
        design -copy-from gold -as gold gold; design -copy-from gate -as gate gate;
        async2sync; equiv_make gold gate equiv; hierarchy -top equiv;
        equiv_simple -seq 2; equiv_induct -seq 2; equiv_status -assert"
else
    yosys -q -p "
        read_verilog $SRC; hierarchy -auto-top; proc; flatten; rename -top gold; design -stash gold;
        read_verilog $TMP/netlist.v; proc; rename -top gate; design -stash gate;
        design -copy-from gold -as gold gold; design -copy-from gate -as gate gate;
        miter -equiv -flatten -make_assert gold gate miter; hierarchy -top miter;
        sat -verify -prove-asserts miter"
fi
