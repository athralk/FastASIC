#!/usr/bin/env bash
# Formal cross-check against Yosys: writes our optimized NAND/INV netlist for a design and
# asks Yosys's SAT solver to prove it computes the same function Yosys reads from the RTL.
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

yosys -q -p "
    read_verilog $SRC; proc; rename -top gold; design -stash gold;
    read_verilog $TMP/netlist.v; proc; rename -top gate; design -stash gate;
    design -copy-from gold -as gold gold; design -copy-from gate -as gate gate;
    miter -equiv -flatten -make_assert gold gate miter; hierarchy -top miter;
    sat -verify -prove-asserts miter"
