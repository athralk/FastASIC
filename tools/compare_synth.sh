#!/usr/bin/env bash
# Synthesis quality and speed: our flow (parse -> NAND/INV -> optimize -> tree covering onto
# sg13cmos5l) versus Yosys + ABC (tools/synth_yosys.sh) on the same RTL. Both netlists are
# timed by lsta with the same constraints (bench/sta/virtual.sdc), and ours is SAT-checked
# against the RTL by the pipeline itself.
# usage: tools/compare_synth.sh [design.v ...]   (default: a set of tests/cases designs)
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
SDC=$ROOT/bench/sta/virtual.sdc
designs=("$@")
[ ${#designs[@]} -eq 0 ] && designs=("$ROOT"/tests/cases/{full_adder,mux4,adder8,alu4,compare12,parity22,all_operators,ripple_adder4}.v "$ROOT/bench/sta/add64.v")

stats() {   # stats <netlist.v> -> "cells area min_period"
    "$ROOT/lsta" -q -sdc "$SDC" "$1" | awk '/^Design:/ {c=$2; a=$9} /^Minimum period/ {p=$3} END {printf "%s %s %s", c, a, p}'
}
printf "%-18s | %-28s | %-28s | %-28s\n" "" "ours --map area" "ours --map delay" "yosys + abc"
printf "%-18s | %5s %9s %7s %4s | %5s %9s %7s %4s | %5s %9s %7s %4s\n" design cells "area um2" "delay" ms cells "area um2" "delay" ms cells "area um2" "delay" ms
for f in "${designs[@]}"; do
    top=$(grep -m1 -oE '^\s*module\s+[A-Za-z_][A-Za-z0-9_]*' "$f" | awk '{print $2}')
    row=$(printf "%-18s" "$(basename "$f" .v)")
    for goal in area delay; do
        s=$(date +%s%N)
        "$ROOT/FastASIC" -q --map $goal -o "$TMP/$goal.v" "$f" > "$TMP/$goal.log" 2>&1 || { row+=" | failed"; continue; }
        ms=$((($(date +%s%N) - s) / 1000000))
        grep -q "original vs cells: PASS" "$TMP/$goal.log" || row+=" (NOT VERIFIED)"
        read -r c a p <<< "$(stats "$TMP/$goal.v")"
        row+=$(printf " | %5s %9s %7s %4s" "$c" "$a" "$p" "$ms")
    done
    s=$(date +%s%N)
    "$ROOT/tools/synth_yosys.sh" "$f" "$top" "$TMP/yosys.v" > /dev/null 2>&1
    ms=$((($(date +%s%N) - s) / 1000000))
    read -r c a p <<< "$(stats "$TMP/yosys.v")"
    row+=$(printf " | %5s %9s %7s %4s" "$c" "$a" "$p" "$ms")
    echo "$row"
done
echo "delay = minimum clock period (ps) for the virtual-clock SDC: the critical path plus 1 ns of I/O delay"
