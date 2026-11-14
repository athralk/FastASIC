#!/usr/bin/env bash
# Synthesis runtime: our flow (--map area) vs Yosys + ABC (tools/synth_yosys.sh) on the same RTL.
# Wall-clock ms per design (best of 3) and the speed-up.
# usage: tools/bench_synth_speed.sh [design.v ...]   (default: tests/cases + bench/arith designs)
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
designs=("$@")
[ ${#designs[@]} -eq 0 ] && designs=("$ROOT"/tests/cases/{full_adder,adder8,alu4,compare12}.v "$ROOT"/bench/sta/add64.v "$ROOT"/bench/arith/*.v)
best() { local b=""; for _ in 1 2 3; do local s=$(date +%s%N); "$@" > /dev/null 2>&1; local e=$((($(date +%s%N)-s)/1000000)); [ -z "$b" ] || [ $e -lt $b ] && b=$e; done; echo $b; }
printf "%-14s %12s %14s %9s\n" design "mycompiler (ms)" "yosys+abc (ms)" speed-up
tot_o=0; tot_y=0
for f in "${designs[@]}"; do
    top=$(grep -m1 -oE '^\s*module\s+[A-Za-z_][A-Za-z0-9_]*' "$f" | awk '{print $2}')
    o=$(best "$ROOT/FastASIC" -q --map area -o "$TMP/o.v" "$f")
    y=$(best "$ROOT/tools/synth_yosys.sh" "$f" "$top" "$TMP/y.v")
    tot_o=$((tot_o+o)); tot_y=$((tot_y+y))
    printf "%-14s %12s %14s %8sx\n" "$(basename "$f" .v)" "$o" "$y" "$(awk "BEGIN{printf \"%.1f\", $y/($o>0?$o:1)}")"
done
printf "%-14s %12s %14s %8sx\n" TOTAL "$tot_o" "$tot_y" "$(awk "BEGIN{printf \"%.1f\", $tot_y/$tot_o}")"
