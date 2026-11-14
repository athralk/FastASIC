#!/usr/bin/env bash
# Benchmarks our SAT equivalence checker against Yosys and ABC on the same design pairs:
#   ours   FastASIC --equiv a.v b.v         (our frontend + AIG + incremental CDCL)
#   yosys  miter -equiv + sat -prove-asserts    (Yosys frontend, word-level cells bit-blasted
#                                                into MiniSat by ezSAT)
#   abc    ABC's &cec (simulation + SAT sweeping), on files written by Yosys, in two modes:
#            miter -m a.blif b.blif; strash; &get; &cec -m   (multi-output miter)
#            &cec a.aig b.aig                                 (two AIGER files)
#          ABC's speed varies by up to 200x between them, so the faster one is reported
#          (marked m or a).
#
# Two rounds:
#   1. RTL: every tool reads the RTL with its own frontend, so each one checks a different
#      circuit (Yosys/ABC build their own adders). This is the end-to-end user experience.
#   2. Same netlist: both sides are first turned into our NAND/INV netlist (-o) and all three
#      tools check those identical gate-level files. This compares the solvers themselves.
# Pairs: bench/equiv/*.{a,b}.v (tools/gen_equiv_bench.py); round 1 also checks a few designs
# against the netlist we generate for them.
#
# "engine" is the checker's own reported time; "wall" is the whole process (parsing, startup).
# Yosys's engine time is estimated as wall(with sat) - wall(setup only).
# Each command is run up to 3 times and the fastest kept.
#
# usage: tools/bench_equiv.sh [timeout seconds per run, default 60]
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BIN=$ROOT/FastASIC
LIMIT=${1:-60}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

for tool in yosys yosys-abc; do
    command -v "$tool" > /dev/null || { echo "$tool not found"; exit 2; }
done
[ -x "$BIN" ] || { echo "build first: make"; exit 2; }

# measure <command...>: sets OUT, STATUS and MS (fastest wall time in ms)
measure() {
    local best="" start elapsed
    for _ in 1 2 3; do
        start=$(date +%s%N)
        OUT=$(timeout "$LIMIT" "$@" 2>&1)
        STATUS=$?
        elapsed=$((($(date +%s%N) - start) / 1000000))
        if [ -z "$best" ] || [ "$elapsed" -lt "$best" ]; then best=$elapsed; fi
        [ "$STATUS" -eq 124 ] && break
        [ "$elapsed" -gt 2000 ] && break
    done
    MS=$best
}

verdict() {   # verdict <status> <equivalent?> <different?>
    if [ "$1" -eq 124 ]; then echo "timeout"
    elif [ "$2" = yes ]; then echo "equiv"
    elif [ "$3" = yes ]; then echo "DIFF"
    else echo "error"
    fi
}

yosys_setup() {
    echo "read_verilog $1; proc; rename -top gold; design -stash gold;
          read_verilog $2; proc; rename -top gate; design -stash gate;
          design -copy-from gold -as gold gold; design -copy-from gate -as gate gate;
          miter -equiv -flatten -make_assert gold gate miter; hierarchy -top miter"
}

row() {   # row <name> <a.v> <b.v>
    local name=$1 a=$2 b=$3

    measure "$BIN" --equiv "$a" "$b"
    local ours_v ours_engine
    ours_v=$(verdict "$STATUS" "$([ "$STATUS" -eq 0 ] && echo yes)" "$([ "$STATUS" -eq 1 ] && echo yes)")
    ours_engine=$(grep -oE '[0-9.]+ ms\)' <<< "$OUT" | head -n 1 | cut -d' ' -f1)
    local ours_wall=$MS

    measure yosys -q -p "$(yosys_setup "$a" "$b")"
    local setup=$MS
    measure yosys -q -p "$(yosys_setup "$a" "$b"); sat -verify -prove-asserts miter"
    local ys_v ys_engine
    ys_v=$(verdict "$STATUS" "$([ "$STATUS" -eq 0 ] && echo yes)" \
                   "$(grep -q 'proof did fail' <<< "$OUT" && echo yes)")
    ys_engine=$((MS > setup ? MS - setup : 0))
    local ys_wall=$MS

    local abc_v="error" abc_engine="" abc_wall="" mode
    local blif_a="read_verilog $a; proc; flatten; techmap; opt_clean; write_blif $TMP/a.blif; aigmap; write_aiger $TMP/a.aig"
    local blif_b="read_verilog $b; proc; flatten; techmap; opt_clean; write_blif $TMP/b.blif; aigmap; write_aiger $TMP/b.aig"
    if yosys -q -p "$blif_a" > /dev/null 2>&1 && yosys -q -p "$blif_b" > /dev/null 2>&1; then
        for mode in m a; do
            if [ $mode = m ]; then
                measure yosys-abc -c "miter -m $TMP/a.blif $TMP/b.blif; strash; &get; &cec -m"
            else
                measure yosys-abc -c "&cec $TMP/a.aig $TMP/b.aig"
            fi
            local v e
            v=$(verdict "$STATUS" "$(grep -q 'Networks are equivalent' <<< "$OUT" && echo yes)" \
                        "$(grep -q 'NOT EQUIVALENT' <<< "$OUT" && echo yes)")
            e=$(grep -oE 'Time = +[0-9.]+' <<< "$OUT" | tail -n 1 | grep -oE '[0-9.]+' | awk '{printf "%.0f", $1 * 1000}')
            [ "$v" = timeout ] && e=$((LIMIT * 1000))
            if [ -z "$abc_engine" ] || { [ -n "$e" ] && [ "$e" -lt "$abc_engine" ]; }; then
                abc_v=$v; abc_engine=${e:-0}; abc_wall=$MS; abc_mode=$mode
            fi
        done
        abc_engine="$abc_engine $abc_mode"
    fi

    printf "%-22s | %-7s %9s %7s | %-7s %9s %7s | %-7s %9s %7s\n" "$name" \
        "$ours_v" "${ours_engine:--}" "$ours_wall" "$ys_v" "$ys_engine" "$ys_wall" \
        "$abc_v" "${abc_engine:--}" "${abc_wall:--}"
}

header() {
    echo
    echo "$1"
    printf "%-22s | %-25s | %-25s | %-25s\n" "" "ours (CDCL)" "yosys sat (MiniSat)" "abc &cec"
    printf "%-22s | %-7s %9s %7s | %-7s %9s %7s | %-7s %9s %7s\n" "pair" \
        verdict "engine ms" "wall" verdict "engine ms" "wall" verdict "engine ms" "wall"
    printf -- "-%.0s" {1..106}; echo
}

header "Round 1: RTL (each tool uses its own frontend)"
for a in "$ROOT"/bench/equiv/*.a.v; do
    row "$(basename "${a%.a.v}")" "$a" "${a%.a.v}.b.v"
done
for f in adder8 alu4 compare12 parity22; do
    "$BIN" -q -o "$TMP/$f.net.v" "$ROOT/tests/cases/$f.v" > /dev/null
    row "$f vs netlist" "$ROOT/tests/cases/$f.v" "$TMP/$f.net.v"
done
"$BIN" -q -o "$TMP/example.net.v" "$ROOT/example.v" > /dev/null
row "example vs netlist" "$ROOT/example.v" "$TMP/example.net.v"

header "Round 2: same NAND/INV netlists for every tool"
for a in "$ROOT"/bench/equiv/*.a.v; do
    name=$(basename "${a%.a.v}")
    "$BIN" -q -o "$TMP/$name.a.net.v" "$a" > /dev/null
    "$BIN" -q -o "$TMP/$name.b.net.v" "${a%.a.v}.b.v" > /dev/null
    row "$name" "$TMP/$name.a.net.v" "$TMP/$name.b.net.v"
done
