#!/usr/bin/env bash
# Regression suite:
#   cases/, seq/, hier/, lang/  every design must parse, map and optimize with equivalence intact (exit 0);
#                 lang/ covers the preprocessor, signed arithmetic, shifters, multipliers, casez/casex,
#                 functions / tasks and generate;
#                 seq/ holds sequential designs (registers, memories, always blocks), hier/
#                 multi-module designs (instances, parameter overrides), flattened
#   errors/       malformed designs must be rejected (exit 2) with the message named in
#                 their "// expect-error: ..." header line
#   equiv/pass/   x.a.v and x.b.v must be reported equivalent (exit 0)
#   equiv/fail/   x.a.v and x.b.v must be reported different (exit 1)
#                 both with the SAT prover and with simulation (except "// sat-only" pairs,
#                 whose difference is too rare for random simulation to find)
#   sta/          timing reports must contain the hand-calculated "// expect: ..." lines
#   + the CDCL solver is fuzzed against brute force (sat_fuzz.cpp)
#   + the STA engine is checked against brute-force path enumeration (sta_bruteforce.cpp)
#   + if yosys is installed, every design is formally checked against Yosys (yosys_equiv.sh)
#   + if the IHP sg13cmos5l PDK is installed: hard-macro (SRAM) designs in macro/, SRAM inference
#     in infer/ (co-simulated against the RTL with the PDK's SRAM model, cosim_sram.sh), lsta against
#     OpenSTA golden values (lsta_golden.sh),
#     tree-covering onto real cells (SAT-checked against the RTL), netlist write/read round trip,
#     physical design (fastpnr floorplan, placement and global routing, self-checked;
#     FASTASIC_OPENROAD=1 also hands the floorplans to OpenROAD in docker)
set -u

BIN=${1:-./FastASIC}
DIR=$(cd "$(dirname "$0")" && pwd)
passed=0
failed=0

# check <description> <expected exit code> <expected output substring or ""> <command...>
check() {
    local desc=$1 expected=$2 needle=$3
    shift 3
    local output status
    output=$("$@" 2>&1)
    status=$?
    if [ "$status" -eq "$expected" ] && { [ -z "$needle" ] || grep -qF -- "$needle" <<< "$output"; }; then
        passed=$((passed + 1))
        echo "  ok    $desc"
    else
        failed=$((failed + 1))
        echo "  FAIL  $desc (exit $status, expected $expected${needle:+, output should contain \"$needle\"})"
        tail -n 5 <<< "$output" | sed 's/^/        /'
    fi
}

echo "Pipeline (parse -> map -> optimize, checked for equivalence):"
check "example.v" 0 "" "$BIN" "$DIR/../example.v"
for f in "$DIR"/cases/*.v "$DIR"/seq/*.v "$DIR"/hier/*.v "$DIR"/lang/*.v; do
    check "$(basename "$f")" 0 "" "$BIN" "$f"
done

echo "Rejected inputs:"
for f in "$DIR"/errors/*.v; do
    needle=$(sed -n 's|^// expect-error: ||p' "$f" | head -n 1)
    check "$(basename "$f")" 2 "$needle" "$BIN" "$f"
done

echo "SAT solver vs brute force (random CNFs, incremental solving with assumptions, pigeonhole):"
if [ -x "$DIR/sat_fuzz" ]; then
    check "sat_fuzz" 0 "solver checks consistent" "$DIR/sat_fuzz"
else
    echo "  skipped (build it with: make tests/sat_fuzz)"
fi

echo "Equivalence checker (SAT proof, the default):"
for a in "$DIR"/equiv/pass/*.a.v; do
    check "equivalent  $(basename "${a%.a.v}")" 0 "PASS (SAT proof" "$BIN" --equiv "$a" "${a%.a.v}.b.v"
done
for a in "$DIR"/equiv/fail/*.a.v; do
    check "different   $(basename "${a%.a.v}")" 1 "FAIL" "$BIN" --equiv "$a" "${a%.a.v}.b.v"
done

echo "Equivalence checker (simulation):"
for a in "$DIR"/equiv/pass/*.a.v; do
    check "equivalent  $(basename "${a%.a.v}")" 0 "PASS" "$BIN" --equiv-method sim --equiv "$a" "${a%.a.v}.b.v"
done
for a in "$DIR"/equiv/fail/*.a.v; do
    grep -q "sat-only" "$a" && continue
    check "different   $(basename "${a%.a.v}")" 1 "FAIL" "$BIN" --equiv-method sim --equiv "$a" "${a%.a.v}.b.v"
done

echo "Static timing (hand-calculated values from each file's '// expect:' lines):"
for f in "$DIR"/sta/*.v; do
    args=$(sed -n 's|^// sta-args: ||p' "$f" | head -n 1)
    # shellcheck disable=SC2086  # args is a list of options
    output=$("$BIN" -q --sta $args "$f" 2>&1)
    status=$?
    squeezed=$(tr -s ' ' <<< "$output")   # compare numbers, not column alignment
    missing=""
    while IFS= read -r needle; do
        grep -qF -- "$needle" <<< "$squeezed" || missing+="\"$needle\" "
    done < <(sed -n 's|^// expect: ||p' "$f")
    if [ "$status" -eq 0 ] && [ -z "$missing" ]; then
        passed=$((passed + 1))
        echo "  ok    $(basename "$f")"
    else
        failed=$((failed + 1))
        echo "  FAIL  $(basename "$f") (exit $status${missing:+, missing $missing})"
        sed -n '/Static Timing/,$p' <<< "$output" | head -n 20 | sed 's/^/        /'
    fi
done

echo "Static timing vs brute-force path enumeration (arrival and required time at every node):"
if [ -x "$DIR/sta_bruteforce" ]; then
    for f in c17 full_adder mux2 mux4 all_operators alu4 shared_outputs constant_folding use_before_define reductions_logic; do
        check "$f.v" 0 "consistent" "$DIR/sta_bruteforce" "$DIR/cases/$f.v"
    done
else
    echo "  skipped (build it with: make tests/sta_bruteforce)"
fi

echo "Formal cross-check against Yosys (SAT proof that our netlist matches Yosys's reading of the RTL):"
if command -v yosys > /dev/null; then
    for f in "$DIR/../example.v" "$DIR"/cases/*.v "$DIR"/seq/*.v "$DIR"/hier/*.v "$DIR"/lang/*.v; do
        if grep -q "yosys-equiv: skip" "$f"; then echo "  skip  $(basename "$f") (checked by co-simulation)"; continue; fi
        check "$(basename "$f")" 0 "" bash "$DIR/yosys_equiv.sh" "$BIN" "$f"
    done
else
    echo "  skipped (yosys not found)"
fi

PDK_LIB=${PDK_ROOT:-$HOME/ttsetup/pdk}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_typ_1p20V_25C.lib
LSTA=$(dirname "$BIN")/lsta
echo "Gate-level STA (lsta) against OpenSTA golden values:"
if [ -f "$PDK_LIB" ] && [ -x "$LSTA" ]; then
    while IFS= read -r line; do
        case $line in
            *"  ok "*) passed=$((passed + 1)); echo "$line" ;;
            *FAIL*) failed=$((failed + 1)); echo "$line" ;;
        esac
    done < <(bash "$DIR/lsta_golden.sh" "$LSTA")
    tmpnet=$(mktemp)
    "$LSTA" -q -sdc "$DIR/../bench/sta/propagated.sdc" -cts sg13cmos5l_buf_8 -cts_fanout 6 -write "$tmpnet" "$DIR/../bench/sta/netlists/mul16.v" > /dev/null
    check "netlist write + re-read keeps every slack" 0 "setup         -547.25" "$LSTA" -q -sdc "$DIR/../bench/sta/propagated.sdc" "$tmpnet"
    rm -f "$tmpnet"
else
    echo "  skipped (IHP sg13cmos5l PDK not found at \$PDK_ROOT or ~/ttsetup/pdk)"
fi

echo "Hard macros (IHP SRAMs, found in the PDK by name):"
if [ -f "$PDK_LIB" ]; then
    for f in "$DIR"/macro/*.v; do
        check "$(basename "$f") pipeline" 0 "original vs optimized: PASS" "$BIN" -q "$f"
        check "$(basename "$f") --map (SAT-checked)" 0 "original vs cells: PASS" "$BIN" -q --map area "$f"
        if command -v yosys > /dev/null; then
            check "$(basename "$f") Yosys proof (macro stand-in model)" 0 "" bash "$DIR/yosys_equiv.sh" "$BIN" "$f"
        fi
    done
    for f in "$DIR"/macro/errors/*.v; do
        needle=$(sed -n 's|^// expect-error: ||p' "$f" | head -n 1)
        check "$(basename "$f")" 2 "$needle" "$BIN" "$f"
    done
else
    echo "  skipped (IHP PDK not found)"
fi

echo "SRAM inference (reg memories -> PDK SRAM macros, checked by co-simulation with IHP's model):"
if [ -f "$PDK_LIB" ]; then
    for f in "$DIR"/infer/*.v; do
        note=$(sed -n 's|^// expect-note: ||p' "$f" | head -n 1)
        check "$(basename "$f") decision" 0 "$note" "$BIN" -q "$f"
        check "$(basename "$f") --map (SAT-checked)" 0 "original vs cells: PASS" "$BIN" -q --map area "$f"
        if command -v iverilog > /dev/null && command -v yosys > /dev/null && grep -q -- "-> RM_" <<< "$note"; then
            check "$(basename "$f") co-simulation vs RTL" 0 "PASS 4000 cycles" bash "$DIR/cosim_sram.sh" "$BIN" "$f"
        fi
    done
    check "ram_1p.v --no-sram-infer keeps flip-flops" 0 "Registers: 8200 flip-flops" "$BIN" -q --no-sram-infer "$DIR/infer/ram_1p.v"
    if command -v iverilog > /dev/null && command -v yosys > /dev/null; then
        export COSIM_MUTATE='s/\.A_REN\([^)]*\)/.A_REN(1'"'"'b1)/'
        check "co-simulation catches a planted bug (read enable forced high)" 1 "FAIL" bash "$DIR/cosim_sram.sh" "$BIN" "$DIR/infer/ram_1p.v"
        unset COSIM_MUTATE
    else
        echo "  co-simulation skipped (needs iverilog and yosys)"
    fi
else
    echo "  skipped (IHP PDK not found)"
fi

echo "Technology mapping onto sg13cmos5l cells (every result SAT-checked against the RTL):"
if [ -f "$PDK_LIB" ]; then
    for f in "$DIR/../example.v" "$DIR"/cases/*.v "$DIR"/seq/*.v "$DIR"/hier/*.v "$DIR"/lang/*.v; do
        for goal in area delay; do
            check "$(basename "$f") --map $goal" 0 "original vs cells: PASS" "$BIN" -q --map $goal "$f"
        done
    done
    check "adder8.v --sta --delay-model nldm" 0 "Summary (ps)" "$BIN" -q --sta --delay-model nldm "$DIR/cases/adder8.v"
    check "counter.v --sta --delay-model nldm (register paths)" 0 "rising edge-triggered flip-flop" "$BIN" -q --sta --delay-model nldm "$DIR/seq/counter.v"
else
    echo "  skipped (IHP sg13cmos5l PDK not found)"
fi

PNR=$(dirname "$BIN")/fastpnr
echo "Physical design (fastpnr: floorplan, placement, global routing; every result self-checked;"
echo "OpenROAD also checks the floorplans with FASTASIC_OPENROAD=1, IHP's DRC and LVS decks the routes with FASTASIC_SIGNOFF=1):"
if [ -f "$PDK_LIB" ] && [ -x "$PNR" ]; then
    FP_TMP=$(mktemp -d)
    # a 52k-cell design: 8 copies of mac32 (generated, too large to keep in the repo)
    python3 "$DIR/../tools/replicate_netlist.py" "$DIR/../bench/sta/netlists/mac32.v" 8 > "$FP_TMP/mac32_x8.v"
    for f in "$DIR"/../bench/sta/netlists/*.v "$FP_TMP/mac32_x8.v"; do
        check "$(basename "$f"): placed legally, routed without overflow" 0 "2D overflow 0" "$PNR" -o "$FP_TMP/$(basename "$f" .v).def" "$f"
    done
    for f in "$DIR"/../bench/sta/netlists/mul16.v "$DIR"/../bench/sta/netlists/sram_buffer.v "$DIR"/../bench/sta/netlists/fifo_2p.v; do
        check "$(basename "$f"): floorplan only" 0 "Power grid" "$PNR" -no-place -o "$FP_TMP/$(basename "$f" .v).fp.def" "$f"
    done
    for f in "$DIR"/cases/alu4.v "$DIR"/seq/counter.v "$DIR"/lang/multipliers.v; do
        "$BIN" -q --map area -o "$FP_TMP/$(basename "$f" .v).cells.v" "$f" > /dev/null 2>&1
        check "$(basename "$f"): synthesized, then placed and routed" 0 "2D overflow 0" "$PNR" -o "$FP_TMP/$(basename "$f" .v).def" "$FP_TMP/$(basename "$f" .v).cells.v"
    done
    for f in "$DIR"/lang/shifters.v "$DIR"/../bench/arith/mul8.v; do
        "$BIN" -q --map area -o "$FP_TMP/$(basename "$f" .v).dr.cells.v" "$f" > /dev/null 2>&1
        check "$(basename "$f"): detailed routing, no conflicts or opens" 0 "0 conflicts, 0 unconnected nets, 0 pins without access" "$PNR" -droute -o "$FP_TMP/$(basename "$f" .v).routed.def" "$FP_TMP/$(basename "$f" .v).dr.cells.v"
    done
    for d in acc16 add64 mul16 sram_buffer fifo_2p; do
        check "$d.v: detailed routing, every pin reached (SRAM escapes too), no conflicts or opens" 0 "0 conflicts, 0 unconnected nets, 0 pins without access" \
            "$PNR" -droute -o "$FP_TMP/$d.routed.def" "$DIR/../bench/sta/netlists/$d.v"
    done
    if [ "${FASTASIC_SIGNOFF:-0}" = 1 ]; then
        for d in acc16 sram_buffer fifo_2p; do
            check "$d: IHP KLayout DRC deck, 0 violations" 0 "DRC violations (excluding metal density): 0" bash "$DIR/../tools/drc_klayout.sh" "$FP_TMP/$d.routed.def"
            check "$d: IHP KLayout LVS deck matches (SRAM pins probed)" 0 "netlists match" bash "$DIR/../tools/lvs_klayout.sh" "$FP_TMP/$d.routed.def"
        done
        # planted bug: one Via1 removed must be caught
        python3 - "$FP_TMP/acc16.routed.def" "$FP_TMP/acc16.broken.def" <<'PY'
import sys
t = open(sys.argv[1]).read(); i = t.index('\nNETS '); j = t.index('Via1_', i)
open(sys.argv[2], 'w').write(t[:t.rindex('\n', 0, j)] + t[t.index('\n', j):])
PY
        check "LVS catches a planted open (one via removed)" 1 "MISMATCH" bash "$DIR/../tools/lvs_klayout.sh" "$FP_TMP/acc16.broken.def"
    fi
    check "route guides written for every routed net" 0 "Metal3" bash -c "'$PNR' -q -guide '$FP_TMP/mul16.guide' '$DIR/../bench/sta/netlists/mul16.v' && grep -c '^(' '$FP_TMP/mul16.guide' > /dev/null && head -5 '$FP_TMP/mul16.guide'"
    check "tie cells replace constant inputs (fifo_2p)" 0 "86 tie cells added" "$PNR" "$DIR/../bench/sta/netlists/fifo_2p.v"
    check "fixed die too small for the cells" 1 "do not fit in the rows" "$PNR" -die 60 60 "$DIR/../bench/sta/netlists/mul16.v"
    check "fixed die too small for the pins" 1 "not enough room" "$PNR" -die 24 24 "$DIR/../bench/sta/netlists/mul16.v"
    check "fixed die, utilization reported" 0 "Die         250.00 x 250.00 um" "$PNR" -die 250 250 "$DIR/../bench/sta/netlists/mul16.v"
    check "cell without a LEF macro" 1 "unknown cell" "$PNR" "$DIR/pnr/unknown_cell.v"
    if [ "${FASTASIC_OPENROAD:-0}" = 1 ]; then
        for d in mul16 sram_buffer fifo_2p; do
            check "OpenROAD accepts $d.fp.def (placement, power grid, global route)" 0 "global_route ok" bash "$DIR/../tools/check_def_openroad.sh" "$FP_TMP/$d.fp.def"
        done
    fi
    rm -rf "$FP_TMP"
else
    echo "  skipped (IHP sg13cmos5l PDK not found)"
fi

echo "Co-simulation against iverilog (RTL operators vs our netlist; exhaustive up to 16 input bits):"
if command -v iverilog > /dev/null && command -v yosys > /dev/null; then
    for f in "$DIR"/lang/*.v "$DIR"/cases/adder8.v "$DIR"/cases/compare12.v "$DIR"/cases/alu4.v; do
        check "$(basename "$f")" 0 "PASS" bash "$DIR/cosim.sh" "$BIN" "$f" 20000
    done
    for style in brent-kung sklansky kogge-stone; do
        for f in "$DIR"/lang/multipliers.v "$DIR"/lang/signed_math.v "$DIR"/cases/compare12.v; do
            check "$(basename "$f") --adder $style" 0 "PASS" bash "$DIR/cosim.sh" "$BIN" "$f" 20000 --adder "$style"
        done
    done
    export COSIM_MUTATE='0,/~\(/s//(/'
    check "co-simulation catches a planted bug (an inversion removed)" 1 "FAIL" bash "$DIR/cosim.sh" "$BIN" "$DIR/lang/multipliers.v" 20000
    unset COSIM_MUTATE
else
    echo "  skipped (needs iverilog and yosys)"
fi

echo
echo "$passed passed, $failed failed"
[ "$failed" -eq 0 ]
