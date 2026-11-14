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
#     tree-covering onto real cells (SAT-checked against the RTL), netlist write/read round trip
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
