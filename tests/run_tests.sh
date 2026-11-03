#!/usr/bin/env bash
# Regression suite:
#   cases/        every design must parse, map and optimize with equivalence intact (exit 0)
#   errors/       malformed designs must be rejected (exit 2) with the message named in
#                 their "// expect-error: ..." header line
#   equiv/pass/   x.a.v and x.b.v must be reported equivalent (exit 0)
#   equiv/fail/   x.a.v and x.b.v must be reported different (exit 1)
#   sta/          timing reports must contain the hand-calculated "// expect: ..." lines
#   + the STA engine is checked against brute-force path enumeration (sta_bruteforce.cpp)
#   + if yosys is installed, every design is formally checked against Yosys (yosys_equiv.sh)
set -u

BIN=${1:-./output_test}
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
for f in "$DIR"/cases/*.v; do
    check "$(basename "$f")" 0 "" "$BIN" "$f"
done

echo "Rejected inputs:"
for f in "$DIR"/errors/*.v; do
    needle=$(sed -n 's|^// expect-error: ||p' "$f" | head -n 1)
    check "$(basename "$f")" 2 "$needle" "$BIN" "$f"
done

echo "Equivalence checker:"
for a in "$DIR"/equiv/pass/*.a.v; do
    check "equivalent  $(basename "${a%.a.v}")" 0 "PASS" "$BIN" --equiv "$a" "${a%.a.v}.b.v"
done
for a in "$DIR"/equiv/fail/*.a.v; do
    check "different   $(basename "${a%.a.v}")" 1 "FAIL" "$BIN" --equiv "$a" "${a%.a.v}.b.v"
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
    for f in "$DIR/../example.v" "$DIR"/cases/*.v; do
        check "$(basename "$f")" 0 "" bash "$DIR/yosys_equiv.sh" "$BIN" "$f"
    done
else
    echo "  skipped (yosys not found)"
fi

echo
echo "$passed passed, $failed failed"
[ "$failed" -eq 0 ]
