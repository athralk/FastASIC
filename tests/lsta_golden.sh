#!/usr/bin/env bash
# Checks lsta's WNS / TNS against the OpenSTA golden values in tests/lsta/opensta_golden.txt
# usage: tests/lsta_golden.sh <lsta binary>     prints one "ok"/"FAIL" line per case
set -u
LSTA=$1
ROOT=$(cd "$(dirname "$0")/.." && pwd)
status=0
while IFS= read -r line; do
    [[ "$line" =~ ^# || -z "$line" ]] && continue
    read -r netlist sdc corner extra _ ws wh ts <<< "$line"
    args=(-q -corner "$corner" -sdc "$ROOT/bench/sta/$sdc")
    if [ "$extra" != "-" ]; then IFS=: read -r _ buf fan <<< "$extra"; args+=(-cts "$buf" -cts_fanout "$fan"); fi
    out=$("$LSTA" "${args[@]}" "$ROOT/$netlist" 2>&1)
    read -r gs gt <<< "$(awk '/^  setup/ {print $2, $3}' <<< "$out")"
    gh=$(awk '/^  hold/ {print $2}' <<< "$out")
    ok=$(python3 -c "import sys; a=[float(x) for x in sys.argv[1:]]; print(abs(a[0]-a[3])<=0.02 and abs(a[1]-a[4])<=0.02 and abs(a[2]-a[5])<=max(0.02, 1e-6*abs(a[5])))" "$gs" "$gh" "$gt" "$ws" "$wh" "$ts" 2>/dev/null)
    name="$(basename "$netlist" .v) $sdc $corner${extra:+ $extra}"
    name=${name% -}
    if [ "$ok" = True ]; then
        echo "  ok    $name (WNS $gs / hold $gh / TNS $gt ps)"
    else
        echo "  FAIL  $name: lsta $gs $gh $gt, OpenSTA $ws $wh $ts"
        status=1
    fi
done < "$ROOT/tests/lsta/opensta_golden.txt"
exit $status
