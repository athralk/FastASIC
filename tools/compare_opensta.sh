#!/usr/bin/env bash
# Compares lsta against OpenSTA (from the OpenLane 2 docker image) on one gate-level netlist:
# every endpoint's worst slack, max (setup/recovery/output) and min (hold/removal/output),
# plus WNS / TNS and the runtime of both tools.
#
# usage: tools/compare_opensta.sh <netlist.v> <constraints.sdc> [corner: typ|fast|slow]
#        EXTRA_LIBS="sg13cmos5l_sram/lib/X.lib ..." adds libraries (paths under the PDK's libs.ref)
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
NETLIST=$(realpath "$1")
SDC=$(realpath "$2")
CORNER=${3:-typ}
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
REF=$PDK_DIR/ihp-sg13cmos5l/libs.ref
PDK=$REF/sg13cmos5l_stdcell/lib
case $CORNER in
    fast) LIBFILE=sg13cmos5l_stdcell_fast_1p32V_m40C.lib ;;
    slow) LIBFILE=sg13cmos5l_stdcell_slow_1p08V_125C.lib ;;
    *)    LIBFILE=sg13cmos5l_stdcell_typ_1p20V_25C.lib ;;
esac
TOP=$(grep -m1 -oE '^\s*module\s+[A-Za-z_][A-Za-z0-9_$]*' "$NETLIST" | awk '{print $2}')
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

cp "$NETLIST" "$TMP/netlist.v"
cp "$SDC" "$TMP/constraints.sdc"
cat > "$TMP/run.tcl" <<EOF
set t0 [clock milliseconds]
read_liberty /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/$LIBFILE
$(for l in ${EXTRA_LIBS:-}; do echo "read_liberty /pdk/ihp-sg13cmos5l/libs.ref/$l"; done)
set t1 [clock milliseconds]
read_verilog /w/netlist.v
link_design $TOP
read_sdc /w/constraints.sdc
set t2 [clock milliseconds]
report_checks -path_delay max -digits 4 > /w/opensta_path.txt
puts "WNS_MAX [sta::format_time [sta::worst_slack_cmd max] 6]"
puts "WNS_MIN [sta::format_time [sta::worst_slack_cmd min] 6]"
puts "TNS_MAX [sta::format_time [sta::total_negative_slack_cmd max] 6]"
set t3 [clock milliseconds]
foreach mm {max min} {
  foreach pe [find_timing_paths -path_delay \$mm -group_count 1000000 -endpoint_count 1 -unique_paths_to_endpoint] {
    puts "EP [get_full_name [[\$pe vertex] pin]] \$mm [sta::format_time [\$pe slack] 6]"
  }
}
puts "TIME liberty [expr \$t1 - \$t0] netlist_sdc [expr \$t2 - \$t1] analysis [expr \$t3 - \$t2]"
EOF

start=$(date +%s%N)
# the whole PDK is mounted: sg13cmos5l_sram is a symlink into the sg13g2 PDK
docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" sta -no_init -exit /w/run.tcl > "$TMP/opensta.out" 2>&1 || {
    cat "$TMP/opensta.out"; exit 2; }
opensta_wall=$((($(date +%s%N) - start) / 1000000))

start=$(date +%s%N)
LSTA_LIBS=(-lib "$PDK/$LIBFILE")
for l in ${EXTRA_LIBS:-}; do LSTA_LIBS+=(-lib "$REF/$l"); done
"$ROOT/lsta" -q "${LSTA_LIBS[@]}" -sdc "$SDC" -endpoints "$TMP/lsta.ep" "$NETLIST" > "$TMP/lsta.out"
lsta_wall=$((($(date +%s%N) - start) / 1000000))

# OpenSTA prints times in ns: convert to ps and compare endpoint by endpoint (worst over edges)
python3 - "$TMP" "$opensta_wall" "$lsta_wall" <<'PY'
import sys, re, collections
tmp, ow, lw = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
osta = {}
info = {}
for line in open(f"{tmp}/opensta.out"):
    f = line.split()
    if not f: continue
    if f[0] == "EP" and len(f) == 4:
        osta[(f[1], f[2])] = float(f[3]) * 1000
    elif f[0] in ("WNS_MAX", "WNS_MIN", "TNS_MAX"):
        info[f[0]] = float(f[1]) * 1000
    elif f[0] == "TIME":
        info["time"] = " ".join(f[1:])
ours = collections.defaultdict(lambda: float("inf"))
for line in open(f"{tmp}/lsta.ep"):
    if line.startswith("#"): continue
    pin, check, mm, edge, arr, req, slack = line.split()
    ours[(pin, mm)] = min(ours[(pin, mm)], float(slack))
keys = sorted(set(osta) | set(ours))
diffs, missing = [], []
for k in keys:
    if k in osta and k in ours: diffs.append((abs(osta[k] - ours[k]), k, osta[k], ours[k]))
    else: missing.append((k, osta.get(k), ours.get(k) if k in ours else None))
diffs.sort(reverse=True)
def stat(mm):
    d = [x[0] for x in diffs if x[1][1] == mm]
    return (len(d), max(d) if d else 0, sum(d) / len(d) if d else 0)
lsta_out = open(f"{tmp}/lsta.out").read()
m = re.search(r"setup\s+(\S+)\s+(\S+)", lsta_out); h = re.search(r"hold\s+(\S+)", lsta_out)
print(f"endpoints compared: {len(diffs)}  (only in one tool: {len(missing)})")
for mm in ("max", "min"):
    n, mx, avg = stat(mm)
    print(f"  {mm}: {n} endpoints, max |slack difference| {mx:.3f} ps, mean {avg:.4f} ps")
print(f"WNS setup: OpenSTA {info.get('WNS_MAX', float('nan')):.2f}  lsta {m.group(1) if m else '?'}")
print(f"WNS hold:  OpenSTA {info.get('WNS_MIN', float('nan')):.2f}  lsta {h.group(1) if h else '?'}")
print(f"TNS setup: OpenSTA {info.get('TNS_MAX', float('nan')):.2f}  lsta {m.group(2) if m else '?'}")
print(f"wall time: OpenSTA {ow} ms (docker start included; internal {info.get('time','?')} ms), lsta {lw} ms")
for d, k, o, l in diffs[:5]:
    if d > 0.01: print(f"  worst: {k[0]} {k[1]}: OpenSTA {o:.2f}  lsta {l:.2f}")
for k, o, l in missing[:5]:
    print(f"  only {'OpenSTA' if l is None else 'lsta'}: {k[0]} {k[1]} {o if o is not None else l}")
PY
