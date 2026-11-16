#!/usr/bin/env bash
# Global routing against OpenROAD's FastRoute on the same placement (fastpnr's):
# same layers (Metal2-Metal4), same 30% capacity adjustment. Prints overflow, routed demand
# (gcell edges, the wire both routers place) and routing time, measured inside each program.
#
# usage: tools/compare_groute_openroad.sh <cells.v> [adjustment, default 0.3]
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
NETLIST=$(realpath "$1")
ADJ=${2:-0.3}
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

"$ROOT/fastpnr" -adjust "$ADJ" -o "$TMP/placed.def" "$NETLIST" > "$TMP/ours.log"

LEFS="read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef"
for m in $(grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$TMP/placed.def" | sort -u); do
    LEFS="$LEFS
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lef/$m.lef"
done
cat > "$TMP/run.tcl" <<EOF
$LEFS
read_def /w/placed.def
set_routing_layers -signal Metal2-Metal4
set_global_routing_layer_adjustment Metal2-Metal4 $ADJ
set t0 [clock milliseconds]
if {[catch {global_route -allow_congestion -verbose} err]} { puts "GRFAIL \$err" }
puts "GRTIME [expr {[clock milliseconds] - \$t0}]"
EOF
docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/run.tcl > "$TMP/or.log" 2>&1 || { tail -20 "$TMP/or.log"; exit 2; }
python3 - "$TMP/ours.log" "$TMP/or.log" "$(basename "$NETLIST")" <<'PY'
import re, sys
ours, orl, name = open(sys.argv[1]).read(), open(sys.argv[2]).read(), sys.argv[3]
# ours
our_dem = int(re.search(r'Total\s+(\d+)\s+(\d+)', ours).group(2))
our_ovf = int(re.search(r'Total\s+\d+\s+\d+\s+[\d.]+%\s+(\d+)', ours).group(1))
our_ms = float(re.search(r'routing ([\d.]+) \[', ours).group(1))
our_rounds = int(re.search(r'(\d+) negotiation rounds', ours).group(1))
# FastRoute final congestion report
rep = orl[orl.index('Final congestion report'):]
tot = re.search(r'Total\s+(\d+)\s+(\d+)\s+[\d.]+%\s+(\d+)\s*/\s*(\d+)\s*/\s*(\d+)', rep)
or_dem, or_ovf = int(tot.group(2)), int(tot.group(5))
or_ms = float(re.search(r'GRTIME (\d+)', orl).group(1))
print(f"{name}: overflow fastpnr {our_ovf}, FastRoute {or_ovf}; routed demand (gcell edges) fastpnr {our_dem}, FastRoute {or_dem} ({(our_dem / or_dem - 1) * 100:+.1f}%)")
print(f"  routing time fastpnr {our_ms:.0f} ms ({our_rounds} negotiation rounds), FastRoute {or_ms:.0f} ms: {or_ms / max(our_ms, 1e-3):.1f}x")
PY
