#!/usr/bin/env bash
# Detailed routing against OpenROAD's TritonRoute on the same placement (fastpnr's), Metal2-Metal4.
# OpenROAD uses all CPU threads (OPENROAD_THREADS, default nproc); fastpnr runs on one. Both results then go through IHP's KLayout DRC deck.
#
# usage: tools/compare_droute_openroad.sh <cells.v>
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
NETLIST=$(realpath "$1")
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
THREADS=${OPENROAD_THREADS:-$(nproc)}   # OpenROAD runs on all threads (fastpnr: one)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
"$ROOT/fastpnr" -no-route -o "$TMP/placed.def" "$NETLIST" > /dev/null
# OURS_DEF=<routed.def> OURS_LOG=<its fastpnr log>: reuse a fastpnr routing done before (same netlist)
if [ -n "${OURS_DEF:-}" ]; then cp "$OURS_DEF" "$TMP/ours.def"; cp "$OURS_LOG" "$TMP/ours.log"
else "$ROOT/fastpnr" -droute -o "$TMP/ours.def" "$NETLIST" > "$TMP/ours.log" || true; fi
LEFS="read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef"
for m in $( (grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$TMP/placed.def" || true) | sort -u); do
    LEFS="$LEFS
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lef/$m.lef"
done
cat > "$TMP/run.tcl" <<EOF
$LEFS
read_def /w/placed.def
set_thread_count $THREADS
set_routing_layers -signal Metal2-Metal4
set_global_routing_layer_adjustment Metal2-Metal4 0.3
global_route
set t0 [clock milliseconds]
detailed_route -output_drc /w/drt.rpt
puts "DRTIME [expr {[clock milliseconds] - \$t0}]"
write_def /w/openroad.def
EOF
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/run.tcl > "$TMP/or.log" 2>&1 || { tail -20 "$TMP/or.log"; exit 2; }
or_ms=$(grep -oE 'DRTIME [0-9]+' "$TMP/or.log" | awk '{print $2}')
or_wl=$(grep -oE 'Total wire length = [0-9.]+' "$TMP/or.log" | tail -1 | awk '{print $5}')
or_vias=$(grep -oE 'Total number of vias = [0-9]+' "$TMP/or.log" | tail -1 | awk '{print $6}')
or_drc=$(grep -oE 'Number of violations = [0-9]+' "$TMP/or.log" | tail -1 | awk '{print $5}')
our_ms=$(grep -oE 'time [0-9.]+ ms \(setup' "$TMP/ours.log" | awk '{print $2}')
our_wl=$(grep -oE '[0-9]+ um of wire' "$TMP/ours.log" | awk '{print $1}')
our_vias=$(grep -oE '[0-9]+ vias, [0-9]+ negotiation' "$TMP/ours.log" | awk '{print $1}')
our_state=$(grep -oE '[0-9]+ conflicts, [0-9]+ unconnected nets' "$TMP/ours.log")
our_drc=$(bash "$ROOT/tools/drc_klayout.sh" "$TMP/ours.def" | head -1 | grep -oE '[0-9]+$')
or_kdrc=$(bash "$ROOT/tools/drc_klayout.sh" "$TMP/openroad.def" | head -1 | grep -oE '[0-9]+$')
# KEEP=<dir>: keep both routed layouts (ours.def, openroad.def) and the placement they share
[ -n "${KEEP:-}" ] && mkdir -p "$KEEP" && cp "$TMP/placed.def" "$TMP/ours.def" "$TMP/openroad.def" "$KEEP"/
echo "$(basename "$NETLIST"): detailed routing, fastpnr 1 thread, TritonRoute $THREADS threads"
echo "  time        fastpnr ${our_ms} ms, TritonRoute ${or_ms} ms"
echo "  wire        fastpnr ${our_wl} um, TritonRoute ${or_wl} um"
echo "  vias        fastpnr ${our_vias}, TritonRoute ${or_vias}"
echo "  IHP DRC     fastpnr ${our_drc} violations, TritonRoute ${or_kdrc} violations (TritonRoute's own count: ${or_drc})"
echo "  fastpnr     ${our_state}"
