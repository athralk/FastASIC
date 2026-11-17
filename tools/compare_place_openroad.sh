#!/usr/bin/env bash
# Placement against OpenROAD on the same floorplan:
#   1. fastpnr writes the floorplan (cells unplaced) and its own placement
#   2. OpenROAD places the floorplan (global_placement + detailed_placement)
#   3. both placements are measured by the same HPWL routine inside OpenROAD, and ours must pass
#      OpenROAD's check_placement
#
# usage: tools/compare_place_openroad.sh <cells.v> [density, default 0.6]
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
NETLIST=$(realpath "$1")
DENSITY=${2:-0.6}
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
THREADS=${OPENROAD_THREADS:-$(nproc)}   # OpenROAD runs on all threads (fastpnr: one)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

"$ROOT/fastpnr" -q -no-place -o "$TMP/fp.def" "$NETLIST"
start=$(date +%s%N)
"$ROOT/fastpnr" -density "$DENSITY" -o "$TMP/ours.def" "$NETLIST" > "$TMP/ours.log"
ours_wall=$((($(date +%s%N) - start) / 1000000))
ours_place=$(grep -oE 'placement [0-9.]+' "$TMP/ours.log" | awk '{print $2}')

LEFS="read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef"
for m in $(grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$TMP/fp.def" | sort -u); do
    LEFS="$LEFS
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lef/$m.lef"
done
HPWL='proc hpwl {} {
    set total 0
    foreach net [[ord::get_db_block] getNets] {
        if {[$net getSigType] ne "SIGNAL"} continue
        set xs {}; set ys {}
        foreach it [$net getITerms] {
            set r [$it getAvgXY]
            if {[lindex $r 0]} { lappend xs [lindex $r 1]; lappend ys [lindex $r 2] }
        }
        foreach bt [$net getBTerms] {
            set b [$bt getBBox]
            lappend xs [expr {([$b xMin] + [$b xMax]) / 2.0}]; lappend ys [expr {([$b yMin] + [$b yMax]) / 2.0}]
        }
        if {[llength $xs] < 2} continue
        set xs [lsort -real $xs]; set ys [lsort -real $ys]
        set total [expr {$total + [lindex $xs end] - [lindex $xs 0] + [lindex $ys end] - [lindex $ys 0]}]
    }
    return [expr {$total / 1000.0}]
}'

cat > "$TMP/openroad.tcl" <<EOF
$LEFS
$HPWL
read_def /w/fp.def
set_thread_count $THREADS
set t0 [clock milliseconds]
global_placement -density $DENSITY -skip_io
set t1 [clock milliseconds]
detailed_placement
set t2 [clock milliseconds]
puts "RESULT openroad [hpwl] [expr {\$t1 - \$t0}] [expr {\$t2 - \$t1}]"
EOF
cat > "$TMP/ours.tcl" <<EOF
$LEFS
$HPWL
read_def /w/ours.def
if {[catch {check_placement -verbose} err]} { puts "RESULT check FAILED \$err" } else { puts "RESULT check ok" }
puts "RESULT ours [hpwl]"
EOF
docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/openroad.tcl > "$TMP/or.log" 2>&1 || { tail -20 "$TMP/or.log"; exit 2; }
docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/ours.tcl > "$TMP/check.log" 2>&1 || { tail -20 "$TMP/check.log"; exit 2; }
read -r _ _ or_hpwl or_gp or_dp < <(grep '^RESULT openroad' "$TMP/or.log")
read -r _ _ our_hpwl < <(grep '^RESULT ours' "$TMP/check.log")
check=$(grep '^RESULT check' "$TMP/check.log" | cut -d' ' -f3-)
python3 - "$or_hpwl" "$our_hpwl" "$or_gp" "$or_dp" "$ours_place" "$check" "$(basename "$NETLIST")" <<'PY'
import sys
orh, ourh, gp, dp, ours_ms, check, name = sys.argv[1:]
orh, ourh, gp, dp, ours_ms = float(orh), float(ourh), float(gp), float(dp), float(ours_ms)
print(f"{name}: HPWL fastpnr {ourh:.0f} um, OpenROAD {orh:.0f} um ({(ourh / orh - 1) * 100:+.1f}%)")
print(f"  placement time fastpnr {ours_ms:.0f} ms (1 thread), OpenROAD {gp + dp:.0f} ms (global {gp:.0f} + detailed {dp:.0f}, all threads): {(gp + dp) / max(ours_ms, 1e-3):.1f}x")
print(f"  OpenROAD check_placement on fastpnr's result: {check}")
PY
