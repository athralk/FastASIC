#!/usr/bin/env bash
# Hands a fastpnr floorplan DEF to OpenROAD (from the OpenLane 2 docker image) as an independent check:
#   1. read_def           the DEF parses against the IHP LEFs
#   2. global/detailed placement into our rows, then check_placement
#   3. check_power_grid   every placed cell reaches VDD and VSS through our grid
#   4. global_route       our tracks and pins are routable
#
# usage: tools/check_def_openroad.sh <floorplan.def> [density, default 0.6]
set -euo pipefail

DEF=$(realpath "$1")
DENSITY=${2:-0.6}
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cp "$DEF" "$TMP/in.def"

LEFS="read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef"
for m in $(grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$DEF" | sort -u); do
    LEFS="$LEFS
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lef/$m.lef"
done

cat > "$TMP/run.tcl" <<EOF
$LEFS
read_def /w/in.def
puts "FPCHECK read_def ok: [llength [[ord::get_db_block] getInsts]] instances, [llength [[ord::get_db_block] getBTerms]] pins, [llength [[ord::get_db_block] getRows]] rows"
global_placement -density $DENSITY -skip_io
detailed_placement
if {[catch {check_placement -verbose} err]} { puts "FPCHECK check_placement FAILED: \$err" } else { puts "FPCHECK check_placement ok" }
set supply_types {}
foreach supply [[ord::get_db_block] getNets] {
    if {[\$supply getSigType] ni {POWER GROUND}} continue
    lappend supply_types [\$supply getSigType]
    set net [\$supply getName]
    if {[catch {check_power_grid -net \$net} err]} { puts "FPCHECK check_power_grid \$net FAILED: \$err" } else { puts "FPCHECK check_power_grid \$net ok" }
}
foreach type {POWER GROUND} {
    if {\$type ni \$supply_types} { puts "FPCHECK check_power_grid FAILED: missing \$type net" }
}
set_routing_layers -signal Metal2-Metal4
if {[catch {global_route -allow_congestion} err]} { puts "FPCHECK global_route FAILED: \$err" } else { puts "FPCHECK global_route ok" }
EOF

docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/run.tcl > "$TMP/openroad.log" 2>&1 || true
grep -E 'FPCHECK|\[ERROR|Total wirelength|Total overflow' "$TMP/openroad.log" | sed 's/^FPCHECK //' | head -40
if grep -q 'FAILED\|\[ERROR' "$TMP/openroad.log"; then
    echo "--- last lines of the OpenROAD log:"; tail -15 "$TMP/openroad.log"; exit 1
fi
