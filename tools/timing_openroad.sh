#!/usr/bin/env bash
# Post-route timing of a routed DEF, judged by OpenROAD: OpenRCX extracts the wires' R and C with
# IHP's own extraction rules, OpenSTA then times the design with those parasitics.
# Works on any routed DEF (fastpnr's or OpenROAD's own), so two layouts can be compared under the
# same constraint.
#
# usage: tools/timing_openroad.sh <routed.def> <period_ns> [corner: typ|fast|slow]
#   A virtual clock of the given period; every input arrives at 0 driven by sg13cmos5l_buf_4,
#   every output must arrive by the period with a 6 fF load. (A clocked design: pass SDC=<file>.)
set -euo pipefail

DEF=$(realpath "$1")
PERIOD=$2
CORNER=${3:-typ}
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
THREADS=${OPENROAD_THREADS:-$(nproc)}
case $CORNER in
    fast) LIB=sg13cmos5l_stdcell_fast_1p32V_m40C.lib ;;
    slow) LIB=sg13cmos5l_stdcell_slow_1p08V_125C.lib ;;
    *)    LIB=sg13cmos5l_stdcell_typ_1p20V_25C.lib ;;
esac
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cp "$DEF" "$TMP/in.def"
if [ -n "${SDC:-}" ]; then cp "$SDC" "$TMP/constraints.sdc"
else cat > "$TMP/constraints.sdc" <<EOF
create_clock -name vclk -period $PERIOD
set_input_delay 0 -clock vclk [all_inputs]
set_output_delay 0 -clock vclk [all_outputs]
set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs]
set_load 0.006 [all_outputs]
EOF
fi
LEFS="read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef"
LIBS="read_liberty /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/$LIB"
for m in $( (grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$DEF" || true) | sort -u); do
    LEFS="$LEFS
read_lef /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lef/$m.lef"
    LIBS="$LIBS
read_liberty /pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lib/${m}_${CORNER/typ/typ_1p20V_25C}.lib"
done
cat > "$TMP/run.tcl" <<EOF
$LEFS
$LIBS
read_def /w/in.def
read_sdc /w/constraints.sdc
set_thread_count $THREADS
define_process_corner -ext_model_index 0 X
extract_parasitics -ext_model_file /pdk/ihp-sg13cmos5l/libs.tech/librelane/IHP_rcx_patterns.rules
write_spef /w/out.spef
read_spef /w/out.spef
report_worst_slack -max -digits 3
report_tns -digits 3
report_checks -path_delay max -digits 1 -fields {slew cap}
EOF
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/run.tcl > "$TMP/or.log" 2>&1 || { tail -20 "$TMP/or.log"; exit 2; }
[ -n "${KEEP:-}" ] && cp "$TMP/or.log" "$TMP/out.spef" "$KEEP"/ 2>/dev/null
wns=$(grep -E '^worst slack ' "$TMP/or.log" | tail -1 | awk '{print $3}')
tns=$(grep -E '^tns ' "$TMP/or.log" | tail -1 | awk '{print $2}')
arrival=$(grep -E 'data arrival time' "$TMP/or.log" | head -1 | awk '{print $1}')
startp=$(grep -E '^Startpoint:' "$TMP/or.log" | head -1 | cut -d' ' -f2-)
endp=$(grep -E '^Endpoint:' "$TMP/or.log" | head -1 | cut -d' ' -f2-)
python3 - "$wns" "$tns" "$arrival" "$PERIOD" "$startp" "$endp" "$(basename "$DEF")" <<'PY'
import sys
wns, tns, arrival, period, sp, ep, name = sys.argv[1:]
wns, tns, arrival, period = float(wns), float(tns), float(arrival), float(period)
print(f"{name}: period {period:g} ns, worst slack {wns:+.3f} ns ({'met' if wns >= 0 else 'VIOLATED'}), total negative slack {tns:.3f} ns")
crit = period - wns   # exact (the path report rounds)
print(f"  critical path {crit:.3f} ns ({sp} -> {ep}): fastest clock {1000 / crit:.1f} MHz")
PY
