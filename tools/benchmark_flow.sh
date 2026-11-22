#!/usr/bin/env bash
# Full-flow benchmark of one Verilog design: FastASIC against Yosys + ABC and OpenROAD.
#
#   synthesis   FastASIC vs Yosys + ABC (time, cells, area)
#   routing     fastpnr vs OpenROAD FastRoute + TritonRoute, on the
#               same netlist and placement (fastpnr's), so only the routers differ
#   quality     IHP KLayout DRC on both layouts, IHP KLayout LVS on fastpnr's, wire, vias
#   timing      both routed layouts extracted with OpenRCX (IHP rules) and timed with OpenSTA,
#               typical and slow corner
#
# usage: tools/benchmark_flow.sh <design.v> [options]
#   --map area|delay    FastASIC mapping goal (default area)
#   --period NS         clock period for the timing check (default 10)
#   --top MODULE        top module (default: the one no other module instantiates)
#   --name NAME         output folder runs/NAME (default: file name, plus _delay for --map delay)
#   --equiv sat|sim     FastASIC's built-in equivalence check (default sim: random simulation;
#                       sat proves it, but cannot finish on large multipliers)
#   --threads N         the same N threads for fastpnr's detailed routing (-j N) and OpenROAD
#                       (default: fastpnr 1 thread, OpenROAD all threads)
# Everything is kept in runs/NAME/. OpenROAD's routing (the slow part) is reused when the netlist
# and placement are unchanged; delete runs/NAME/openroad.* to force a new run.
# Needs docker with the OpenLane 2 image, yosys and the IHP PDK.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
RTL=""; MAP=area; PERIOD=10; TOP=""; NAME=""; EQUIV=sim; BOTH_THREADS=""
while [ $# -gt 0 ]; do
    case $1 in
        --map) MAP=$2; shift 2 ;;
        --period) PERIOD=$2; shift 2 ;;
        --top) TOP=$2; shift 2 ;;
        --name) NAME=$2; shift 2 ;;
        --equiv) EQUIV=$2; shift 2 ;;
        --threads) BOTH_THREADS=$2; shift 2 ;;
        -*) echo "unknown option $1"; exit 2 ;;
        *) RTL=$(realpath "$1"); shift ;;
    esac
done
[ -n "$RTL" ] || { sed -n '2,20p' "$0"; exit 2; }
base=$(basename "$RTL" .v)
[ -n "$NAME" ] || NAME=$base$([ "$MAP" = delay ] && echo _delay || true)
[ -n "$TOP" ] || TOP=$(python3 - "$RTL" <<'PY'
import re, sys
t = re.sub(r'//[^\n]*|/\*.*?\*/', '', open(sys.argv[1]).read(), flags=re.S)
mods = re.findall(r'\bmodule\s+(\w+)', t)
body = re.sub(r'\bmodule\s+\w+', '', t)   # drop the definitions' own names
used = {m for m in mods if re.search(r'\b%s\s*(#\s*\(.*?\)\s*)?\w+\s*\(' % re.escape(m), body, re.S)}
print(next((m for m in mods if m not in used), mods[-1]))
PY
)
OUT=$ROOT/runs/$NAME
mkdir -p "$OUT"
cp "$RTL" "$OUT/$base.v"
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
THREADS=${BOTH_THREADS:-${OPENROAD_THREADS:-$(nproc)}}
FA_THREADS=${BOTH_THREADS:-${FASTPNR_THREADS:-1}}
ms() { python3 -c "import sys; print(round(float(sys.argv[1]) * 1000))" "$1"; }
log() { echo "[$(date +%H:%M:%S)] $*" >&2; }

# ---------------------------------------------------------------- synthesis
log "synthesis: FastASIC (--map $MAP)"
t0=$(date +%s%N)
"$ROOT/FastASIC" -q --map "$MAP" --equiv-method "$EQUIV" --top "$TOP" -o "$OUT/fastasic.cells.v" "$OUT/$base.v" > "$OUT/fastasic.synth.log" 2>&1
fa_syn=$((($(date +%s%N) - t0) / 1000000))
grep -q 'FAIL' "$OUT/fastasic.synth.log" && { echo "FastASIC equivalence check failed"; exit 1; }
log "synthesis: Yosys + ABC"
t0=$(date +%s%N)
bash "$ROOT/tools/synth_yosys.sh" "$OUT/$base.v" "$TOP" "$OUT/yosys.cells.v" > "$OUT/yosys.synth.log" 2>&1
yo_syn=$((($(date +%s%N) - t0) / 1000000))
read -r fa_cells fa_area < <("$ROOT/lsta" -q "$OUT/fastasic.cells.v" 2>&1 | grep -oE '^Design: +[0-9]+ cells.*area [0-9.]+' | awk '{print $2, $NF}')
read -r yo_cells yo_area < <("$ROOT/lsta" -q "$OUT/yosys.cells.v" 2>&1 | grep -oE '^Design: +[0-9]+ cells.*area [0-9.]+' | awk '{print $2, $NF}')

# ---------------------------------------------------------------- routing: fastpnr
log "place and route: fastpnr"
"$ROOT/fastpnr" -q -no-route -o "$OUT/placed.new.def" "$OUT/fastasic.cells.v"
[ -s "$OUT/placed.new.def" ] || { echo "fastpnr did not write the placement (runs/$NAME/placed.new.def)"; exit 1; }
"$ROOT/fastpnr" -droute -j "$FA_THREADS" -o "$OUT/fastasic.routed.def" "$OUT/fastasic.cells.v" > "$OUT/fastpnr.log" 2>&1 || true
fa_gr=$(grep -oE 'routing [0-9.]+ \[capacity' "$OUT/fastpnr.log" | awk '{print $2}')
fa_dr=$(grep -oE 'time [0-9.]+ ms \(setup' "$OUT/fastpnr.log" | awk '{print $2}')
fa_wire=$(grep -oE '[0-9]+ um of wire' "$OUT/fastpnr.log" | awk '{print $1}')
fa_vias=$(grep -oE '[0-9]+ vias, [0-9]+ negotiation' "$OUT/fastpnr.log" | awk '{print $1}')
fa_state=$(grep -oE '[0-9]+ conflicts, [0-9]+ unconnected nets' "$OUT/fastpnr.log")

# ---------------------------------------------------------------- routing: OpenROAD (cached)
if [ -f "$OUT/placed.def" ] && cmp -s "$OUT/placed.def" "$OUT/placed.new.def" && [ -f "$OUT/openroad.routed.def" ] && [ -f "$OUT/openroad.times" ] && [ "$(awk '{print $5}' "$OUT/openroad.times")" = "$THREADS" ]; then
    log "routing: OpenROAD (reusing the earlier run on this placement)"
else
    mv "$OUT/placed.new.def" "$OUT/placed.def"
    log "routing: OpenROAD FastRoute + TritonRoute, $THREADS threads (this is the slow part)"
    TMP=$(mktemp -d)
    trap 'rm -rf "$TMP"' EXIT
    cp "$OUT/placed.def" "$TMP/placed.def"
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
set t0 [clock milliseconds]
global_route
set t1 [clock milliseconds]
detailed_route -output_drc /w/drt.rpt
set t2 [clock milliseconds]
puts "ORTIMES [expr {\$t1 - \$t0}] [expr {\$t2 - \$t1}]"
write_def /w/openroad.def
EOF
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" openroad -no_init -exit /w/run.tcl > "$TMP/or.log" 2>&1 || { tail -20 "$TMP/or.log"; exit 2; }
    cp "$TMP/openroad.def" "$OUT/openroad.routed.def"
    cp "$TMP/or.log" "$OUT/openroad.route.log"
    read -r gr dr < <(grep '^ORTIMES' "$TMP/or.log" | awk '{print $2, $3}')
    wire=$(grep -oE 'Total wire length = [0-9.]+' "$TMP/or.log" | tail -1 | awk '{print $5}')
    vias=$(grep -oE 'Total number of vias = [0-9]+' "$TMP/or.log" | tail -1 | awk '{print $6}')
    echo "$gr $dr $wire $vias $THREADS" > "$OUT/openroad.times"
fi
rm -f "$OUT/placed.new.def"
read -r or_gr or_dr or_wire or_vias or_threads < "$OUT/openroad.times"

# ---------------------------------------------------------------- checks and timing
log "IHP DRC on both layouts, IHP LVS on fastpnr's"
fa_drc=$(bash "$ROOT/tools/drc_klayout.sh" "$OUT/fastasic.routed.def" | head -1 | grep -oE '[0-9]+$')
or_drc=$(bash "$ROOT/tools/drc_klayout.sh" "$OUT/openroad.routed.def" | head -1 | grep -oE '[0-9]+$')
fa_lvs=$(bash "$ROOT/tools/lvs_klayout.sh" "$OUT/fastasic.routed.def" 2>&1 | grep -q 'netlists match' && echo match || echo MISMATCH)
log "timing: OpenRCX + OpenSTA, typical and slow corner"
crit() { bash "$ROOT/tools/timing_openroad.sh" "$1" "$PERIOD" "$2" 2>/dev/null | grep -oE 'critical path [0-9.]+' | awk '{print $3}'; }
fa_typ=$(crit "$OUT/fastasic.routed.def" typ); fa_slow=$(crit "$OUT/fastasic.routed.def" slow)
or_typ=$(crit "$OUT/openroad.routed.def" typ); or_slow=$(crit "$OUT/openroad.routed.def" slow)

# ---------------------------------------------------------------- report
python3 - "$OUT/benchmark.txt" <<PY
import sys
d = dict(name="$NAME", map="$MAP", period=float("$PERIOD"),
         fa_syn=float("$fa_syn"), yo_syn=float("$yo_syn"), fa_cells=int("$fa_cells"), yo_cells=int("$yo_cells"),
         fa_area=float("$fa_area"), yo_area=float("$yo_area"),
         fa_gr=float("${fa_gr:-0}"), fa_dr=float("${fa_dr:-0}"), or_gr=float("$or_gr"), or_dr=float("$or_dr"),
         fa_wire=float("${fa_wire:-0}"), or_wire=float("$or_wire"), fa_vias=int("${fa_vias:-0}"), or_vias=int("$or_vias"),
         fa_drc="$fa_drc", or_drc="$or_drc", fa_lvs="$fa_lvs", state="$fa_state", threads="$or_threads", fa_threads=int("$FA_THREADS"),
         fa_typ=float("$fa_typ"), fa_slow=float("$fa_slow"), or_typ=float("$or_typ"), or_slow=float("$or_slow"))
s = lambda ms: f"{ms / 1000:.1f} s" if ms >= 1000 else f"{ms:.0f} ms"
pct = lambda a, b: f"{(a / b - 1) * 100:+.1f}%"
fa_route, or_route = d['fa_gr'] + d['fa_dr'], d['or_gr'] + d['or_dr']
lines = [
    f"Benchmark {d['name']} (FastASIC --map {d['map']})",
    "",
    "Speed                     FastASIC              Yosys / OpenROAD                 Speedup",
    f"  Synthesis               {s(d['fa_syn']):<21} Yosys + ABC: {s(d['yo_syn']):<19} {d['yo_syn'] / d['fa_syn']:.1f}x",
    f"  Routing (global+detail) {s(fa_route):<21} OpenROAD: {s(or_route):<22} {or_route / fa_route:.1f}x",
    f"    global                {s(d['fa_gr']):<21} FastRoute: {s(d['or_gr']):<21} {d['or_gr'] / max(d['fa_gr'], 1e-3):.1f}x",
    f"    detailed              {s(d['fa_dr']):<21} TritonRoute: {s(d['or_dr']):<19} {d['or_dr'] / max(d['fa_dr'], 1e-3):.1f}x",
    f"  (fastpnr {d['fa_threads']} thread{'s' if d['fa_threads'] > 1 else ''}, OpenROAD {d['threads']} threads)",
    "",
    "Result                    FastASIC              Yosys / OpenROAD                 FastASIC vs",
    f"  Cells                   {d['fa_cells']:<21} {d['yo_cells']:<32} {pct(d['fa_cells'], d['yo_cells'])}",
    f"  Area (um^2)             {d['fa_area']:<21.0f} {d['yo_area']:<32.0f} {pct(d['fa_area'], d['yo_area'])}",
    f"  Wire (um)               {d['fa_wire']:<21.0f} {d['or_wire']:<32.0f} {pct(d['fa_wire'], d['or_wire'])}",
    f"  Vias                    {d['fa_vias']:<21} {d['or_vias']:<32} {pct(d['fa_vias'], d['or_vias'])}",
    f"  IHP DRC violations      {d['fa_drc']:<21} {d['or_drc']:<32}",
    f"  IHP LVS                 {d['fa_lvs']:<21} {'-':<32}",
    f"  Router state            {d['state']}",
    "",
    "Timing (OpenRCX + OpenSTA, same netlist and placement; critical path, fastest clock)",
    f"  Typical corner          {d['fa_typ']:.3f} ns, {1000 / d['fa_typ']:.1f} MHz     {d['or_typ']:.3f} ns, {1000 / d['or_typ']:.1f} MHz               {pct(d['fa_typ'], d['or_typ'])}",
    f"  Slow corner (sign-off)  {d['fa_slow']:.3f} ns, {1000 / d['fa_slow']:.1f} MHz     {d['or_slow']:.3f} ns, {1000 / d['or_slow']:.1f} MHz               {pct(d['fa_slow'], d['or_slow'])}",
    "",
    f"Outputs: runs/{d['name']}/",
]
text = "\n".join(lines)
print(text)
open(sys.argv[1], "w").write(text + "\n")
PY
