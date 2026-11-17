#!/usr/bin/env bash
# Design-rule check of a routed fastpnr DEF with IHP's own KLayout DRC deck (in the OpenLane 2
# docker image): the DEF is turned into GDS with the real standard-cell layouts, then checked.
# Prints the number of violations per rule. Metal density rules are listed separately: they
# need metal fill, which no flow step adds yet.
#
# usage: tools/drc_klayout.sh <routed.def>
set -euo pipefail

DEF=$(realpath "$1")
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cp "$DEF" "$TMP/in.def"
TOP=$(awk '/^DESIGN /{print $2; exit}' "$DEF")

SRAMS=$( (grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$DEF" || true) | sort -u | tr '\n' ' ')
cat > "$TMP/def2gds.py" <<EOF
import pya
ref = '/pdk/ihp-sg13cmos5l/libs.ref'
opt = pya.LoadLayoutOptions()
cfg = opt.lefdef_config
cfg.map_file = '/pdk/ihp-sg13cmos5l/libs.tech/klayout/tech/sg13cmos5l.map'
lefs = [ref + '/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef', ref + '/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef']
gds = [ref + '/sg13cmos5l_stdcell/gds/sg13cmos5l_stdcell.gds']
for m in "$SRAMS".split():
    lefs.append(ref + '/sg13cmos5l_sram/lef/' + m + '.lef')
    gds.append(ref + '/sg13cmos5l_sram/gds/' + m + '.gds')
cfg.lef_files = lefs
cfg.macro_layout_files = gds
cfg.macro_resolution_mode = 2
cfg.read_lef_with_def = False
layout = pya.Layout()
layout.read('/w/in.def', opt)
layout.write('/w/out.gds')
EOF
docker run --rm -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" bash -c "
  klayout -b -r /w/def2gds.py > /w/convert.log 2>&1 &&
  klayout -b -r /pdk/ihp-sg13cmos5l/libs.tech/klayout/tech/drc/ihp-sg13cmos5l.drc -rd input=/w/out.gds -rd topcell=$TOP -rd report=/w/drc.lyrdb -rd run_mode=deep > /w/drc.log 2>&1" || {
    tail -20 "$TMP/convert.log" "$TMP/drc.log" 2>/dev/null; exit 2; }
[ -n "${DRC_KEEP:-}" ] && cp "$TMP/drc.lyrdb" "$TMP/out.gds" "$DRC_KEEP"/
python3 - "$TMP/drc.lyrdb" <<'PY'
import sys, xml.etree.ElementTree as ET
from collections import Counter
root = ET.parse(sys.argv[1]).getroot()
c = Counter(item.findtext('category').strip("'") for item in root.iter('item'))
density = {k: v for k, v in c.items() if 'density' in k.lower() or k.lower().startswith(('m1.j', 'm2.j', 'm3.j', 'm4.j', 'tm1.c', 'tm1.d')) or '.j' in k.lower()}
rules = {k: v for k, v in c.items() if k not in density}
print(f"DRC violations (excluding metal density): {sum(rules.values())}")
for k, v in sorted(rules.items(), key=lambda kv: -kv[1]):
    print(f"  {k}: {v}")
if density:
    print(f"metal density rules (need fill): {', '.join(f'{k} {v}' for k, v in sorted(density.items()))}")
PY
