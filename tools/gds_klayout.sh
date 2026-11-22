#!/usr/bin/env bash
# GDS of a routed DEF, and pictures of it, with KLayout (in the OpenLane 2 docker image): the DEF
# is merged with the real standard-cell (and SRAM) layouts into one GDS, then drawn with IHP's
# layer colours: the whole chip, and a close-up of its centre.
#
# usage: tools/gds_klayout.sh <routed.def> [out_dir, default: next to the DEF] [--gui]
#   writes <name>.gds, <name>.png and <name>.zoom.png
#   --gui opens the GDS in KLayout's window instead of drawing pictures (WSLg or any X display)
set -euo pipefail

DEF=$(realpath "$1")
OUT=$(realpath "${2:-$(dirname "$DEF")}")
GUI=""
for a in "$@"; do [ "$a" = --gui ] && GUI=1; done
[ "${2:-}" = --gui ] && OUT=$(dirname "$DEF")
IMAGE=${OPENLANE_IMAGE:-ghcr.io/efabless/openlane2:2.3.10}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
NAME=$(basename "$DEF" .def)
TECH=/pdk/ihp-sg13cmos5l/libs.tech/klayout/tech
mkdir -p "$OUT"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cp "$DEF" "$TMP/in.def"
SRAMS=$( (grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$DEF" || true) | sort -u | tr '\n' ' ')

cat > "$TMP/def2gds.py" <<EOF
import pya
ref = '/pdk/ihp-sg13cmos5l/libs.ref'
opt = pya.LoadLayoutOptions()
cfg = opt.lefdef_config
cfg.map_file = '$TECH/sg13cmos5l.map'
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
top = layout.top_cell()
b = top.dbbox()
print('GDS %s: %.1f x %.1f um, %d cells placed' % (top.name, b.width(), b.height(), sum(1 for _ in top.each_inst())))
EOF

cat > "$TMP/draw.py" <<EOF
import pya
lv = pya.LayoutView()
lv.load_layout('/w/out.gds', True)
lv.load_layer_props('$TECH/sg13cmos5l.lyp')
lv.max_hier()
lv.set_config('background-color', '#ffffff')
lv.set_config('grid-visible', 'false')
lv.set_config('text-visible', 'false')
lv.zoom_fit()
lv.save_image_with_options('/w/out.png', 2400, 2400, 0, 2, 0, pya.DBox(), False)
b = lv.cellview(0).cell.dbbox()
c, h = b.center(), 15.0   # a 30 x 30 um window in the middle
lv.save_image_with_options('/w/out.zoom.png', 2400, 2400, 0, 2, 0, pya.DBox(c.x - h, c.y - h, c.x + h, c.y + h), False)
EOF

docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" \
    klayout -b -r /w/def2gds.py > "$TMP/convert.log" 2>&1 || { tail -20 "$TMP/convert.log"; exit 2; }
grep '^GDS ' "$TMP/convert.log"
cp "$TMP/out.gds" "$OUT/$NAME.gds"
echo "wrote $OUT/$NAME.gds"

if [ -n "$GUI" ]; then
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$OUT:/o:ro" -v "$PDK_DIR:/pdk:ro" "$IMAGE" klayout -l "$TECH/sg13cmos5l.lyp" "/o/$NAME.gds"
    exit 0
fi
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -e QT_QPA_PLATFORM=offscreen -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" \
    klayout -b -r /w/draw.py > "$TMP/draw.log" 2>&1 || { tail -20 "$TMP/draw.log"; exit 2; }
cp "$TMP/out.png" "$OUT/$NAME.png"
cp "$TMP/out.zoom.png" "$OUT/$NAME.zoom.png"
echo "wrote $OUT/$NAME.png and $OUT/$NAME.zoom.png"
