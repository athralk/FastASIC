#!/usr/bin/env bash
# Layout-versus-schematic check of a routed fastpnr DEF with IHP's own KLayout LVS deck (in the
# OpenLane 2 docker image). The layout is the DEF turned into GDS with the real cell layouts; the
# schematic is the same design's connectivity, written from the DEF's components and nets with
# every cell expanded to transistors from the PDK's CDL. A match means no opens and no shorts.
#
# usage: tools/lvs_klayout.sh <routed.def>
set -euo pipefail

DEF=$(realpath "$1")
# The LVS driver needs a recent KLayout: the LibreLane image has one
IMAGE=${LIBRELANE_IMAGE:-ghcr.io/librelane/librelane:3.1.0.dev3}
PDK_DIR=${PDK_ROOT:-$HOME/ttsetup/pdk}
REF=$PDK_DIR/ihp-sg13cmos5l/libs.ref
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cp "$DEF" "$TMP/in.def"
TOP=$(awk '/^DESIGN /{print $2; exit}' "$DEF")
SRAMS=$( (grep -oE 'RM_IHPSG13_[A-Za-z0-9_]+' "$DEF" || true) | sort -u | tr '\n' ' ')

# Schematic: the DEF's connectivity, one subcircuit call per cell (fillers have no devices)
python3 - "$DEF" "$REF" "$TMP/top.cdl" "$SRAMS" <<'PY'
import re, sys
def_path, ref, out_path, srams = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4].split()
text = open(def_path).read()
# SRAMs are black boxes: their own GDS and CDL do not compare clean with this deck, so the layout
# keeps only their LEF pins and the schematic an empty subcircuit with the same pins
cdls = [ref + '/sg13cmos5l_stdcell/cdl/sg13cmos5l_stdcell.cdl']
order = {}
blackbox = {}
for m in srams:
    lef = open(ref + '/sg13cmos5l_sram/lef/' + m + '.lef').read()
    pins = re.findall(r'\n  PIN (\S+)', lef)
    order[m] = pins
    blackbox[m] = pins
for path in cdls:
    lines = open(path).read().replace('\n+', ' ').split('\n')
    for line in lines:
        m = re.match(r'\s*\.SUBCKT\s+(\S+)\s+(.*)', line, re.I)
        if m: order[m.group(1)] = m.group(2).split()
def section(name):
    m = re.search(r'\n%s \d+ ;(.*?)\nEND %s' % (name, name), text, re.S)
    return m.group(1) if m else ''
comps = re.findall(r'-\s+(\S+)\s+(\S+)', section('COMPONENTS'))
conn = {}   # (instance, pin) -> net
ports = []
pins_on = {}   # net -> the top-level pins on it
for stmt in section('PINS').split(';'):
    m = re.search(r'-\s+(\S+)\s+\+\s+NET\s+(\S+)', stmt)
    if m and m.group(1) not in ('VDD', 'VSS'):
        ports.append(m.group(2))
        pins_on.setdefault(m.group(2), []).append(m.group(1))
# A net with several top-level pins (an input wired straight to an output, `assign y = a`):
# the extracted layout names it after all its labels, sorted and joined by commas, so the
# schematic does the same (LVS then still checks that those pins share one net)
alias = {n: ','.join(sorted(p)) for n, p in pins_on.items() if len(p) > 1}
for stmt in section('NETS').split(';'):
    m = re.match(r'\s*-\s+(\S+)', stmt)
    if not m: continue
    net = m.group(1)
    for inst, pin in re.findall(r'\(\s*(\S+)\s+(\S+)\s*\)', stmt.split('+')[0]):
        if inst != 'PIN': conn[(inst, pin)] = net
unc = 0
def spice(n): return alias.get(n, n).replace('\\', '')
with open(out_path, 'w') as f:
    # KLayout's SPICE reader stops a path at '-': the CDLs are copied next to the schematic
    for path in cdls: f.write('.INCLUDE %s\n' % path.split('/')[-1])
    for m, pins in blackbox.items(): f.write('.SUBCKT %s %s\n.ENDS\n' % (m, ' '.join(pins)))
    uniq = list(dict.fromkeys(ports))
    f.write('.SUBCKT %s %s VDD VSS\n' % (re.search(r'DESIGN (\S+)', text).group(1), ' '.join(spice(p) for p in uniq)))
    for inst, master in comps:
        if master not in order or 'fill' in master or 'decap' in master: continue
        nets = []
        for pin in order[master]:
            if pin in ('VDD', 'VDD!', 'VDDARRAY!'): nets.append('VDD'); continue   # SRAM supplies join the grid
            if pin in ('VSS', 'VSS!'): nets.append('VSS'); continue
            n = conn.get((inst, pin)) or conn.get((inst, pin.replace('<', '[').replace('>', ']')))   # CDL buses use <k>
            if n is None: unc += 1; n = 'unconnected_%d' % unc
            nets.append(spice(n))
        f.write('X%s %s %s\n' % (spice(inst), ' '.join(nets), master))
    f.write('.ENDS\n')
PY

# SRAM pins: the LVS above treats SRAMs as black boxes, so their connections are checked here
# by probing the extracted metal: each SRAM pin must sit on the same net as another pin of its
# DEF net (no open), and pins of different nets must never share one (no short)
if [ -n "$SRAMS" ]; then
python3 - "$DEF" "$REF" "$TMP/probes.json" "$SRAMS" <<'PY'
import re, sys, json
def_path, ref, out_path, srams = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4].split()
text = open(def_path).read()
dbu = int(re.search(r'UNITS DISTANCE MICRONS (\d+)', text).group(1))
def lef_pins(path):
    out = {}
    t = open(path).read()
    for mm in re.finditer(r'\nMACRO (\S+)(.*?)\nEND \1', t, re.S):
        name, body = mm.group(1), mm.group(2)
        size = re.search(r'SIZE ([\d.]+) BY ([\d.]+)', body)
        pins = {}
        for pm in re.finditer(r'\n  PIN (\S+)(.*?)\n  END \1', body, re.S):
            layer = None
            for line in pm.group(2).split('\n'):
                l = re.match(r'\s*LAYER (\S+)', line)
                if l: layer = l.group(1)
                r = re.match(r'\s*RECT ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)', line)
                if r and layer and pm.group(1) not in pins: pins[pm.group(1)] = (layer, *map(float, r.groups()))
        out[name] = (float(size.group(1)), float(size.group(2)), pins)
    return out
lib = lef_pins(ref + '/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef')
for m in srams: lib.update(lef_pins(ref + '/sg13cmos5l_sram/lef/' + m + '.lef'))
def section(name):
    m = re.search(r'\n%s \d+ ;(.*?)\nEND %s' % (name, name), text, re.S)
    return m.group(1) if m else ''
place = {}
for inst, master, x, y, o in re.findall(r'-\s+(\S+)\s+(\S+)\s+\+\s+(?:PLACED|FIXED)\s+\(\s*(-?\d+)\s+(-?\d+)\s*\)\s+(\w+)', section('COMPONENTS')):
    place[inst] = (master, int(x) / dbu, int(y) / dbu, o)
def pin_point(inst, pin):
    master, x, y, o = place[inst]
    w, h, pins = lib[master]
    if pin not in pins: return None
    layer, x0, y0, x1, y1 = pins[pin]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if o in ('FS', 'S'): cy = h - cy
    if o in ('FN', 'S'): cx = w - cx
    return layer, x + cx, y + cy
probes = []
for stmt in section('NETS').split(';'):
    m = re.match(r'\s*-\s+(\S+)', stmt)
    if not m: continue
    conns = [c for c in re.findall(r'\(\s*(\S+)\s+(\S+)\s*\)', stmt.split('+')[0]) if c[0] != 'PIN']
    sram = [c for c in conns if c[0] in place and place[c[0]][0] in srams]
    other = [c for c in conns if c[0] in place and place[c[0]][0] not in srams]
    for inst, pin in sram:
        a = pin_point(inst, pin)
        b = pin_point(*other[0]) if other else None
        if a: probes.append({'net': m.group(1), 'sram': [inst + '/' + pin, *a], 'other': [other[0][0] + '/' + other[0][1], *b] if b else None})
json.dump(probes, open(out_path, 'w'))
PY
cat > "$TMP/probe.py" <<'EOF2'
import pya, json
ly = pya.Layout()
ly.read('/w/out.gds')
top = ly.top_cell()
l2n = pya.LayoutToNetlist(pya.RecursiveShapeIterator(ly, top, []))
layers = {}
stack = [('Metal1', 8), ('Via1', 19), ('Metal2', 10), ('Via2', 29), ('Metal3', 30), ('Via3', 49), ('Metal4', 50), ('TopVia1', 125), ('TopMetal1', 126)]
for name, n in stack:
    shapes = [l2n.make_layer(ly.layer(n, d), '%s_%d' % (name, d)) for d in ((0, 2) if not name.startswith(('Via', 'Top' 'Via')) else (0,))]
    for r in shapes: l2n.connect(r)
    for a in shapes:
        for b in shapes:
            if a is not b: l2n.connect(a, b)
    layers[name] = shapes
for (a, _), (b, _) in zip(stack, stack[1:]):
    for ra in layers[a]:
        for rb in layers[b]: l2n.connect(ra, rb)
l2n.extract_netlist()
def probe(layer, x, y):
    for r in layers[layer]:
        net = l2n.probe_net(r, pya.DPoint(x, y))
        if net: return net.cluster_id
    return None
opens, shorts, owner = 0, 0, {}
for p in json.load(open('/w/probes.json')):
    if not p['other']: continue   # an unused SRAM output: nothing else on its net
    a = probe(*p['sram'][1:])
    if a is None:
        print('OPEN  %s: nothing at the SRAM pin' % p['sram'][0]); opens += 1; continue
    if p['other']:
        b = probe(*p['other'][1:])
        if b != a: print('OPEN  %s (net %s) does not reach %s' % (p['sram'][0], p['net'], p['other'][0])); opens += 1
    if a in owner and owner[a] != p['net']:
        print('SHORT %s: nets %s and %s' % (p['sram'][0], owner[a], p['net'])); shorts += 1
    owner[a] = p['net']
print('SRAM pins probed: %d, opens %d, shorts %d' % (sum(1 for q in json.load(open('/w/probes.json')) if q['other']), opens, shorts))
EOF2
fi
cp "$REF/sg13cmos5l_stdcell/cdl/sg13cmos5l_stdcell.cdl" "$TMP/"
cat > "$TMP/def2gds.py" <<EOF
import pya
ref = '/pdk/ihp-sg13cmos5l/libs.ref'
opt = pya.LoadLayoutOptions()
cfg = opt.lefdef_config
cfg.map_file = '/pdk/ihp-sg13cmos5l/libs.tech/klayout/tech/sg13cmos5l.map'
lefs = [ref + '/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef', ref + '/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef']
gds = [ref + '/sg13cmos5l_stdcell/gds/sg13cmos5l_stdcell.gds']
for m in "$SRAMS".split():
    lefs.append(ref + '/sg13cmos5l_sram/lef/' + m + '.lef')   # black box: LEF pins only
cfg.lef_files = lefs
cfg.macro_layout_files = gds
cfg.read_lef_with_def = False
cfg.produce_lef_pins = True
cfg.macro_resolution_mode = 2   # standard cells: their real layouts
layout = pya.Layout()
layout.read('/w/in.def', opt)
if "$SRAMS".split():
    # SRAMs: the LEF abstract (pin shapes; obstructions sit on their own datatype), from a second read
    cfg.macro_resolution_mode = 1
    abstract = pya.Layout()
    abstract.read('/w/in.def', opt)
    lm = pya.LayerMapping()
    lm.create_full(layout, abstract)
    for m in "$SRAMS".split():
        if layout.has_cell(m) and abstract.has_cell(m):
            layout.cell(m).copy_shapes(abstract.cell(m), lm)
layout.write('/w/out.gds')
EOF
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/w -v "$TMP:/w" -v "$PDK_DIR:/pdk:ro" "$IMAGE" bash -c "
  cd /w && klayout -b -r /w/def2gds.py > /w/convert.log 2>&1 &&
  python3 /pdk/ihp-sg13cmos5l/libs.tech/klayout/tech/lvs/run_lvs.py --layout=/w/out.gds --netlist=/w/top.cdl --run_dir=/w/run \
    --topcell=$TOP --run_mode=deep --disable_tap_extraction > /w/lvs.log 2>&1;
  if [ -f /w/probe.py ]; then klayout -b -r /w/probe.py > /w/probe.log 2>&1; fi" || true
[ -n "${LVS_KEEP:-}" ] && cp -r "$TMP"/lvs.log "$TMP"/top.cdl "$TMP"/run "$TMP"/out.gds "$TMP"/probes.json "$TMP"/probe.py "$TMP"/probe.log "$TMP"/convert.log "$LVS_KEEP"/ 2>/dev/null || true   # some files exist only with SRAMs
status=0
if [ -n "$SRAMS" ]; then
    if grep -q 'opens 0, shorts 0' "$TMP/probe.log" 2>/dev/null; then grep 'SRAM pins probed' "$TMP/probe.log"
    else echo "SRAM pin check FAILED"; grep -E 'OPEN|SHORT|probed|Error|ERROR' "$TMP/probe.log" | head -8; status=1; fi
fi
if grep -q 'Congratulations! Netlists match' "$TMP/lvs.log"; then
    echo "LVS: netlists match ($(grep -oE 'Devices to compare: layout=[0-9]+' "$TMP/lvs.log" | grep -oE '[0-9]+$') device types compared)"
else
    echo "LVS: MISMATCH"; grep -E 'ERROR|Outcome' "$TMP/lvs.log" | head -8; exit 1
fi
exit $status
