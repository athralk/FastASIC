#!/usr/bin/env bash
# Co-simulation check: the RTL (simulated by iverilog with its own operators) against the gate
# netlist we write, on the same inputs, comparing every output (4-state, X included).
#   combinational designs (no clk input): every input combination when they fit in 2^16,
#     otherwise random vectors mixed with all-zeros / all-ones / single-bit corner cases
#   clocked designs: random inputs every cycle, outputs compared after every clock edge;
#     address-like inputs are kept small so memory reads hit written words
# IHP SRAM macros in the netlist are simulated with the PDK's own functional model.
#
# usage: tests/cosim.sh <FastASIC> <design.v> [vectors] [extra FastASIC options]
#        COSIM_MUTATE='sed expression' edits the netlist first (to check that the check can fail)
set -euo pipefail
BIN=$1
SRC=$2
VECTORS=${3:-20000}
shift $(( $# >= 3 ? 3 : $# ))
VDIR=${PDK_ROOT:-$HOME/ttsetup/pdk}/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

"$BIN" -q "$@" -o "$TMP/netlist.v" "$SRC" > "$TMP/log.txt"
if [ -n "${COSIM_MUTATE:-}" ]; then sed -i -E "$COSIM_MUTATE" "$TMP/netlist.v"; fi
yosys -q -p "read_verilog $SRC; hierarchy -auto-top; proc; write_json $TMP/ports.json"

python3 - "$TMP" "$VECTORS" <<'PY'
import json, re, sys
tmp, vectors = sys.argv[1], int(sys.argv[2])
mods = json.load(open(f"{tmp}/ports.json"))["modules"]
top = next(n for n, m in mods.items() if m.get("attributes", {}).get("top"))
ports = mods[top]["ports"]
net = open(f"{tmp}/netlist.v").read()
open(f"{tmp}/netlist.v", "w").write(re.sub(rf"\bmodule\s+{top}\b", f"module {top}_gate", net, count=1))
ins = {n: len(p["bits"]) for n, p in ports.items() if p["direction"] == "input"}
outs = {n: len(p["bits"]) for n, p in ports.items() if p["direction"] == "output"}
clk = next((n for n in ins if n in ("clk", "clock")), None)
decl = "".join(f"  reg [{w-1}:0] {n};\n" for n, w in ins.items())
decl += "".join(f"  wire [{w-1}:0] {n}_rtl, {n}_net;\n" for n, w in outs.items())
conn = lambda suffix: ", ".join([f".{n}({n})" for n in ins] + [f".{n}({n}_{suffix})" for n in outs])
check = "".join(f'''      if ({n}_rtl !== {n}_net) begin
        errors = errors + 1;
        if (errors <= 5) $display("vector %0d: {n} rtl=%h netlist=%h", t, {n}_rtl, {n}_net);
      end
''' for n in outs)
data = [n for n in ins if n != clk]
total = sum(ins[n] for n in data)
if clk is None:
    exhaustive = total <= 16
    count = (1 << total) if exhaustive else vectors
    drive = []
    if exhaustive:
        pos = 0
        for n in data:
            drive.append(f"      {n} = t[{pos + ins[n] - 1}:{pos}];")
            pos += ins[n]
    else:
        for n in data:
            w = ins[n]
            drive.append(f"      case (t % 8) 0: {n} = 0; 1: {n} = ~0; 2: {n} = 1 << ($random & 31); "
                         f"3: {n} = {{1'b1, {{{max(w-1,1)}{{1'b0}}}}}}; default: {n} = {{$random, $random}}; endcase")
    body = "\n".join(drive) + f"\n      #1;\n{check}"
    kind = "exhaustive" if exhaustive else "random"
else:
    count = vectors
    drive = []
    for n in data:
        if n in ("rst_n", "resetn", "rst_ni"): drive.append(f"      {n} = t > 2;")
        elif n in ("rst", "reset"): drive.append(f"      {n} = t <= 2;")
        elif re.search("addr|ptr", n): drive.append(f"      {n} = $random & 7;")
        else: drive.append(f"      {n} = {{$random, $random}};")
    body = "\n".join(drive) + f"\n      #5 {clk} = 1;\n      #5 {clk} = 0;\n{check}"
    kind = "cycles"
open(f"{tmp}/tb.v", "w").write(f'''`timescale 1ns/1ps
module tb;
{decl}  integer t, errors;
  {top} rtl ({conn("rtl")});
  {top}_gate net ({conn("net")});
  initial begin
    errors = 0;
{f"    {clk} = 0;" if clk else ""}
    for (t = 0; t < {count}; t = t + 1) begin
{body}    end
    if (errors == 0) $display("PASS {count} {kind}");
    else $display("FAIL %0d mismatches in {count} {kind}", errors);
    $finish;
  end
endmodule
''')
PY
models=()
for m in $(grep -oE '\bRM_IHPSG13_[A-Za-z0-9_]+' "$TMP/netlist.v" | sort -u); do models+=("$VDIR/$m.v"); done
if [ ${#models[@]} -gt 0 ]; then
    models+=("$VDIR/RM_IHPSG13_1P_core_behavioral_bm_bist.v" "$VDIR/RM_IHPSG13_2P_core_behavioral_bm_bist_ideal.v")
fi
iverilog -g2005 -DFUNCTIONAL -I "$(dirname "$SRC")" -o "$TMP/sim" "$TMP/tb.v" "$SRC" "$TMP/netlist.v" "${models[@]}" 2> "$TMP/iverilog.log" || {
    cat "$TMP/iverilog.log"; exit 2; }
vvp -n "$TMP/sim" > "$TMP/sim.log"
grep -vE "^VCD|A_DLY" "$TMP/sim.log" | tail -n 6
grep -q "^PASS" "$TMP/sim.log"
