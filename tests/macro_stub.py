#!/usr/bin/env python3
"""Writes a stand-in Verilog model for a hard macro, for formal checks of how a design is
wired to it. Each output bit is the XOR of a different pseudo-random subset of all input
bits (every input column distinct), so any swapped, missing or miswired input changes the
function. The ports come from the PDK's Verilog model of the macro.
usage: macro_stub.py <macro name> <pdk verilog dir>   (writes the module to stdout)"""
import random, re, sys

name, vdir = sys.argv[1], sys.argv[2]
text = open(f"{vdir}/{name}.v").read()
body = text[text.index(f"module {name}"):]
body = body[:body.index("endmodule")]
ports = []
for m in re.finditer(r"^\s*(input|output)\s*(\[(\d+):(\d+)\])?\s*(\w+)\s*;", body, re.M):
    ports.append((m.group(1), int(m.group(3) or 0), int(m.group(4) or 0), m.group(5), m.group(2) is not None))
ins = [(p, i) for d, hi, lo, p, vec in ports if d == "input" for i in (range(lo, hi + 1) if vec else [None])]
outs = [(p, hi, lo, vec) for d, hi, lo, p, vec in ports if d == "output"]
bit = lambda p, i: p if i is None else f"{p}[{i}]"
rng = random.Random(name)
print(f"module {name} (" + ", ".join(p for _, _, _, p, _ in ports) + ");")
for d, hi, lo, p, vec in ports:
    print(f"  {d} " + (f"[{hi}:{lo}] " if vec else "") + f"{p};")
out_bits = [bit(p, i) for p, hi, lo, vec in outs for i in (range(lo, hi + 1) if vec else [None])]
# distinct non-zero column per input bit (so no two inputs can be swapped unnoticed)
cols, seen = [], set()
while len(cols) < len(ins):
    c = rng.getrandbits(len(out_bits)) or 1
    if c not in seen:
        seen.add(c)
        cols.append(c)
for k, o in enumerate(out_bits):
    terms = [bit(p, i) for (p, i), c in zip(ins, cols) if (c >> k) & 1]
    print(f"  assign {o} = " + (" ^ ".join(terms) if terms else "1'b0") + ";")
print("endmodule")
