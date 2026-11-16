#!/usr/bin/env python3
"""Builds a larger flat netlist from a Yosys one: N copies sharing the inputs, each with its own
outputs (out -> out_0 .. out_{N-1}). Internal nets and instances (_123_) get a copy prefix.

usage: tools/replicate_netlist.py <netlist.v> <copies> > big.v"""
import re
import sys

src = open(sys.argv[1]).read()
n = int(sys.argv[2])
m = re.search(r'module\s+(\w+)\s*\((.*?)\);(.*?)endmodule', src, re.S)
name, body = m.group(1), m.group(3)
inputs, outputs, rest = [], [], []
for stmt in body.split(';'):
    s = stmt.strip()
    if not s:
        continue
    d = re.match(r'(input|output)\s*(\[[^\]]*\])?\s*(\w+)$', s)
    if d:
        (inputs if d.group(1) == 'input' else outputs).append((d.group(2) or '', d.group(3)))
        continue
    if re.match(r'wire\s*(\[[^\]]*\])?\s*(\w+)$', s) and not re.match(r'wire\s+_\d+_$', s):
        continue   # port wires are redeclared below
    rest.append(s)
outs = {o for _, o in outputs}
lines = []
for k in range(n):
    def ren(t):
        w = t.group(0)
        if re.fullmatch(r'_\d+_', w):
            return f'_c{k}{w}'
        if w in outs:
            return f'{w}_{k}'
        return w
    for s in rest:
        lines.append('  ' + re.sub(r'\b[A-Za-z_]\w*\b', ren, s) + ';')
ports = [p for _, p in inputs] + [f'{o}_{k}' for k in range(n) for _, o in outputs]
print(f'module {name}_x{n}({", ".join(ports)});')
for w, p in inputs:
    print(f'  input {w} {p};')
for k in range(n):
    for w, o in outputs:
        print(f'  output {w} {o}_{k};')
print('\n'.join(lines))
print('endmodule')
