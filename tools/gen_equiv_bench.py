#!/usr/bin/env python3
"""Writes the equivalence-checking benchmark pairs in bench/equiv/ (name.a.v, name.b.v).

Each pair is chosen to stress a different part of a checker:
  addN_vs_sub     a + b vs a - ~b - 1: same function, different adder structure
  add3_reassocN   (a + b) + c vs a + (b + c): reassociated adders, no internal net matches
  add4_reassocN   ((a + b) + c) + d vs (a + c) + (b + d)
  ltN_vs_sub      a < b vs the borrow out of a - b
  parityN_chain   reduction XOR tree vs a linear XOR chain: XOR is hard for plain CDCL
  backdoorN       differs on a single input pattern (expected: NOT equivalent)
usage: tools/gen_equiv_bench.py [out_dir]
"""
import os
import sys

out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "bench", "equiv")
os.makedirs(out, exist_ok=True)


def pair(name, a, b):
    for suffix, body in (("a", a), ("b", b)):
        with open(os.path.join(out, f"{name}.{suffix}.v"), "w") as f:
            f.write(body.strip() + "\n")


for n in (32, 64, 256):
    ports = f"module m (input [{n-1}:0] a, b, output [{n-1}:0] y);"
    pair(f"add{n}_vs_sub", f"{ports}\n    assign y = a + b;\nendmodule",
         f"{ports}\n    assign y = a - ~b - 1;\nendmodule")

for n in (8, 16, 32, 128):
    ports = f"module m (input [{n-1}:0] a, b, c, output [{n-1}:0] y);"
    pair(f"add3_reassoc{n}", f"{ports}\n    assign y = (a + b) + c;\nendmodule",
         f"{ports}\n    assign y = a + (b + c);\nendmodule")

for n in (16, 32):
    ports = f"module m (input [{n-1}:0] a, b, c, d, output [{n-1}:0] y);"
    pair(f"add4_reassoc{n}", f"{ports}\n    assign y = ((a + b) + c) + d;\nendmodule",
         f"{ports}\n    assign y = (a + c) + (b + d);\nendmodule")

for n in (32, 64):
    ports = f"module m (input [{n-1}:0] a, b, output y);"
    pair(f"lt{n}_vs_sub", f"{ports}\n    assign y = a < b;\nendmodule",
         f"{ports}\n    wire [{n}:0] d = {{1'b0, a}} - {{1'b0, b}};\n    assign y = d[{n}];\nendmodule")

for n in (16, 32, 64):
    chain = f"a[0]"
    for i in range(1, n):
        chain = f"({chain} ^ a[{i}])"
    ports = f"module m (input [{n-1}:0] a, output y);"
    pair(f"parity{n}_chain", f"{ports}\n    assign y = ^a;\nendmodule",
         f"{ports}\n    assign y = {chain};\nendmodule")

for n, key in ((32, "32'hDEADBEEF"), (64, "64'hDEADBEEFCAFEF00D")):
    ports = f"module m (input [{n-1}:0] key, input x, output y);"
    pair(f"backdoor{n}", f"{ports}\n    assign y = x ^ (key == {key});\nendmodule",
         f"{ports}\n    assign y = x;\nendmodule")
