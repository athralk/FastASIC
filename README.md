# 1. Verilog to gate level
This is a very light weight compiler for understanding how Verilog actually translates to gates and that too NAND and INVERT - the universal gates.

Currently the repo is able to parse verilog (combinational mostly, sequential is under work) and develop optimised canonical forms of the universal gate representation for each root (forest of gate -> technology mapper).

![technology mapper](images/technology_mapper.jpg) <br>
<br>
![corresponding verilog](images/example_verilog.jpg)

## Pipeline
```
Verilog -> lexer -> parser (AST) -> elaborator (bit-level gate DAG) -> NAND/INV mapping -> optimizer -> equivalence check
                                                                                                  |-> structural Verilog (-o)
                                                                                                  \-> static timing analysis (--sta)
```
- **Parser** builds a syntax tree (`include/ast.hpp`). It only checks syntax.
- **Elaborator** evaluates parameters, sizes every expression using Verilog's width rules, and *bit-blasts* vectors into one node per bit (`mat_a[255:0]` becomes `mat_a[0]` … `mat_a[255]`). Everything after it only sees single-bit gates. Each net is a single node with fanouts, so the result is a DAG. Combinational loops, undeclared nets, multiple drivers and undriven outputs are reported per bit.
- **Technology mapper** rewrites AND / OR / NOR / XOR / XNOR into NAND2 + INV, memoized so shared logic stays shared.
- **Optimizer** runs one pass over the whole netlist: structural hashing (commutative, so `NAND(a,b)` = `NAND(b,a)`), constant propagation, double-inversion removal and `x & ~x` style simplifications.
- **Equivalence checker** simulates 64 patterns at a time (one per bit of a 64-bit word). Designs with up to 20 inputs are checked exhaustively; larger ones with 65,536 random patterns. Every run checks original vs mapped vs optimized and prints a counterexample on mismatch.

## Supported Verilog
A single combinational module:
- ANSI or non-ANSI ports, `#(parameter ...)` lists, `parameter` / `localparam`
- `input` / `output` / `wire`, scalar or vector (`[7:0]`, `[0:7]`, `[W-1:0]`), and `wire w = expr;`
- continuous `assign`, including partial targets (`assign y[3:0] = ...`) and concatenated targets (`assign {cout, sum} = a + b;`)
- bit selects `a[3]`, part selects `a[7:4]`, indexed part selects `a[i +: 4]` / `a[i -: 4]` (constant indices)
- concatenation `{a, b}` and replication `{4{a}}`
- literals: `42`, `8'hFF`, `4'b10_10`, `8'o17`, `'d7`
- operators: `~ & | ^ ~^`, reductions `&a ~&a |a ~|a ^a ~^a`, logical `! && ||`, `== != < <= > >=`, `+ -` (ripple-carry), `<< >>` by a constant, `?:`, and `* / %` between constants
- `` `timescale `` and `` `default_nettype `` (skipped)

Not supported yet (reported with a clear error): `always` blocks / `reg`, module instances, variable indices and shifts, multiplication of signals, signed arithmetic, x/z values, `` `define ``.

## Static timing analysis
`--sta` times the optimized netlist with real cells from the SkyWater sky130 library: every NAND becomes `sky130_fd_sc_hd__nand2_1` (first input on pin A, second on pin B) and every inverter `sky130_fd_sc_hd__inv_1`, using the typical corner (25 °C, 1.8 V).

**Delay model (RC):** each timing arc's delay is `t = R_eff·C_par + R_eff·C_load`, the switch-level model from ECE 6745.
- `C_load` is the sum of the input-pin capacitances the net drives, plus the output load on primary outputs.
- `R_eff` and `R_eff·C_par` are not in the Liberty file, which only stores delay tables. They are fitted from its NLDM tables (`tools/fit_rc_from_liberty.py`), and rise and fall get separate values:

| Arc | R_eff rise / fall (kΩ) | R_eff·C_par rise / fall (ps) | Input pin cap (fF) |
|---|---|---|---|
| inv_1 A→Y | 5.627 / 3.330 | 18.78 / 13.32 | A 2.302 |
| nand2_1 A→Y | 6.167 / 5.065 | 22.41 / 18.42 | A 2.315 |
| nand2_1 B→Y | 6.120 / 5.040 | 29.27 / 23.16 | B 2.324 |

The fit is within about 1.5 ps of the tables for a sharp (10 ps) input edge. It ignores input slew, so it underestimates by roughly 15–50 ps per gate at the 50–120 ps slews typical inside a circuit. Moving to the full table lookup (NLDM) is the next step.

**Analysis:**
- A forward pass over the gates in topological order computes the latest arrival time of every net, separately for rising and falling transitions. Both cells invert, so a rising output is caused by a falling input.
- A backward pass computes required times.
- The report lists the critical path stage by stage and every output's slack.
- `--delay-model unit` costs every cell 1, so the critical path delay becomes the logic depth.

Constraints default to OpenLane's sky130 flow: a 10 ns clock, and 33.442 fF on every output (the input capacitance of an `inv_16`).

```
./output_test -q --sta tests/cases/adder8.v
Critical path: b[1] -> cout (23 cells)
       Delay     Arrival  Edge  Load fF  Pin
        0.00        0.00     v     4.64  b[1] (input)
       71.88       71.88     ^     6.96  _g7015_ nand2_1 B->Y
       ...
       57.66      275.74     ^     4.64  _g7038_ nand2_1 B->Y      <- 2 NANDs (~116 ps) per bit of carry ripple
       58.29      334.03     v     6.97  _g7039_ nand2_1 B->Y
       ...
      233.92     1424.29     ^    33.44  _g7151_ nand2_1 B->Y
  data arrival time        1424.29
```
Gate names match the `-o` netlist, so a path can be followed in the written Verilog.

## Usage
To run, use it in Linux:
1. `make`
2. `./output_test example.v`

basically -> `./output_test [-q] [-o netlist.v] [--sta] <verilog_file>.v`
- `-q` prints only gate counts and equivalence results (useful for wide designs)
- `-o netlist.v` writes the optimized NAND + INV netlist as structural Verilog with the same ports as the source
- `--sta` runs static timing analysis, with `--delay-model rc|unit`, `--period PS`, `--input-delay PS`, `--output-delay PS`, `--load FF`

To check whether two designs implement the same logic (outputs are matched by name, bit by bit):
```
./output_test --equiv first.v second.v
```
Exit codes: `0` success / equivalent, `1` not equivalent, `2` parse or usage error.

## Tests
```
make test
```
- Runs every design in `tests/cases` through the full pipeline.
- Checks that malformed or unsupported designs in `tests/errors` are rejected with the right message.
- Checks known equivalent / different pairs in `tests/equiv`.
- Checks timing reports in `tests/sta` against hand-calculated values written in each file.
- Checks the STA engine against brute-force path enumeration (`tests/sta_bruteforce.cpp`): arrival and required times must agree at every node and edge.
- If `yosys` is installed, **formally proves** each design's generated netlist equivalent to Yosys's reading of the original RTL (`tests/yosys_equiv.sh`, using a SAT miter). This is an independent oracle: it catches mistakes in how we interpret Verilog, not just in mapping.

CI runs the same suite, Yosys proofs included, on every push.
