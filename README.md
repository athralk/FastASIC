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
`--sta` times the optimized netlist with real cells from IHP's open **SG13CMOS5L** process (130 nm, CMOS only): every NAND becomes `sg13cmos5l_nand2_1` (first input on pin A, second on pin B) and every inverter `sg13cmos5l_inv_1`.

| | |
|---|---|
| PDK | `ihp-sg13cmos5l`, IHP-Open-PDK commit `2bbec755dc67ca3db0261c3d6163e15735d66710` (the one Tiny Tapeout's CI uses) |
| Standard-cell library | `sg13cmos5l_stdcell` rev 0.1.4 |
| Timing corners | `sg13cmos5l_stdcell_typ_1p20V_25C.lib` (sign-off, default), `..._fast_1p32V_m40C.lib`, `..._slow_1p08V_125C.lib` |

**Delay model (RC):** each timing arc's delay is `t = R_eff·C_par + R_eff·C_load`, the switch-level model from ECE 6745.
- `C_load` is the sum of the input-pin capacitances the net drives, plus the output load on primary outputs.
- `R_eff` and `R_eff·C_par` are not in the Liberty file, which only stores delay tables. They are fitted from its NLDM tables (`tools/fit_rc_from_liberty.py`), and rise and fall get separate values:

| Arc | R_eff rise / fall (kΩ) | R_eff·C_par rise / fall (ps) | Input pin cap (fF) |
|---|---|---|---|
| inv_1 A→Y | 2.768 / 2.211 | 19.59 / 19.70 | A 2.867 |
| nand2_1 A→Y | 2.768 / 4.081 | 21.59 / 28.06 | A 2.886 |
| nand2_1 B→Y | 2.768 / 4.080 | 25.11 / 32.35 | B 2.978 |

Those are the typ values. `--corner fast|slow` switches every constant, including the pin capacitances, to that corner's fit. The spread is large: the 8-bit adder's critical path is **865 ps fast, 1173 ps typ, 1715 ps slow**. Setup (max-delay) sign-off normally looks at slow, and hold (min-delay) at fast, once there are flip-flops.

| inv_1 | fast 1.32 V −40 °C | typ 1.20 V 25 °C | slow 1.08 V 125 °C |
|---|---|---|---|
| R_eff rise / fall (kΩ) | 1.762 / 1.482 | 2.768 / 2.211 | 4.331 / 3.685 |
| Input cap (fF) | 3.058 | 2.867 | 2.703 |

In this library the NAND's pull-down (two NMOS in series) is the weak side, so falling outputs are usually the critical ones.

The fit is within about 2 ps of the tables at the sharpest characterized input edge (19 ps). It ignores input slew, so it underestimates by roughly 45 ps per gate at a 97 ps input slew. Moving to the full table lookup (NLDM) is the next step.

**Analysis:**
- A forward pass over the gates in topological order computes the latest arrival time of every net, separately for rising and falling transitions. Both cells invert, so a rising output is caused by a falling input.
- A backward pass computes required times.
- The report lists the critical path stage by stage and every output's slack.
- `--delay-model unit` costs every cell 1, so the critical path delay becomes the logic depth.

Constraints default to a 10 ns clock and 6.0 fF on every output (`OUTPUT_CAP_LOAD` from the PDK's LibreLane config, `libs.tech/librelane/sg13cmos5l_stdcell/config.tcl`).

```
./output_test -q --sta tests/cases/adder8.v
Critical path: b[1] -> cout (23 cells)
       Delay     Arrival  Edge  Load fF  Pin
        0.00        0.00     ^     5.86  b[1] (input)
       68.42       68.42     v     8.84  _g7015_ sg13cmos5l_nand2_1 B->Y
       ...
       56.27      260.26     v     5.86  _g7038_ sg13cmos5l_nand2_1 B->Y      <- 2 NANDs (~106 ps) per bit of carry ripple
       49.84      310.10     ^     8.93  _g7039_ sg13cmos5l_nand2_1 B->Y
       ...
       56.82     1172.96     v     6.00  _g7151_ sg13cmos5l_nand2_1 B->Y
  data arrival time        1172.96
```
Gate names match the `-o` netlist, so a path can be followed in the written Verilog.

## Usage
To run, use it in Linux:
1. `make`
2. `./output_test example.v`

basically -> `./output_test [-q] [-o netlist.v] [--sta] <verilog_file>.v`
- `-q` prints only gate counts and equivalence results (useful for wide designs)
- `-o netlist.v` writes the optimized NAND + INV netlist as structural Verilog with the same ports as the source
- `--sta` runs static timing analysis, with `--corner typ|fast|slow`, `--delay-model rc|unit`, `--period PS`, `--input-delay PS`, `--output-delay PS`, `--load FF`

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
- Checks the STA engine against brute-force path enumeration (`tests/sta_bruteforce.cpp`): arrival and required times must agree at every node and edge, at all three corners.
- If `yosys` is installed, **formally proves** each design's generated netlist equivalent to Yosys's reading of the original RTL (`tests/yosys_equiv.sh`, using a SAT miter). This is an independent oracle: it catches mistakes in how we interpret Verilog, not just in mapping.

CI runs the same suite, Yosys proofs included, on every push.
