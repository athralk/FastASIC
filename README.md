# 1. Verilog to gate level
This is a very light weight compiler for understanding how Verilog actually translates to gates and that too NAND and INVERT - the universal gates.

Currently the repo is able to parse verilog (combinational mostly, sequential is under work) and develop optimised canonical forms of the universal gate representation for each root (forest of gate -> technology mapper).

![technology mapper](images/technology_mapper.jpg) <br>
<br>
![corresponding verilog](images/example_verilog.jpg)

## Pipeline
```
Verilog -> lexer -> parser (AST) -> elaborator (bit-level gate DAG) -> [balancing] -> NAND/INV mapping -> optimizer -> SAT equivalence check
                                                                                                  |-> tree covering onto sg13cmos5l cells (--map) -> SAT check -> cell netlist (-o)
                                                                                                  |-> static timing analysis (--sta: RC model, or Liberty tables with --delay-model nldm)
gate-level netlist (ours, Yosys, OpenLane) + .lib + .sdc -> lsta -> setup/hold/recovery/removal slack, paths, Fmax
```
- **Parser** builds a syntax tree (`include/ast.hpp`). It only checks syntax.
- **Elaborator** evaluates parameters, sizes every expression using Verilog's width rules, and *bit-blasts* vectors into one node per bit (`mat_a[255:0]` becomes `mat_a[0]` … `mat_a[255]`). Everything after it only sees single-bit gates. Each net is a single node with fanouts, so the result is a DAG. Combinational loops, undeclared nets, multiple drivers and undriven outputs are reported per bit.
- **Technology mapper** rewrites AND / OR / NOR / XOR / XNOR into NAND2 + INV, memoized so shared logic stays shared.
- **Optimizer** runs one pass over the whole netlist: structural hashing (commutative, so `NAND(a,b)` = `NAND(b,a)`), constant propagation, double-inversion removal and `x & ~x` style simplifications.
- **Equivalence checker** *proves* original vs mapped vs optimized equivalent with its own SAT solver (below) and prints a counterexample on mismatch. `--equiv-method sim` uses bit-parallel simulation instead (64 patterns per 64-bit word; exhaustive up to 20 inputs, 65,536 random patterns above, which is a sample, not a proof).

## SAT-based equivalence checking

Written from scratch, no external solver:
```
both netlists -> one AIG (structurally hashed) -> random simulation -> per output: Tseitin CNF of its cone + miter -> CDCL solver
```
- **AIG** (`include/aig.hpp`): every node is a 2-input AND and inversion is a flag on the edge, so the NAND2 + INV netlist maps onto it with inverters for free. Nodes live in flat arrays and refer to each other by index. Both designs share one structural hash table, so logic that is built the same way in both merges before any solving happens.
- **Random simulation** of the AIG (1024 patterns) catches most real differences without a SAT call.
- **Miters, incrementally**: for each output pair `x`, `y` the solver is asked "can `x XOR y` be 1?" under an *assumption*, on one solver instance that is kept alive. Only the cones of influence are encoded (Tseitin: `n = a & b` becomes `(!n | a) (!n | b) (n | !a | !b)`). Outputs go smallest cone first, and each proven pair is added back as the clauses `x <-> y`, which helps prove the larger cones built on top of it.
- **SAT sweeping**, for outputs that resist: each output is first tried with a 100-conflict budget, which settles nearly every design. Only for the hard ones are internal nodes with equal simulation signatures (up to inversion, 512 random patterns) proven equal bottom-up in their cones, each with a small budget. Each proven pair is added as clauses, so the final output proofs only cross small gaps.
  - checking a mapped 16×16 multiplier went from 58 s to under 0.5 s
  - it proves our Booth/Dadda multiplier equal to a hand-written shift-and-add one: 6×6 in 0.3 s, 8×8 in 6 s, where Yosys's `sat` gives up after 120 s
  - two multipliers that share no internal signals remain exponentially hard; that is research territory (algebraic methods)
- **CDCL solver** (`src/cdcl_solver.cpp`, MiniSat/Glucose style): two watched literals with blocker literals and a fast path for binary clauses, all clauses in one contiguous arena, first-UIP learning with clause minimization, VSIDS with phase saving, Luby restarts, LBD-based clause deletion, assumptions and failed-assumption analysis.
- **IPASIR interface** (`include/sat_solver.hpp`): the solver sits behind the standard incremental SAT API (`add`, `assume`, `solve`, `val`, `failed`), so CaDiCaL or MiniSat can be dropped in and benchmarked against it.
- Every SAT counterexample is **replayed on both gate netlists** with the simulator before it is reported, so an encoding bug shows up as an internal error, never as a wrong answer.

Why it matters: a 32-bit "backdoor" that differs on one pattern in 2^33 (`tests/equiv/fail/backdoor32`) passes 65,536 random patterns but is found by SAT in milliseconds, with the exact key. SAT is also faster than exhaustive simulation here (`example.v`: about 16 ms vs 244 ms for the whole run).

### Against Yosys and ABC
`tools/bench_equiv.sh` runs the pairs in `bench/equiv/` (generated by `tools/gen_equiv_bench.py`) through three checkers. All three got the same answer on every pair. Times below are each engine's own solve time, with all three tools fed the *same* NAND/INV netlists. Yosys is `miter -equiv` + `sat -prove-asserts` (MiniSat). ABC is `&cec` (simulation + SAT sweeping), the faster of its two modes, with a 10 ms timer resolution.

| pair | gates (AND nodes) | ours | Yosys `sat` | ABC `&cec` |
|---|---|---|---|---|
| `a + b` vs `a - ~b - 1`, 64-bit | 1,137 | 12 ms | 1232 ms | 20 ms |
| same, 256-bit | 4,593 | 169 ms | 12.5 s | **20 ms** |
| `(a+b)+c` vs `a+(b+c)`, 32-bit | 1,128 | **34 ms** | 2.5 s | 150 ms |
| same, 128-bit | 4,584 | **202 ms** | 17.9 s | 530 ms |
| `((a+b)+c)+d` vs `(a+c)+(b+d)`, 32-bit | 1,692 | **195 ms** | 14.4 s | 2740 ms |
| 64-input XOR tree vs chain | 500 | **7 ms** | 314 ms | 60 ms |
| 64-bit backdoor (not equivalent) | small | 0.4 ms | 13 ms | 10 ms |

What this shows, and what it doesn't:
- **Versus Yosys `sat`** we are 40–100x faster across the board. Yosys proves the whole miter in one call. We prove output by output on one incremental solver, smallest cone first, keeping learned clauses and adding each proven pair back as a fact. For adders, bit *k* is then easy once bits below it are proven.
- **Versus ABC** we win on reassociated adders and XOR chains, where the two sides share almost no internal nets, so sweeping finds little to merge. ABC wins clearly on `a + b` vs `a - ~b - 1`: its AIG rewriting makes the two sides almost identical before any SAT call, while our structural hashing only catches identically built logic. That is the next thing to add: AIG rewriting and SAT sweeping of internal nodes.
- These are small (under 5,000 AND nodes) arithmetic benchmarks, which suit our output-by-output strategy. ABC is built for million-gate industrial netlists with many internal equivalences, and these numbers say nothing about that regime.

## Supported Verilog
Any number of modules per file, combinational or sequential:
- ANSI or non-ANSI ports, `#(parameter ...)` lists, `parameter` / `localparam`
- `input` / `output` / `output reg` / `wire` / `reg`, scalar or vector (`[7:0]`, `[0:7]`, `[W-1:0]`), `wire w = expr;`, and memories `reg [7:0] mem [0:15]`
- continuous `assign`, including partial targets (`assign y[3:0] = ...`) and concatenated targets (`assign {cout, sum} = a + b;`)
- `always @(posedge clk)`, `always @(posedge clk or negedge rst_n)` (asynchronous reset, either polarity, to any constant), `always @(negedge clk)`, `always @(*)` / `@(a or b)`
- statements: `begin`/`end`, blocking `=` and non-blocking `<=`, `if`/`else`, `case` (with or without `default`), `for` loops over an `integer` (unrolled)
- bit selects `a[3]`, part selects `a[7:4]`, indexed part selects `a[i +: 4]` / `a[i -: 4]`; a **variable** bit index `a[i]` and memory word index `mem[addr]`, read or written (a decoder plus muxes)
- concatenation `{a, b}` and replication `{4{a}}`
- literals: `42`, `8'hFF`, `4'b10_10`, `8'o17`, `'d7`, signed `8'sd5` / `-3`, and `x`/`z`/`?` digits in `casez` / `casex` labels
- operators: `~ & | ^ ~^`, reductions `&a ~&a |a ~|a ^a ~^a`, logical `! && ||`, `== != < <= > >=`, `+ -`, `*` (between signals too), `<< >> <<< >>>` by a constant or a signal, `?:`, and `/ %` between constants
- **`signed`** ports, wires and regs, `$signed` / `$unsigned`, `$clog2`, with Verilog's rules: an expression is signed only if all its operands are; then operands are sign-extended to the context width, comparisons are signed, and `>>>` shifts in the sign bit
- **`casez` / `casex`**: wildcard label bits match anything; a case listing every value needs no `default`
- **`function`** (ANSI or classic ports, `integer`, loops, calls inside calls, signed results) and **`task`** (inputs and outputs), inlined
- **`generate`**: `for` with `genvar`, `if` / `else`, `case`, named and nested blocks, `localparam` inside. Contents get Yosys-style names (`stage[3].fa.co`).
- **preprocessor**: `` `define `` (with arguments), `` `undef ``, `` `ifdef `` / `` `ifndef `` / `` `elsif `` / `` `else `` / `` `endif ``, `` `include `` (`-I DIR`), `-DNAME=value`; `` `timescale `` and `` `default_nettype `` are skipped
- Verilog's reserved words (`small`, `and`, `event`, ...) are rejected as names, as other tools do
- **module instances**, with named or positional ports and parameter overrides `#(.W(8))` / `#(8)` (not `localparam`); `parameter [7:0] P = ...`
- **hard macros**: instances of IHP SRAMs (`RM_IHPSG13_1P_*`, `RM_IHPSG13_2P_*`), found in the PDK by name (or `--macro-lib FILE`)

Not supported yet (reported with a clear error): instance arrays, division by a signal, gate primitives (`and g (y, a, b);`), x/z values outside `casez`/`casex` labels, `initial` blocks, latches, more than one clock or reset per always block, SystemVerilog.

### Arithmetic
- **Adders, subtractors and comparators**, chosen with `--adder`:
  - `ripple`: the smallest, linear depth. This is the default.
  - `brent-kung`: parallel prefix, depth 2 log n, about 2n carry operators.
  - `sklansky`: depth log n, fanout grows with width. The default with `--map delay`.
  - `kogge-stone`: depth log n, low fanout, the largest.

  Comparators only need the carry out, so they use a balanced tree of the same carry operator. A 1-bit term of a sum becomes the carry-in instead of a second adder.
- **Multipliers:**
  - radix-4 **Booth** partial products, so half as many rows as bits
  - sign extensions replaced by inverting each row's sign bit plus one constant correction row
  - constant ones folded column by column
  - a **Dadda** tree of full and half adders down to two rows
  - a final adder in the chosen style
  - operand bits that are only sign or zero extension are not multiplied, so a 16×16 product in a 32-bit context costs a 16×16 multiplier
  - multiplying by a constant folds down to shift-and-add
- **Variable shifts**: a logarithmic barrel shifter (one 2:1 mux stage per amount bit). Amounts beyond the width shift everything out, and `>>>` fills with the sign bit when signed.

**How it is checked:**
- Every feature has a design in `tests/lang` that Yosys proves equivalent to its own reading of the RTL. That checks our *semantics* (sign rules, generate scoping, `casez` priority), not just consistency.
- Multipliers are beyond Yosys's SAT, so `tests/cosim.sh` simulates the RTL with iverilog's own operators against our netlist:
  - exhaustively when the inputs fit in 16 bits: an 8×8 multiplier, signed and unsigned, is checked on all 65,536 input pairs
  - otherwise on 20,000 random and corner-case vectors
- All four adder styles are co-simulated on multipliers, signed arithmetic and comparators. (This caught a bug in the first Brent–Kung version.)

### How registers are handled
The elaborator **symbolically executes** every always block:
- blocking assignments update values that later statements read; non-blocking ones take effect at the end of the block
- `if`/`case` become muxes, and `for` loops are unrolled
- in a combinational block, a reg that is not assigned on every path is reported as an inferred latch
- in a clocked block, an unassigned path holds the register's value, and an asynchronous reset is recognised from the standard `if (!rst_n) ... else ...` shape

The netlist is then **cut at every flip-flop**: the register's output Q is a pseudo-input named after it (`count[3]`, `mem[2][7]`), its next state a pseudo-output `count[3]$next`. The clock and reset are kept on the side. Every combinational pass (optimizer, mapping, simulation, SAT equivalence) therefore works on sequential designs unchanged. Registers that cannot affect an output are removed, as Yosys's `opt_clean` does.

On the way out:
- `-o` writes the registers back as `always` blocks
- `--map` turns them into sg13cmos5l flops:
  - `dfrbpq_1`, with RESET_B tied high when there is no reset
  - `dfrbp_1` storing the inverted value for reset-to-1 registers
  - an inverter for falling-edge clocks or active-high resets
- `--equiv` matches registers by name and compares their clocking and reset too

Checks:
- each of the 8 sequential designs in `tests/seq` is proven sequentially equivalent to its RTL by Yosys (`equiv_simple` + `equiv_induct`)
- every mapped cell netlist is SAT-checked against the RTL, next-state logic included
- lsta matches OpenSTA on all of them, including half-cycle paths into falling-edge flops

A small `reg` memory becomes flip-flops. A large one is **mapped onto an SRAM macro automatically** (below), unless `--no-sram-infer` is given.

### SRAM inference
A memory is turned into the smallest fitting PDK SRAM when that is smaller than the flip-flops it replaces, and its RTL has the shape an SRAM can implement exactly:
- exactly one write, a whole-word `mem[wa] <= d` in a rising-edge block, not in a loop
- exactly one read, a registered `q <= mem[ra]` on the same clock; `q` has no reset and is assigned nowhere else
- no other use (an asynchronous read `assign y = mem[a]` cannot come from an SRAM, whose read data is registered)
- word indices from 0, and an address that cannot point past the last word

Which macro is used comes from IHP's functional model. On a cycle with both write and read enabled, a 1-port SRAM *writes through*: it returns the new word, where the RTL's non-blocking read returns the old one. So:
- **1-port** when the read uses the same address *and our SAT solver proves the write and read conditions are never true together*; then `MEN = W | R`, `WEN = W`, `REN = R`
- **2-port** otherwise (A writes, B reads); in the functional model a same-address collision returns the old word, like the RTL. Real silicon collision behaviour should be checked in the datasheet, so the note says when they can coincide.

Every decision is reported:
```
SRAM inference: memory 'mem' (1024 x 8) -> RM_IHPSG13_1P_1024x8_c2_bm_bist (1-port), 49419 um^2 instead of about 401316 um^2 of flip-flops
SRAM inference: memory 'big' (512 x 8) kept as flip-flops: it is read asynchronously (an SRAM's read data is registered: use q <= big[addr])
```
The memory's accesses are recorded during symbolic execution, together with the path condition under which each one runs. They become the macro's enables, address and data pins. Every pass then treats the macro as a black box, as for an instantiated one.

**Check:** our SAT checks compare against the design *after* inference, so the inference itself is checked by **co-simulation** (`tests/cosim_sram.sh`):
- iverilog runs the original RTL (memory as flip-flops) next to our gate netlist, with the SRAM simulated by **IHP's own functional model**
- 4000 random cycles, comparing every output after every clock edge in 4-state logic
- address inputs are kept to a few words, so reads hit written data
- all inferred designs in `tests/infer` match, and planted bugs (a read enable forced high, the FIFO writing at its read pointer) give thousands of mismatches

The mapped 1024×8 RAM is 5 cells; lsta and OpenSTA agree on it endpoint by endpoint.

### Hierarchy
The design is flattened during elaboration, as Yosys's `flatten` does:
- the top is the one module nobody instantiates (or `--top`)
- every instance is elaborated with its own parameter values, and its contents are named `instance.signal` (`u_core.u_alu.sum[3]`), Yosys's convention, so the two tools' netlists can be compared by name
- an instance's ports become nets: inputs are driven by the parent's expressions (truncated or zero-extended to the port width), outputs drive the parent's wires
- every register's clock and reset is traced back through the port connections to a top-level input; gated or derived clocks are reported, and a reset tied to its inactive level means no reset
- written netlists use escaped identifiers for the hierarchical names (`\u_core.acc [3]`), which Yosys and OpenSTA read

The designs in `tests/hier` (ripple adder, parameterized registers three levels deep, a small CPU datapath with two register files) are each proven equivalent to their RTL by Yosys (`hierarchy; flatten; equiv_*`), SAT-checked after mapping, and timed identically by lsta and OpenSTA.

### Hard macros (SRAM)
The PDK ships 30 SRAM macros, single and dual port, 64×16 to 8192×32, with typ/fast/slow `.lib` files. In this PDK `sg13cmos5l_sram` is a link to the SG13G2 SRAMs. Instantiate one like any module:
```verilog
RM_IHPSG13_1P_1024x8_c2_bm_bist u_mem (.A_CLK(clk), .A_MEN(en), .A_WEN(we), .A_REN(re),
    .A_ADDR(addr), .A_DIN(wdata), .A_DOUT(rdata), .A_BM(8'hFF), .A_DLY(1'b1), /* BIST pins tied to 0 */ ...);
```
- **Elaboration:** a module that is defined nowhere is looked up as `<name>_<corner>*.lib` among the PDK's SRAMs. Its ports come from the Liberty `bus()`/`pin()` groups, and the instance becomes a black box. Like a register, it cuts the netlist: each output bit is a pseudo-input (`u_mem.A_DOUT[3]`), each input bit a pseudo-output (`u_mem.A_ADDR[3]$in`). The SAT checker therefore proves every macro pin connection along with the logic, and every pass keeps working.
- **Mapping:** `--map` places the macro cell next to the standard cells. Bus pins are written as concatenations (`.A_ADDR({...})`), which lsta's netlist reader and OpenSTA both accept.
- **Timing:** lsta reads the macro libraries:
  - Liberty `bus`/`bus_type`, and timing on whole bit ranges (`pin(A_ADDR[9:0])`)
  - the `memory()` group
  - attributes missing their `;`
  - slews converted between libraries with different slew thresholds (the SRAMs measure 30–70% with derate 0.5, the standard cells 20–80%), exactly as OpenSTA's `thresholdAdjust` describes
- **Formal check:** `tests/yosys_equiv.sh` gives Yosys a stand-in model of each macro (`tests/macro_stub.py`: every output a distinct XOR mix of all inputs). The Yosys proof therefore also catches a swapped or miswired macro input; a plain Liberty black box would not.

Against OpenSTA on the two macro designs in `tests/macro`:
- WNS, TNS and every hold slack agree.
- A few setup endpoints at macro pins differ by up to 47 ps. These are traced to **OpenSTA 2.6.0 converting the slew again for each additional load from another library on the same net**: on one net the first SRAM pin gets ×4/3 and the second ×(4/3)², while standard-cell loads are unchanged, so the result depends on load order. lsta converts each load once.

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

## Gate-level static timing: `lsta`
`lsta` is a standalone timer for flat gate-level netlists: what Yosys or OpenLane write after synthesis, and what our own `--map` writes. It is checked endpoint by endpoint against **OpenSTA**, the timer inside OpenLane/OpenROAD.
```
./lsta -sdc constraints.sdc netlist.v              # IHP sg13cmos5l typ corner by default
./lsta -corner slow -sdc c.sdc -paths 5 netlist.v   # 5 worst setup and hold paths
./lsta -sdc c.sdc -cts sg13cmos5l_buf_8 -write with_tree.v netlist.v
```
- **Liberty reader** (`src/liberty.cpp`): parses the full 1.7 MB `.lib` (84 cells, 384 arcs) in about 20 ms, and the SRAM macro libraries (buses, memory groups). It reads the timing tables in all three corners (NLDM: delay and output slew as 2D tables of input slew × load), setup/hold/recovery/removal tables, pin capacitance ranges, functions and flip-flop definitions.
- **Timing engine** (`src/timer.cpp`), on a flat array-based graph:
  - rise/fall arrival and slew at every pin, late (max) and early (min)
  - delay looked up in the table at (input slew, load), with OpenSTA's interpolation and extrapolation
  - flip-flops launch through clock→Q arcs and are checked with setup/hold tables at (data slew, clock slew); async resets get recovery/removal checks
  - reports WNS/TNS, the worst paths and Fmax
- **SDC subset** (`src/sdc.cpp`): `create_clock`, `set_input/output_delay`, `set_load`, `set_driving_cell`, `set_input_transition`, `set_clock_uncertainty`/`transition`/`latency`, `set_propagated_clock`, plus `get_ports`/`all_inputs`/`all_outputs`, Tcl variables and `[expr]`.
- **Clocks**: ideal (latency + transition), or propagated through a real clock tree with skew and CRPR (clock reconvergence pessimism removal). `-cts BUF` builds a balanced buffer tree first (`src/clock_tree.cpp`).

The OpenSTA behaviours that had to be matched exactly to agree to the picosecond:
- the upper bound of each pin's capacitance range for late analysis, the lower bound for early
- a driving cell contributes only its load-dependent delay (intrinsic delay is part of the input delay) but its full slew
- reset→Q (`clear`/`preset`) arcs are not data paths
- a propagated clock starts from the port's input transition, not the clock transition
- CRPR credit taken at the point where the launch and capture clock paths split

**Against OpenSTA 2.6.0** (`tools/compare_opensta.sh`, Yosys-synthesised netlists, every endpoint compared):

| design | cells | corner, clock | endpoints | max slack difference | lsta graph + analysis | OpenSTA analysis |
|---|---|---|---|---|---|---|
| acc16 (async reset) | 89 | typ / slow, ideal | 96 | 0.005 ps | 0.2 ms | 5 ms |
| mul16 | 1,480 | typ / slow, ideal | 320 | 0.005 ps | 2 ms | 37 ms |
| mul16 + clock tree | 1,493 | typ, propagated | 320 | setup 0.005 ps, hold ≤ 4 ps on 2 endpoints | 3 ms | 42 ms |
| mac32 | 6,490 | typ / slow, ideal | 256 | 0.007 ps | 11 ms | 156 ms |
| add64 | 344 | fast, virtual clock | 130 | 0.006 ps | 0.6 ms | 10 ms |
| 8 × mac32 | 51,920 | typ, ideal | 2,048 | 0.006 ps | **150 ms** | **1,944 ms** |

On the 52k-cell netlist the whole `lsta` process (Liberty + netlist + analysis) takes 0.5 s and 92 MB, against OpenSTA's 2.6 s of internal time, not counting docker. The remaining hold difference with propagated clocks comes from CRPR: lsta applies it to the earliest-arriving path, while OpenSTA tracks clock tags so it can find a slower launch path with less credit. `tests/lsta/opensta_golden.txt` freezes OpenSTA's numbers, and the test suite checks lsta against them.

Clock edges are tracked through the clock network, so inverted clocks and falling-edge registers work (rising-to-falling paths get half a cycle, as in OpenSTA). Not modelled yet: wire RC (SPEF), multiple clocks, false/multicycle paths, latches, on-chip variation, signal integrity, CCS current-source models.

## Synthesis onto real cells: `--map`
```
./output_test -q --map area  -o cells.v design.v
./output_test -q --map delay --sta --delay-model nldm --sdc c.sdc design.v
```
1. **Arithmetic** (elaborator): a sum `a + b + c ...` is built as one carry chain per pair of wide terms, and a 1-bit term becomes the carry-in instead of a second adder, as Yosys's `alumacc` does. This alone made `a + b + cin` 30% smaller.
2. **Balancing** (`src/balance.cpp`): AND/OR/XOR chains are rebuilt as depth-balanced trees, two shallowest operands first. A 22-input parity chain goes from 21 levels to 5.
3. **NAND2/INV mapping, then optimization** (structural hashing, constant propagation).
4. **Cut-based mapping with Boolean matching** (`src/cut_mapper.cpp`, the default; the approach of ABC's mappers):
   - The logic becomes an AIG.
   - Every node gets its **cuts**, sets of up to 6 leaves that determine it, each with the **truth table** of the node over those leaves (64-bit words). Only the 8 cheapest ("priority cuts") are kept for the nodes above.
   - Every library cell's Liberty function is compiled and evaluated under **all input orders and input inversions** (720 × 64 variants for `mux4`), into a table from (leaf count, truth table) to the smallest and fastest cell. A cut therefore matches a cell whenever the *functions* agree, however the logic happens to be built. Tri-state cells are excluded.
   - Dynamic programming chooses, for each node and each polarity, the best cut and cell. The other polarity falls back to an inverter.
   - **Area mode:** area flow (shared logic charged in proportion to its fanout), repeated with the cover's real fanouts, then two passes of **exact area recovery**: each used node's implementation is freed, every alternative is costed by reference counting what it would really add, and the cheapest is kept. This stops shared logic (e.g. an adder's `a ^ b`) from being duplicated.
   - **Delay mode:** minimum estimated arrival (Liberty delay at a typical slew and load), then the same exact area recovery, accepting only options that still meet each node's required time. The critical path stays as fast while the rest of the logic shrinks.
   - `--mapper tree` selects the older tree covering (`src/tech_mapper.cpp`, Keutzer's DAGON, as in Cornell ECE 6745).
5. The cell netlist is **SAT-checked against the original RTL** (each cell's Liberty `function` is turned back into gates), and written as structural Verilog that OpenSTA reads. lsta and OpenSTA agree on it to 0.01 ps.

**Against Yosys + ABC** (`tools/compare_synth.sh`, both timed by lsta with the same SDC; area in um², min period in ps):

| design | ours `--map area` | ours `--map delay` | Yosys + ABC | ours runtime | Yosys + ABC runtime |
|---|---|---|---|---|---|
| full_adder | 50.8, 1238 ps | 50.8, 1238 ps | 50.8, 1268 ps | 38 ms | 539 ms |
| mux4 | 163, 1426 ps | 374, 1409 ps | 152, 1270 ps | 44 ms | 663 ms |
| adder8 | 443, 2303 ps | 515, **2007 ps** | 412, 2116 ps | 47 ms | 666 ms |
| alu4 | 894, 2246 ps | 1101, 2104 ps | 786, 1961 ps | 52 ms | 1031 ms |
| compare12 | 1030, 2833 ps | 1143, **2486 ps** | 880, 2590 ps | 47 ms | 970 ms |
| parity22 | 310, 1566 ps | 310, **1566 ps** | 305, 1581 ps | 120 ms | 929 ms |
| all_operators | **74**, 1289 ps | 74, 1223 ps | 78, 1148 ps | 38 ms | 605 ms |
| ripple_adder4 | 218, 1695 ps | 240, **1599 ps** | 207, 1604 ps | 39 ms | 584 ms |
| add64 | 3593, 10823 ps | 4379, 8513 ps | 3440, 8086 ps | 102 ms | 1565 ms |

- **Area mode:** over these 9 designs we are 7% above ABC in total (6,777 vs 6,310 um²), and smaller on `all_operators`. With the old tree covering it was 36%.
- **Delay mode:** we are faster than ABC on 5 of the 9 designs.
- **Runtime:** our flow is 8–20× faster.
- **What is left:** ABC also rewrites the AIG before mapping and maps over several equivalent structures at once (`dch`, "structural choices"), which helps most on control logic (`alu4`, `compare12`). That is the next step here, using the SAT sweeping the equivalence checker already needs.

## Usage
To run, use it in Linux:
1. `make`
2. `./output_test example.v`

basically -> `./output_test [-q] [-o netlist.v] [--sta] <verilog_file>.v`
- `-q` prints only gate counts and equivalence results (useful for wide designs)
- `--equiv-method sat|sim` picks how the pipeline's equivalence checks are done (default `sat`)
- `--top MODULE`, `--macro-lib FILE`, `--no-sram-infer`: hierarchy top, extra macro libraries, keep memories as flip-flops
- `-o netlist.v` writes the optimized NAND + INV netlist as structural Verilog with the same ports as the source
- `--sta` runs static timing analysis, with `--corner typ|fast|slow`, `--delay-model rc|unit|nldm`, `--period PS`, `--input-delay PS`, `--output-delay PS`, `--load FF`; `nldm` maps onto real cells and uses the Liberty timer (`--sdc FILE` for constraints)
- `--map area|delay` maps onto real sg13cmos5l cells (`--lib FILE` for another library, `--mapper cut|tree` for the algorithm); `-o` then writes the cell netlist
- `./lsta` times any flat gate-level netlist (see above); `tools/synth_yosys.sh` makes one from RTL with Yosys

To check whether two designs implement the same logic (outputs are matched by name, bit by bit):
```
./output_test --equiv first.v second.v
```
```
PASS (SAT proof, 32 outputs: 1 structural, 31 by SAT, 466 AIG ANDs, 411 conflicts, 4.3 ms)
```
"structural" outputs were merged by the AIG's hashing; "by SAT" ones needed an UNSAT miter.
Exit codes: `0` success / equivalent, `1` not equivalent, `2` parse or usage error.

## Tests
```
make test
```
- Runs every design in `tests/cases` through the full pipeline.
- Checks that malformed or unsupported designs in `tests/errors` are rejected with the right message.
- Checks known equivalent / different pairs in `tests/equiv`, with both the SAT prover and simulation.
- Fuzzes the SAT solver against brute force (`tests/sat_fuzz.cpp`): 2000 random formulas, 4000 incremental solves with random assumptions (every failed-assumption set must itself be unsatisfiable), pigeonhole formulas up to 9 into 8, and random 3-SAT at the phase transition with every model checked.
- Checks timing reports in `tests/sta` against hand-calculated values written in each file.
- Checks the STA engine against brute-force path enumeration (`tests/sta_bruteforce.cpp`): arrival and required times must agree at every node and edge, at all three corners.
- If `yosys` is installed, **formally proves** each design's generated netlist equivalent to Yosys's reading of the original RTL (`tests/yosys_equiv.sh`, using a SAT miter). This is an independent oracle: it catches mistakes in how we interpret Verilog, not just in mapping.

- With the IHP PDK installed: lsta against the frozen OpenSTA results (9 design/corner/clock cases), mapping of every design in both modes with a SAT proof against the RTL, and a netlist write/read round trip.

CI runs the same suite, Yosys proofs included, on every push.
