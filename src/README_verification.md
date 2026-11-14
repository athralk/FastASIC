# Step 3: Verification (proving the result is correct)

Files: `aig.cpp`, `cdcl_solver.cpp`, `cec.cpp`, `simulator.cpp`.
Test scripts: `tests/yosys_equiv.sh`, `tests/cosim.sh`, `tests/cosim_sram.sh`.

Every run of `FastASIC` proves each step's netlist equivalent to the original Verilog. `--equiv a.v b.v` compares two designs.

## The SAT solver (`cdcl_solver.cpp`)

A CDCL solver in the style of MiniSat and Glucose:
- clause learning and VSIDS branching
- restarts and clause deletion
- incremental solving with assumptions

It sits behind the standard IPASIR interface (`include/sat_solver.hpp`), so another solver such as CaDiCaL could be plugged in. `tests/sat_fuzz.cpp` checks it against brute force on thousands of random formulas.

## The equivalence checker (`cec.cpp`)

1. Both designs go into one AIG (`aig.cpp`). Logic that is built the same way in both merges automatically.
2. A quick random simulation catches most real differences without any proof.
3. Each output pair is asked "can these two ever differ?" by the SAT solver, smallest first. Every proven pair helps the next proof.
4. **SAT sweeping**, only for outputs that resist a quick proof: matching signals inside both designs are proven equal first. This turns one huge proof into many small ones; a mapped 16 × 16 multiplier checks in under half a second instead of a minute.
5. Every counterexample is replayed in the simulator before it is reported, so a bug in the checker shows up as an error, never as a wrong answer.

Registers and macros are matched by name. Their clocks and resets are compared separately.

## The simulator (`simulator.cpp`)

It simulates 64 input patterns at once, one per bit of a 64-bit word. `--equiv-method sim` uses it instead of SAT: exhaustive up to 20 inputs, random beyond that.

## Independent checks in the tests

- **`tests/yosys_equiv.sh`:** Yosys proves our netlist equivalent to *its own* reading of the Verilog. This catches mistakes in how we interpret the language. Sequential designs use Yosys's induction-based check, and SRAM macros get a stand-in model so their wiring is checked too.
- **`tests/cosim.sh`:** iverilog simulates the Verilog against our netlist, exhaustively when the inputs are small. It covers multipliers, which are too hard for any SAT solver to prove, and SRAMs, using IHP's own model.
