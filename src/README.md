# How FastASIC works

FastASIC follows the usual ASIC flow. Each step has its own document:

| Step | What happens | Document |
|---|---|---|
| 1. Front end | Verilog text becomes a bit-level gate netlist | [README_frontend.md](README_frontend.md) |
| 2. Synthesis | the netlist is optimized and mapped onto real sg13cmos5l cells | [README_synthesis.md](README_synthesis.md) |
| 3. Verification | every result is proven equivalent to the Verilog | [README_verification.md](README_verification.md) |
| 4. Timing | setup, hold, clocks and Fmax of a cell netlist | [README_timing.md](README_timing.md) |
| 5. Physical design | floorplan, placement, global and detailed routing, written as DEF; checked with IHP's DRC and LVS decks | [README_physical.md](README_physical.md) |
| Power | dynamic and leakage power | not implemented yet (only total leakage is reported) |

## The pipeline

```
Verilog ── preprocessor ── lexer ── parser ── elaborator ─────────────── gate netlist (DAG)
                                                │  registers, hierarchy, memories → SRAMs
                                                ▼
gate netlist ── balancing ── NAND/INV mapping ── optimizer ── cut mapper ── sg13cmos5l cell netlist
                       └──────────── SAT equivalence check at every step ────────────┘
                                                                                     ▼
cell netlist (ours, Yosys or OpenLane) + .lib + .sdc ── lsta ── slack, paths, Fmax
cell netlist + LEF ── fastpnr ── floorplan ── placement ── global routing ── detailed routing ── routed DEF
```

## Three programs

- **`FastASIC`** (`main.cpp`) runs the whole pipeline: parse, synthesize, verify, write the netlist, and optionally time it.
- **`lsta`** (`lsta_main.cpp`) only does timing, on any flat gate-level netlist.
- **`fastpnr`** (`fastpnr_main.cpp`) does physical design: floorplan, placement, global and detailed routing.

## The one idea used everywhere

Every pass works on a directed acyclic graph of logic. Flip-flops and hard macros would make cycles, so the netlist is **cut** at each one:
- a register's output becomes an input of the graph
- its next state becomes an extra output, named `reg$next`
- a macro's input pins become outputs named `inst.pin$in`

The clock and reset are kept separately. Thanks to this, the optimizer, the mapper, the simulator and the SAT checker never need to know about sequential logic.
