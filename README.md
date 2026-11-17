# FastASIC

FastASIC is a Verilog compiler and ASIC back-end toolkit, written from scratch in C++ with no dependencies. It turns Verilog into a netlist of real standard cells for IHP's open 130 nm process (SG13CMOS5L, the one Tiny Tapeout uses) and checks the timing of that netlist. It does the same jobs as Yosys and OpenSTA (the timing engine inside OpenROAD and OpenLane), and it is much faster than both.

Every netlist it produces is mathematically proven to behave like your Verilog before you see it.

## What it does

- **Synthesis:** Verilog in, a netlist of sg13cmos5l cells out, optimized for area or speed. Large memories become IHP SRAM macros automatically.
- **Timing:** `lsta` reports setup, hold, worst paths and maximum clock speed for any gate-level netlist, including ones from Yosys or OpenLane. Its numbers match OpenSTA's.
- **Verification:** a built-in SAT solver proves that two designs are equivalent, or shows an input where they differ.
- **Physical design:** `fastpnr` floorplans, places and routes a netlist (die, rows, tracks, I/O pins, SRAMs, power grid, every wire and via). The routed layouts pass IHP's own design-rule (DRC) and layout-versus-schematic (LVS) checks.

How each step works is explained in [`src/`](src/README.md).

## Setup

You need a C++17 compiler and `make`. On Ubuntu:

```bash
sudo apt install g++ make
```

Build:

```bash
make
```

This creates three programs: `FastASIC` (synthesis and verification), `lsta` (timing) and `fastpnr` (physical design).

**The IHP PDK** is needed for anything that uses real cells. FastASIC looks for it in `~/ttsetup/pdk`, the folder Tiny Tapeout's setup creates. If yours is somewhere else, set `PDK_ROOT`:

```bash
export PDK_ROOT=/path/to/pdk
```

The PDK must contain `ihp-sg13cmos5l`. The version used and tested here is IHP-Open-PDK commit `2bbec755dc67ca3db0261c3d6163e15735d66710`.

**Optional tools.** None are needed to use FastASIC, only to run the comparisons and the full test suite:
- `yosys` (with `yosys-abc`) for the synthesis and equivalence benchmarks
- `iverilog` for the simulation checks
- `docker` with the `ghcr.io/efabless/openlane2:2.3.10` image for the OpenSTA comparison

Run all tests:

```bash
make test
```

## Using it

Synthesize a design onto sg13cmos5l cells, optimized for area, and write the netlist:

```bash
./FastASIC -q --map area -o cells.v design.v
```

Use `--map delay` to optimize for speed instead. Add `--sta --delay-model nldm --sdc constraints.sdc` to check the timing straight away.

Check the timing of any gate-level netlist:

```bash
./lsta -sdc constraints.sdc netlist.v
```

Floorplan, place and route a cell netlist, and write the routed layout as DEF:

```bash
./fastpnr -droute -o design.def cells.v
```

Check the result with IHP's own rule decks (needs docker):

```bash
tools/drc_klayout.sh design.def
```

```bash
tools/lvs_klayout.sh design.def
```

Check whether two designs behave the same:

```bash
./FastASIC --equiv first.v second.v
```

Run `./FastASIC`, `./lsta` or `./fastpnr` with no arguments to see every option.

## Benchmarks

All numbers below were measured on the same machine with the same inputs. Area is in square micrometres, delay in picoseconds.

### Synthesis speed against Yosys + ABC

FastASIC and Yosys + ABC each turn the same Verilog into sg13cmos5l cells. Times are the full run, best of three.

| Design | FastASIC | Yosys + ABC | FastASIC is |
|---|---|---|---|
| full adder | 35 ms | 530 ms | 15× faster |
| 4-bit ALU | 56 ms | 986 ms | 18× faster |
| 12-bit comparator | 45 ms | 972 ms | 22× faster |
| 64-bit adder | 92 ms | 1,599 ms | 17× faster |
| 32-bit barrel shifter | 56 ms | 1,171 ms | 21× faster |
| 16 × 16 multiplier | 934 ms | 3,348 ms | 3.6× faster |
| all 11 designs | 3.8 s | 20.4 s | 5.3× faster |

Multipliers gain less because FastASIC also proves the result correct, which Yosys does not do.

Run it yourself: `tools/bench_synth_speed.sh`

### Result quality against Yosys + ABC

Smaller area and smaller delay are better.

| Design | FastASIC | Yosys + ABC | Result |
|---|---|---|---|
| 16 × 16 multiplier, area | 12,126 | 14,570 | FastASIC 17% smaller |
| 16 × 16 multiply-add, area | 13,709 | 15,951 | FastASIC 14% smaller |
| 32-bit barrel shifter, area | 2,578 | 2,703 | FastASIC 5% smaller |
| 32-bit adder, delay | 2,450 ps | 4,767 ps | FastASIC 49% faster |
| 16 × 16 multiplier, delay | 4,724 ps | 5,942 ps | FastASIC 20% faster |
| small control logic, area | 6,777 (total of 9) | 6,310 | Yosys + ABC 7% smaller |

Area numbers come from `--map area`, delay numbers from `--map delay`, so they are two different netlists.

Run it yourself: `tools/compare_synth.sh` and `tools/compare_synth.sh bench/arith/*.v`

### Timing against OpenROAD's OpenSTA

`lsta` and OpenSTA read the same netlist, cell library and constraints, and report the slack at every endpoint.

| Check | Result |
|---|---|
| Accuracy | Same answer as OpenSTA at every endpoint, within 0.007 ps, on 6 designs at 3 corners, with ideal and real clock trees and with SRAM macros |
| 52,000-cell design, timing analysis | 150 ms for lsta, 1,944 ms for OpenSTA: 13× faster |
| 52,000-cell design, whole run | 0.5 s and 92 MB for lsta, 2.6 s for OpenSTA |

Run it yourself: `tools/compare_opensta.sh bench/sta/netlists/mac32.v bench/sta/default.sdc typ`

### Floorplan against OpenROAD

Both tools read the same netlist and do the same job: tie cells, die and rows, routing tracks, I/O pins and the same power grid (Metal1 rails, Metal4 and TopMetal1 straps). Times are best of three on `mac32` (6,490 cells), measured inside each program.

| Check | fastpnr | OpenROAD |
|---|---|---|
| Whole run (read, floorplan, write DEF) | 66 ms | 285 ms: fastpnr 4.3× faster |
| Floorplan step alone | 0.6 ms | 64 ms: fastpnr about 100× faster |
| OpenROAD accepts fastpnr's DEF | placement, `check_placement`, `check_power_grid` on VDD and VSS, and global routing all pass, on 6 designs including two with SRAMs | |

Run it yourself: `./fastpnr -no-place -o mac32.def bench/sta/netlists/mac32.v`, then `tools/check_def_openroad.sh mac32.def`

### Placement against OpenROAD

Both tools place the same floorplan. Wire length (HPWL) is measured by the same routine inside OpenROAD, and shorter is better. Times are measured inside each program.

| Design | Cells | Wire length | Time: fastpnr vs OpenROAD |
|---|---|---|---|
| 16 × 16 multiplier | 1,480 | fastpnr 6.8% shorter | 123 vs 231 ms: 1.9× faster |
| 32-bit multiply-accumulate | 6,490 | fastpnr 1.5% shorter | 584 vs 1,199 ms: 2.1× faster |
| 8 × multiply-accumulate | 51,920 | fastpnr 17.8% shorter | 3.8 vs 10.9 s: 2.9× faster |
| FIFO with a 2-port SRAM | 244 | fastpnr 4.2% shorter | 12 vs 91 ms: 7.6× faster |
| buffer with a 1-port SRAM | 66 | fastpnr 46.8% shorter | 1 vs 533 ms |

OpenROAD's own `check_placement` accepts every fastpnr placement.

Run it yourself: `tools/compare_place_openroad.sh bench/sta/netlists/mac32.v`. The 51,920-cell design is 8 copies of mac32: `tools/replicate_netlist.py bench/sta/netlists/mac32.v 8 > mac32_x8.v`

### Global routing against OpenROAD's FastRoute

Both routers route fastpnr's placement on Metal2–Metal4, with the same 30% of tracks held back for detailed routing. Overflow means more wires than tracks somewhere, and 0 is what you need.

| Design | Overflow (fastpnr / FastRoute) | Routed wire | Time: fastpnr vs FastRoute |
|---|---|---|---|
| 16 × 16 multiplier | 0 / 0 | fastpnr 1.6% less | 8 vs 199 ms: 26× faster |
| 32-bit multiply-accumulate | 0 / 0 | fastpnr 1.9% less | 53 vs 492 ms: 9× faster |
| 8 × multiply-accumulate | 0 / 0 | fastpnr 2.4% less | 1.2 vs 3.6 s: 3× faster |
| FIFO with a 2-port SRAM | 0 / 0 | fastpnr 1.6% less | 9 vs 126 ms: 13× faster |

Run it yourself: `tools/compare_groute_openroad.sh bench/sta/netlists/mac32.v`

### Detailed routing against OpenROAD's TritonRoute

An earlier measurement, with **both routers on one thread** and before the router's latest improvements; the full-flow ALU benchmark below gives OpenROAD all 16 threads. Both routers route the same placement on Metal2–Metal4, and both results are checked with IHP's own design-rule deck, fastpnr's also with IHP's LVS deck (no opens, no shorts).

| Design | fastpnr | TritonRoute | Design-rule violations (fastpnr / TritonRoute) |
|---|---|---|---|
| 16 × 16 multiplier, 1,514 nets | 1.3 s | 251 s: fastpnr 190× faster | 0 / 0 |
| 32-bit multiply-accumulate, 6,556 nets | 8.1 s | 18.6 min: fastpnr 138× faster | 0 / 0 |

TritonRoute's layouts use 1–2% less wire and about 20% fewer vias.

Run it yourself: `tools/compare_droute_openroad.sh bench/sta/netlists/mul16.v`

### Full flow on a real design: a 64-bit vector ALU

`bench/flow/alu.v`, one ALU lane of a user's GPU project, is mostly a 64 × 64 multiplier (about 8,000–13,000 cells). FastASIC is compared with Yosys + ABC for synthesis and with OpenROAD for routing, on the same netlist and placement. fastpnr uses 1 thread and OpenROAD 16.

| | Area-optimised | Speed-optimised (`--map delay`) |
|---|---|---|
| Synthesis: FastASIC vs Yosys + ABC | 10.9 s vs 31.9 s: **2.9× faster**, 29% smaller | 17.1 s vs 36.5 s: **2.1× faster**, 3% smaller |
| Routing: fastpnr vs OpenROAD | 35.5 s vs 575 s: **16× faster** | 119 s vs 753 s: **6.3× faster** |
| Design-rule violations (IHP deck), both tools | 0 | 0 |
| Wire and vias vs OpenROAD | 0.9% less wire, 6.5% fewer vias | 1.8% less wire, 6.4% fewer vias |
| Fastest clock after routing, slow corner | 36.9 MHz vs 36.5 MHz | 47.2 MHz vs 46.7 MHz |

The clock speeds come from OpenROAD's own extraction (OpenRCX) and timing (OpenSTA) on both layouts, so the same tools judge both.

Run it yourself (about 15 minutes; OpenROAD's routing is the slow part):

```bash
tools/benchmark_flow.sh bench/flow/alu.v --period 17.6
```

```bash
tools/benchmark_flow.sh bench/flow/alu.v --map delay --period 17.6
```

### Equivalence checking against Yosys

| Check | FastASIC | Yosys `sat` |
|---|---|---|
| Two ways of adding three 32-bit numbers | 195 ms | 14.4 s |
| 8 × 8 multiplier against a shift-and-add multiplier | proven in 6.2 s | gave up after 120 s |

Run it yourself: `tools/bench_equiv.sh`

### What FastASIC does not do yet

Routing is not timing-driven yet, and there is no metal fill or GDS output: OpenROAD or KLayout finish the layout from FastASIC's routed DEF. Power analysis is not implemented yet.

## Licenses

**FastASIC is licensed under the [Apache License 2.0](LICENSE).** You may use, modify and distribute it, including in commercial and closed-source products, as long as you keep the copyright and license notices (see [NOTICE](NOTICE)). The license also grants a patent license from every contributor.

Third-party files and tools:
- **IHP-Open-PDK** (cell libraries and SRAM macros): Apache License 2.0. FastASIC reads the PDK but does not include it.
- **Yosys** (ISC license), **OpenSTA** (GPL-3.0) and **iverilog** (GPL-2.0) are only run as separate programs for comparisons and tests. None of their code is part of FastASIC.
