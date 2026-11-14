# FastASIC

FastASIC is a Verilog compiler and ASIC back-end toolkit, written from scratch in C++ with no dependencies. It turns Verilog into a netlist of real standard cells for IHP's open 130 nm process (SG13CMOS5L, the one Tiny Tapeout uses) and checks the timing of that netlist. It does the same jobs as Yosys and OpenSTA (the timing engine inside OpenROAD and OpenLane), and it is much faster than both.

Every netlist it produces is mathematically proven to behave like your Verilog before you see it.

## What it does

- **Synthesis:** Verilog in, a netlist of sg13cmos5l cells out, optimized for area or speed. Large memories become IHP SRAM macros automatically.
- **Timing:** `lsta` reports setup, hold, worst paths and maximum clock speed for any gate-level netlist, including ones from Yosys or OpenLane. Its numbers match OpenSTA's.
- **Verification:** a built-in SAT solver proves that two designs are equivalent, or shows an input where they differ.

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

This creates two programs: `FastASIC` (synthesis and verification) and `lsta` (timing).

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

Check whether two designs behave the same:

```bash
./FastASIC --equiv first.v second.v
```

Run `./FastASIC` or `./lsta` with no arguments to see every option.

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

### Equivalence checking against Yosys

| Check | FastASIC | Yosys `sat` |
|---|---|---|
| Two ways of adding three 32-bit numbers | 195 ms | 14.4 s |
| 8 × 8 multiplier against a shift-and-add multiplier | proven in 6.2 s | gave up after 120 s |

Run it yourself: `tools/bench_equiv.sh`

### What FastASIC does not do yet

It stops at the gate-level netlist. Placement, routing and layout are still done by OpenROAD, which reads FastASIC's netlists directly. Power analysis is not implemented yet.

## Licenses

**FastASIC is licensed under the [Apache License 2.0](LICENSE).** You may use, modify and distribute it, including in commercial and closed-source products, as long as you keep the copyright and license notices (see [NOTICE](NOTICE)). The license also grants a patent license from every contributor.

Third-party files and tools:
- **IHP-Open-PDK** (cell libraries and SRAM macros): Apache License 2.0. FastASIC reads the PDK but does not include it.
- **Yosys** (ISC license), **OpenSTA** (GPL-3.0) and **iverilog** (GPL-2.0) are only run as separate programs for comparisons and tests. None of their code is part of FastASIC.
