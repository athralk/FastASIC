# Step 4: Timing (static timing analysis)

Files: `timer.cpp`, `liberty.cpp`, `sdc.cpp`, `clock_tree.cpp`, `lsta_main.cpp`. The older simple model is in `sta.cpp` and `cell_library.cpp`.

## lsta: the timer (`timer.cpp`)

lsta works on any flat gate-level netlist: our own, Yosys's or OpenLane's. Its results match OpenSTA's to within 0.007 ps, and it runs about 13× faster on large designs.

**What it tracks.** For every pin it keeps:
- the rise and fall arrival times, both latest and earliest
- the rise and fall slew

**Delays.** Cell delays come from the library's tables, looked up at the input slew and output load, exactly as OpenSTA does it.

**Checks.**
- flip-flops start paths at their clock pin
- setup and hold at flip-flop inputs
- recovery and removal for asynchronous resets
- output delays at the ports

**Reports.** Worst slack, total negative slack, the worst paths in OpenSTA's format, and the maximum clock frequency.

**Clocks.**
- An ideal clock arrives everywhere at once.
- A propagated clock goes through a real clock tree, which gives clock skew.
- Inverted clocks and falling-edge registers are handled: a rising-to-falling path gets half a clock cycle.
- With propagated clocks, the shared part of the clock tree is not counted twice (CRPR).

**Details that had to match OpenSTA exactly:**
- pin capacitance uses the top of its range for setup and the bottom for hold
- an input's driving cell adds only its load-dependent delay
- resets are not timed as data paths
- slews are converted when a net connects cells whose libraries measure slew differently (the SRAMs use 30–70% thresholds, the standard cells 20–80%)

## Constraints (`sdc.cpp`)

It reads the usual SDC commands:
- clocks: `create_clock`, `set_clock_uncertainty`, `set_clock_transition`, `set_clock_latency`, `set_propagated_clock`
- ports: `set_input_delay`, `set_output_delay`, `set_load`, `set_driving_cell`, `set_input_transition`
- object queries: `get_ports`, `all_inputs`, `all_outputs`, `get_clocks`, `all_clocks`
- Tcl variables and `[expr]`

## Clock tree (`clock_tree.cpp`)

`lsta -cts BUF` builds a balanced tree of clock buffers before timing. No buffer drives more than the given fanout. It doesn't use placement yet, so the tree balances fanout, not wire length.

## Not modelled yet

- wire delay from layout (SPEF files)
- multiple clocks, false paths and multicycle paths
- latches
- on-chip variation
- power
