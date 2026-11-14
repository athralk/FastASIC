# Step 2: Synthesis (gates to standard cells)

Files: `balance.cpp`, `mapper.cpp`, `optimizer.cpp`, `cut_mapper.cpp`, `tech_mapper.cpp`, `netlist_builder.cpp`, `cell_netlist.cpp`, `verilog_writer.cpp`, `liberty.cpp`.

## 1. Balancing (`balance.cpp`)

Chains of the same gate (AND, OR, XOR) are rebuilt as balanced trees. A 22-input XOR chain goes from 21 levels of logic to 5.

## 2. NAND/INV form (`mapper.cpp`) and optimization (`optimizer.cpp`)

Every gate is rewritten as 2-input NANDs and inverters. The optimizer then:
- merges identical logic (structural hashing)
- propagates constants
- removes double inverters and simplifies patterns like `x & ~x`

`-o` without `--map` writes this netlist.

## 3. Mapping onto real cells (`cut_mapper.cpp`, the default)

This is the same approach as ABC's mappers:
1. The logic becomes an AIG, a graph of 2-input ANDs with inverted edges (`aig.cpp`).
2. For every node, the mapper lists its **cuts**: sets of up to 6 signals that fully determine it. It stores the **truth table** of the node over those signals.
3. Every cell in the `.lib` file is evaluated under every input order and inversion, once, into a lookup table. A cut matches a cell when the truth tables are equal, however the logic happens to be built.
4. Dynamic programming picks the best cell for every node.
   - **Area mode:** "exact area recovery" then removes duplicated logic: each choice is costed by what it really adds to the netlist.
   - **Delay mode:** the fastest choice wins, and area is recovered only where timing allows.

`--mapper tree` selects the older tree-covering mapper (`tech_mapper.cpp`, the algorithm taught in Cornell ECE 6745).

## 4. Building the cell netlist (`netlist_builder.cpp`)

The mapped logic is wrapped with the design's ports, one flip-flop cell per register and the SRAM macro instances:
- `dfrbpq_1` for most registers, with the reset tied high when there is none
- `dfrbp_1` for registers that reset to 1
- an inverter for falling-edge clocks and active-high resets

## 5. Reading and writing netlists

- `cell_netlist.cpp` reads and writes flat cell netlists (with buses for macro pins) and turns one back into gates for checking.
- `verilog_writer.cpp` writes the NAND/INV netlist, with registers as `always` blocks.

Both outputs can be read by Yosys, OpenSTA and OpenROAD.

## The cell library (`liberty.cpp`)

It reads `.lib` files:
- cells, pins, buses
- logic functions
- flip-flops and macro memories
- timing tables
- capacitance ranges
- slew thresholds

The 1.7 MB sg13cmos5l library loads in about 20 ms.
