# Step 1: Front end (Verilog to gates)

Files: `preprocessor.cpp`, `lexer.cpp`, `parser.cpp`, `elaborator.cpp`, `file_reader.cpp`.
Syntax tree: `include/ast.hpp`. Netlist types: `include/gate_forest.hpp`.

## Preprocessor (`preprocessor.cpp`)

It runs on the raw text before anything else and handles:
- `` `define `` (also with arguments) and `` `undef ``
- `` `ifdef `` / `` `ifndef `` / `` `elsif `` / `` `else `` / `` `endif ``
- `` `include ``, searched next to the file, then in the `-I` folders
- `-DNAME=value` from the command line

Lines it removes are kept as blank lines, so error messages still point at the right line.

## Lexer and parser (`lexer.cpp`, `parser.cpp`)

The lexer splits text into tokens. The parser builds a syntax tree and only checks grammar; widths, names and drivers are checked later. It reads:
- modules with ANSI or classic port lists, and parameters
- `wire`, `reg`, `integer`, `signed`, memories `reg [7:0] mem [0:255]`
- `assign`, `always` blocks, and statements: `begin`/`end`, `=`, `<=`, `if`, `case`/`casez`/`casex`, `for`, task calls
- `function`, `task`, `generate` (`for`, `if`, `case`, named blocks), `genvar`
- module instances with named or positional ports and parameter overrides
- every expression operator, with Verilog's precedence

Reserved words (`small`, `and`, ...) are rejected as names, as other tools do.

## Elaborator (`elaborator.cpp`)

This is where the meaning of the Verilog is worked out. Its output is a bit-level gate netlist: one node per bit and one gate per operation.

**Widths and signedness.** Every expression gets Verilog's width: operands are extended to the size of the result before the operation, so `{carry, sum} = a + b` keeps the carry. An expression is signed only if all its operands are; then values are sign-extended, comparisons are signed, and `>>>` shifts in the sign bit.

**Arithmetic.**
- **`+`, `-` and comparisons:** ripple carry, or a parallel-prefix adder (Brent–Kung, Sklansky, Kogge–Stone) chosen with `--adder`. A 1-bit term in a sum becomes the carry-in instead of a second adder.
- **`*`:** a radix-4 Booth multiplier, then a Dadda tree of full adders, then the final adder. Bits that are only sign or zero extension are not multiplied.
- **Variable shifts:** a barrel shifter.

**`always` blocks** are run symbolically:
- blocking assignments update values that later statements see; non-blocking ones take effect at the end
- `if` and `case` become multiplexers, and `for` loops are unrolled
- a combinational block that leaves a variable unassigned on some path is reported as a latch
- in a clocked block, an unassigned register keeps its value, and an asynchronous reset is recognized from the usual `if (!rst_n)` shape

**Functions and tasks** are inlined: their body is run the same way, with the arguments bound.

**`generate`** is unrolled before anything else. Loop variables become constants, and names inside get scoped names such as `stage[3].sum`.

**Hierarchy** is flattened:
- each instance is elaborated with its own parameters, and its signals are named `instance.signal`
- every register's clock and reset is traced back to a top-level input

**Memories.** A small memory becomes flip-flops. A large one with one write and one registered read becomes an IHP SRAM macro, when that is smaller (`inferSrams`):
- 1-port when a SAT proof shows the read and write never happen in the same cycle
- 2-port otherwise, because a 1-port SRAM would return the new data instead of the old one
- the decision is printed

**Black boxes.** Modules that are not defined anywhere are looked up among the PDK's SRAM macros and kept as black boxes.

**Checks:** undeclared names, multiple drivers, undriven outputs, combinational loops, unconnected instance inputs, derived clocks.
