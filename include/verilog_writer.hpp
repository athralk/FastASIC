#pragma once
#include "gate_forest.hpp"
#include <ostream>

// Writes a design back out as structural Verilog (one assign per gate) with the same
// ports as the source, so it can be simulated or formally compared against the RTL.
void writeVerilog(std::ostream& out, const Design& design);
