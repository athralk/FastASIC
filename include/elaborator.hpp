#pragma once
#include "ast.hpp"
#include "gate_forest.hpp"

// Turns a Module syntax tree into a bit-level gate netlist: evaluates parameters,
// sizes every expression with Verilog's width rules and splits vectors into one
// node per bit ("bit-blasting"), so everything downstream only sees single-bit gates.
Design elaborate(const Module& module);
