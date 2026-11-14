#pragma once
#include "ast.hpp"
#include "gate_forest.hpp"
#include <string>
#include <vector>

// Turns a Module syntax tree into a bit-level gate netlist: evaluates parameters,
// sizes every expression with Verilog's width rules and splits vectors into one
// node per bit ("bit-blasting"), so everything downstream only sees single-bit gates.
// A whole file: the hierarchy under `top` (default: the one module no other module
// instantiates) is flattened into one netlist; instance contents get "instance." names
// (u_core.u_alu.sum[3]), the same convention as Yosys's flatten.
// How + / - / comparisons / the final adder of multipliers are built
enum class AdderStyle {
    RIPPLE,        // n full adders in a chain: smallest, depth n
    BRENT_KUNG,    // parallel prefix, depth 2 log n, about 2n carry operators
    SKLANSKY,      // parallel prefix, depth log n, (n/2) log n operators (high fanout)
    KOGGE_STONE,   // parallel prefix, depth log n, n log n operators (low fanout, largest)
};

// A memory macro the elaborator may map a reg memory onto
struct SramMacro {
    std::string name;
    long long words = 0, width = 0;
    int ports = 1;            // 1: one shared read/write port, 2: separate ports (A writes, B reads)
    double area = 0;          // um^2
};

// SRAM inference: a reg memory with one whole-word write and one registered read on the same
// clock (and no other use) becomes the smallest fitting macro, when that is smaller than the
// flip-flops it replaces. Its ports must be among the blackboxes.
struct SramInference {
    bool enabled = false;
    std::vector<SramMacro> macros;
    double flop_area = 0;     // um^2 of one flip-flop, to compare against
};

// Instances of modules that are not defined in the source but appear in `blackboxes`
// (hard macros such as SRAMs) are kept as MacroInstance black boxes.
Design elaborate(const std::vector<Module>& modules, const std::string& top = "",
                 const std::vector<BlackBoxModule>& blackboxes = {}, const SramInference& srams = {},
                 AdderStyle adder = AdderStyle::RIPPLE);
Design elaborate(const Module& module);
