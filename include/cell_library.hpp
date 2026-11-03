#pragma once
#include "gate_forest.hpp"
#include <string>
#include <vector>

// Timing data for the two standard cells the mapper targets, from IHP's SG13CMOS5L
// standard-cell library (130 nm, CMOS only):
//   PDK:     ihp-sg13cmos5l, IHP-Open-PDK commit 2bbec755dc67ca3db0261c3d6163e15735d66710
//   Library: sg13cmos5l_stdcell rev 0.1.4
//   Corners: libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_<corner>.lib
//              typ_1p20V_25C (sign-off), fast_1p32V_m40C, slow_1p08V_125C
//
// Units: capacitance in fF, resistance in kOhm, time in ps (kOhm * fF = ps).
//
// Liberty stores delay as 2D tables (input slew x output load), not as R and C. For the
// RC delay model each arc's table row at the sharpest input edge (19 ps slew) was fitted
// with a straight line, delay = intrinsic + R_eff * C_load (tools/fit_rc_from_liberty.py).
// The intercept is R_eff * C_par: the gate charging its own output (parasitic) capacitance.

// Transition of a net; also used as an array index
enum Edge { RISE = 0, FALL = 1 };

// Process / voltage / temperature corner, each characterized in its own .lib file
enum class Corner { TYP, FAST, SLOW };

// e.g. "typ 1.20V 25C"
std::string cornerName(Corner corner);

struct RcArc {
    double intrinsic_ps[2];   // R_eff * C_par, indexed by output edge
    double r_kohm[2];         // effective drive resistance, indexed by output edge
};

struct CellTiming {
    std::string name;
    double area_um2;
    std::vector<std::string> input_pins;   // input k of a gate node connects to input_pins[k]
    std::vector<double> input_cap_ff;      // capacitance of each input pin
    std::string output_pin;
    double max_cap_ff;                     // largest load the cell is characterized for
    std::vector<RcArc> arcs;               // arcs[k]: input_pins[k] -> output pin, inverting
};

// The cell a mapped gate is built from at the given corner: NOT -> inv_1, NAND -> nand2_1.
// Throws for other gates.
const CellTiming& cellFor(GateType type, Corner corner);
