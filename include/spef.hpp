#pragma once
// Lightweight IEEE 1481 SPEF extraction (from fastpnr detailed routing + tech LEF R/C)
// and SPEF reader + Elmore RC delay calculator for lsta post-route timing.
#include "cell_netlist.hpp"
#include "droute.hpp"
#include "floorplan.hpp"
#include "lef.hpp"
#include <ostream>
#include <string>
#include <unordered_map>

struct NetParasitics {
    double wire_cap_ff = 0.0;                           // total extracted wire capacitance (fF)
    double wire_res_ohm = 0.0;                          // total routed resistance (ohm)
    double default_elmore_ps = 0.0;                     // fallback wire delay if a pin is unmatched (ps)
    std::unordered_map<std::string, double> pin_elmore_ps; // load pin ("inst/PIN" or port) -> Elmore delay (ps)
};

struct SpefData {
    std::unordered_map<std::string, NetParasitics> nets; // canonical net name -> parasitics
    size_t net_count = 0;
    double ms_read = 0.0;
};

// Extract RC trees from fastpnr's routed segments/vias + tech LEF R/C and write IEEE 1481 SPEF.
// `cap_scale` accounts for lateral coupling capacitance not captured by isolated LEF CPERSQDIST+EDGECAP.
void writeSpef(std::ostream& out, const Floorplan& fp, const CellNetlist& nl, const Lef& lef,
               const DrouteResult& dr, double cap_scale = 1.35);

// Read an IEEE 1481 SPEF file (from fastpnr or OpenRCX) and compute exact Elmore RC delays per load pin.
SpefData readSpef(const std::string& path);
