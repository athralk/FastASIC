#pragma once
// Placement: global placement (quadratic, SimPL-style), legalization (Abacus) and detailed
// placement. Works on a Floorplan: macros and I/O pins stay fixed, standard cells get positions.
#include "cell_netlist.hpp"
#include "floorplan.hpp"
#include "lef.hpp"
#include <string>
#include <vector>

struct PlaceOptions {
    double target_density = 0.6;   // maximum cell area / free area in any bin after spreading
    int max_iterations = 60;       // global placement outer iterations
    double stop_gap = 0.08;        // stop when (upper - lower bound HPWL) / upper is below this
    int max_net_degree = 100;      // larger nets (clocks, resets) are ignored by global placement
    bool detailed = true;
    int detailed_passes = 6;
};

struct PlaceResult {
    double hpwl_global = 0, hpwl_legal = 0, hpwl_final = 0;   // um, pin to pin
    double ms_global = 0, ms_legal = 0, ms_detailed = 0;
    int iterations = 0;
    double avg_displacement = 0, max_displacement = 0;          // um, legalization
    std::vector<std::string> notes;
};

PlaceResult placeDesign(Floorplan& fp, const CellNetlist& netlist, const Lef& lef, const PlaceOptions& options);

// Half-perimeter wirelength over all signal nets, in um, using the average of each pin's shape
// centres (as OpenROAD's dbITerm::getAvgXY) and the centre of each I/O pin.
double placementHpwl(const Floorplan& fp, const CellNetlist& netlist, const Lef& lef);

// Every standard cell placed in a row, on the site grid, in the row's orientation, no overlaps
std::vector<std::string> checkPlacement(const Floorplan& fp, const CellNetlist& netlist, const Lef& lef);
