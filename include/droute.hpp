#pragma once
// Detailed routing on the track grid, inside the global-routing guides.
//
// Correct by construction:
//  - wires run on routing tracks, at minimum width, in each layer's preferred direction
//  - vias are the PDK's single-cut vias whose pads lie along each layer's direction
//    (Via1_XY, Via2_YX, Via3_XY), so side-by-side tracks keep their spacing
//  - two different nets never use adjacent nodes of the same track (end-of-line spacing)
//  - standard-cell pins are reached by a Via1 whose Metal1 pad lies inside the pin
//  - obstacles (power grid, macros) block every node whose wire or via would come too close
//  - short wire pieces are extended to the minimum-area rule
// Conflicts are resolved by negotiation (PathFinder) with A* searches from each net's tree.
#include "cell_netlist.hpp"
#include "floorplan.hpp"
#include "groute.hpp"
#include "lef.hpp"
#include <string>
#include <vector>

struct DrouteOptions {
    int max_iterations = 80;
    int patience = 20;              // rounds without improvement before giving up
    double via_cost = 4.0;          // in track pitches
    double outside_guide = 3.0;     // cost multiplier for leaving the net's guides
    double wrong_way = 2.5;         // cost of a wrong-way jog per unit of length (0: none)
    double access_penalty = 0.5;    // wire over another net's pin access point (makes it harder to reach)
    double top_via_factor = 1.0;    // extra cost of the via up to the top routing layer
    double other_layer = 1.0;       // inside a guide but not on the layer global routing chose (higher values added vias on mul16)
    double astar_weight = 1.2;      // A* estimate weight: 1.2 halves the search for 0.2% more wire (mac32)
};

struct DrouteSegment {
    int layer;                      // LEF layer
    Dbu x0, y0, x1, y1;             // along one track; x0 == x1 or y0 == y1
};

struct DrouteVia {
    std::string name;               // LEF via
    int bottom;                     // LEF layer of its bottom metal
    Dbu x, y;
};

struct DrouteNet {
    int netlist_net = -1;
    std::vector<DrouteSegment> wires;
    std::vector<DrouteVia> vias;
    std::vector<std::pair<int, Rect>> patches;   // (LEF layer, rectangle): metal filling notches between close via pads
};

struct DrouteResult {
    std::vector<DrouteNet> nets;
    std::vector<Dbu> wire_width;    // drawn wire width per grid layer (wider than the minimum where via pads are)
    std::vector<int> wire_layer;    // the LEF layer of each grid layer
    int iterations = 0;
    long conflicts = 0;             // nodes shared, or adjacent on a track, by different nets
    int unrouted = 0;               // nets with a pin that could not be reached
    int pins_without_access = 0;
    int min_area_fixes = 0, min_area_left = 0;
    double wirelength_um = 0;
    long vias = 0;
    double ms_setup = 0, ms_route = 0, ms_finish = 0;
    std::vector<std::string> notes;
};

DrouteResult detailedRoute(const Floorplan& fp, const CellNetlist& netlist, const Lef& lef, const GrouteResult& groute, const DrouteOptions& options);
