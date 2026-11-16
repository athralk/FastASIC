#pragma once
// Global routing: every signal net is routed over a grid of gcells (tiles about 15 tracks wide).
//  1. capacity: routing tracks per gcell edge and layer, minus the power grid and macro blockages,
//     derated by `adjustment` (like OpenROAD's set_global_routing_layer_adjustment)
//  2. 2D routing: pattern routes (L shapes to the nearest point of the net's tree), then negotiated
//     rip-up and reroute (PathFinder costs, A* maze search from the whole tree)
//  3. layer assignment onto the preferred-direction layers, via counting, route guides
#include "cell_netlist.hpp"
#include "floorplan.hpp"
#include "lef.hpp"
#include <ostream>
#include <string>
#include <vector>

struct GrouteOptions {
    std::string min_layer = "Metal2", max_layer = "Metal4";
    int gcell_tracks = 15;        // gcell size in tracks of the first horizontal routing layer above Metal1
    double adjustment = 0.3;      // fraction of every edge's tracks held back for detailed routing
    int max_iterations = 60;      // negotiation rounds
    double via_cost = 2.0;        // cost of a bend, in gcell edges
};

struct GrouteLayerReport {
    std::string name;
    bool horizontal = true;
    long resource_original = 0, resource = 0, demand = 0, overflow = 0;
    int max_overflow = 0;
};

struct GrouteResult {
    std::vector<GrouteLayerReport> layers;
    long overflow_2d = 0;
    int iterations = 0;
    double wirelength_um = 0;     // gcell edges times the gcell size
    long vias = 0;
    int routed_nets = 0, local_nets = 0;
    double ms_capacity = 0, ms_pattern = 0, ms_negotiation = 0, ms_layers = 0;
    int gx = 0, gy = 0;
    Dbu gcell = 0;
    // Route guides: for each routed net, gcell-aligned rectangles per layer
    struct Guide {
        int layer;
        Rect rect;
    };
    std::vector<std::pair<int, std::vector<Guide>>> guides;   // (netlist net, guides)
};

GrouteResult globalRoute(const Floorplan& fp, const CellNetlist& netlist, const Lef& lef, const GrouteOptions& options);

// OpenROAD / TritonRoute route-guide format
void writeGuides(std::ostream& out, const GrouteResult& result, const CellNetlist& netlist, const Lef& lef);
