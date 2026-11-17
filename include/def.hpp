#pragma once
// DEF (Design Exchange Format) writer: the design's physical state (die, rows, tracks, components,
// pins, power grid, nets), readable by OpenROAD, KLayout and Magic.
#include "cell_netlist.hpp"
#include "droute.hpp"
#include "floorplan.hpp"
#include "lef.hpp"
#include <ostream>

// With `routes`, every routed net also gets its wires and vias (+ ROUTED)
void writeDef(std::ostream& out, const Floorplan& fp, const CellNetlist& netlist, const Lef& lef, const DrouteResult* routes = nullptr);
