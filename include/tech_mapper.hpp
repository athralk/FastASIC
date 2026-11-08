#pragma once
#include "cell_netlist.hpp"
#include "gate_forest.hpp"
#include "liberty.hpp"
#include <map>
#include <string>

// Technology mapping by tree covering (Keutzer's DAGON, the algorithm Cornell ECE 6745 uses):
// the optimized NAND2 + INV netlist is the "subject graph"; every library cell is described as
// a small NAND2/INV pattern; dynamic programming picks, at every node, the cheapest cell whose
// pattern matches there. Nodes with several fanouts split the DAG into trees (a match may not
// swallow a node another gate also reads), except inside the XOR/XNOR pattern, whose shared
// NAND is part of the cell.
//
// area mode:  minimize total cell area
// delay mode: minimize the estimated arrival time at every node (cell delay from the NLDM
//             tables at a typical slew and load), area as tie-breaker
enum class MapGoal { AREA, DELAY };

struct MapResult {
    CellNetlist netlist;
    std::map<std::string, int> cell_counts;
    double area = 0;
};

MapResult mapToCells(const Design& design, const GateForest& nand_inv, const Library& lib, MapGoal goal);
