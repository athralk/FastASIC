#pragma once
#include "gate_forest.hpp"
#include <map>
#include <string>
#include <vector>

std::string gateTypeName(GateType type);

// Name a gate goes by in written netlists and timing reports: _g<id>_
std::string instanceName(const GateNode& node);

// Fanins-before-fanouts ordering of every node reachable from the roots.
// Throws if the netlist contains a combinational loop.
std::vector<std::shared_ptr<GateNode>> topologicalOrder(const GateForest& forest);

// Sorted, de-duplicated names of the primary inputs reachable from the roots
std::vector<std::string> collectInputs(const GateForest& forest);

// Number of logic gates per type (ports, wires and constants are not counted)
std::map<GateType, int> gateCounts(const GateForest& forest);

// Drops fanout entries pointing at nodes that were discarded during a rewrite
void compactFanouts(const GateForest& forest);
