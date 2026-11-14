#pragma once
#include "gate_forest.hpp"
#include "liberty.hpp"
#include "tech_mapper.hpp"

// Cut-based technology mapping with Boolean matching (the approach of ABC's mappers):
//   1. the logic becomes an AIG
//   2. every AND node gets its k-feasible cuts (up to 6 leaves), each with the truth table of
//      the node as a function of the leaves; only the best few ("priority cuts") are kept
//   3. every library cell's function is precomputed under all input orders and inversions, so
//      a cut matches a cell when their truth tables are equal: matching is functional, not
//      structural, and the leaves may be shared logic
//   4. dynamic programming picks, for each node and each polarity, the cheapest cut + cell
//      (area flow: shared logic is charged to its readers in proportion), an inverter being
//      the fallback for the other polarity
//   5. area recovery: the mapping is repeated with the real fanout counts of the chosen cover
// delay mode minimizes the estimated arrival time instead, area breaking ties.
MapResult mapByCuts(const Design& design, const GateForest& forest, const Library& lib, MapGoal goal);
