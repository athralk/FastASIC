#pragma once
#include "gate_forest.hpp"
#include "sat_solver.hpp"
#include <string>

// Combinational equivalence checking by SAT: a proof for any number of inputs, where
// simulation can only be exhaustive up to about 20.
struct CecResult {
    bool equivalent = false;
    std::string message;        // counterexample or reason when not equivalent
    size_t outputs = 0;
    size_t structural = 0;      // proved with no SAT call: both sides hashed to the same AIG node
    size_t by_sat = 0;          // proved by an UNSAT miter
    size_t aig_ands = 0;        // AND nodes in the shared AIG of both designs
    SatSolver::Stats sat;
    double milliseconds = 0;
};

// Both designs go into one structurally hashed AIG (inputs shared by name), so identical
// logic merges for free. A short random simulation catches easy differences. Every remaining
// output pair (matched by name) is then a miter question "can x XOR y be 1?" asked of one
// incremental solver under an assumption; only the cones of influence are Tseitin-encoded,
// and each proven pair is added back as x <-> y to help prove the next one.
// A SAT counterexample is replayed on both netlists with the simulator before it is reported.
CecResult proveEquivalence(const GateForest& a, const GateForest& b, const std::string& solver = "cdcl");
