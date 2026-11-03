#pragma once
#include "gate_forest.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// Bit-parallel logic simulator: every net carries a 64-bit word, so one pass over
// the netlist evaluates 64 input patterns at once (bit i of every word = pattern i).
class Simulator {
    std::vector<std::shared_ptr<GateNode>> order;
    std::unordered_map<const GateNode*, size_t> index;
    std::vector<uint64_t> values;

public:
    explicit Simulator(const GateForest& forest);

    // Inputs missing from the map are driven to 0
    void run(const std::unordered_map<std::string, uint64_t>& input_words);
    uint64_t value(const GateNode* node) const;
};

struct EquivalenceResult {
    bool equivalent = false;
    bool exhaustive = false;   // true: every input combination was tried (a proof, not a sample)
    uint64_t patterns = 0;
    std::string message;       // counterexample or reason when not equivalent
};

// Compares two netlists output-by-output (matched by port name)
EquivalenceResult checkEquivalence(const GateForest& a, const GateForest& b);
