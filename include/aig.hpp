#pragma once
#include "gate_forest.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// And-Inverter Graph: the netlist form used by formal tools (ABC, Yosys's `abc` pass).
// Every node is a 2-input AND; inversion is a flag on the edge, so NOT gates cost nothing.
// Nodes live in one array and refer to each other by index, so walking the graph is a
// linear scan instead of chasing shared_ptrs.
//
// An edge ("literal") is 2 * node + complemented:
//   literal 0 = constant false, literal 1 = constant true (node 0 is the constant).
// A node's fanins always have smaller indices, so index order is a topological order.
using AigLit = uint32_t;

constexpr AigLit kAigFalse = 0;
constexpr AigLit kAigTrue = 1;

inline AigLit aigNot(AigLit lit) { return lit ^ 1u; }
inline uint32_t aigNode(AigLit lit) { return lit >> 1; }
inline bool aigIsComplemented(AigLit lit) { return lit & 1u; }
inline AigLit aigLiteral(uint32_t node, bool complemented = false) { return (node << 1) | (complemented ? 1u : 0u); }

class Aig {
public:
    Aig();

    // Primary input with this name; asking twice returns the same input
    AigLit input(const std::string& name);

    // Structurally hashed AND: identical (or trivially simplifiable) requests return an
    // existing literal, so equal sub-circuits built from both designs merge automatically.
    AigLit makeAnd(AigLit a, AigLit b);
    AigLit makeOr(AigLit a, AigLit b) { return aigNot(makeAnd(aigNot(a), aigNot(b))); }
    AigLit makeXor(AigLit a, AigLit b);

    size_t nodeCount() const { return fanin0_.size(); }   // includes the constant and the inputs
    size_t andCount() const { return nodeCount() - 1 - input_nodes_.size(); }
    bool isAnd(uint32_t node) const { return node != 0 && fanin0_[node] != kNoFanin; }
    bool isInput(uint32_t node) const { return node != 0 && fanin0_[node] == kNoFanin; }
    AigLit fanin0(uint32_t node) const { return fanin0_[node]; }
    AigLit fanin1(uint32_t node) const { return fanin1_[node]; }

    const std::vector<uint32_t>& inputNodes() const { return input_nodes_; }
    const std::vector<std::string>& inputNames() const { return input_names_; }

private:
    static constexpr AigLit kNoFanin = ~0u;
    std::vector<AigLit> fanin0_, fanin1_;              // structure-of-arrays, indexed by node
    std::vector<uint32_t> input_nodes_;
    std::vector<std::string> input_names_;
    std::unordered_map<std::string, uint32_t> input_by_name_;
    std::unordered_map<uint64_t, uint32_t> strash_;    // (fanin0, fanin1) -> AND node
};

// Adds a gate netlist's logic to the AIG (inputs are shared with anything already there
// by name) and returns each output root's name and literal, in forest order.
std::vector<std::pair<std::string, AigLit>> addToAig(Aig& aig, const GateForest& forest);

// Bit-parallel simulation: input_words[i] drives inputNodes()[i] (64 patterns per word).
// Returns one word per node; the value of literal l is word[aigNode(l)] ^ (complemented ? ~0 : 0).
std::vector<uint64_t> simulateAig(const Aig& aig, const std::vector<uint64_t>& input_words);

inline uint64_t aigValue(const std::vector<uint64_t>& node_words, AigLit lit) {
    return node_words[aigNode(lit)] ^ (aigIsComplemented(lit) ? ~0ULL : 0ULL);
}
