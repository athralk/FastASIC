#include "../include/simulator.hpp"
#include "../include/netlist_utils.hpp"
#include <algorithm>
#include <map>
#include <random>
#include <set>
#include <stdexcept>

namespace {

constexpr size_t kMaxExhaustiveInputs = 20;   // 2^20 patterns = 16384 words per netlist
constexpr uint64_t kRandomWords = 1024;       // 65536 random patterns above that

// For exhaustive simulation, pattern number p = word * 64 + bit. Input i (< 6) takes bit i of p,
// which is the same fixed mask in every word; higher inputs are constant within a word.
constexpr uint64_t kPatternMasks[6] = {
    0xAAAAAAAAAAAAAAAAULL, 0xCCCCCCCCCCCCCCCCULL, 0xF0F0F0F0F0F0F0F0ULL,
    0xFF00FF00FF00FF00ULL, 0xFFFF0000FFFF0000ULL, 0xFFFFFFFF00000000ULL,
};

} // namespace

Simulator::Simulator(const GateForest& forest) : order(topologicalOrder(forest)), values(order.size(), 0) {
    for (size_t i = 0; i < order.size(); ++i) index[order[i].get()] = i;
}

void Simulator::run(const std::unordered_map<std::string, uint64_t>& input_words) {
    // Topological order guarantees every fanin is evaluated before the gate that reads it
    for (size_t i = 0; i < order.size(); ++i) {
        const auto& node = order[i];
        auto in = [&](size_t k) {
            if (k >= node->inputs.size()) {
                throw std::runtime_error("'" + node->name + "' (" + gateTypeName(node->type) + ") has no driver");
            }
            return values[index.at(node->inputs[k].get())];
        };

        uint64_t v = 0;
        switch (node->type) {
        case GateType::INPUT: {
            auto it = input_words.find(node->name);
            v = it == input_words.end() ? 0 : it->second;
            break;
        }
        case GateType::CONST0: v = 0; break;
        case GateType::CONST1: v = ~0ULL; break;
        case GateType::ASSIGN:
        case GateType::OUTPUT:
        case GateType::BUF:    v = in(0); break;
        case GateType::NOT:    v = ~in(0); break;
        case GateType::AND:
        case GateType::NAND:
            v = ~0ULL;
            for (size_t k = 0; k < node->inputs.size(); ++k) v &= in(k);
            if (node->type == GateType::NAND) v = ~v;
            break;
        case GateType::OR:
        case GateType::NOR:
            for (size_t k = 0; k < node->inputs.size(); ++k) v |= in(k);
            if (node->type == GateType::NOR) v = ~v;
            break;
        case GateType::XOR:
        case GateType::XNOR:
            for (size_t k = 0; k < node->inputs.size(); ++k) v ^= in(k);
            if (node->type == GateType::XNOR) v = ~v;
            break;
        default:
            throw std::runtime_error("simulator does not support " + gateTypeName(node->type) + " gates yet");
        }
        values[i] = v;
    }
}

uint64_t Simulator::value(const GateNode* node) const {
    return values[index.at(node)];
}

EquivalenceResult checkEquivalence(const GateForest& a, const GateForest& b) {
    EquivalenceResult result;

    std::map<std::string, const GateNode*> outs_a, outs_b;
    for (const auto& root : a) outs_a[root->name] = root.get();
    for (const auto& root : b) outs_b[root->name] = root.get();
    for (const auto& [name, node] : outs_a) {
        if (!outs_b.count(name)) {
            result.message = "output '" + name + "' exists only in the first design";
            return result;
        }
    }
    for (const auto& [name, node] : outs_b) {
        if (!outs_a.count(name)) {
            result.message = "output '" + name + "' exists only in the second design";
            return result;
        }
    }

    // Drive the union of both designs' inputs (an input one side ignores is still a free variable)
    std::set<std::string> name_set;
    for (const auto& name : collectInputs(a)) name_set.insert(name);
    for (const auto& name : collectInputs(b)) name_set.insert(name);
    std::vector<std::string> input_names(name_set.begin(), name_set.end());
    size_t n = input_names.size();

    Simulator sim_a(a), sim_b(b);
    result.exhaustive = n <= kMaxExhaustiveInputs;
    uint64_t words = result.exhaustive ? std::max<uint64_t>(1, (1ULL << n) / 64) : kRandomWords;
    result.patterns = result.exhaustive ? (1ULL << n) : words * 64;

    std::mt19937_64 rng(0x5eed);   // fixed seed keeps runs reproducible
    std::unordered_map<std::string, uint64_t> input_words;
    for (uint64_t w = 0; w < words; ++w) {
        for (size_t i = 0; i < n; ++i) {
            uint64_t word;
            if (!result.exhaustive) word = rng();
            else if (i < 6) word = kPatternMasks[i];
            else word = ((w >> (i - 6)) & 1) ? ~0ULL : 0;
            input_words[input_names[i]] = word;
        }
        sim_a.run(input_words);
        sim_b.run(input_words);

        for (const auto& [name, node_a] : outs_a) {
            uint64_t value_a = sim_a.value(node_a);
            uint64_t value_b = sim_b.value(outs_b[name]);
            uint64_t diff = value_a ^ value_b;
            if (!diff) continue;

            // Report the first failing pattern as a counterexample
            int bit = 0;
            while (!((diff >> bit) & 1)) ++bit;
            std::string assignment;
            for (const auto& input : input_names) {
                assignment += input + "=" + (((input_words[input] >> bit) & 1) ? "1" : "0") + " ";
            }
            result.message = "output '" + name + "' differs when " + assignment +
                             "(first design gives " + std::to_string((value_a >> bit) & 1) +
                             ", second gives " + std::to_string((value_b >> bit) & 1) + ")";
            return result;
        }
    }

    result.equivalent = true;
    return result;
}
