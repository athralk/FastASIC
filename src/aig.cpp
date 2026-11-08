#include "../include/aig.hpp"
#include "../include/netlist_utils.hpp"
#include <stdexcept>

Aig::Aig() {
    fanin0_.push_back(kNoFanin - 1);   // node 0: the constant (neither an input nor an AND)
    fanin1_.push_back(kNoFanin - 1);
}

AigLit Aig::input(const std::string& name) {
    auto it = input_by_name_.find(name);
    if (it != input_by_name_.end()) return aigLiteral(it->second);
    uint32_t node = static_cast<uint32_t>(nodeCount());
    fanin0_.push_back(kNoFanin);
    fanin1_.push_back(kNoFanin);
    input_nodes_.push_back(node);
    input_names_.push_back(name);
    input_by_name_[name] = node;
    return aigLiteral(node);
}

AigLit Aig::makeAnd(AigLit a, AigLit b) {
    if (a > b) std::swap(a, b);           // AND is commutative: one canonical order
    if (a == kAigFalse) return kAigFalse; // 0 & x = 0
    if (a == kAigTrue) return b;          // 1 & x = x
    if (a == b) return a;                 // x & x = x
    if (a == aigNot(b)) return kAigFalse; // x & !x = 0

    uint64_t key = (static_cast<uint64_t>(a) << 32) | b;
    auto it = strash_.find(key);
    if (it != strash_.end()) return aigLiteral(it->second);

    uint32_t node = static_cast<uint32_t>(nodeCount());
    fanin0_.push_back(a);
    fanin1_.push_back(b);
    strash_.emplace(key, node);
    return aigLiteral(node);
}

AigLit Aig::makeXor(AigLit a, AigLit b) {
    // a ^ b = !(a & b) & !(!a & !b): three AND nodes
    return makeAnd(aigNot(makeAnd(a, b)), aigNot(makeAnd(aigNot(a), aigNot(b))));
}

std::vector<std::pair<std::string, AigLit>> addToAig(Aig& aig, const GateForest& forest) {
    std::unordered_map<const GateNode*, AigLit> lit_of;
    for (const auto& node : topologicalOrder(forest)) {
        auto in = [&](size_t k) {
            if (k >= node->inputs.size()) {
                throw std::runtime_error("'" + node->name + "' (" + gateTypeName(node->type) + ") has no driver");
            }
            return lit_of.at(node->inputs[k].get());
        };

        AigLit lit = kAigFalse;
        switch (node->type) {
        case GateType::INPUT:  lit = aig.input(node->name); break;
        case GateType::CONST0: lit = kAigFalse; break;
        case GateType::CONST1: lit = kAigTrue; break;
        case GateType::ASSIGN:
        case GateType::OUTPUT:
        case GateType::BUF:    lit = in(0); break;
        case GateType::NOT:    lit = aigNot(in(0)); break;
        case GateType::AND:
        case GateType::NAND:
            lit = kAigTrue;
            for (size_t k = 0; k < node->inputs.size(); ++k) lit = aig.makeAnd(lit, in(k));
            if (node->type == GateType::NAND) lit = aigNot(lit);
            break;
        case GateType::OR:
        case GateType::NOR:
            for (size_t k = 0; k < node->inputs.size(); ++k) lit = aig.makeOr(lit, in(k));
            if (node->type == GateType::NOR) lit = aigNot(lit);
            break;
        case GateType::XOR:
        case GateType::XNOR:
            for (size_t k = 0; k < node->inputs.size(); ++k) lit = aig.makeXor(lit, in(k));
            if (node->type == GateType::XNOR) lit = aigNot(lit);
            break;
        default:
            throw std::runtime_error("AIG conversion does not support " + gateTypeName(node->type) + " gates yet");
        }
        lit_of[node.get()] = lit;
    }

    std::vector<std::pair<std::string, AigLit>> outputs;
    for (const auto& root : forest) outputs.emplace_back(root->name, lit_of.at(root.get()));
    return outputs;
}

std::vector<uint64_t> simulateAig(const Aig& aig, const std::vector<uint64_t>& input_words) {
    std::vector<uint64_t> words(aig.nodeCount(), 0);   // node 0 (constant false) stays 0
    const auto& inputs = aig.inputNodes();
    for (size_t i = 0; i < inputs.size() && i < input_words.size(); ++i) words[inputs[i]] = input_words[i];
    for (uint32_t node = 1; node < aig.nodeCount(); ++node) {
        if (aig.isAnd(node)) words[node] = aigValue(words, aig.fanin0(node)) & aigValue(words, aig.fanin1(node));
    }
    return words;
}
