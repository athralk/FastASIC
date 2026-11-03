#include "../include/netlist_utils.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <unordered_map>

std::string gateTypeName(GateType type) {
    switch (type) {
    case GateType::INPUT:  return "INPUT";
    case GateType::OUTPUT: return "OUTPUT";
    case GateType::ASSIGN: return "ASSIGN";
    case GateType::AND:    return "AND";
    case GateType::OR:     return "OR";
    case GateType::NOT:    return "NOT";
    case GateType::NAND:   return "NAND";
    case GateType::NOR:    return "NOR";
    case GateType::XOR:    return "XOR";
    case GateType::XNOR:   return "XNOR";
    case GateType::BUF:    return "BUF";
    case GateType::MUX:    return "MUX";
    case GateType::DFF:    return "DFF";
    case GateType::CONST0: return "CONST0";
    case GateType::CONST1: return "CONST1";
    }
    return "GATE";
}

std::string instanceName(const GateNode& node) {
    return "_g" + std::to_string(node.id) + "_";
}

namespace {

enum class VisitState { IN_PROGRESS, DONE };

void visit(const std::shared_ptr<GateNode>& node,
           std::unordered_map<const GateNode*, VisitState>& state,
           std::vector<std::shared_ptr<GateNode>>& order) {
    auto it = state.find(node.get());
    if (it != state.end()) {
        // Reaching a node that is still on the DFS stack means we walked around a cycle
        if (it->second == VisitState::IN_PROGRESS) {
            throw std::runtime_error("combinational loop detected through '" + node->name + "'");
        }
        return;
    }
    state[node.get()] = VisitState::IN_PROGRESS;
    for (const auto& fanin : node->inputs) visit(fanin, state, order);
    state[node.get()] = VisitState::DONE;
    order.push_back(node);
}

bool isLogicGate(GateType type) {
    return type != GateType::INPUT && type != GateType::OUTPUT && type != GateType::ASSIGN &&
           type != GateType::CONST0 && type != GateType::CONST1;
}

} // namespace

std::vector<std::shared_ptr<GateNode>> topologicalOrder(const GateForest& forest) {
    std::unordered_map<const GateNode*, VisitState> state;
    std::vector<std::shared_ptr<GateNode>> order;
    for (const auto& root : forest) {
        if (root) visit(root, state, order);
    }
    return order;
}

std::vector<std::string> collectInputs(const GateForest& forest) {
    std::set<std::string> names;
    for (const auto& node : topologicalOrder(forest)) {
        if (node->type == GateType::INPUT) names.insert(node->name);
    }
    return {names.begin(), names.end()};
}

std::map<GateType, int> gateCounts(const GateForest& forest) {
    std::map<GateType, int> counts;
    for (const auto& node : topologicalOrder(forest)) {
        if (isLogicGate(node->type)) counts[node->type]++;
    }
    return counts;
}

void compactFanouts(const GateForest& forest) {
    for (const auto& node : topologicalOrder(forest)) {
        auto& outs = node->outputs;
        outs.erase(std::remove_if(outs.begin(), outs.end(),
                                  [](const std::weak_ptr<GateNode>& w) { return w.expired(); }),
                   outs.end());
    }
}
