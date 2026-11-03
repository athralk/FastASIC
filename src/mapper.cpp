#include "../include/mapper.hpp"
#include "../include/netlist_utils.hpp"
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <unordered_map>

// Rewrites gates into NAND and NOT primitives.
// Results are memoized per source node so shared logic is mapped once and stays shared;
// without this, every reconvergent path would duplicate its whole fanin cone.
class MapperPass {
    int next_id;
    std::unordered_map<const GateNode*, std::shared_ptr<GateNode>> memo;

    std::shared_ptr<GateNode> make(GateType type, const std::string& name,
                                   std::initializer_list<std::shared_ptr<GateNode>> fanins) {
        auto node = std::make_shared<GateNode>(next_id++, type, name);
        for (const auto& fanin : fanins) node->addFanin(fanin);
        return node;
    }
    std::shared_ptr<GateNode> makeNot(const std::shared_ptr<GateNode>& a) {
        return make(GateType::NOT, "not", {a});
    }
    std::shared_ptr<GateNode> makeNand(const std::shared_ptr<GateNode>& a, const std::shared_ptr<GateNode>& b) {
        return make(GateType::NAND, "nand", {a, b});
    }

    // 4-NAND implementation of XOR:
    // T1 = NAND(A, B), T2 = NAND(A, T1), T3 = NAND(B, T1), XOR = NAND(T2, T3)
    std::shared_ptr<GateNode> makeXor(const std::shared_ptr<GateNode>& a, const std::shared_ptr<GateNode>& b) {
        auto t1 = makeNand(a, b);
        auto t2 = makeNand(a, t1);
        auto t3 = makeNand(b, t1);
        return makeNand(t2, t3);
    }

    std::shared_ptr<GateNode> rewrite(const std::shared_ptr<GateNode>& node) {
        // Pass through inputs/constants directly
        if (node->type == GateType::INPUT || node->type == GateType::CONST0 || node->type == GateType::CONST1) {
            return std::make_shared<GateNode>(next_id++, node->type, node->name);
        }

        // Pass through structural nodes (ASSIGN / OUTPUT) so net names survive mapping
        if (node->type == GateType::ASSIGN || node->type == GateType::OUTPUT) {
            auto newNode = std::make_shared<GateNode>(next_id++, node->type, node->name);
            if (!node->inputs.empty()) newNode->addFanin(map(node->inputs[0]));
            return newNode;
        }

        auto operand = [&](size_t i) {
            if (i >= node->inputs.size()) {
                throw std::runtime_error(gateTypeName(node->type) + " gate is missing input " + std::to_string(i));
            }
            return map(node->inputs[i]);
        };

        switch (node->type) {
        // A buffer is logically a wire
        case GateType::BUF:  return operand(0);
        // Inverter remains an Inverter (NOT)
        case GateType::NOT:  return makeNot(operand(0));
        case GateType::NAND: return makeNand(operand(0), operand(1));
        // AND(A, B) -> NOT(NAND(A, B))
        case GateType::AND:  return makeNot(makeNand(operand(0), operand(1)));
        // OR(A, B) -> NAND(NOT(A), NOT(B)) using De Morgan's Law
        case GateType::OR:   return makeNand(makeNot(operand(0)), makeNot(operand(1)));
        // NOR(A, B) -> NOT(OR(A, B))
        case GateType::NOR:  return makeNot(makeNand(makeNot(operand(0)), makeNot(operand(1))));
        case GateType::XOR:  return makeXor(operand(0), operand(1));
        // XNOR(A, B) -> NOT(XOR(A, B))
        case GateType::XNOR: return makeNot(makeXor(operand(0), operand(1)));
        default:
            throw std::runtime_error("technology mapper does not support " + gateTypeName(node->type) + " gates yet");
        }
    }

public:
    explicit MapperPass(int start_id) : next_id(start_id) {}

    std::shared_ptr<GateNode> map(const std::shared_ptr<GateNode>& node) {
        if (!node) return nullptr;
        auto it = memo.find(node.get());
        if (it != memo.end()) return it->second;
        auto mapped = rewrite(node);
        memo[node.get()] = mapped;
        return mapped;
    }
};

GateForest TechnologyMapper::mapToNandInverter(const GateForest& forest) {
    GateForest mappedForest;
    MapperPass pass(5000); // Unique ID offset for mapped canonical nodes
    for (const auto& root : forest) {
        mappedForest.push_back(pass.map(root));
    }
    return mappedForest;
}
