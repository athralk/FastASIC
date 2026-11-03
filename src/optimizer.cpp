#include "../include/optimizer.hpp"
#include "../include/netlist_utils.hpp"
#include <memory>
#include <unordered_map>
#include <utility>

// Key for Structural Hashing (Strash)
struct NodeKey {
    GateType type;
    int left_id;
    int right_id;

    bool operator==(const NodeKey& other) const {
        return type == other.type && left_id == other.left_id && right_id == other.right_id;
    }
};

struct NodeKeyHash {
    std::size_t operator()(const NodeKey& k) const {
        std::size_t h = std::hash<int>()(static_cast<int>(k.type));
        h ^= std::hash<int>()(k.left_id) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(k.right_id) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

using NodePtr = std::shared_ptr<GateNode>;

class OptimizerPass {
    int next_id;
    std::unordered_map<const GateNode*, NodePtr> memo;          // source node -> optimized node
    std::unordered_map<NodeKey, NodePtr, NodeKeyHash> strash_table;
    std::unordered_map<std::string, NodePtr> inputs;            // one node per primary input
    NodePtr const0, const1;

    NodePtr constant(bool value) {
        auto& slot = value ? const1 : const0;
        if (!slot) {
            slot = std::make_shared<GateNode>(next_id++, value ? GateType::CONST1 : GateType::CONST0,
                                              value ? "1'b1" : "1'b0");
        }
        return slot;
    }

    static bool isConst(const NodePtr& n, bool value) {
        return n->type == (value ? GateType::CONST1 : GateType::CONST0);
    }

    // True when one operand is NOT of the other
    static bool isComplement(const NodePtr& a, const NodePtr& b) {
        return (a->type == GateType::NOT && a->inputs[0] == b) || (b->type == GateType::NOT && b->inputs[0] == a);
    }

    // Returns the existing equivalent gate if there is one, otherwise builds it.
    // Commutative gates sort their fanins so NAND(a, b) and NAND(b, a) hash identically.
    NodePtr hashed(GateType type, const std::string& name, NodePtr a, NodePtr b, bool commutative) {
        if (commutative && b && b->id < a->id) std::swap(a, b);
        NodeKey key{type, a->id, b ? b->id : -1};
        auto it = strash_table.find(key);
        if (it != strash_table.end()) return it->second;   // Equivalent gate already exists; reuse it!

        auto newNode = std::make_shared<GateNode>(next_id++, type, name);
        newNode->addFanin(a);
        if (b) newNode->addFanin(b);
        strash_table[key] = newNode;
        return newNode;
    }

    NodePtr makeNot(const NodePtr& a) {
        if (a->type == GateType::CONST0 || a->type == GateType::CONST1) return constant(isConst(a, false));
        // --- Peephole Optimization: Double Inversion Elimination (NOT(NOT(x)) -> x) ---
        if (a->type == GateType::NOT) return a->inputs[0];
        return hashed(GateType::NOT, "not", a, nullptr, false);
    }

    NodePtr makeNand(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false) || isConst(b, false)) return constant(true);
        if (isConst(a, true)) return makeNot(b);
        if (isConst(b, true)) return makeNot(a);
        if (a == b) return makeNot(a);                    // NAND(x, x) = NOT(x)
        if (isComplement(a, b)) return constant(true);   // NAND(x, ~x) = 1
        return hashed(GateType::NAND, "nand", a, b, true);
    }

    NodePtr makeAnd(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false) || isConst(b, false)) return constant(false);
        if (isConst(a, true)) return b;
        if (isConst(b, true)) return a;
        if (a == b) return a;
        if (isComplement(a, b)) return constant(false);
        return hashed(GateType::AND, "and", a, b, true);
    }

    NodePtr makeOr(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, true) || isConst(b, true)) return constant(true);
        if (isConst(a, false)) return b;
        if (isConst(b, false)) return a;
        if (a == b) return a;
        if (isComplement(a, b)) return constant(true);
        return hashed(GateType::OR, "or", a, b, true);
    }

    NodePtr makeXor(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false)) return b;
        if (isConst(b, false)) return a;
        if (isConst(a, true)) return makeNot(b);
        if (isConst(b, true)) return makeNot(a);
        if (a == b) return constant(false);
        if (isComplement(a, b)) return constant(true);
        return hashed(GateType::XOR, "xor", a, b, true);
    }

    NodePtr rewrite(const NodePtr& node) {
        switch (node->type) {
        case GateType::INPUT: {
            auto& slot = inputs[node->name];
            if (!slot) slot = std::make_shared<GateNode>(next_id++, GateType::INPUT, node->name);
            return slot;
        }
        case GateType::CONST0: return constant(false);
        case GateType::CONST1: return constant(true);
        // Wires, buffers and outputs used as fanins carry no logic: look straight through them
        case GateType::ASSIGN:
        case GateType::OUTPUT:
        case GateType::BUF:    return optimizeNode(node->inputs[0]);
        case GateType::NOT:    return makeNot(optimizeNode(node->inputs[0]));
        case GateType::NAND:
        case GateType::AND:
        case GateType::OR:
        case GateType::XOR:
        case GateType::NOR:
        case GateType::XNOR: {
            // Optimize the operands in a fixed order: it decides their ids, the ids decide the
            // commutative fanin order, and that decides which NAND pin (A or B) each signal uses.
            // Inside a single call's argument list C++ leaves the evaluation order unspecified.
            NodePtr a = optimizeNode(node->inputs[0]);
            NodePtr b = optimizeNode(node->inputs[1]);
            if (node->type == GateType::NAND) return makeNand(a, b);
            if (node->type == GateType::AND) return makeAnd(a, b);
            if (node->type == GateType::OR) return makeOr(a, b);
            if (node->type == GateType::XOR) return makeXor(a, b);
            return hashed(node->type, node->name, a, b, true);
        }
        default: {
            // Gates without simplification rules are copied as-is
            auto newNode = std::make_shared<GateNode>(next_id++, node->type, node->name);
            for (const auto& fanin : node->inputs) newNode->addFanin(optimizeNode(fanin));
            return newNode;
        }
        }
    }

public:
    OptimizerPass(int start_id) : next_id(start_id) {}

    NodePtr optimizeNode(const NodePtr& node) {
        auto it = memo.find(node.get());
        if (it != memo.end()) return it->second;
        auto optimized = rewrite(node);
        memo[node.get()] = optimized;
        return optimized;
    }

    // Roots keep their identity (an OUTPUT node with the port name) even when their logic is shared
    NodePtr optimizeRoot(const NodePtr& root) {
        auto opt_root = std::make_shared<GateNode>(next_id++, root->type, root->name);
        if (!root->inputs.empty()) opt_root->addFanin(optimizeNode(root->inputs[0]));
        return opt_root;
    }
};

GateForest NetlistOptimizer::optimize(const GateForest& forest) {
    GateForest optimized_forest;
    {
        // One pass over the whole netlist, so identical logic feeding different outputs is shared
        OptimizerPass pass(7000);
        for (const auto& root : forest) {
            if (root) optimized_forest.push_back(pass.optimizeRoot(root));
        }
    }
    // Gates that were built and then simplified away die with the pass; drop their stale fanout links
    compactFanouts(optimized_forest);
    return optimized_forest;
}
