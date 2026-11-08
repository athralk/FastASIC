#include "../include/balance.hpp"
#include "../include/netlist_utils.hpp"
#include <queue>
#include <unordered_map>

namespace {

bool associative(GateType t) { return t == GateType::AND || t == GateType::OR || t == GateType::XOR; }

class Balancer {
    std::unordered_map<const GateNode*, int> fanout;
    std::unordered_map<const GateNode*, std::shared_ptr<GateNode>> built;
    std::unordered_map<const GateNode*, int> depth_of;   // keyed by the rebuilt node
    int next_id = 9000;

    int depth(const std::shared_ptr<GateNode>& n) {
        auto it = depth_of.find(n.get());
        return it == depth_of.end() ? 0 : it->second;
    }

    // Operands of the maximal single-type chain rooted at n
    void collect(const GateNode* n, GateType type, std::vector<const GateNode*>& leaves) {
        for (const auto& in : n->inputs) {
            if (in->type == type && fanout[in.get()] == 1) collect(in.get(), type, leaves);
            else leaves.push_back(in.get());
        }
    }

public:
    explicit Balancer(const GateForest& forest) {
        for (const auto& n : topologicalOrder(forest)) {
            for (const auto& in : n->inputs) fanout[in.get()]++;
        }
    }

    std::shared_ptr<GateNode> rebuild(const GateNode* n) {
        auto it = built.find(n);
        if (it != built.end()) return it->second;
        std::shared_ptr<GateNode> result;
        if (associative(n->type) && n->inputs.size() >= 2) {
            std::vector<const GateNode*> leaves;
            collect(n, n->type, leaves);
            using Item = std::pair<int, std::shared_ptr<GateNode>>;
            auto later = [](const Item& a, const Item& b) { return a.first > b.first; };
            std::priority_queue<Item, std::vector<Item>, decltype(later)> heap(later);
            for (const GateNode* leaf : leaves) {
                auto r = rebuild(leaf);
                heap.push({depth(r), r});
            }
            while (heap.size() > 1) {
                Item a = heap.top(); heap.pop();
                Item b = heap.top(); heap.pop();
                auto g = std::make_shared<GateNode>(next_id++, n->type, n->name);
                g->addFanin(a.second);
                g->addFanin(b.second);
                int d = std::max(a.first, b.first) + 1;
                depth_of[g.get()] = d;
                heap.push({d, g});
            }
            result = heap.top().second;
        } else {
            result = std::make_shared<GateNode>(n->id, n->type, n->name);
            int d = 0;
            for (const auto& in : n->inputs) {
                auto r = rebuild(in.get());
                result->addFanin(r);
                d = std::max(d, depth(r));
            }
            if (!n->inputs.empty() && n->type != GateType::OUTPUT && n->type != GateType::ASSIGN && n->type != GateType::BUF) d++;
            depth_of[result.get()] = d;
        }
        return built[n] = result;
    }
};

} // namespace

GateForest balanceAssociative(const GateForest& forest) {
    Balancer b(forest);
    GateForest out;
    for (const auto& root : forest) out.push_back(b.rebuild(root.get()));
    return out;
}
