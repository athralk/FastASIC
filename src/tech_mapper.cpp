#include "../include/tech_mapper.hpp"
#include "../include/netlist_utils.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

using Node = const GateNode*;

struct Match {
    const char* cell;                 // library cell name without the sg13cmos5l_ prefix and drive
    std::vector<Node> leaves;         // in the cell's pin order
};

// Cell pin order for each pattern (inputs only)
const std::unordered_map<std::string, std::vector<std::string>>& pinOrder() {
    static const std::unordered_map<std::string, std::vector<std::string>> pins = {
        {"inv_1", {"A"}}, {"nand2_1", {"A", "B"}}, {"nand3_1", {"A", "B", "C"}}, {"nand4_1", {"A", "B", "C", "D"}},
        {"and2_1", {"A", "B"}}, {"and3_1", {"A", "B", "C"}}, {"and4_1", {"A", "B", "C", "D"}},
        {"or2_1", {"A", "B"}}, {"or3_1", {"A", "B", "C"}}, {"or4_1", {"A", "B", "C", "D"}},
        {"nor2_1", {"A", "B"}}, {"nor3_1", {"A", "B", "C"}}, {"nor4_1", {"A", "B", "C", "D"}},
        {"a21oi_1", {"A1", "A2", "B1"}}, {"o21ai_1", {"A1", "A2", "B1"}}, {"a21o_1", {"A1", "A2", "B1"}},
        {"a22oi_1", {"A1", "A2", "B1", "B2"}}, {"xor2_1", {"A", "B"}}, {"xnor2_1", {"A", "B"}},
        {"mux2_1", {"A0", "A1", "S"}}, {"nand2b_1", {"A_N", "B"}}, {"nor2b_1", {"A", "B_N"}},
    };
    return pins;
}

class Mapper {
    const Design& design;
    const GateForest& forest;
    const Library& lib;
    MapGoal goal;
    std::string prefix = "sg13cmos5l_";

    std::unordered_map<Node, int> fanout;
    std::unordered_map<std::string, const LibCell*> cells;   // short name -> cell (present in the library)
    std::unordered_map<std::string, double> cell_delay;      // typical delay estimate

    struct Best {
        double cost = kInf;       // area of the tree below (area mode) or arrival (delay mode)
        double area = kInf;
        Match match{nullptr, {}};
    };
    std::unordered_map<Node, Best> best;

    bool isNand(Node n) const { return n->type == GateType::NAND && n->inputs.size() == 2; }
    bool isInv(Node n) const { return n->type == GateType::NOT; }
    Node in(Node n, size_t k) const { return n->inputs[k].get(); }
    bool single(Node n) const { auto it = fanout.find(n); return it != fanout.end() && it->second == 1; }   // may be absorbed
    bool leafKind(Node n) const { return !isNand(n) && !isInv(n); }   // inputs, constants

    void add(std::vector<Match>& out, const char* cell, std::vector<Node> leaves) const {
        if (cells.count(cell)) out.push_back({cell, std::move(leaves)});
    }

    // INV(x) inside a pattern: x if n is an inverter. Shared inverters may be absorbed too
    // (inverter duplication): the cell builds its own, and the shared one is only instantiated
    // if some other gate still uses it
    Node invOf(Node n) const { return isInv(n) ? in(n, 0) : nullptr; }
    // NAND inside a pattern
    bool nandIn(Node n) const { return isNand(n) && single(n); }

    std::vector<Match> matches(Node v) const {
        std::vector<Match> out;
        if (isInv(v)) {
            Node x = in(v, 0);
            add(out, "inv_1", {x});
            if (nandIn(x)) {
                Node a = in(x, 0), b = in(x, 1);
                add(out, "and2_1", {a, b});
                Node na = invOf(a), nb = invOf(b);
                if (na && nb) add(out, "nor2_1", {na, nb});
                if (na && !nb) add(out, "nor2b_1", {na, b});              // !A & B_N
                if (nb && !na) add(out, "nor2b_1", {nb, a});
                for (int s = 0; s < 2; ++s) {
                    Node p = s ? b : a, q = s ? a : b;
                    Node ip = invOf(p);
                    if (ip && nandIn(ip)) add(out, "and3_1", {in(ip, 0), in(ip, 1), q});
                    if (nandIn(p)) {
                        Node iq = invOf(q);
                        if (iq) add(out, "a21oi_1", {in(p, 0), in(p, 1), iq});
                    }
                    // NOR3: x = NAND(INV a, INV(NAND(INV b, INV c)))
                    Node iq = invOf(q);
                    if (ip && iq && nandIn(iq)) {
                        Node b2 = invOf(in(iq, 0)), c2 = invOf(in(iq, 1));
                        if (b2 && c2) add(out, "nor3_1", {ip, b2, c2});
                    }
                }
                Node ia = invOf(a), ib = invOf(b);
                if (ia && ib && nandIn(ia) && nandIn(ib)) {
                    add(out, "and4_1", {in(ia, 0), in(ia, 1), in(ib, 0), in(ib, 1)});
                    Node w = invOf(in(ia, 0)), x2 = invOf(in(ia, 1)), y = invOf(in(ib, 0)), z = invOf(in(ib, 1));
                    if (w && x2 && y && z) add(out, "nor4_1", {w, x2, y, z});
                }
                if (nandIn(a) && nandIn(b)) add(out, "a22oi_1", {in(a, 0), in(a, 1), in(b, 0), in(b, 1)});
                if (Node xa = xorLeaves(x, 0)) add(out, "xnor2_1", {xa, xorLeaves(x, 1)});
            }
            return out;
        }
        if (!isNand(v)) return out;
        Node a = in(v, 0), b = in(v, 1);
        add(out, "nand2_1", {a, b});
        Node ia = invOf(a), ib = invOf(b);
        if (ia && ib) add(out, "or2_1", {ia, ib});
        if (ia && !ib) add(out, "nand2b_1", {ia, b});
        if (ib && !ia) add(out, "nand2b_1", {ib, a});
        for (int s = 0; s < 2; ++s) {
            Node p = s ? b : a, q = s ? a : b;
            Node ip = invOf(p), iq = invOf(q);
            if (ip && nandIn(ip)) add(out, "nand3_1", {in(ip, 0), in(ip, 1), q});
            if (nandIn(p)) {
                Node x0 = invOf(in(p, 0)), x1 = invOf(in(p, 1));
                if (x0 && x1) add(out, "o21ai_1", {x0, x1, q});
                if (iq) add(out, "a21o_1", {in(p, 0), in(p, 1), iq});
            }
            // OR3: NAND(INV a, INV(NAND(INV b, INV c)))
            if (ip && iq && nandIn(iq)) {
                Node b2 = invOf(in(iq, 0)), c2 = invOf(in(iq, 1));
                if (b2 && c2) add(out, "or3_1", {ip, b2, c2});
            }
        }
        if (ia && ib && nandIn(ia) && nandIn(ib)) {
            add(out, "nand4_1", {in(ia, 0), in(ia, 1), in(ib, 0), in(ib, 1)});
            Node w = invOf(in(ia, 0)), x = invOf(in(ia, 1)), y = invOf(in(ib, 0)), z = invOf(in(ib, 1));
            if (w && x && y && z) add(out, "or4_1", {w, x, y, z});
        }
        if (Node xa = xorLeaves(v, 0)) add(out, "xor2_1", {xa, xorLeaves(v, 1)});
        // MUX2: NAND(NAND(INV s, a0), NAND(s, a1))
        if (nandIn(a) && nandIn(b)) {
            for (int s = 0; s < 2; ++s) {
                Node p = s ? b : a, q = s ? a : b;   // p holds INV(sel)
                for (int k = 0; k < 2; ++k) {
                    Node sel = invOf(in(p, k));
                    if (!sel) continue;
                    Node a0 = in(p, 1 - k);
                    for (int j = 0; j < 2; ++j) {
                        if (in(q, j) == sel) add(out, "mux2_1", {a0, in(q, 1 - j), sel});
                    }
                }
            }
        }
        return out;
    }

    // XOR(a, b) = NAND(NAND(a, n), NAND(b, n)) with n = NAND(a, b) read only by those two NANDs.
    // Returns leaf k (0: a, 1: b) or nullptr when v is not that structure.
    Node xorLeaves(Node v, int k) const {
        if (!isNand(v)) return nullptr;
        Node p = in(v, 0), q = in(v, 1);
        if (!nandIn(p) || !nandIn(q)) return nullptr;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                Node n = in(p, i);
                if (n != in(q, j) || !isNand(n)) continue;   // n may have other readers: then it stays as well
                Node a = in(p, 1 - i), b = in(q, 1 - j);
                if ((in(n, 0) == a && in(n, 1) == b) || (in(n, 0) == b && in(n, 1) == a)) return k == 0 ? a : b;
            }
        }
        return nullptr;
    }

    // A node is a tree root (mapped on its own) when several gates read it
    bool isRoot(Node n) const { return !single(n); }

    void solve(Node v) {
        Best b;
        for (Match& m : matches(v)) {
            double area = cells.at(m.cell)->area;
            double cost;
            if (goal == MapGoal::AREA) {
                cost = area;
                for (Node l : m.leaves) {
                    if (!leafKind(l) && !isRoot(l)) cost += best.at(l).cost;
                }
            } else {
                double latest = 0;
                for (Node l : m.leaves) {
                    if (!leafKind(l)) latest = std::max(latest, best.at(l).cost);
                }
                cost = latest + cell_delay.at(m.cell);
            }
            for (Node l : m.leaves) {
                if (!leafKind(l) && !isRoot(l)) area += best.at(l).area;
            }
            bool better = cost < b.cost - 1e-9 || (std::abs(cost - b.cost) <= 1e-9 && area < b.area);
            if (better) {
                b.cost = cost;
                b.area = area;
                b.match = std::move(m);
            }
        }
        if (!b.match.cell) throw std::logic_error("no cell matches gate " + std::to_string(v->id));
        best[v] = std::move(b);
    }

public:
    Mapper(const Design& d, const GateForest& f, const Library& l, MapGoal g) : design(d), forest(f), lib(l), goal(g) {
        for (const auto& [name, pins] : pinOrder()) {
            if (const LibCell* c = lib.find(prefix + name)) cells[name] = c;
        }
        if (!cells.count("nand2_1") || !cells.count("inv_1")) throw std::runtime_error("library has no nand2_1 / inv_1 cells");
        // Typical delay: worst arc at a 60 ps input slew driving two average input pins (~6 fF)
        for (const auto& [name, cell] : cells) {
            double worst = 0;
            TableArgs args;
            args.input_transition = 60;
            args.output_cap = 6;
            for (const LibArc& arc : cell->arcs) {
                if (arc.type != TimingType::COMBINATIONAL) continue;
                worst = std::max({worst, arc.cell_rise.lookup(args), arc.cell_fall.lookup(args)});
            }
            cell_delay[name] = worst;
        }
    }

    MapResult run() {
        auto order = topologicalOrder(forest);
        for (const auto& n : order) {
            for (const auto& f : n->inputs) fanout[f.get()]++;
        }
        for (const auto& n : order) {
            if (isNand(n.get()) || isInv(n.get())) solve(n.get());
            else if (n->type != GateType::INPUT && n->type != GateType::OUTPUT && n->type != GateType::CONST0 &&
                     n->type != GateType::CONST1) {
                throw std::runtime_error("tech mapping needs the optimized NAND2 + INV netlist, found " + gateTypeName(n->type));
            }
        }

        MapResult result;
        CellNetlist& nl = result.netlist;
        nl.name = design.name;
        nl.ports = design.ports;
        std::unordered_map<std::string, int> input_net;
        for (const auto& p : design.ports) {
            int width = std::abs(p.msb - p.lsb) + 1;
            for (int k = 0; k < width; ++k) {
                int bit = p.msb >= p.lsb ? p.lsb + k : p.lsb - k;
                std::string name = p.is_vector ? p.name + "[" + std::to_string(bit) + "]" : p.name;
                if (p.is_input) {
                    int net = nl.addNet(name);
                    input_net[name] = net;
                    nl.port_bits.push_back({name, true, net});
                }
            }
        }

        std::unordered_map<Node, int> net_of;
        std::function<int(Node)> emit = [&](Node v) -> int {
            if (v->type == GateType::CONST0) return CellNetlist::kConst0;
            if (v->type == GateType::CONST1) return CellNetlist::kConst1;
            if (v->type == GateType::INPUT) return input_net.at(v->name);
            auto it = net_of.find(v);
            if (it != net_of.end()) return it->second;
            const Match& m = best.at(v).match;
            const LibCell* cell = cells.at(m.cell);
            CellNetlist::Instance inst;
            inst.cell = cell;
            inst.pin_net.assign(cell->pins.size(), -1);
            const auto& pins = pinOrder().at(m.cell);
            for (size_t k = 0; k < m.leaves.size(); ++k) inst.pin_net[cell->pinIndex(pins[k])] = emit(m.leaves[k]);
            inst.name = "_m" + std::to_string(nl.instances.size()) + "_";   // after the fanin cells are numbered
            int net = nl.addNet("_n" + std::to_string(v->id) + "_");
            inst.pin_net[cell->outputPin()] = net;
            nl.instances.push_back(std::move(inst));
            result.cell_counts[cell->name]++;
            result.area += cell->area;
            return net_of[v] = net;
        };
        for (const auto& root : forest) {
            int net = emit(root->inputs.at(0).get());
            nl.port_bits.push_back({root->name, false, net});
        }
        return result;
    }
};

} // namespace

MapResult mapToCells(const Design& design, const GateForest& nand_inv, const Library& lib, MapGoal goal) {
    return Mapper(design, nand_inv, lib, goal).run();
}
