#include "../include/cut_mapper.hpp"
#include "../include/aig.hpp"
#include "../include/netlist_builder.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace {

using TT = uint64_t;   // truth table over up to 6 variables, replicated to 64 bits
constexpr int kMaxLeaves = 6;
constexpr int kPriorityCuts = 8;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr TT kVar[6] = {0xAAAAAAAAAAAAAAAAULL, 0xCCCCCCCCCCCCCCCCULL, 0xF0F0F0F0F0F0F0F0ULL,
                        0xFF00FF00FF00FF00ULL, 0xFFFF0000FFFF0000ULL, 0xFFFFFFFF00000000ULL};

TT replicate(TT t, int vars) {
    for (int b = 1 << vars; b < 64; b <<= 1) t |= t << b;
    return t;
}

bool dependsOn(TT t, int v) {
    int shift = 1 << v;
    return ((t & kVar[v]) >> shift) != (t & ~kVar[v]);
}

// ---------------------------------------------------------------------------------------------
// Liberty functions compiled to a tiny stack program, evaluated on truth tables
// ---------------------------------------------------------------------------------------------
struct Program {
    enum Op : uint8_t { VAR, ZERO, ONE, NOT, AND, OR, XOR };
    std::vector<std::pair<Op, int>> ops;
    std::vector<std::string> vars;   // input pin names, in variable order

    TT eval(const TT* values) const {
        TT stack[32] = {};
        int sp = 0;
        for (const auto& [op, arg] : ops) {
            switch (op) {
            case VAR: stack[sp++] = values[arg]; break;
            case ZERO: stack[sp++] = 0; break;
            case ONE: stack[sp++] = ~0ULL; break;
            case NOT: stack[sp - 1] = ~stack[sp - 1]; break;
            case AND: sp--; stack[sp - 1] &= stack[sp]; break;
            case OR: sp--; stack[sp - 1] |= stack[sp]; break;
            case XOR: sp--; stack[sp - 1] ^= stack[sp]; break;
            }
        }
        return stack[0];
    }
};

class Compiler {   // ! (and postfix ') > ^ > * & juxtaposition > + |
    const std::string& f;
    size_t i = 0;
    Program& p;
    void space() { while (i < f.size() && f[i] == ' ') i++; }
    int var(const std::string& name) {
        for (size_t k = 0; k < p.vars.size(); ++k) {
            if (p.vars[k] == name) return static_cast<int>(k);
        }
        p.vars.push_back(name);
        return static_cast<int>(p.vars.size()) - 1;
    }
    void primary() {
        space();
        if (i >= f.size()) throw std::runtime_error("bad function " + f);
        if (f[i] == '!') { i++; primary(); p.ops.push_back({Program::NOT, 0}); }
        else if (f[i] == '(') { i++; orExpr(); space(); i++; }
        else if (f[i] == '0' || f[i] == '1') { p.ops.push_back({f[i] == '1' ? Program::ONE : Program::ZERO, 0}); i++; }
        else {
            size_t start = i;
            while (i < f.size() && (std::isalnum(static_cast<unsigned char>(f[i])) || f[i] == '_')) i++;
            if (start == i) throw std::runtime_error("bad function " + f);
            p.ops.push_back({Program::VAR, var(f.substr(start, i - start))});
        }
        space();
        while (i < f.size() && f[i] == '\'') { i++; p.ops.push_back({Program::NOT, 0}); space(); }
    }
    void xorExpr() { primary(); for (space(); i < f.size() && f[i] == '^'; space()) { i++; primary(); p.ops.push_back({Program::XOR, 0}); } }
    void andExpr() {
        xorExpr();
        for (;;) {
            space();
            if (i < f.size() && (f[i] == '*' || f[i] == '&')) { i++; xorExpr(); p.ops.push_back({Program::AND, 0}); }
            else if (i < f.size() && (f[i] == '(' || f[i] == '!' || std::isalnum(static_cast<unsigned char>(f[i])))) { xorExpr(); p.ops.push_back({Program::AND, 0}); }
            else return;
        }
    }
    void orExpr() { andExpr(); for (space(); i < f.size() && (f[i] == '+' || f[i] == '|'); space()) { i++; andExpr(); p.ops.push_back({Program::OR, 0}); } }

public:
    Compiler(const std::string& fn, Program& prog) : f(fn), p(prog) {}
    void run() { orExpr(); }
};

// ---------------------------------------------------------------------------------------------
// The library as a table: (leaf count, truth table) -> the cells that implement it
// ---------------------------------------------------------------------------------------------
struct CellMatch {
    const LibCell* cell = nullptr;
    int inputs = 0;
    std::array<int, kMaxLeaves> pin{};      // cell input k is library pin pin[k] ...
    std::array<uint8_t, kMaxLeaves> leaf{};  // ... driven by cut leaf leaf[k]
    uint8_t neg = 0;                         // bit k: that leaf is used inverted
    double delay = 0;
};

struct KeyHash {
    size_t operator()(const std::pair<int, TT>& k) const { return std::hash<TT>()(k.second * 0x9E3779B97F4A7C15ULL + k.first); }
};

class MatchTable {
    // For each function: the smallest cell and the fastest cell
    std::unordered_map<std::pair<int, TT>, std::array<CellMatch, 2>, KeyHash> table;

public:
    const LibCell* inverter = nullptr;
    double inverter_delay = 0;

    explicit MatchTable(const Library& lib) {
        for (const LibCell& cell : lib.cells) {
            if (cell.is_sequential || cell.is_macro) continue;
            int out = -1, outputs = 0;
            for (size_t k = 0; k < cell.pins.size(); ++k) {
                if (cell.pins[k].dir == PinDir::OUTPUT) { outputs++; out = static_cast<int>(k); }
            }
            if (outputs != 1 || cell.pins[out].function.empty() || !cell.pins[out].three_state.empty()) continue;
            Program prog;
            try { Compiler(cell.pins[out].function, prog).run(); } catch (const std::exception&) { continue; }
            int n = static_cast<int>(prog.vars.size());
            if (n < 1 || n > kMaxLeaves) continue;
            std::array<int, kMaxLeaves> pins{};
            bool ok = true;
            for (int k = 0; k < n; ++k) {
                pins[k] = cell.pinIndex(prog.vars[k]);
                ok &= pins[k] >= 0 && cell.pins[pins[k]].dir == PinDir::INPUT;
            }
            if (!ok) continue;
            TT base = replicate(prog.eval(kVar) & ((n == 6) ? ~0ULL : ((1ULL << (1 << n)) - 1)), n);
            bool essential = true;
            for (int v = 0; v < n; ++v) essential &= dependsOn(base, v);
            if (!essential) continue;
            if (n == 1 && base == kVar[0]) continue;   // buffers do nothing for the logic

            // Typical delay: worst arc at a 60 ps input slew driving two average inputs (~6 fF)
            double delay = 0;
            TableArgs args;
            args.input_transition = 60;
            args.output_cap = 6;
            for (const LibArc& arc : cell.arcs) {
                if (arc.type == TimingType::COMBINATIONAL) delay = std::max({delay, arc.cell_rise.lookup(args), arc.cell_fall.lookup(args)});
            }
            if (n == 1 && base == ~kVar[0] && (!inverter || cell.area < inverter->area)) {
                inverter = &cell;
                inverter_delay = delay;
            }

            // Every way of connecting the cut leaves to the cell's inputs
            std::array<int, kMaxLeaves> perm{};
            for (int k = 0; k < n; ++k) perm[k] = k;
            do {
                for (int neg = 0; neg < (1 << n); ++neg) {
                    TT values[kMaxLeaves];
                    for (int k = 0; k < n; ++k) values[k] = kVar[perm[k]] ^ (((neg >> k) & 1) ? ~0ULL : 0ULL);
                    TT t = prog.eval(values);
                    CellMatch m;
                    m.cell = &cell;
                    m.inputs = n;
                    m.neg = static_cast<uint8_t>(neg);
                    m.delay = delay;
                    for (int k = 0; k < n; ++k) {
                        m.pin[k] = pins[k];
                        m.leaf[k] = static_cast<uint8_t>(perm[k]);
                    }
                    auto& slot = table[{n, t}];
                    if (!slot[0].cell || cell.area < slot[0].cell->area) slot[0] = m;
                    if (!slot[1].cell || delay < slot[1].delay) slot[1] = m;
                }
            } while (std::next_permutation(perm.begin(), perm.begin() + n));
        }
        if (!inverter) throw std::runtime_error("the library has no inverter");
    }

    const std::array<CellMatch, 2>* find(int n, TT t) const {
        auto it = table.find({n, t});
        return it == table.end() ? nullptr : &it->second;
    }
};

// ---------------------------------------------------------------------------------------------
// Cuts
// ---------------------------------------------------------------------------------------------
struct Cut {
    std::array<uint32_t, kMaxLeaves> leaves{};
    int size = 0;
    TT truth = 0;
    double cost = kInf;   // best match cost (either polarity), for ranking
};

// t is a function of a's leaves; re-express it over the leaves L (a superset)
TT expand(TT t, const Cut& a, const std::array<uint32_t, kMaxLeaves>& L, int n) {
    if (a.size == n) return t;   // same leaves (L is sorted and contains a's)
    int pos[kMaxLeaves];
    for (int j = 0; j < a.size; ++j) pos[j] = static_cast<int>(std::find(L.begin(), L.begin() + n, a.leaves[j]) - L.begin());
    TT out = 0;
    for (int m = 0; m < (1 << n); ++m) {
        int idx = 0;
        for (int j = 0; j < a.size; ++j) idx |= ((m >> pos[j]) & 1) << j;
        if ((t >> idx) & 1) out |= 1ULL << m;
    }
    return replicate(out, n);
}

// Drops leaves the function does not depend on
void minimize(Cut& c) {
    for (int v = c.size - 1; v >= 0; --v) {
        if (dependsOn(c.truth, v)) continue;
        TT out = 0;
        int n = c.size - 1;
        for (int m = 0; m < (1 << n); ++m) {
            int full = (m & ((1 << v) - 1)) | ((m >> v) << (v + 1));   // insert a 0 at position v
            if ((c.truth >> full) & 1) out |= 1ULL << m;
        }
        for (int j = v; j < n; ++j) c.leaves[j] = c.leaves[j + 1];
        c.size = n;
        c.truth = replicate(out, n);
    }
}

struct Choice {   // how (node, polarity) is implemented
    enum Kind { NONE, PASS, INVERT, CELL } kind = NONE;
    double cost = kInf;      // area flow (area mode) or arrival (delay mode)
    double area = kInf;      // area flow, the tie-breaker in delay mode
    Cut cut;
    const CellMatch* match = nullptr;
};

class CutMapper {
    const Design& design;
    const GateForest& forest;
    const Library& lib;
    MapGoal goal;
    MatchTable table;
    Aig aig;
    std::vector<std::pair<std::string, AigLit>> outputs;
    std::vector<std::vector<Cut>> cuts;
    std::vector<std::array<Choice, 2>> best;
    std::vector<double> refs;

    double leafCost(uint32_t leaf, int phase, double& area) const {
        const Choice& c = best[leaf][phase];
        double r = std::max(1.0, refs[leaf]);
        area += c.area / r;
        return goal == MapGoal::AREA ? c.cost / r : c.cost;
    }

    // Best implementation of the cut's function in each polarity
    void matchCut(const Cut& cut, std::array<Choice, 2>& direct) const {
        for (int phase = 0; phase < 2; ++phase) {
            TT f = phase ? ~cut.truth : cut.truth;
            const auto* m = table.find(cut.size, f);
            if (!m) continue;
            for (const CellMatch& cm : *m) {
                double area = cm.cell->area, cost;
                if (goal == MapGoal::AREA) {
                    cost = cm.cell->area;
                    for (int k = 0; k < cm.inputs; ++k) cost += leafCost(cut.leaves[cm.leaf[k]], (cm.neg >> k) & 1, area);
                    area = cost;
                } else {
                    double latest = 0;
                    for (int k = 0; k < cm.inputs; ++k) latest = std::max(latest, leafCost(cut.leaves[cm.leaf[k]], (cm.neg >> k) & 1, area));
                    cost = latest + cm.delay;
                }
                Choice& d = direct[phase];
                if (cost < d.cost - 1e-9 || (std::abs(cost - d.cost) <= 1e-9 && area < d.area)) {
                    d.kind = Choice::CELL;
                    d.cost = cost;
                    d.area = area;
                    d.cut = cut;
                    d.match = &cm;
                }
            }
        }
    }

    void relaxInverter(std::array<Choice, 2>& b) const {
        double inv_cost = goal == MapGoal::AREA ? table.inverter->area : table.inverter_delay;
        std::array<Choice, 2> direct = b;
        for (int phase = 0; phase < 2; ++phase) {
            const Choice& other = direct[1 - phase];
            double cost = other.cost + inv_cost, area = other.area + table.inverter->area;
            if (cost < b[phase].cost - 1e-9 || (std::abs(cost - b[phase].cost) <= 1e-9 && area < b[phase].area)) {
                b[phase] = Choice();
                b[phase].kind = Choice::INVERT;
                b[phase].cost = cost;
                b[phase].area = area;
            }
        }
    }

    void mapOnce() {
        size_t n = aig.nodeCount();
        cuts.assign(n, {});
        best.assign(n, {});
        for (uint32_t node = 0; node < n; ++node) {
            Cut trivial;
            trivial.leaves[0] = node;
            trivial.size = 1;
            trivial.truth = kVar[0];
            trivial.cost = 0;
            if (!aig.isAnd(node)) {   // inputs (and the constant) are free in their own polarity
                best[node][0].kind = Choice::PASS;
                best[node][0].cost = best[node][0].area = 0;
                relaxInverter(best[node]);
                cuts[node] = {trivial};
                continue;
            }
            AigLit f0 = aig.fanin0(node), f1 = aig.fanin1(node);
            std::vector<Cut> found;
            for (const Cut& a : cuts[aigNode(f0)]) {
                for (const Cut& b : cuts[aigNode(f1)]) {
                    Cut c;
                    int i = 0, j = 0, k = 0;
                    bool fits = true;
                    while (i < a.size || j < b.size) {
                        uint32_t next = j >= b.size || (i < a.size && a.leaves[i] < b.leaves[j]) ? a.leaves[i++]
                                      : i >= a.size || b.leaves[j] < a.leaves[i]                  ? b.leaves[j++]
                                                                                                  : (j++, a.leaves[i++]);
                        if (k == kMaxLeaves) { fits = false; break; }
                        c.leaves[k++] = next;
                    }
                    if (!fits) continue;
                    c.size = k;
                    TT ta = expand(a.truth, a, c.leaves, k) ^ (aigIsComplemented(f0) ? ~0ULL : 0ULL);
                    TT tb = expand(b.truth, b, c.leaves, k) ^ (aigIsComplemented(f1) ? ~0ULL : 0ULL);
                    c.truth = ta & tb;
                    minimize(c);
                    if (c.size == 0) continue;   // constant: strash already removes these
                    bool duplicate = false;
                    for (const Cut& e : found) duplicate |= e.size == c.size && e.leaves == c.leaves;
                    if (duplicate) continue;
                    std::array<Choice, 2> direct;
                    matchCut(c, direct);
                    c.cost = std::min(direct[0].cost, direct[1].cost);
                    for (int p = 0; p < 2; ++p) {
                        const Choice& d = direct[p];
                        Choice& b2 = best[node][p];
                        if (d.cost < b2.cost - 1e-9 || (std::abs(d.cost - b2.cost) <= 1e-9 && d.area < b2.area)) b2 = d;
                    }
                    found.push_back(c);
                }
            }
            relaxInverter(best[node]);
            if (best[node][0].kind == Choice::NONE && best[node][1].kind == Choice::NONE) {
                throw std::logic_error("cut mapping: no cell implements node " + std::to_string(node));
            }
            // Priority cuts: the cheapest few, plus the node itself for its readers
            std::stable_sort(found.begin(), found.end(), [](const Cut& x, const Cut& y) {
                return x.cost != y.cost ? x.cost < y.cost : x.size < y.size;
            });
            if (found.size() > static_cast<size_t>(kPriorityCuts)) found.resize(kPriorityCuts);
            found.push_back(trivial);
            cuts[node] = std::move(found);
        }
    }

    // Uses of each node by the cover rooted at the outputs
    std::vector<double> coverRefs() const {
        std::vector<double> uses(aig.nodeCount(), 0);
        std::vector<std::array<bool, 2>> done(aig.nodeCount(), {false, false});
        std::function<void(uint32_t, int)> visit = [&](uint32_t node, int phase) {
            if (done[node][phase]) return;
            done[node][phase] = true;
            const Choice& c = best[node][phase];
            if (c.kind == Choice::INVERT) { visit(node, 1 - phase); return; }
            if (c.kind != Choice::CELL) return;
            for (int k = 0; k < c.match->inputs; ++k) {
                uint32_t leaf = c.cut.leaves[c.match->leaf[k]];
                uses[leaf] += 1;
                visit(leaf, (c.match->neg >> k) & 1);
            }
        };
        for (const auto& [name, lit] : outputs) {
            uses[aigNode(lit)] += 1;
            visit(aigNode(lit), aigIsComplemented(lit));
        }
        return uses;
    }

    // ---- Exact area recovery (reference counting over the cover) ----
    std::vector<std::array<int, 2>> nref;

    double inverterArea() const { return table.inverter->area; }

    double refNode(uint32_t node, int phase) { return nref[node][phase]++ > 0 ? 0 : refChoice(node, phase, +1); }
    double derefNode(uint32_t node, int phase) { return --nref[node][phase] > 0 ? 0 : refChoice(node, phase, -1); }

    // Adds (dir +1) or removes (-1) the references the chosen implementation of (node, phase)
    // makes; returns the area that comes into (or goes out of) existence
    double refChoice(uint32_t node, int phase, int dir) {
        const Choice& c = best[node][phase];
        if (c.kind == Choice::PASS || c.kind == Choice::NONE) return 0;
        if (c.kind == Choice::INVERT) return inverterArea() + (dir > 0 ? refNode(node, 1 - phase) : derefNode(node, 1 - phase));
        double area = c.match->cell->area;
        for (int k = 0; k < c.match->inputs; ++k) {
            uint32_t leaf = c.cut.leaves[c.match->leaf[k]];
            int ph = (c.match->neg >> k) & 1;
            area += dir > 0 ? refNode(leaf, ph) : derefNode(leaf, ph);
        }
        return area;
    }

    // Area that implementing (node, phase) with `option` would add, given the rest of the cover
    double exactArea(uint32_t node, int phase, const Choice& option) {
        Choice saved = best[node][phase];
        best[node][phase] = option;
        double area = refChoice(node, phase, +1);
        refChoice(node, phase, -1);
        best[node][phase] = saved;
        return area;
    }

    // ---- Timing for delay mode: arrival of each choice, required times from the cover ----
    std::vector<std::array<double, 2>> arrival, required;

    double optionArrival(uint32_t node, int phase, const Choice& c) const {
        if (c.kind == Choice::PASS) return 0;
        if (c.kind == Choice::INVERT) return arrival[node][1 - phase] + table.inverter_delay;
        double latest = 0;
        for (int k = 0; k < c.match->inputs; ++k) latest = std::max(latest, arrival[c.cut.leaves[c.match->leaf[k]]][(c.match->neg >> k) & 1]);
        return latest + c.match->delay;
    }

    void computeArrivals() {
        arrival.assign(aig.nodeCount(), {0.0, 0.0});
        for (uint32_t node = 1; node < aig.nodeCount(); ++node) {
            // the phase implemented by an inverter is computed after the other one
            int order[2] = {best[node][0].kind == Choice::INVERT ? 1 : 0, best[node][0].kind == Choice::INVERT ? 0 : 1};
            for (int phase : order) arrival[node][phase] = optionArrival(node, phase, best[node][phase]);
        }
    }

    // Latest each (node, phase) of the cover may settle for every output to meet `target`
    void computeRequired(double target) {
        required.assign(aig.nodeCount(), {kInf, kInf});
        for (const auto& [name, lit] : outputs) {
            double& r = required[aigNode(lit)][aigIsComplemented(lit)];
            r = std::min(r, target);
        }
        for (uint32_t node = static_cast<uint32_t>(aig.nodeCount()); node-- > 1;) {
            for (int phase = 1; phase >= 0; --phase) {   // phase 1 may be an inverter of phase 0
                double r = required[node][phase];
                if (r == kInf) continue;
                const Choice& c = best[node][phase];
                if (c.kind == Choice::INVERT) {
                    required[node][1 - phase] = std::min(required[node][1 - phase], r - table.inverter_delay);
                } else if (c.kind == Choice::CELL) {
                    for (int k = 0; k < c.match->inputs; ++k) {
                        double& lr = required[c.cut.leaves[c.match->leaf[k]]][(c.match->neg >> k) & 1];
                        lr = std::min(lr, r - c.match->delay);
                    }
                }
            }
        }
    }

    // Exact area recovery; with `timed`, an option must still meet the node's required time
    void recoverArea(bool timed = false) {
        size_t n = aig.nodeCount();
        nref.assign(n, {0, 0});
        for (const auto& [name, lit] : outputs) refNode(aigNode(lit), aigIsComplemented(lit));
        for (uint32_t node = 1; node < n; ++node) {
            if (!aig.isAnd(node)) continue;
            for (int phase = 0; phase < 2; ++phase) {
                if (nref[node][phase] == 0) continue;
                refChoice(node, phase, -1);   // free the current implementation
                Choice chosen = best[node][phase];
                double chosen_area = exactArea(node, phase, chosen);
                auto meets = [&](const Choice& option) {
                    return !timed || optionArrival(node, phase, option) <= required[node][phase] + 1e-6;
                };
                for (const Cut& cut : cuts[node]) {
                    if (cut.size == 1 && cut.leaves[0] == node) continue;   // the trivial cut
                    const auto* m = table.find(cut.size, phase ? ~cut.truth : cut.truth);
                    if (!m) continue;
                    for (const CellMatch& cm : *m) {
                        Choice option;
                        option.kind = Choice::CELL;
                        option.cut = cut;
                        option.match = &cm;
                        if (!meets(option)) continue;
                        double area = exactArea(node, phase, option);
                        if (area < chosen_area - 1e-9) {
                            chosen_area = area;
                            chosen = option;
                        }
                    }
                }
                if (best[node][1 - phase].kind != Choice::INVERT) {   // or invert the other polarity
                    Choice option;
                    option.kind = Choice::INVERT;
                    double area = meets(option) ? exactArea(node, phase, option) : kInf;
                    if (area < chosen_area - 1e-9) {
                        chosen_area = area;
                        chosen = option;
                    }
                }
                best[node][phase] = chosen;
                if (timed) arrival[node][phase] = optionArrival(node, phase, chosen);
                refChoice(node, phase, +1);
            }
        }
    }

    double coverArea() const {
        double total = 0;
        std::vector<std::array<bool, 2>> done(aig.nodeCount(), {false, false});
        std::function<void(uint32_t, int)> visit = [&](uint32_t node, int phase) {
            if (done[node][phase]) return;
            done[node][phase] = true;
            const Choice& c = best[node][phase];
            if (c.kind == Choice::INVERT) { total += table.inverter->area; visit(node, 1 - phase); return; }
            if (c.kind != Choice::CELL) return;
            total += c.match->cell->area;
            for (int k = 0; k < c.match->inputs; ++k) visit(c.cut.leaves[c.match->leaf[k]], (c.match->neg >> k) & 1);
        };
        for (const auto& [name, lit] : outputs) visit(aigNode(lit), aigIsComplemented(lit));
        return total;
    }

public:
    CutMapper(const Design& d, const GateForest& f, const Library& l, MapGoal g) : design(d), forest(f), lib(l), goal(g), table(l) {}

    MapResult run() {
        outputs = addToAig(aig, forest);
        refs.assign(aig.nodeCount(), 0);
        for (uint32_t node = 1; node < aig.nodeCount(); ++node) {
            if (!aig.isAnd(node)) continue;
            refs[aigNode(aig.fanin0(node))] += 1;
            refs[aigNode(aig.fanin1(node))] += 1;
        }
        for (const auto& [name, lit] : outputs) refs[aigNode(lit)] += 1;

        // Area flow, then again with the cover's real fanouts, then exact area recovery
        mapOnce();
        if (goal == MapGoal::AREA) {
            std::vector<double> uses = coverRefs();
            for (size_t i = 0; i < refs.size(); ++i) refs[i] = std::max(1.0, uses[i]);
            mapOnce();
            for (int pass = 0; pass < 2; ++pass) recoverArea();
        } else {
            // Delay: keep the fastest cover's critical path, recover area everywhere else
            computeArrivals();
            double target = 0;
            for (const auto& [name, lit] : outputs) target = std::max(target, arrival[aigNode(lit)][aigIsComplemented(lit)]);
            for (int pass = 0; pass < 2; ++pass) {
                computeArrivals();
                computeRequired(target);
                recoverArea(true);
            }
        }

        NetlistBuilder nb(design, lib);
        std::vector<std::array<int, 2>> net(aig.nodeCount(), {-1, -1});
        std::function<int(uint32_t, int)> emit = [&](uint32_t node, int phase) -> int {
            if (node == 0) return phase ? CellNetlist::kConst1 : CellNetlist::kConst0;
            if (net[node][phase] >= 0) return net[node][phase];
            const Choice& c = best[node][phase];
            int out;
            if (c.kind == Choice::PASS) {
                out = nb.inputNet(aig.inputNames()[std::find(aig.inputNodes().begin(), aig.inputNodes().end(), node) - aig.inputNodes().begin()]);
            } else if (c.kind == Choice::INVERT) {
                out = nb.invert(emit(node, 1 - phase));
            } else {
                std::vector<int> leaf_nets;
                for (int k = 0; k < c.match->inputs; ++k) leaf_nets.push_back(emit(c.cut.leaves[c.match->leaf[k]], (c.match->neg >> k) & 1));
                out = nb.newNet("_a" + std::to_string(node) + (phase ? "n_" : "_"));
                auto& inst = nb.addCell(c.match->cell);
                for (int k = 0; k < c.match->inputs; ++k) inst.pin_net[c.match->pin[k]] = leaf_nets[k];
                inst.pin_net[c.match->cell->outputPin()] = out;
            }
            return net[node][phase] = out;
        };
        std::vector<std::pair<std::string, int>> roots;
        for (const auto& [name, lit] : outputs) roots.emplace_back(name, emit(aigNode(lit), aigIsComplemented(lit)));
        nb.finish(roots);
        MapResult result;
        result.netlist = std::move(nb.nl);
        result.cell_counts = std::move(nb.cell_counts);
        result.area = nb.area;
        return result;
    }
};

} // namespace

MapResult mapByCuts(const Design& design, const GateForest& forest, const Library& lib, MapGoal goal) {
    return CutMapper(design, forest, lib, goal).run();
}
