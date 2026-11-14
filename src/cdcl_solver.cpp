// Conflict-driven clause learning (CDCL) SAT solver, in the style of MiniSat / Glucose:
//   - two watched literals per clause, with a "blocker" literal cached in each watch so most
//     satisfied clauses are skipped without touching clause memory; binary clauses never
//     touch clause memory at all
//   - all clauses in one contiguous uint32_t arena, literals as 2 * var + negated
//   - first-UIP conflict analysis with basic clause minimization
//   - VSIDS branching (binary heap on variable activity) with phase saving
//   - Luby restarts; learned clauses ranked by LBD ("glue") and the worse half deleted
//     periodically (clauses with LBD <= 2 are kept forever)
//   - assumptions (decided first, one per level) and failed-assumption analysis, so one
//     solver can answer many related questions incrementally
#include "../include/sat_solver.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace {

using Lit = uint32_t;                 // 2 * var + 1 if negated
constexpr Lit kNoLit = ~0u;
constexpr uint32_t kNoReason = ~0u;   // decision, assumption or level-0 fact

inline Lit negate(Lit lit) { return lit ^ 1u; }
inline uint32_t varOf(Lit lit) { return lit >> 1; }

// Luby sequence 1 1 2 1 1 2 4 1 1 2 1 1 2 4 8 ...: restart intervals that are provably
// within a log factor of the best fixed schedule
double luby(double base, int index) {
    int size = 1, seq = 0;
    while (size < index + 1) {
        seq++;
        size = 2 * size + 1;
    }
    while (size - 1 != index) {
        size = (size - 1) >> 1;
        seq--;
        index = index % size;
    }
    return std::pow(base, seq);
}

// Max-heap of variables keyed on activity, with positions so a bumped variable can move up
class VarHeap {
    const std::vector<double>& activity;
    std::vector<uint32_t> heap;
    std::vector<int> position;   // -1: not in the heap

    bool before(uint32_t a, uint32_t b) const { return activity[a] > activity[b]; }

    void up(size_t i) {
        uint32_t v = heap[i];
        while (i > 0) {
            size_t parent = (i - 1) / 2;
            if (!before(v, heap[parent])) break;
            heap[i] = heap[parent];
            position[heap[i]] = static_cast<int>(i);
            i = parent;
        }
        heap[i] = v;
        position[v] = static_cast<int>(i);
    }

    void down(size_t i) {
        uint32_t v = heap[i];
        for (;;) {
            size_t child = 2 * i + 1;
            if (child >= heap.size()) break;
            if (child + 1 < heap.size() && before(heap[child + 1], heap[child])) child++;
            if (!before(heap[child], v)) break;
            heap[i] = heap[child];
            position[heap[i]] = static_cast<int>(i);
            i = child;
        }
        heap[i] = v;
        position[v] = static_cast<int>(i);
    }

public:
    explicit VarHeap(const std::vector<double>& a) : activity(a) {}
    void grow(size_t vars) { position.resize(vars, -1); }
    bool empty() const { return heap.empty(); }
    bool contains(uint32_t v) const { return position[v] >= 0; }
    void insert(uint32_t v) {
        position[v] = static_cast<int>(heap.size());
        heap.push_back(v);
        up(heap.size() - 1);
    }
    void increased(uint32_t v) {
        if (contains(v)) up(static_cast<size_t>(position[v]));
    }
    uint32_t pop() {
        uint32_t top = heap[0];
        position[top] = -1;
        uint32_t last = heap.back();
        heap.pop_back();
        if (!heap.empty()) {
            heap[0] = last;
            position[last] = 0;
            down(0);
        }
        return top;
    }
};

class CdclSolver final : public SatSolver {
    // Clause arena: [size][lbd << 2 | deleted << 1 | learned][literals...]
    std::vector<uint32_t> arena;
    std::vector<uint32_t> originals, learneds;   // clause offsets ("crefs") into the arena

    struct Watch {
        uint32_t cref;
        Lit blocker;   // another literal of the clause: if it is true, the clause is satisfied
        bool binary;
    };
    std::vector<std::vector<Watch>> watches;   // by literal: clauses to visit when it becomes true

    // Assignment
    std::vector<int8_t> value;         // by literal: 1 true, -1 false, 0 unassigned
    std::vector<int> level;            // by variable
    std::vector<uint32_t> reason;      // by variable: clause that implied it
    std::vector<Lit> trail;            // assigned literals in order
    std::vector<size_t> trail_limits;  // trail size at the start of each decision level
    size_t propagated = 0;             // trail[0 .. propagated) have had their watches visited

    // Heuristics
    std::vector<double> activity;
    double activity_increment = 1.0;
    VarHeap order{activity};
    std::vector<uint8_t> saved_phase;  // by variable: 1 = last assigned false
    uint64_t next_reduce = 2000;
    uint64_t reductions = 0;

    // Scratch space
    std::vector<uint8_t> seen;
    std::vector<uint32_t> level_stamp;
    uint32_t stamp = 0;
    std::vector<Lit> clause_buffer;

    // Incremental interface
    std::vector<Lit> assumptions;
    std::vector<int8_t> model;         // by variable after SATISFIABLE
    std::vector<uint8_t> failed_flag;  // by literal after UNSATISFIABLE
    std::vector<Lit> failed_lits;
    bool ok = true;                    // false once the clauses alone are unsatisfiable
    uint64_t conflict_limit = 0;       // per solve(); 0: none
    Stats counters;

    size_t varCount() const { return level.size(); }
    int decisionLevel() const { return static_cast<int>(trail_limits.size()); }
    uint32_t clauseSize(uint32_t cref) const { return arena[cref]; }
    uint32_t* clauseLits(uint32_t cref) { return &arena[cref + 2]; }
    bool isDeleted(uint32_t cref) const { return arena[cref + 1] & 2u; }

    Lit internal(int lit) {
        if (lit == 0) throw std::invalid_argument("0 is not a literal");
        uint32_t v = static_cast<uint32_t>(std::abs(lit)) - 1;
        ensureVars(v + 1);
        return 2 * v + (lit < 0 ? 1u : 0u);
    }

    void ensureVars(size_t vars) {
        if (vars <= varCount()) return;
        size_t old = varCount();
        value.resize(2 * vars, 0);
        watches.resize(2 * vars);
        failed_flag.resize(2 * vars, 0);
        level.resize(vars, 0);
        reason.resize(vars, kNoReason);
        activity.resize(vars, 0.0);
        saved_phase.resize(vars, 1);   // try false first: miters are mostly 0
        seen.resize(vars, 0);
        level_stamp.resize(vars + 1, 0);
        order.grow(vars);
        for (size_t v = old; v < vars; ++v) order.insert(static_cast<uint32_t>(v));
    }

    uint32_t allocClause(const std::vector<Lit>& lits, bool learned, uint32_t lbd) {
        uint32_t cref = static_cast<uint32_t>(arena.size());
        arena.push_back(static_cast<uint32_t>(lits.size()));
        arena.push_back((lbd << 2) | (learned ? 1u : 0u));
        arena.insert(arena.end(), lits.begin(), lits.end());
        return cref;
    }

    void attach(uint32_t cref) {
        const uint32_t* lits = clauseLits(cref);
        bool binary = clauseSize(cref) == 2;
        watches[negate(lits[0])].push_back({cref, lits[1], binary});
        watches[negate(lits[1])].push_back({cref, lits[0], binary});
    }

    void assign(Lit lit, uint32_t why) {
        value[lit] = 1;
        value[negate(lit)] = -1;
        level[varOf(lit)] = decisionLevel();
        reason[varOf(lit)] = why;
        trail.push_back(lit);
    }

    void cancelUntil(int target) {
        if (decisionLevel() <= target) return;
        for (size_t i = trail.size(); i-- > trail_limits[target];) {
            Lit lit = trail[i];
            uint32_t v = varOf(lit);
            value[lit] = value[negate(lit)] = 0;
            reason[v] = kNoReason;
            saved_phase[v] = lit & 1u;
            if (!order.contains(v)) order.insert(v);
        }
        trail.resize(trail_limits[target]);
        trail_limits.resize(target);
        propagated = trail.size();
    }

    // Unit propagation over the watch lists. Returns the conflicting clause, or kNoReason.
    uint32_t propagate() {
        uint32_t conflict = kNoReason;
        while (propagated < trail.size() && conflict == kNoReason) {
            Lit p = trail[propagated++];
            Lit false_lit = negate(p);
            std::vector<Watch>& ws = watches[p];
            counters.propagations++;

            size_t i = 0, j = 0, n = ws.size();
            while (i < n) {
                Watch w = ws[i++];
                if (value[w.blocker] == 1) {
                    ws[j++] = w;
                    continue;
                }
                if (w.binary) {   // the other literal is the whole story
                    ws[j++] = w;
                    if (value[w.blocker] == -1) {
                        conflict = w.cref;
                        break;
                    }
                    assign(w.blocker, w.cref);
                    continue;
                }

                // Keep the false literal in position 1, the other watch in position 0
                uint32_t* lits = clauseLits(w.cref);
                uint32_t size = clauseSize(w.cref);
                if (lits[0] == false_lit) std::swap(lits[0], lits[1]);
                Lit first = lits[0];
                Watch kept{w.cref, first, false};
                if (first != w.blocker && value[first] == 1) {
                    ws[j++] = kept;
                    continue;
                }

                // Look for a replacement watch that is not false
                bool moved = false;
                for (uint32_t k = 2; k < size; ++k) {
                    if (value[lits[k]] != -1) {
                        lits[1] = lits[k];
                        lits[k] = false_lit;
                        watches[negate(lits[1])].push_back(kept);
                        moved = true;
                        break;
                    }
                }
                if (moved) continue;

                // Every other literal is false: the clause is unit (or conflicting)
                ws[j++] = kept;
                if (value[first] == -1) {
                    conflict = w.cref;
                    break;
                }
                assign(first, w.cref);
            }
            while (i < n) ws[j++] = ws[i++];
            ws.resize(j);
        }
        if (conflict != kNoReason) propagated = trail.size();
        return conflict;
    }

    void bumpVar(uint32_t v) {
        activity[v] += activity_increment;
        if (activity[v] > 1e100) {   // rescale everything before doubles overflow
            for (double& a : activity) a *= 1e-100;
            activity_increment *= 1e-100;
        }
        order.increased(v);
    }

    // First-UIP learning: resolve the conflict clause with reasons on the current level until
    // exactly one literal of that level remains. Returns the backjump level.
    int analyze(uint32_t conflict, std::vector<Lit>& learned, uint32_t& lbd) {
        learned.assign(1, kNoLit);   // slot 0 is filled with the asserting literal at the end
        int open = 0;
        Lit p = kNoLit;
        size_t index = trail.size();

        do {
            uint32_t size = clauseSize(conflict);
            const uint32_t* lits = clauseLits(conflict);
            for (uint32_t k = 0; k < size; ++k) {
                Lit q = lits[k];
                uint32_t v = varOf(q);
                if (p != kNoLit && v == varOf(p)) continue;   // the literal this clause implied
                if (seen[v] || level[v] == 0) continue;
                seen[v] = 1;
                bumpVar(v);
                if (level[v] >= decisionLevel()) open++;
                else learned.push_back(q);
            }
            while (!seen[varOf(trail[--index])]) {}
            p = trail[index];
            conflict = reason[varOf(p)];
            seen[varOf(p)] = 0;
            open--;
        } while (open > 0);
        learned[0] = negate(p);

        // Minimization: drop a literal whose reason is covered by the rest of the clause
        std::vector<Lit> to_clear(learned.begin() + 1, learned.end());
        size_t keep = 1;
        for (size_t i = 1; i < learned.size(); ++i) {
            uint32_t v = varOf(learned[i]);
            uint32_t why = reason[v];
            bool redundant = why != kNoReason;
            if (redundant) {
                uint32_t size = clauseSize(why);
                const uint32_t* lits = clauseLits(why);
                for (uint32_t k = 0; k < size && redundant; ++k) {
                    uint32_t u = varOf(lits[k]);
                    if (u != v && !seen[u] && level[u] > 0) redundant = false;
                }
            }
            if (!redundant) learned[keep++] = learned[i];
        }
        learned.resize(keep);
        for (Lit lit : to_clear) seen[varOf(lit)] = 0;

        // Backjump to the second-highest level, with that literal in watch position 1
        int backjump = 0;
        if (learned.size() > 1) {
            size_t max_i = 1;
            for (size_t i = 2; i < learned.size(); ++i) {
                if (level[varOf(learned[i])] > level[varOf(learned[max_i])]) max_i = i;
            }
            std::swap(learned[1], learned[max_i]);
            backjump = level[varOf(learned[1])];
        }

        // LBD: number of distinct decision levels in the clause
        if (level_stamp.size() <= static_cast<size_t>(decisionLevel())) level_stamp.resize(decisionLevel() + 1, 0);
        stamp++;
        lbd = 0;
        for (Lit lit : learned) {
            int l = level[varOf(lit)];
            if (level_stamp[l] != stamp) {
                level_stamp[l] = stamp;
                lbd++;
            }
        }
        return backjump;
    }

    // Assumption `failed` came out false: collect the assumptions that forced it
    void analyzeFinal(Lit failed) {
        failed_lits.push_back(failed);
        failed_flag[failed] = 1;
        if (decisionLevel() == 0) return;
        seen[varOf(failed)] = 1;
        for (size_t i = trail.size(); i-- > trail_limits[0];) {
            uint32_t v = varOf(trail[i]);
            if (!seen[v]) continue;
            seen[v] = 0;
            if (reason[v] == kNoReason) {   // a decision below the assumption levels: an assumption
                Lit assumed = trail[i];
                if (!failed_flag[assumed]) {
                    failed_flag[assumed] = 1;
                    failed_lits.push_back(assumed);
                }
            } else {
                uint32_t size = clauseSize(reason[v]);
                const uint32_t* lits = clauseLits(reason[v]);
                for (uint32_t k = 0; k < size; ++k) {
                    uint32_t u = varOf(lits[k]);
                    if (u != v && level[u] > 0) seen[u] = 1;   // not v itself: it is behind us already
                }
            }
        }
        seen[varOf(failed)] = 0;
    }

    Lit pickBranch() {
        while (!order.empty()) {
            uint32_t v = order.pop();
            if (value[2 * v] == 0) return 2 * v + saved_phase[v];
        }
        return kNoLit;
    }

    // At level 0: delete clauses already satisfied for good and the worse half of the learned
    // clauses, strip literals that are false for good, then compact the arena.
    void reduceDatabase() {
        reductions++;
        next_reduce = counters.conflicts + 2000 + 300 * reductions;

        std::vector<uint32_t> candidates;
        for (uint32_t cref : learneds) {
            if ((arena[cref + 1] >> 2) > 2) candidates.push_back(cref);
        }
        std::sort(candidates.begin(), candidates.end(), [&](uint32_t a, uint32_t b) {
            uint32_t lbd_a = arena[a + 1] >> 2, lbd_b = arena[b + 1] >> 2;
            return lbd_a != lbd_b ? lbd_a > lbd_b : clauseSize(a) > clauseSize(b);
        });
        for (size_t i = 0; i < candidates.size() / 2; ++i) arena[candidates[i] + 1] |= 2u;

        // Level-0 facts need no reasons (analysis never looks at level 0)
        for (Lit lit : trail) reason[varOf(lit)] = kNoReason;

        std::vector<uint32_t> fresh;
        fresh.reserve(arena.size());
        auto compact = [&](std::vector<uint32_t>& list) {
            std::vector<uint32_t> kept;
            for (uint32_t cref : list) {
                if (isDeleted(cref)) continue;
                const uint32_t* lits = clauseLits(cref);
                uint32_t size = clauseSize(cref);
                bool satisfied = false;
                for (uint32_t k = 0; k < size && !satisfied; ++k) satisfied = value[lits[k]] == 1;
                if (satisfied) continue;
                uint32_t out = static_cast<uint32_t>(fresh.size());
                fresh.push_back(0);
                fresh.push_back(arena[cref + 1]);
                for (uint32_t k = 0; k < size; ++k) {
                    if (value[lits[k]] != -1) fresh.push_back(lits[k]);   // the two watches stay first
                }
                fresh[out] = static_cast<uint32_t>(fresh.size() - out - 2);
                kept.push_back(out);
            }
            list.swap(kept);
        };
        compact(originals);
        compact(learneds);
        arena.swap(fresh);

        for (auto& ws : watches) ws.clear();
        for (uint32_t cref : originals) attach(cref);
        for (uint32_t cref : learneds) attach(cref);
    }

    int search(uint64_t conflict_budget) {
        uint64_t conflicts_here = 0;
        std::vector<Lit> learned;
        for (;;) {
            uint32_t conflict = propagate();
            if (conflict != kNoReason) {
                counters.conflicts++;
                conflicts_here++;
                if (decisionLevel() == 0) {
                    ok = false;
                    return UNSATISFIABLE;
                }
                uint32_t lbd = 0;
                int backjump = analyze(conflict, learned, lbd);
                cancelUntil(backjump);
                if (learned.size() == 1) {
                    assign(learned[0], kNoReason);
                } else {
                    uint32_t cref = allocClause(learned, true, lbd);
                    learneds.push_back(cref);
                    attach(cref);
                    assign(learned[0], cref);
                }
                counters.learned++;
                activity_increment /= 0.95;   // VSIDS decay: recent conflicts weigh more
                continue;
            }

            if (conflicts_here >= conflict_budget) {
                cancelUntil(0);
                return UNKNOWN;   // restart
            }

            // Assumptions first, one decision level each
            Lit next = kNoLit;
            while (decisionLevel() < static_cast<int>(assumptions.size())) {
                Lit a = assumptions[decisionLevel()];
                if (value[a] == 1) {
                    trail_limits.push_back(trail.size());   // already true: empty level
                } else if (value[a] == -1) {
                    analyzeFinal(a);
                    return UNSATISFIABLE;
                } else {
                    next = a;
                    break;
                }
            }
            if (next == kNoLit) {
                next = pickBranch();
                if (next == kNoLit) return SATISFIABLE;   // every variable assigned, no conflict
                counters.decisions++;
            }
            trail_limits.push_back(trail.size());
            assign(next, kNoReason);
        }
    }

public:
    std::string signature() const override { return "cdcl (ASIC_backend)"; }

    void add(int lit) override {
        if (lit != 0) {
            clause_buffer.push_back(internal(lit));
            return;
        }
        std::vector<Lit> lits;
        lits.swap(clause_buffer);
        if (!ok) return;
        cancelUntil(0);

        std::sort(lits.begin(), lits.end());
        std::vector<Lit> clause;
        for (size_t i = 0; i < lits.size(); ++i) {
            Lit lit = lits[i];
            if (i + 1 < lits.size() && lits[i + 1] == negate(lit)) return;   // x | !x: always true
            if (!clause.empty() && clause.back() == lit) continue;            // duplicate
            if (value[lit] == 1) return;                                      // already satisfied
            if (value[lit] == -1) continue;                                   // false for good
            clause.push_back(lit);
        }

        if (clause.empty()) {
            ok = false;
        } else if (clause.size() == 1) {
            assign(clause[0], kNoReason);
            if (propagate() != kNoReason) ok = false;
        } else {
            uint32_t cref = allocClause(clause, false, 0);
            originals.push_back(cref);
            attach(cref);
        }
    }

    void assume(int lit) override { assumptions.push_back(internal(lit)); }

    int solve() override {
        counters.solves++;
        model.clear();
        for (Lit lit : failed_lits) failed_flag[lit] = 0;
        failed_lits.clear();

        int result = UNSATISFIABLE;
        if (ok) {
            cancelUntil(0);
            if (propagate() != kNoReason) ok = false;
        }
        if (ok) {
            uint64_t start = counters.conflicts;
            for (int restart = 0;; ++restart) {
                result = search(static_cast<uint64_t>(luby(2, restart) * 100));
                if (result != UNKNOWN) break;
                if (conflict_limit && counters.conflicts - start >= conflict_limit) break;   // gave up
                counters.restarts++;
                if (counters.conflicts >= next_reduce) reduceDatabase();
            }
        }

        if (result == SATISFIABLE) {
            model.resize(varCount());
            for (size_t v = 0; v < varCount(); ++v) model[v] = value[2 * v];
        }
        cancelUntil(0);
        assumptions.clear();
        return result;
    }

    int val(int lit) const override {
        size_t v = static_cast<size_t>(std::abs(lit)) - 1;
        bool positive_true = v < model.size() && model[v] == 1;
        return (lit > 0) == positive_true ? lit : -lit;
    }

    bool failed(int lit) const override {
        size_t v = static_cast<size_t>(std::abs(lit)) - 1;
        if (lit == 0 || v >= varCount()) return false;
        return failed_flag[2 * v + (lit < 0 ? 1 : 0)];
    }

    Stats stats() const override { return counters; }
    void setConflictLimit(uint64_t conflicts) override { conflict_limit = conflicts; }
};

} // namespace

std::unique_ptr<SatSolver> makeSatSolver(const std::string& name) {
    if (name == "cdcl") return std::make_unique<CdclSolver>();
    throw std::runtime_error("unknown SAT solver '" + name + "' (available: cdcl)");
}
