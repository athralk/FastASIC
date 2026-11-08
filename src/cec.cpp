#include "../include/cec.hpp"
#include "../include/aig.hpp"
#include "../include/simulator.hpp"
#include <algorithm>
#include <chrono>
#include <map>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace {

constexpr int kRandomWords = 16;   // 1024 random patterns before any SAT call

// Tseitin encoding of AIG nodes into CNF, on demand: a node is encoded the first time a
// question needs it, so the solver only ever sees the cones of influence.
class CnfEncoder {
    const Aig& aig;
    SatSolver& solver;
    std::vector<int> var_of;   // AIG node -> SAT variable (0: not encoded yet)
    int last_var = 0;

    void clause(std::initializer_list<int> lits) {
        for (int lit : lits) solver.add(lit);
        solver.add(0);
    }

    int encode(uint32_t root) {
        std::vector<uint32_t> stack{root};
        while (!stack.empty()) {
            uint32_t node = stack.back();
            if (var_of[node]) {
                stack.pop_back();
                continue;
            }
            if (aig.isAnd(node)) {
                uint32_t f0 = aigNode(aig.fanin0(node)), f1 = aigNode(aig.fanin1(node));
                if (!var_of[f0] || !var_of[f1]) {   // fanins first
                    if (!var_of[f0]) stack.push_back(f0);
                    if (!var_of[f1]) stack.push_back(f1);
                    continue;
                }
                int n = var_of[node] = newVar();
                int x = literal(aig.fanin0(node)), y = literal(aig.fanin1(node));
                clause({-n, x});        // n -> x
                clause({-n, y});        // n -> y
                clause({n, -x, -y});    // x & y -> n
            } else {
                int v = var_of[node] = newVar();
                if (node == 0) clause({-v});   // the constant-false node
            }
            stack.pop_back();
        }
        return var_of[root];
    }

public:
    CnfEncoder(const Aig& g, SatSolver& s) : aig(g), solver(s), var_of(g.nodeCount(), 0) {}

    int newVar() { return ++last_var; }

    int literal(AigLit lit) {
        int v = var_of[aigNode(lit)] ? var_of[aigNode(lit)] : encode(aigNode(lit));
        return aigIsComplemented(lit) ? -v : v;
    }

    // Value of an input in the last model (inputs outside every encoded cone are free: 0)
    bool inputValue(uint32_t node) const { return var_of[node] && solver.val(var_of[node]) > 0; }
};

struct OutputPair {
    std::string name;
    AigLit a, b;
};

// Turns one failing pattern into the same message format the simulator uses
std::string describeCounterexample(const Aig& aig, const std::string& output, const std::vector<bool>& pattern,
                                   AigLit a, AigLit b) {
    std::vector<uint64_t> words(pattern.size());
    for (size_t i = 0; i < pattern.size(); ++i) words[i] = pattern[i] ? ~0ULL : 0;
    std::vector<uint64_t> values = simulateAig(aig, words);

    std::map<std::string, bool> by_name;   // sorted by name, like the simulator's report
    for (size_t i = 0; i < pattern.size(); ++i) by_name[aig.inputNames()[i]] = pattern[i];
    std::string assignment;
    for (const auto& [name, bit] : by_name) assignment += name + "=" + (bit ? "1" : "0") + " ";
    return "output '" + output + "' differs when " + assignment + "(first design gives " +
           std::to_string(aigValue(values, a) & 1) + ", second gives " + std::to_string(aigValue(values, b) & 1) + ")";
}

// Independent confirmation: replay the pattern on the original gate netlists
void confirmCounterexample(const GateForest& fa, const GateForest& fb, const Aig& aig, const std::string& output,
                           const std::vector<bool>& pattern) {
    std::unordered_map<std::string, uint64_t> words;
    for (size_t i = 0; i < pattern.size(); ++i) words[aig.inputNames()[i]] = pattern[i] ? ~0ULL : 0;
    Simulator sim_a(fa), sim_b(fb);
    sim_a.run(words);
    sim_b.run(words);
    auto value = [&](const GateForest& forest, const Simulator& sim) {
        for (const auto& root : forest) {
            if (root->name == output) return sim.value(root.get()) & 1;
        }
        throw std::logic_error("output vanished");
    };
    if (value(fa, sim_a) == value(fb, sim_b)) {
        throw std::logic_error("internal error: SAT counterexample for '" + output + "' does not reproduce in simulation");
    }
}

} // namespace

CecResult proveEquivalence(const GateForest& fa, const GateForest& fb, const std::string& solver_name) {
    auto start = std::chrono::steady_clock::now();
    CecResult result;
    auto finish = [&]() -> CecResult {
        result.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return result;
    };

    Aig aig;
    auto outs_a = addToAig(aig, fa);
    auto outs_b = addToAig(aig, fb);
    result.aig_ands = aig.andCount();

    std::map<std::string, AigLit> lit_b;
    for (const auto& [name, lit] : outs_b) lit_b[name] = lit;
    std::map<std::string, AigLit> lit_a;
    for (const auto& [name, lit] : outs_a) lit_a[name] = lit;
    for (const auto& [name, lit] : lit_a) {
        if (!lit_b.count(name)) {
            result.message = "output '" + name + "' exists only in the first design";
            return finish();
        }
    }
    for (const auto& [name, lit] : lit_b) {
        if (!lit_a.count(name)) {
            result.message = "output '" + name + "' exists only in the second design";
            return finish();
        }
    }

    std::vector<OutputPair> pairs;
    for (const auto& [name, lit] : lit_a) pairs.push_back({name, lit, lit_b[name]});
    result.outputs = pairs.size();
    size_t n_inputs = aig.inputNodes().size();

    // Cheap first: random simulation finds most real differences without any SAT call
    std::mt19937_64 rng(0x5eed);
    for (int w = 0; w < kRandomWords; ++w) {
        std::vector<uint64_t> inputs(n_inputs);
        for (auto& word : inputs) word = rng();
        std::vector<uint64_t> values = simulateAig(aig, inputs);
        for (const OutputPair& pair : pairs) {
            uint64_t diff = aigValue(values, pair.a) ^ aigValue(values, pair.b);
            if (!diff) continue;
            int bit = __builtin_ctzll(diff);
            std::vector<bool> pattern(n_inputs);
            for (size_t i = 0; i < n_inputs; ++i) pattern[i] = (inputs[i] >> bit) & 1;
            confirmCounterexample(fa, fb, aig, pair.name, pattern);
            result.message = describeCounterexample(aig, pair.name, pattern, pair.a, pair.b);
            return finish();
        }
    }

    // Small cones first, so each proven pair can help with the bigger ones built on top of it
    std::stable_sort(pairs.begin(), pairs.end(), [](const OutputPair& x, const OutputPair& y) {
        return std::max(aigNode(x.a), aigNode(x.b)) < std::max(aigNode(y.a), aigNode(y.b));
    });

    std::unique_ptr<SatSolver> solver = makeSatSolver(solver_name);
    CnfEncoder cnf(aig, *solver);
    for (const OutputPair& pair : pairs) {
        if (pair.a == pair.b) {
            result.structural++;
            continue;
        }

        // Miter: m <-> (x XOR y); ask whether m can be true
        int x = cnf.literal(pair.a), y = cnf.literal(pair.b);
        int m = cnf.newVar();
        for (auto clause : {std::vector<int>{-m, x, y}, {-m, -x, -y}, {m, -x, y}, {m, x, -y}}) {
            for (int lit : clause) solver->add(lit);
            solver->add(0);
        }
        solver->assume(m);
        int status = solver->solve();

        if (status == SatSolver::SATISFIABLE) {
            std::vector<bool> pattern(n_inputs);
            for (size_t i = 0; i < n_inputs; ++i) pattern[i] = cnf.inputValue(aig.inputNodes()[i]);
            confirmCounterexample(fa, fb, aig, pair.name, pattern);
            result.message = describeCounterexample(aig, pair.name, pattern, pair.a, pair.b);
            result.sat = solver->stats();
            return finish();
        }
        if (status != SatSolver::UNSATISFIABLE) throw std::runtime_error("SAT solver gave up on output '" + pair.name + "'");

        // Proven: keep the fact. The miter can never fire, and x <-> y helps later cones.
        for (auto clause : {std::vector<int>{-m}, {-x, y}, {x, -y}}) {
            for (int lit : clause) solver->add(lit);
            solver->add(0);
        }
        result.by_sat++;
    }

    result.equivalent = true;
    result.sat = solver->stats();
    return finish();
}
