// Independent check of the CDCL solver:
//   1. random small CNFs: SAT/UNSAT must agree with brute-force enumeration, every model
//      must satisfy every clause
//   2. incremental use: clauses added between solves, random assumptions each time; a failed
//      assumption set must be a subset of the assumptions that is itself unsatisfiable
//   3. pigeonhole formulas (n+1 pigeons, n holes): always UNSAT and exponentially hard for
//      resolution, so they exercise learning, restarts and clause deletion
//   4. larger random 3-SAT near the phase transition: models checked clause by clause
//
// usage: sat_fuzz [rounds]    exit 0 when everything agrees
#include "../include/sat_solver.hpp"
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

namespace {

using Clause = std::vector<int>;
using Cnf = std::vector<Clause>;

std::mt19937 rng(12345);

int randomInt(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }

Clause randomClause(int vars, int size) {
    Clause c;
    for (int k = 0; k < size; ++k) c.push_back(randomInt(1, vars) * (randomInt(0, 1) ? 1 : -1));
    return c;
}

bool satisfiedBy(const Clause& c, uint32_t assignment) {
    for (int lit : c) {
        bool value = (assignment >> (std::abs(lit) - 1)) & 1;
        if (value == (lit > 0)) return true;
    }
    return false;
}

// Is cnf satisfiable with every literal in `forced` true? (vars <= ~20)
bool bruteForce(const Cnf& cnf, int vars, const std::vector<int>& forced) {
    for (uint32_t a = 0; a < (1u << vars); ++a) {
        bool ok = true;
        for (int lit : forced) ok &= ((a >> (std::abs(lit) - 1)) & 1) == (lit > 0);
        for (size_t i = 0; ok && i < cnf.size(); ++i) ok = satisfiedBy(cnf[i], a);
        if (ok) return true;
    }
    return false;
}

void addClause(SatSolver& s, const Clause& c) {
    for (int lit : c) s.add(lit);
    s.add(0);
}

bool modelSatisfies(const SatSolver& s, const Cnf& cnf, const std::vector<int>& forced) {
    for (int lit : forced) {
        if (s.val(lit) != lit) return false;
    }
    for (const Clause& c : cnf) {
        bool sat = false;
        for (int lit : c) sat |= s.val(lit) == lit;
        if (!sat) return false;
    }
    return true;
}

int failures = 0;

void fail(const std::string& what) {
    if (failures++ < 10) std::cout << "  FAIL " << what << "\n";
}

void randomFormulas(int rounds) {
    for (int r = 0; r < rounds; ++r) {
        int vars = randomInt(1, 12);
        int clauses = randomInt(0, vars * 5);
        Cnf cnf;
        for (int i = 0; i < clauses; ++i) cnf.push_back(randomClause(vars, randomInt(1, 4)));

        auto solver = makeSatSolver();
        for (const Clause& c : cnf) addClause(*solver, c);
        int result = solver->solve();
        bool expected = bruteForce(cnf, vars, {});
        if ((result == SatSolver::SATISFIABLE) != expected) fail("random formula " + std::to_string(r) + ": wrong answer");
        else if (expected && !modelSatisfies(*solver, cnf, {})) fail("random formula " + std::to_string(r) + ": bad model");
    }
    std::cout << "random formulas vs brute force: " << rounds << " formulas\n";
}

void incremental(int rounds) {
    int solves = 0, unsat_with_failed = 0;
    for (int r = 0; r < rounds; ++r) {
        int vars = randomInt(3, 12);
        auto solver = makeSatSolver();
        Cnf cnf;
        for (int step = 0; step < 8; ++step) {
            for (int i = randomInt(0, vars); i > 0; --i) {
                cnf.push_back(randomClause(vars, randomInt(2, 3)));
                addClause(*solver, cnf.back());
            }
            std::vector<int> assumptions;
            for (int i = randomInt(0, 4); i > 0; --i) {
                assumptions.push_back(randomInt(1, vars) * (randomInt(0, 1) ? 1 : -1));
                solver->assume(assumptions.back());
            }
            int result = solver->solve();
            solves++;
            bool expected = bruteForce(cnf, vars, assumptions);
            std::string where = "incremental " + std::to_string(r) + "." + std::to_string(step);
            if ((result == SatSolver::SATISFIABLE) != expected) {
                fail(where + ": wrong answer");
            } else if (expected && !modelSatisfies(*solver, cnf, assumptions)) {
                fail(where + ": bad model");
            } else if (!expected) {
                std::vector<int> failed;
                for (int lit : assumptions) {
                    if (solver->failed(lit)) failed.push_back(lit);
                }
                if (bruteForce(cnf, vars, failed)) fail(where + ": failed assumptions are satisfiable");
                if (!failed.empty()) unsat_with_failed++;
            }
        }
    }
    std::cout << "incremental solving with assumptions: " << solves << " solves (" << unsat_with_failed
              << " UNSAT answers blamed a subset of the assumptions)\n";
}

void pigeonhole(int holes) {
    // var(p, h) is true when pigeon p sits in hole h
    int pigeons = holes + 1;
    auto var = [&](int p, int h) { return p * holes + h + 1; };
    auto solver = makeSatSolver();
    for (int p = 0; p < pigeons; ++p) {
        Clause somewhere;
        for (int h = 0; h < holes; ++h) somewhere.push_back(var(p, h));
        addClause(*solver, somewhere);
    }
    for (int h = 0; h < holes; ++h) {
        for (int p = 0; p < pigeons; ++p) {
            for (int q = p + 1; q < pigeons; ++q) addClause(*solver, {-var(p, h), -var(q, h)});
        }
    }
    int result = solver->solve();
    if (result != SatSolver::UNSATISFIABLE) fail("pigeonhole " + std::to_string(holes) + " should be UNSAT");
    std::cout << "pigeonhole " << pigeons << " into " << holes << ": UNSAT after " << solver->stats().conflicts
              << " conflicts\n";
}

void phaseTransition(int vars, int instances) {
    int sat = 0;
    uint64_t conflicts = 0;
    for (int i = 0; i < instances; ++i) {
        Cnf cnf;
        for (int k = 0; k < vars * 426 / 100; ++k) cnf.push_back(randomClause(vars, 3));
        auto solver = makeSatSolver();
        for (const Clause& c : cnf) addClause(*solver, c);
        int result = solver->solve();
        conflicts += solver->stats().conflicts;
        if (result == SatSolver::SATISFIABLE) {
            sat++;
            if (!modelSatisfies(*solver, cnf, {})) fail("random 3-SAT model violates a clause");
        } else if (result != SatSolver::UNSATISFIABLE) {
            fail("random 3-SAT: no answer");
        }
    }
    std::cout << "random 3-SAT, " << vars << " vars at ratio 4.26: " << sat << "/" << instances << " SAT, "
              << conflicts << " conflicts in total, every model checked\n";
}

} // namespace

int main(int argc, char* argv[]) {
    int rounds = argc > 1 ? std::atoi(argv[1]) : 2000;
    randomFormulas(rounds);
    incremental(rounds / 4);
    for (int holes = 5; holes <= 8; ++holes) pigeonhole(holes);
    phaseTransition(150, 20);
    std::cout << (failures ? "solver checks FAILED (" + std::to_string(failures) + ")" : std::string("solver checks consistent"))
              << "\n";
    return failures ? 1 : 0;
}
