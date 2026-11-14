#pragma once
#include <cstdint>
#include <memory>
#include <string>

// Incremental SAT solver with IPASIR semantics, the standard interface of the SAT
// competition's incremental track (implemented by CaDiCaL, MiniSat, Glucose, CryptoMiniSat).
// Keeping to it means another solver can be dropped in behind the same calls and
// benchmarked against ours.
//
// Literals are DIMACS integers: variable v (v >= 1) is v, its negation is -v. Variables
// come into existence the first time they are used.
class SatSolver {
public:
    enum Result { UNKNOWN = 0, SATISFIABLE = 10, UNSATISFIABLE = 20 };

    struct Stats {
        uint64_t solves = 0;
        uint64_t decisions = 0;
        uint64_t propagations = 0;   // literals whose watch lists were visited
        uint64_t conflicts = 0;
        uint64_t restarts = 0;
        uint64_t learned = 0;        // clauses learned from conflicts (some are deleted later)
    };

    virtual ~SatSolver() = default;
    virtual std::string signature() const = 0;

    // Adds a literal to the clause being built; 0 ends the clause. Clauses are permanent.
    virtual void add(int lit) = 0;

    // Assumes lit is true for the next solve() only
    virtual void assume(int lit) = 0;

    // SATISFIABLE, UNSATISFIABLE (under the assumptions) or UNKNOWN
    virtual int solve() = 0;

    // After SATISFIABLE: lit if it is true in the model, -lit if it is false
    virtual int val(int lit) const = 0;

    // After UNSATISFIABLE: whether assumption lit was needed to prove it
    virtual bool failed(int lit) const = 0;

    virtual Stats stats() const = 0;

    // Not part of IPASIR: give up (UNKNOWN) after this many conflicts in the next solve() call;
    // 0 means no limit. Solvers without it just ignore the limit.
    virtual void setConflictLimit(uint64_t /*conflicts*/) {}
};

// The built-in CDCL solver ("cdcl")
std::unique_ptr<SatSolver> makeSatSolver(const std::string& name = "cdcl");
