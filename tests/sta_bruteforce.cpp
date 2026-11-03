// Independent check of the STA engine: recomputes every arrival and required time by
// enumerating paths explicitly (no topological order, no memoization) and compares the
// results with analyzeTiming at every node and edge. Exponential, so small designs only.
//
// usage: sta_bruteforce <design.v> [<design.v> ...]    exit 0 when everything matches
#include "../include/elaborator.hpp"
#include "../include/file_reader.hpp"
#include "../include/lexer.hpp"
#include "../include/mapper.hpp"
#include "../include/netlist_utils.hpp"
#include "../include/optimizer.hpp"
#include "../include/parser.hpp"
#include "../include/sta.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

bool same(double a, double b) { return a == b || std::fabs(a - b) < 1e-9; }

bool isWire(GateType type) {
    return type == GateType::OUTPUT || type == GateType::ASSIGN || type == GateType::BUF;
}

class BruteForce {
    const TimingReport& report;
    const TimingConstraints& constraints;
    std::set<const GateNode*> roots;

    double arcDelay(const GateNode* node, size_t pin, int edge) const {
        const RcArc& arc = cellFor(node->type).arcs[pin];
        return arc.intrinsic_ps[edge] + arc.r_kohm[edge] * report.timing.at(node).load_ff;
    }

public:
    BruteForce(const GateForest& forest, const TimingReport& r, const TimingConstraints& c)
        : report(r), constraints(c) {
        for (const auto& root : forest) roots.insert(root.get());
    }

    // Longest delay over every path from an input to (node, edge)
    double arrival(const GateNode* node, int edge) const {
        if (node->type == GateType::INPUT) return constraints.input_delay_ps;
        if (node->type == GateType::CONST0 || node->type == GateType::CONST1) return -kInf;
        if (isWire(node->type)) return arrival(node->inputs[0].get(), edge);
        double latest = -kInf;
        for (size_t k = 0; k < node->inputs.size(); ++k) {
            latest = std::max(latest, arrival(node->inputs[k].get(), 1 - edge) + arcDelay(node, k, edge));
        }
        return latest;
    }

    // Tightest (output required time - path delay) over every path from (node, edge) to an output
    double required(const GateNode* node, int edge) const {
        double tightest = roots.count(node) ? constraints.clock_period_ps - constraints.output_delay_ps : kInf;
        for (const auto& weak : node->outputs) {
            auto fanout = weak.lock();
            if (!fanout) continue;
            if (isWire(fanout->type)) {
                tightest = std::min(tightest, required(fanout.get(), edge));
                continue;
            }
            for (size_t k = 0; k < fanout->inputs.size(); ++k) {
                if (fanout->inputs[k].get() != node) continue;
                tightest = std::min(tightest, required(fanout.get(), 1 - edge) - arcDelay(fanout.get(), k, 1 - edge));
            }
        }
        return tightest;
    }
};

bool checkDesign(const std::string& path) {
    // Unusual constraints so input delay, output delay and period all show up in the numbers
    TimingConstraints constraints;
    constraints.clock_period_ps = 900;
    constraints.input_delay_ps = 13;
    constraints.output_delay_ps = 40;

    VerilogParser parser(VerilogLexer(readVerilogFile(path)).tokenize());
    Design design = elaborate(parser.parse());
    GateForest forest = NetlistOptimizer::optimize(TechnologyMapper::mapToNandInverter(design.forest));
    TimingReport report = analyzeTiming(forest, constraints, DelayModel::RC);
    BruteForce brute(forest, report, constraints);

    int pairs = 0, mismatches = 0;
    for (const auto& node : topologicalOrder(forest)) {
        for (int edge = 0; edge < 2; ++edge) {
            const NodeTiming& t = report.timing.at(node.get());
            double arrival = brute.arrival(node.get(), edge);
            double required = brute.required(node.get(), edge);
            pairs++;
            if (!same(arrival, t.arrival[edge]) || !same(required, t.required[edge])) {
                mismatches++;
                std::cout << "  mismatch at " << node->name << (edge ? " fall" : " rise")
                          << ": arrival " << arrival << " vs " << t.arrival[edge]
                          << ", required " << required << " vs " << t.required[edge] << "\n";
            }
        }
    }

    // The reported critical path must add up and carry the worst slack at every stage
    bool path_ok = true;
    if (!report.critical_path.empty()) {
        double total = constraints.input_delay_ps;
        for (const PathStage& stage : report.critical_path) {
            total += stage.delay;
            path_ok &= same(report.timing.at(stage.node).slack(stage.edge), report.worst_slack);
        }
        path_ok &= same(total, report.critical_path.back().arrival);
    }

    std::cout << path << ": " << pairs << " node/edge pairs, " << mismatches << " mismatches, critical path "
              << (path_ok ? "consistent" : "INCONSISTENT") << "\n";
    return mismatches == 0 && path_ok;
}

} // namespace

int main(int argc, char* argv[]) {
    bool ok = argc > 1;
    try {
        for (int i = 1; i < argc; ++i) ok &= checkDesign(argv[i]);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
    return ok ? 0 : 1;
}
