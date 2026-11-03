#include "../include/sta.hpp"
#include "../include/netlist_utils.hpp"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr Edge kEdges[] = {RISE, FALL};

Edge opposite(Edge e) { return e == RISE ? FALL : RISE; }

// Nodes with no logic: the net just continues through them with zero delay
bool isWire(GateType type) {
    return type == GateType::ASSIGN || type == GateType::BUF || type == GateType::OUTPUT;
}

bool isCell(GateType type) { return type == GateType::NOT || type == GateType::NAND; }

class TimingAnalyzer {
    const GateForest& forest;
    const TimingConstraints& constraints;
    DelayModel model;
    Corner corner;
    std::vector<std::shared_ptr<GateNode>> order;
    std::unordered_map<const GateNode*, NodeTiming> timing;

    // Capacitance on the net a node drives: every cell input pin connected to it, plus the
    // external load of each primary output it reaches (wires are followed to their own fanouts).
    // Wire capacitance is not modelled yet.
    double loadOf(const GateNode* node) const {
        double load = node->type == GateType::OUTPUT ? constraints.output_load_ff : 0.0;
        std::unordered_set<const GateNode*> seen;
        for (const auto& weak : node->outputs) {
            auto fanout = weak.lock();
            if (!fanout || !seen.insert(fanout.get()).second) continue;
            if (isWire(fanout->type)) {
                load += loadOf(fanout.get());
                continue;
            }
            const CellTiming& cell = cellFor(fanout->type, corner);
            for (size_t k = 0; k < fanout->inputs.size(); ++k) {
                if (fanout->inputs[k].get() == node) load += cell.input_cap_ff[k];
            }
        }
        return load;
    }

    double arcDelay(const CellTiming& cell, int pin, Edge out, double load_ff) const {
        if (model == DelayModel::UNIT) return 1.0;
        const RcArc& arc = cell.arcs[pin];
        return arc.intrinsic_ps[out] + arc.r_kohm[out] * load_ff;   // R*C_par + R*C_load
    }

    // Forward pass: latest arrival time of every net, rise and fall separately
    void propagateArrivals() {
        for (const auto& node : order) {
            NodeTiming& t = timing[node.get()];
            t.load_ff = loadOf(node.get());

            if (node->type == GateType::INPUT) {
                // Ideal input: switches at the input delay with no drive delay of its own
                t.arrival[RISE] = t.arrival[FALL] = constraints.input_delay_ps;
            } else if (node->type == GateType::CONST0 || node->type == GateType::CONST1) {
                // A constant never switches, so it starts no path (arrival stays -inf)
            } else if (isWire(node->type)) {
                const GateNode* in = node->inputs.at(0).get();
                for (Edge e : kEdges) {
                    t.arrival[e] = timing.at(in).arrival[e];
                    t.from[e] = in;
                    t.from_edge[e] = e;
                }
            } else {
                const CellTiming& cell = cellFor(node->type, corner);
                for (Edge e : kEdges) {
                    // Both cells invert (negative unate): a rising output comes from a falling input
                    Edge in_edge = opposite(e);
                    for (size_t k = 0; k < node->inputs.size(); ++k) {
                        const GateNode* in = node->inputs[k].get();
                        double in_arrival = timing.at(in).arrival[in_edge];
                        if (in_arrival == -kInf) continue;
                        double d = arcDelay(cell, static_cast<int>(k), e, t.load_ff);
                        if (in_arrival + d > t.arrival[e]) {
                            t.arrival[e] = in_arrival + d;
                            t.delay[e] = d;
                            t.from[e] = in;
                            t.from_edge[e] = in_edge;
                            t.from_pin[e] = static_cast<int>(k);
                        }
                    }
                }
            }
        }
    }

    // Backward pass: the latest each net may settle and still meet every output it feeds
    void propagateRequired() {
        double output_required = constraints.clock_period_ps - constraints.output_delay_ps;
        for (const auto& root : forest) {
            NodeTiming& t = timing.at(root.get());
            for (Edge e : kEdges) t.required[e] = std::min(t.required[e], output_required);
        }

        for (auto it = order.rbegin(); it != order.rend(); ++it) {
            const GateNode* node = it->get();
            const NodeTiming& t = timing.at(node);
            if (isWire(node->type)) {
                NodeTiming& in = timing.at(node->inputs.at(0).get());
                for (Edge e : kEdges) in.required[e] = std::min(in.required[e], t.required[e]);
            } else if (isCell(node->type)) {
                const CellTiming& cell = cellFor(node->type, corner);
                for (Edge e : kEdges) {
                    for (size_t k = 0; k < node->inputs.size(); ++k) {
                        NodeTiming& in = timing.at(node->inputs[k].get());
                        double d = arcDelay(cell, static_cast<int>(k), e, t.load_ff);
                        in.required[opposite(e)] = std::min(in.required[opposite(e)], t.required[e] - d);
                    }
                }
            }
        }
    }

    // Copies the worst-edge numbers into the GateNode fields the netlist already carries
    void annotateNodes(TimingReport& report) {
        for (const auto& node : order) {
            const NodeTiming& t = timing.at(node.get());
            node->arrival_time = std::max(t.arrival[RISE], t.arrival[FALL]);
            node->required_time = std::min(t.required[RISE], t.required[FALL]);
            node->slack = std::min(t.slack(RISE), t.slack(FALL));
            if (isCell(node->type)) {
                const CellTiming& cell = cellFor(node->type, corner);
                node->mapped_cell_name = cell.name;
                node->cell_area = cell.area_um2;
                report.cell_count++;
                report.total_area_um2 += cell.area_um2;
            }
        }
    }

    void traceCriticalPath(TimingReport& report) {
        const GateNode* end = nullptr;
        Edge end_edge = RISE;
        for (const auto& root : forest) {
            const NodeTiming& t = timing.at(root.get());
            for (Edge e : kEdges) {
                if (t.arrival[e] == -kInf) continue;
                if (!end || t.slack(e) < report.worst_slack ||
                    (t.slack(e) == report.worst_slack && t.arrival[e] > timing.at(end).arrival[end_edge])) {
                    end = root.get();
                    end_edge = e;
                    report.worst_slack = t.slack(e);
                }
            }
        }

        // Walk back along the fanins that set each arrival time
        for (const GateNode* node = end; node;) {
            const NodeTiming& t = timing.at(node);
            report.critical_path.push_back({node, end_edge, t.from_pin[end_edge], t.delay[end_edge],
                                            t.arrival[end_edge], t.load_ff});
            Edge previous = t.from_edge[end_edge];
            node = t.from[end_edge];
            end_edge = previous;
        }
        std::reverse(report.critical_path.begin(), report.critical_path.end());
    }

public:
    TimingAnalyzer(const GateForest& f, const TimingConstraints& c, DelayModel m, Corner k)
        : forest(f), constraints(c), model(m), corner(k), order(topologicalOrder(f)) {}

    TimingReport run() {
        propagateArrivals();
        propagateRequired();

        TimingReport report;
        report.model = model;
        report.corner = corner;
        report.constraints = constraints;
        report.worst_slack = kInf;
        for (const auto& root : forest) report.outputs.push_back(root.get());
        annotateNodes(report);
        traceCriticalPath(report);
        report.timing = std::move(timing);
        return report;
    }
};

std::string fixed2(double value) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(2) << value;
    return s.str();
}

std::string shortCellName(const std::string& name) {
    size_t sep = name.rfind("__");
    return sep == std::string::npos ? name : name.substr(sep + 2);
}

std::string describeStage(const PathStage& stage, Corner corner) {
    const GateNode* node = stage.node;
    switch (node->type) {
    case GateType::INPUT:  return node->name + " (input)";
    case GateType::OUTPUT: return node->name + " (output)";
    case GateType::ASSIGN:
    case GateType::BUF:    return node->name + " (wire)";
    default: {
        const CellTiming& cell = cellFor(node->type, corner);
        return instanceName(*node) + " " + shortCellName(cell.name) + " " +
               cell.input_pins.at(stage.from_pin) + "->" + cell.output_pin;
    }
    }
}

} // namespace

NodeTiming::NodeTiming() : arrival{-kInf, -kInf}, required{kInf, kInf} {}

TimingReport analyzeTiming(const GateForest& forest, const TimingConstraints& constraints, DelayModel model,
                           Corner corner) {
    return TimingAnalyzer(forest, constraints, model, corner).run();
}

void printTimingReport(std::ostream& out, const TimingReport& report, size_t max_outputs) {
    const TimingConstraints& c = report.constraints;
    out << "\n=== Static Timing Analysis ===\n";
    if (report.model == DelayModel::RC) {
        out << "Delay model: RC, t = R_eff*C_par + R_eff*C_load (sg13cmos5l_stdcell, " << cornerName(report.corner)
            << "), times in ps\n";
    } else {
        out << "Delay model: unit, every cell = 1 (the critical path delay is the logic depth)\n";
    }
    out << "Constraints: clock period " << fixed2(c.clock_period_ps) << ", input delay " << fixed2(c.input_delay_ps)
        << ", output delay " << fixed2(c.output_delay_ps) << ", output load " << c.output_load_ff << " fF\n";
    out << "Cells: " << report.cell_count << " (area " << fixed2(report.total_area_um2) << " um^2)\n";

    if (report.critical_path.empty()) {
        out << "No output depends on a switching input (every output is constant).\n";
        return;
    }

    const auto& path = report.critical_path;
    long depth = std::count_if(path.begin(), path.end(), [](const PathStage& s) { return isCell(s.node->type); });
    out << "\nCritical path: " << path.front().node->name << " -> " << path.back().node->name
        << " (" << depth << (depth == 1 ? " cell)\n" : " cells)\n");
    out << "       Delay     Arrival  Edge  Load fF  Pin\n";
    out << "  ----------  ----------  ----  -------  ------------------------------\n";
    for (const PathStage& stage : path) {
        out << std::setw(12) << fixed2(stage.delay) << std::setw(12) << fixed2(stage.arrival)
            << std::setw(6) << (stage.edge == RISE ? "^" : "v") << std::setw(9) << fixed2(stage.load_ff)
            << "  " << describeStage(stage, report.corner) << "\n";
    }
    double arrival = path.back().arrival;
    out << "  data arrival time   " << std::setw(12) << fixed2(arrival) << "\n";
    out << "  data required time  " << std::setw(12) << fixed2(arrival + report.worst_slack) << "\n";
    out << "  slack               " << std::setw(12) << fixed2(report.worst_slack)
        << (report.worst_slack >= 0 ? "  (MET)" : "  (VIOLATED)") << "\n";

    // Every output, worst slack first
    struct Row { const GateNode* node; Edge edge; double slack; };
    std::vector<Row> rows;
    size_t constant_outputs = 0;
    for (const GateNode* output : report.outputs) {
        const NodeTiming& t = report.timing.at(output);
        Edge worst = t.slack(RISE) <= t.slack(FALL) ? RISE : FALL;
        if (t.arrival[worst] == -kInf) {
            constant_outputs++;
            continue;
        }
        rows.push_back({output, worst, t.slack(worst)});
    }
    std::stable_sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.slack < b.slack; });

    out << "\nOutputs by slack (" << std::min(rows.size(), max_outputs) << " of " << rows.size() << " shown):\n";
    out << "  Output                    Edge     Arrival    Required       Slack\n";
    for (size_t i = 0; i < rows.size() && i < max_outputs; ++i) {
        const NodeTiming& t = report.timing.at(rows[i].node);
        out << "  " << std::left << std::setw(24) << rows[i].node->name << std::right
            << std::setw(6) << (rows[i].edge == RISE ? "^" : "v")
            << std::setw(12) << fixed2(t.arrival[rows[i].edge]) << std::setw(12) << fixed2(t.required[rows[i].edge])
            << std::setw(12) << fixed2(rows[i].slack) << "\n";
    }
    if (constant_outputs) out << "  (" << constant_outputs << " constant outputs never switch and are not timed)\n";
}
