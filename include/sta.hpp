#pragma once
#include "cell_library.hpp"
#include "gate_forest.hpp"
#include <ostream>
#include <unordered_map>
#include <vector>

enum class DelayModel {
    UNIT,   // every cell costs 1: the critical path delay is the logic depth
    RC,     // delay = R_eff * C_par + R_eff * C_load, ignores input slew
};

// Boundary conditions. The defaults follow OpenLane's sky130_fd_sc_hd flow.
struct TimingConstraints {
    double clock_period_ps = 10000.0;   // CLOCK_PERIOD = 10 ns
    double input_delay_ps = 0.0;        // when the inputs switch, relative to the clock edge
    double output_delay_ps = 0.0;       // how early the outputs must settle before the next edge
    double output_load_ff = 33.442;     // OUTPUT_CAP_LOAD: the input capacitance of an inv_16
};

// Timing of one node's output net, kept separately for rising and falling transitions
struct NodeTiming {
    double load_ff = 0.0;                            // capacitance the net drives
    double arrival[2];                               // latest time the net settles (-inf: never switches)
    double required[2];                              // latest time it may settle (+inf: unconstrained)
    double delay[2] = {0.0, 0.0};                    // delay of the arc that set the arrival
    const GateNode* from[2] = {nullptr, nullptr};    // fanin on the latest path
    Edge from_edge[2] = {RISE, FALL};                // that fanin's transition
    int from_pin[2] = {-1, -1};                      // which cell input it arrived on

    NodeTiming();
    double slack(Edge e) const { return required[e] - arrival[e]; }
};

struct PathStage {
    const GateNode* node;
    Edge edge;
    int from_pin;     // cell input the transition came in on (-1 for ports and wires)
    double delay;
    double arrival;
    double load_ff;
};

struct TimingReport {
    DelayModel model;
    TimingConstraints constraints;
    std::unordered_map<const GateNode*, NodeTiming> timing;
    std::vector<const GateNode*> outputs;    // the forest roots, in order
    std::vector<PathStage> critical_path;    // input -> output; empty if no output ever switches
    double worst_slack;                      // +inf if no output ever switches
    int cell_count = 0;
    double total_area_um2 = 0.0;
};

// Static timing analysis of a mapped NAND2 + INV netlist. Also fills each node's
// arrival_time / required_time / slack (worst edge) and mapped_cell_name / cell_area.
TimingReport analyzeTiming(const GateForest& forest, const TimingConstraints& constraints, DelayModel model);

void printTimingReport(std::ostream& out, const TimingReport& report, size_t max_outputs = 10);
