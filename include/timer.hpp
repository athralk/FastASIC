#pragma once
#include "cell_library.hpp"
#include "cell_netlist.hpp"
#include "liberty.hpp"
#include "sdc.hpp"
#include <ostream>
#include <string>
#include <vector>

// Static timing analysis of a cell netlist with NLDM tables (the delay model OpenSTA uses
// for Liberty libraries without CCS data):
//   - every pin carries rise/fall arrival and slew, late (max) and early (min)
//   - cell delay and output slew come from 2D tables at (input slew, output load)
//   - flip-flops launch through clock->Q arcs and are checked with setup/hold (and
//     recovery/removal for asynchronous resets) tables at (data slew, clock slew)
//   - clocks are ideal (latency + transition from SDC) or propagated through the clock tree
//   - primary inputs: input delay plus the delay of the SDC driving cell; outputs: output delay
// Wires are ideal (no RC): the timing a netlist has right after synthesis.

struct PathPoint {
    std::string pin;          // "inst/PIN" or a port name
    std::string cell;         // library cell, empty for ports
    Edge edge = RISE;
    double delay = 0;         // ps, from the previous point
    double arrival = 0;       // ps
    double slew = 0;          // ps
    double cap = 0;           // fF on the net it drives (driver pins only)
    bool is_driver = false;
};

struct TimingPath {
    std::string check;        // "setup", "hold", "recovery", "removal"
    bool max = true;          // late (setup-like) or early (hold-like) analysis
    std::string startpoint, endpoint;
    std::string start_kind, end_kind;
    std::vector<PathPoint> points;
    double arrival = 0, required = 0, slack = 0;
    double clock_time = 0;    // capture edge time
    double clock_latency = 0; // capture clock arrival at the endpoint register (propagated or ideal)
    double margin = 0;        // setup/hold time or output delay
    double uncertainty = 0;
};

struct EndpointSlack {
    std::string pin;
    std::string check;
    bool max = true;
    Edge edge = RISE;
    double arrival = 0, required = 0, slack = 0;
};

struct TimingResult {
    std::string clock_name;
    double period = 0;
    bool propagated_clock = false;
    size_t cells = 0, nets = 0, registers = 0;
    double area = 0, leakage = 0;
    double wns_setup = 0, tns_setup = 0, wns_hold = 0, tns_hold = 0;
    int setup_endpoints = 0, hold_endpoints = 0, setup_violations = 0, hold_violations = 0;
    double min_period = 0;                 // smallest period meeting every setup check
    double clock_skew = 0;                 // max - min clock arrival over registers
    std::vector<TimingPath> setup_paths, hold_paths;   // worst first
    std::vector<EndpointSlack> endpoints;  // every checked endpoint and edge
    std::vector<std::string> warnings;
    double ms_build = 0, ms_analyze = 0;
};

struct TimerOptions {
    int paths = 1;   // worst paths to trace per analysis
};

TimingResult analyzeNetlistTiming(const CellNetlist& netlist, const Library& lib, const Sdc& sdc,
                                  const TimerOptions& options = {});

void printTimingResult(std::ostream& out, const TimingResult& result);
void printTimingPath(std::ostream& out, const TimingPath& path);
void writeEndpointSlacks(std::ostream& out, const TimingResult& result);
