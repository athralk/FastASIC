#pragma once
#include <string>
#include <vector>

// Timing constraints: the subset of SDC (Synopsys Design Constraints, a Tcl dialect) that
// single-clock synthesis and sign-off flows use. Values are converted to ps / fF when read
// (SDC numbers are in the Liberty library's units).
//
// Supported: create_clock, set_clock_uncertainty, set_clock_transition, set_clock_latency,
// set_propagated_clock, set_input_delay, set_output_delay, set_load, set_driving_cell,
// set_input_transition; object queries get_ports, get_clocks, all_inputs [-no_clocks],
// all_outputs, all_clocks; Tcl `set` variables with $name substitution and [expr ...].
// Object lists are kept as name patterns (glob * ?) and matched against the netlist later.

struct SdcClock {
    std::string name;
    double period = 0;
    double rise = 0, fall = 0;                // waveform edges, default {0, period/2}
    std::vector<std::string> sources;         // port patterns; empty: a virtual clock
    double uncertainty_setup = 0, uncertainty_hold = 0;
    double transition = 0;                    // slew of the ideal clock
    double latency = 0;                       // ideal network latency
    bool propagated = false;                  // clock arrival computed through the clock tree
};

struct SdcPortDelay {
    std::vector<std::string> ports;
    std::string clock;
    double value = 0;
    bool max = true, min = true;              // -max / -min restrict it to one analysis
};

struct SdcPortValue {
    std::vector<std::string> ports;
    double value = 0;
};

struct SdcDrivingCell {
    std::vector<std::string> ports;
    std::string cell, pin, from_pin;
};

struct Sdc {
    std::vector<SdcClock> clocks;
    std::vector<SdcPortDelay> input_delays, output_delays;
    std::vector<SdcPortValue> loads, input_transitions;
    std::vector<SdcDrivingCell> driving_cells;
    std::vector<std::string> warnings;        // commands that were ignored
};

// time_unit_ps / cap_unit_ff: the Liberty units SDC values are given in
Sdc readSdc(const std::string& path, double time_unit_ps, double cap_unit_ff);
Sdc parseSdc(const std::string& text, double time_unit_ps, double cap_unit_ff);

// Glob match with * and ?
bool globMatch(const std::string& pattern, const std::string& text);
