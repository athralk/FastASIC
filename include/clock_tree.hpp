#pragma once
#include "cell_netlist.hpp"
#include "liberty.hpp"
#include <set>
#include <string>

// Clock tree synthesis without placement: a balanced buffer tree built bottom-up, so every
// register clock pin sits behind the same number of buffers and no buffer drives more than
// max_fanout pins.
struct ClockTreeOptions {
    std::string buffer = "sg13cmos5l_buf_8";
    int max_fanout = 8;
};

struct ClockTreeReport {
    int sinks = 0;
    int buffers = 0;
    int levels = 0;
};

struct BufferNetsReport {
    int buffered_nets = 0;
    int buffers_added = 0;
    int hold_buffers_added = 0;
};

// Rebuilds the net driven by input port `clock_port` as a buffer tree
ClockTreeReport insertClockTree(CellNetlist& netlist, const Library& lib, const std::string& clock_port,
                                const ClockTreeOptions& options = {});

// Rebuilds any non-clock net whose fanout exceeds `max_fanout` into a balanced buffer tree
// so high-fanout resets, enables, and select lines do not suffer multi-picofarad slew violations.
BufferNetsReport bufferHighFanoutNets(CellNetlist& netlist, const Library& lib, int max_fanout = 12,
                                      const std::string& buffer_cell = "sg13cmos5l_buf_4",
                                      const std::set<std::string>& skip_ports = {"clk", "clock", "CLK"});

struct Sdc;
struct HoldRepairReport {
    int buffers = 0;
    int remaining_data_endpoints = 0;
};

// Select violating register inputs at the fast corner and validate each change at both corners.
HoldRepairReport repairFastHoldPaths(CellNetlist& netlist, const Library& lib, const Library& fast, const Library& slow,
                        const Sdc& constraints);
