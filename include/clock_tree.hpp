#pragma once
#include "cell_netlist.hpp"
#include "liberty.hpp"
#include <string>

// Clock tree synthesis without placement: a balanced buffer tree built bottom-up, so every
// register clock pin sits behind the same number of buffers and no buffer drives more than
// max_fanout pins. With a propagated clock this gives real clock latency and skew (skew
// comes from the uneven loads of the last groups); with placement the same structure would
// be built from geometric clusters (an H-tree) instead of fanout groups.
struct ClockTreeOptions {
    std::string buffer = "sg13cmos5l_buf_8";
    int max_fanout = 8;
};

struct ClockTreeReport {
    int sinks = 0;
    int buffers = 0;
    int levels = 0;
};

// Rebuilds the net driven by input port `clock_port` as a buffer tree
ClockTreeReport insertClockTree(CellNetlist& netlist, const Library& lib, const std::string& clock_port,
                                const ClockTreeOptions& options = {});
