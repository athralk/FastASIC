#include "../include/clock_tree.hpp"
#include <stdexcept>
#include <utility>
#include <vector>

ClockTreeReport insertClockTree(CellNetlist& nl, const Library& lib, const std::string& clock_port, const ClockTreeOptions& options) {
    const LibCell* buf = lib.find(options.buffer);
    if (!buf) throw std::runtime_error("clock tree: unknown buffer cell " + options.buffer);
    int in_pin = -1, out_pin = buf->outputPin();
    for (size_t k = 0; k < buf->pins.size(); ++k) {
        if (buf->pins[k].dir == PinDir::INPUT) in_pin = static_cast<int>(k);
    }
    if (in_pin < 0 || out_pin < 0) throw std::runtime_error("clock tree: " + options.buffer + " is not a buffer");
    if (options.max_fanout < 2) throw std::runtime_error("clock tree: fanout must be at least 2");

    int root = -1;
    for (const auto& pb : nl.port_bits) {
        if (pb.is_input && pb.name == clock_port) root = pb.net;
    }
    if (root < 0) throw std::runtime_error("clock tree: no input port " + clock_port);

    // Sinks: every instance input pin on the clock net, as (instance, pin)
    std::vector<std::pair<int, int>> level;
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] == root && inst.cell->pins[k].dir != PinDir::OUTPUT) level.emplace_back(static_cast<int>(i), static_cast<int>(k));
        }
    }
    ClockTreeReport report;
    report.sinks = static_cast<int>(level.size());

    // Group the current level into chunks of max_fanout, each behind a new buffer, until the
    // remaining drivers fit on the root net
    while (static_cast<int>(level.size()) > options.max_fanout) {
        report.levels++;
        size_t groups = (level.size() + options.max_fanout - 1) / options.max_fanout;
        std::vector<std::pair<int, int>> next;
        for (size_t g = 0; g < groups; ++g) {
            std::string name = "cts_L" + std::to_string(report.levels) + "_" + std::to_string(g);
            int net = nl.addNet(name + "_clk");
            CellNetlist::Instance inst;
            inst.name = name;
            inst.cell = buf;
            inst.pin_net.assign(buf->pins.size(), -1);
            inst.pin_net[out_pin] = net;
            inst.pin_net[in_pin] = root;   // reconnected by the next level
            // Spread sinks evenly over the groups so loads stay balanced
            for (size_t s = g; s < level.size(); s += groups) nl.instances[level[s].first].pin_net[level[s].second] = net;
            nl.instances.push_back(std::move(inst));
            next.emplace_back(static_cast<int>(nl.instances.size()) - 1, in_pin);
            report.buffers++;
        }
        level = std::move(next);
    }
    for (const auto& [inst, pin] : level) nl.instances[inst].pin_net[pin] = root;
    return report;
}
