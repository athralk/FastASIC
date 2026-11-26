#include "../include/clock_tree.hpp"
#include <algorithm>
#include "../include/timer.hpp"
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <tuple>
#include <functional>
#include <unordered_set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

// Verilog instances and nets share a namespace, including declared vector port names.
class BufferNames {
    std::unordered_set<std::string> used;
public:
    explicit BufferNames(const CellNetlist& nl) : used(nl.net_names.begin(), nl.net_names.end()) {
        for (const auto& port : nl.ports) used.insert(port.name);
        for (const auto& inst : nl.instances) used.insert(inst.name);
    }

    std::string reserve(const std::string& base) {
        std::string name = base;
        for (size_t suffix = 1; used.count(name) || used.count(name + "_n"); ++suffix) {
            name = base + "_" + std::to_string(suffix);
        }
        used.insert(name);
        used.insert(name + "_n");
        return name;
    }
};

// Rebuilds `sinks` on `root_net` into a balanced tree of `buf` cells with fanout <= `max_fanout`.
void buildBalancedTree(CellNetlist& nl, BufferNames& names, const LibCell* buf, int in_pin, int out_pin, int root_net,
                       std::vector<std::pair<int, int>> level, int max_fanout, const std::string& prefix,
                       int& buffers_out, int& levels_out) {
    while (static_cast<int>(level.size()) > max_fanout) {
        levels_out++;
        size_t groups = (level.size() + max_fanout - 1) / max_fanout;
        std::vector<std::pair<int, int>> next;
        next.reserve(groups);
        for (size_t g = 0; g < groups; ++g) {
            std::string name = names.reserve(prefix + "_L" + std::to_string(levels_out) + "_" + std::to_string(g));
            int net = nl.addNet(name + "_n");
            CellNetlist::Instance inst;
            inst.name = name;
            inst.cell = buf;
            inst.pin_net.assign(buf->pins.size(), -1);
            inst.pin_net[out_pin] = net;
            inst.pin_net[in_pin] = root_net;
            // Assign contiguous chunks (preserving bus/word locality instead of striding across the whole chip)
            size_t begin = (g * level.size()) / groups;
            size_t end = ((g + 1) * level.size()) / groups;
            for (size_t s = begin; s < end; ++s) {
                nl.instances[level[s].first].pin_net[level[s].second] = net;
            }
            nl.instances.push_back(std::move(inst));
            next.emplace_back(static_cast<int>(nl.instances.size()) - 1, in_pin);
            buffers_out++;
        }
        level = std::move(next);
    }
    for (const auto& [inst, pin] : level) nl.instances[inst].pin_net[pin] = root_net;
}

bool findBufferPins(const LibCell* buf, int& in_pin, int& out_pin) {
    if (!buf) return false;
    in_pin = -1;
    out_pin = buf->outputPin();
    for (size_t k = 0; k < buf->pins.size(); ++k) {
        if (buf->pins[k].dir == PinDir::INPUT) in_pin = static_cast<int>(k);
    }
    return in_pin >= 0 && out_pin >= 0;
}

} // namespace

ClockTreeReport insertClockTree(CellNetlist& nl, const Library& lib, const std::string& clock_port, const ClockTreeOptions& options) {
    const LibCell* buf = lib.find(options.buffer);
    int in_pin = -1, out_pin = -1;
    if (!findBufferPins(buf, in_pin, out_pin)) throw std::runtime_error("clock tree: invalid buffer cell " + options.buffer);
    if (options.max_fanout < 2) throw std::runtime_error("clock tree: fanout must be at least 2");

    int root = -1;
    for (const auto& pb : nl.port_bits) {
        if (pb.is_input && pb.name == clock_port) root = pb.net;
    }
    if (root < 0) throw std::runtime_error("clock tree: no input port " + clock_port);

    std::vector<std::pair<int, int>> level;
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] == root && inst.cell->pins[k].dir != PinDir::OUTPUT) {
                level.emplace_back(static_cast<int>(i), static_cast<int>(k));
            }
        }
    }
    ClockTreeReport report;
    report.sinks = static_cast<int>(level.size());
    if (report.sinks <= options.max_fanout) return report;
    BufferNames names(nl);
    buildBalancedTree(nl, names, buf, in_pin, out_pin, root, std::move(level), options.max_fanout, "cts", report.buffers, report.levels);
    return report;
}

BufferNetsReport bufferHighFanoutNets(CellNetlist& nl, const Library& lib, int max_fanout,
                                      const std::string& buffer_cell, const std::set<std::string>& skip_ports) {
    BufferNetsReport report;
    if (max_fanout < 2) return report;
    const LibCell* buf = lib.find(buffer_cell);
    if (!buf) buf = lib.find("sg13cmos5l_buf_4");
    if (!buf) buf = lib.find("sg13cmos5l_buf_2");
    int in_pin = -1, out_pin = -1;
    if (!findBufferPins(buf, in_pin, out_pin)) return report;

    std::set<int> skip_nets = {CellNetlist::kConst0, CellNetlist::kConst1};
    for (const auto& pb : nl.port_bits) {
        if (pb.is_input && skip_ports.count(pb.name)) skip_nets.insert(pb.net);
    }

    size_t orig_nets = nl.netCount();
    size_t orig_insts = nl.instances.size();
    std::vector<std::vector<std::pair<int, int>>> sinks(orig_nets);
    for (size_t i = 0; i < orig_insts; ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int n = inst.pin_net[k];
            if (n >= 2 && static_cast<size_t>(n) < orig_nets && inst.cell->pins[k].dir != PinDir::OUTPUT) {
                sinks[n].emplace_back(static_cast<int>(i), static_cast<int>(k));
            }
        }
    }

    BufferNames names(nl);
    for (size_t n = 2; n < orig_nets; ++n) {
        if (skip_nets.count(static_cast<int>(n))) continue;
        if (static_cast<int>(sinks[n].size()) <= max_fanout) continue;
        int lvls = 0;
        buildBalancedTree(nl, names, buf, in_pin, out_pin, static_cast<int>(n), std::move(sinks[n]), max_fanout,
                          "hfn_" + std::to_string(n), report.buffers_added, lvls);
        report.buffered_nets++;
    }
    return report;
}

namespace {
using Check = std::tuple<std::string, std::string, Edge>;
using Slacks = std::map<Check, double>;

// The timer follows instance cell pointers for delays, slews and constraints.
Slacks cornerSlacks(CellNetlist nl, const Library& corner, const Sdc& sdc, bool late) {
    for (auto& inst : nl.instances) {
        const LibCell* cell = corner.find(inst.cell->name);
        if (!cell || cell->pins.size() != inst.pin_net.size())
            throw std::runtime_error("hold repair: missing or incompatible corner cell " + inst.cell->name);
        std::vector<int> nets(cell->pins.size(), -1);
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            const auto& pin = inst.cell->pins[k];
            int idx = cell->pinIndex(pin.name);
            if (idx < 0 || cell->pins[idx].dir != pin.dir)
                throw std::runtime_error("hold repair: incompatible corner pin " + cell->name + "/" + pin.name);
            nets[idx] = inst.pin_net[k];
        }
        inst.pin_net = std::move(nets);
        inst.cell = cell;
    }
    Slacks result;
    for (const auto& ep : analyzeNetlistTiming(nl, corner, sdc, {0}).endpoints) {
        if (ep.max != late) continue;
        auto entry = result.emplace(Check{ep.pin, ep.check, ep.edge}, ep.slack);
        entry.first->second = std::min(entry.first->second, ep.slack);
    }
    return result;
}

double worstEdge(const Slacks& slacks, const std::string& pin, const char* check) {
    double worst = std::numeric_limits<double>::infinity();
    for (auto edge : {RISE, FALL}) {
        auto it = slacks.find({pin, check, edge});
        if (it != slacks.end()) worst = std::min(worst, it->second);
    }
    return worst;
}

bool preservesChecks(const Slacks& before, const Slacks& after) {
    for (const auto& [key, slack] : before) {
        auto trial = after.find(key);
        if (trial == after.end() || trial->second + 1e-6 < std::min(0.0, slack)) return false;
    }
    return true;
}
} // namespace

HoldRepairReport repairFastHoldPaths(CellNetlist& nl, const Library& lib, const Library& fast, const Library& slow,
                        const Sdc& constraints) {
    if (constraints.clocks.size() != 1 || constraints.clocks.front().sources.empty())
        throw std::runtime_error("hold repair requires one clock bound to a port");
    const LibCell* delay = lib.find("sg13cmos5l_dlygate4sd3_1");
    int in_pin, out_pin;
    if (!findBufferPins(delay, in_pin, out_pin)) throw std::runtime_error("hold repair: missing IHP delay cell");
    auto hold = cornerSlacks(nl, fast, constraints, false);
    auto setup = cornerSlacks(nl, slow, constraints, true);
    struct Sink { size_t inst, pin; std::string name; };
    std::vector<Sink> sinks;
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        if (!inst.cell->is_sequential) continue;
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            auto name = inst.name + "/" + inst.cell->pins[k].name;
            if (std::isfinite(worstEdge(hold, name, "hold"))) sinks.push_back({i, k, name});
        }
    }
    auto eligible = [&](const Sink& sink) {
        double margin = worstEdge(setup, sink.name, "setup");
        return worstEdge(hold, sink.name, "hold") < 0 && std::isfinite(margin) && margin > 0;
    };
    int added = 0;
    // Try independent endpoints together. Split a rejected batch so one
    // setup-limited endpoint cannot veto repairs elsewhere in the design.
    for (int round = 0; round < 8; ++round) {
        std::vector<Sink> todo;
        std::copy_if(sinks.begin(), sinks.end(), std::back_inserter(todo), eligible);
        int previous = added;
        std::function<void(size_t, size_t)> attempt = [&](size_t begin, size_t end) {
            // Earlier batches can change the loading of a shared driver.
            while (begin < end && !eligible(todo[begin])) ++begin;
            if (begin == end) return;
            for (size_t j = begin + 1; j < end; ++j) {
                if (!eligible(todo[j])) {
                    attempt(begin, j);
                    attempt(j + 1, end);
                    return;
                }
            }
            CellNetlist trial = nl;
            BufferNames names(trial);
            for (size_t j = begin; j < end; ++j) {
                const auto& sink = todo[j];
                std::string name = names.reserve("hold_buf_" + std::to_string(added + j - begin));
                int net = trial.addNet(name + "_n");
                auto& input = trial.instances[sink.inst].pin_net[sink.pin];
                CellNetlist::Instance inst{name, delay, std::vector<int>(delay->pins.size(), -1)};
                inst.pin_net[in_pin] = input;
                inst.pin_net[out_pin] = net;
                input = net;
                trial.instances.push_back(std::move(inst));
            }
            auto candidate_hold = cornerSlacks(trial, fast, constraints, false);
            bool improves = preservesChecks(hold, candidate_hold);
            for (size_t j = begin; improves && j < end; ++j)
                improves = worstEdge(candidate_hold, todo[j].name, "hold") >
                           worstEdge(hold, todo[j].name, "hold") + 1e-6;
            if (improves) {
                auto candidate_setup = cornerSlacks(trial, slow, constraints, true);
                if (preservesChecks(setup, candidate_setup)) {
                    nl = std::move(trial);
                    hold = std::move(candidate_hold);
                    setup = std::move(candidate_setup);
                    added += static_cast<int>(end - begin);
                    return;
                }
            }
            if (end - begin > 1) {
                size_t mid = begin + (end - begin) / 2;
                attempt(begin, mid);
                attempt(mid, end);
            }
        };
        if (!todo.empty()) attempt(0, todo.size());
        if (added == previous) break;
    }
    int remaining = std::count_if(sinks.begin(), sinks.end(), [&](const Sink& sink) {
        return worstEdge(hold, sink.name, "hold") < 0;
    });
    return {added, remaining};
}
