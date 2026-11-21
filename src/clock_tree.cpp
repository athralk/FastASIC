#include "../include/clock_tree.hpp"
#include <algorithm>
#include <cstdint>
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

int repairFastHoldPaths(CellNetlist& nl, const Library& lib, int max_short_hops, int max_setup_hops) {
    const LibCell* dly = lib.find("sg13cmos5l_dlygate4sd3_1");
    if (!dly) dly = lib.find("sg13cmos5l_dlygate4sd2_1");
    if (!dly) dly = lib.find("sg13cmos5l_buf_1");
    const LibCell* fast_buf = lib.find("sg13cmos5l_buf_1");
    if (!fast_buf) fast_buf = dly;
    int in_pin = -1, out_pin = -1, fin_pin = -1, fout_pin = -1;
    if (!findBufferPins(dly, in_pin, out_pin)) return 0;
    if (!findBufferPins(fast_buf, fin_pin, fout_pin)) { fast_buf = dly; fin_pin = in_pin; fout_pin = out_pin; }

    size_t n_nets = nl.netCount();
    size_t n_insts = nl.instances.size();
    std::vector<int> net_driver_inst(n_nets, -1);
    for (size_t i = 0; i < n_insts; ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int n = inst.pin_net[k];
            if (n >= 2 && static_cast<size_t>(n) < n_nets && inst.cell->pins[k].dir == PinDir::OUTPUT) {
                net_driver_inst[n] = static_cast<int>(i);
            }
        }
    }

    // Memoized min hops from any sequential output (Q) and max combinational hops from any startpoint
    constexpr int kInf = 1000000;
    std::vector<int> min_seq(n_nets, -1), max_comb(n_nets, -1);
    std::vector<uint8_t> active(n_nets, 0);

    std::function<std::pair<int, int>(int)> analyzeNet = [&](int n) -> std::pair<int, int> {
        if (n < 2 || static_cast<size_t>(n) >= n_nets) return {kInf, 0};
        if (min_seq[n] >= 0) return {min_seq[n], max_comb[n]};
        if (active[n]) return {kInf, 0};
        int drv = net_driver_inst[n];
        if (drv < 0) {
            min_seq[n] = kInf;
            max_comb[n] = 0;
            return {kInf, 0};
        }
        const auto& inst = nl.instances[drv];
        if (inst.cell->is_sequential || inst.cell->is_macro) {
            min_seq[n] = 0;
            max_comb[n] = 0;
            return {0, 0};
        }
        active[n] = 1;
        int best_min = kInf, worst_max = 0;
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.cell->pins[k].dir == PinDir::OUTPUT) continue;
            auto [mn, mx] = analyzeNet(inst.pin_net[k]);
            if (mn < kInf) best_min = std::min(best_min, mn + 1);
            worst_max = std::max(worst_max, mx + 1);
        }
        active[n] = 0;
        min_seq[n] = best_min;
        max_comb[n] = worst_max;
        return {best_min, worst_max};
    };

    BufferNames names(nl);
    int added = 0;
    for (size_t i = 0; i < n_insts; ++i) {
        if (!nl.instances[i].cell->is_sequential) continue;
        const LibCell* cell = nl.instances[i].cell;
        for (size_t k = 0; k < cell->pins.size(); ++k) {
            const LibPin& pin = cell->pins[k];
            if (pin.dir != PinDir::INPUT || pin.is_clock) continue;
            // Only buffer synchronous data inputs (D), not async reset/set pins
            bool is_data_pin = false;
            for (const LibArc& arc : cell->arcs) {
                if (arc.to_pin == static_cast<int>(k) &&
                    (arc.type == TimingType::SETUP_RISING || arc.type == TimingType::SETUP_FALLING ||
                     arc.type == TimingType::HOLD_RISING || arc.type == TimingType::HOLD_FALLING)) {
                    is_data_pin = true;
                }
            }
            if (!is_data_pin) continue;
            int net = nl.instances[i].pin_net[k];
            auto [mn, mx] = analyzeNet(net);
            if (mn <= max_short_hops && mx <= max_setup_hops) {
                // Use full delay cell on shallow-setup flops (mx <= 10), lighter buf_1 on deeper paths
                bool use_dly = (mx <= 10);
                const LibCell* chosen = use_dly ? dly : fast_buf;
                int cp_in = use_dly ? in_pin : fin_pin;
                int cp_out = use_dly ? out_pin : fout_pin;
                std::string bname = names.reserve("hold_buf_" + std::to_string(added++));
                int dnet = nl.addNet(bname + "_n");
                CellNetlist::Instance binst;
                binst.name = bname;
                binst.cell = chosen;
                binst.pin_net.assign(chosen->pins.size(), -1);
                binst.pin_net[cp_in] = net;
                binst.pin_net[cp_out] = dnet;
                nl.instances[i].pin_net[k] = dnet;
                nl.instances.push_back(std::move(binst));
            }
        }
    }
    return added;
}
