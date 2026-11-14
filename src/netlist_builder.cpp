#include "../include/netlist_builder.hpp"
#include <cstdlib>
#include <stdexcept>

NetlistBuilder::NetlistBuilder(const Design& design, const Library& lib, const std::string& prefix)
    : design_(design), lib_(lib), prefix_(prefix) {
    inv_ = lib.find(prefix + "inv_1");
    nl.name = design.name;
    nl.ports = design.ports;
    for (const auto& p : design.ports) {
        int width = std::abs(p.msb - p.lsb) + 1;
        for (int k = 0; k < width; ++k) {
            int bit = p.msb >= p.lsb ? p.lsb + k : p.lsb - k;
            std::string name = p.is_vector ? p.name + "[" + std::to_string(bit) + "]" : p.name;
            if (p.is_input) {
                int net = nl.addNet(name);
                input_net_[name] = net;
                nl.port_bits.push_back({name, true, net});
            }
        }
    }
    // Register outputs are nets named after the register (an output reg's net is its port)
    for (const auto& r : design.registers) input_net_[r.name] = nl.addNet(r.name);
    for (const auto& m : design.macros) {
        for (const auto& [pin, q] : m.outputs) input_net_[q] = nl.addNet(q);
    }
}

CellNetlist::Instance& NetlistBuilder::addCell(const LibCell* cell) {
    CellNetlist::Instance inst;
    inst.name = "_m" + std::to_string(nl.instances.size()) + "_";
    inst.cell = cell;
    inst.pin_net.assign(cell->pins.size(), -1);
    nl.instances.push_back(std::move(inst));
    cell_counts[cell->name]++;
    area += cell->area;
    return nl.instances.back();
}

int NetlistBuilder::invert(int net) {
    if (net == CellNetlist::kConst0) return CellNetlist::kConst1;
    if (net == CellNetlist::kConst1) return CellNetlist::kConst0;
    auto it = inverted_.find(net);
    if (it != inverted_.end()) return it->second;
    if (!inv_) throw std::runtime_error("library has no " + prefix_ + "inv_1");
    int out = nl.addNet("_inv" + std::to_string(nl.netCount()) + "_");
    auto& inst = addCell(inv_);
    inst.pin_net[inv_->pinIndex("A")] = net;
    inst.pin_net[inv_->outputPin()] = out;
    return inverted_[net] = out;
}

void NetlistBuilder::finish(const std::vector<std::pair<std::string, int>>& root_nets) {
    std::unordered_map<std::string, int> next_net, macro_net;
    for (const auto& [name, net] : root_nets) {
        if (isMacroInputName(name)) macro_net[name] = net;
        else if (isNextStateName(name)) next_net[name.substr(0, name.size() - 5)] = net;
        else nl.port_bits.push_back({name, false, net});
    }
    for (const auto& m : design_.macros) {
        const LibCell* cell = lib_.find(m.cell);
        if (!cell) throw std::runtime_error("macro " + m.cell + " is not in the loaded libraries");
        auto& inst = addCell(cell);
        inst.name = m.name;
        auto pin = [&](const std::string& name) {
            int k = cell->pinIndex(name);
            if (k < 0) throw std::runtime_error("macro " + m.cell + " has no pin " + name);
            return k;
        };
        for (const auto& [p, root] : m.inputs) inst.pin_net[pin(p)] = macro_net.at(root);
        for (const auto& [p, q] : m.outputs) inst.pin_net[pin(p)] = input_net_.at(q);
    }

    // Flip-flops. Every sg13cmos5l flop has an active-low asynchronous reset (RESET_B):
    //   no reset          dfrbpq_1, RESET_B tied high
    //   reset to 0        dfrbpq_1
    //   reset to 1        dfrbp_1 storing the inverted value: D = !next, the register is Q_N
    //   active-high reset / falling-edge clock: an inverter in front of RESET_B / CLK
    for (const auto& r : design_.registers) {
        bool set = !r.reset.empty() && r.reset_value;
        const LibCell* ff = lib_.find(prefix_ + (set ? "dfrbp_1" : "dfrbpq_1"));
        if (!ff) throw std::runtime_error("library has no " + prefix_ + (set ? "dfrbp_1" : "dfrbpq_1") + " flip-flop");
        int clk = input_net_.at(r.clock);
        if (r.negedge) clk = invert(clk);
        int rst = CellNetlist::kConst1;
        if (!r.reset.empty()) rst = r.reset_active_low ? input_net_.at(r.reset) : invert(input_net_.at(r.reset));
        int d = next_net.at(r.name);
        if (set) d = invert(d);
        auto& inst = addCell(ff);
        inst.name = r.name + "_reg";
        inst.pin_net[ff->pinIndex("CLK")] = clk;
        inst.pin_net[ff->pinIndex("D")] = d;
        inst.pin_net[ff->pinIndex("RESET_B")] = rst;
        inst.pin_net[ff->pinIndex(set ? "Q_N" : "Q")] = input_net_.at(r.name);
    }
}
