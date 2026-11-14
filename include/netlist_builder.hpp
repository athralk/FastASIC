#pragma once
#include "cell_netlist.hpp"
#include "gate_forest.hpp"
#include "liberty.hpp"
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

// Assembles a cell netlist around mapped combinational logic: the design's ports, one
// sg13cmos5l flip-flop per register and the hard-macro instances. Shared by the mappers.
class NetlistBuilder {
public:
    NetlistBuilder(const Design& design, const Library& lib, const std::string& prefix = "sg13cmos5l_");

    // Net of a primary input bit, register output or macro output, by name
    int inputNet(const std::string& name) const { return input_net_.at(name); }

    // A new cell instance (counted in cell_counts / area); connect it with pin_net
    CellNetlist::Instance& addCell(const LibCell* cell);
    int newNet(const std::string& name) { return nl.addNet(name); }
    int invert(int net);   // shared inverter per net

    // Connects the forest roots (outputs, register next states, macro inputs) given each
    // root's net, then adds the flip-flops and macros
    void finish(const std::vector<std::pair<std::string, int>>& root_nets);

    CellNetlist nl;
    std::map<std::string, int> cell_counts;
    double area = 0;

private:
    const Design& design_;
    const Library& lib_;
    std::string prefix_;
    const LibCell* inv_ = nullptr;
    std::unordered_map<std::string, int> input_net_;
    std::unordered_map<int, int> inverted_;
};
