#pragma once
#include "gate_forest.hpp"
#include "liberty.hpp"
#include <ostream>
#include <string>
#include <vector>
#include <unordered_set>

// A flat gate-level netlist of library cells: what Yosys / OpenLane write after synthesis
// and what static timing analysis works on. Everything is bit-level and index-based.
struct CellNetlist {
    static constexpr int kConst0 = 0;   // net 0 is tied low, net 1 tied high
    static constexpr int kConst1 = 1;

    struct PortBit {
        std::string name;   // "a" or "a[3]"
        bool is_input = true;
        int net = -1;
        bool bus_bit = false;
    };
    struct Instance {
        std::string name;
        const LibCell* cell = nullptr;
        std::vector<int> pin_net;   // by library pin index; -1 when unconnected
    };

    std::string name;
    std::vector<PortInfo> ports;        // as declared, for writing the netlist back out
    std::vector<PortBit> port_bits;
    std::vector<std::string> net_names;
    std::unordered_set<int> vector_nets;  // true bit-selects in the structural Verilog
    std::vector<Instance> instances;

    CellNetlist() : net_names{"1'b0", "1'b1"} {}
    int addNet(const std::string& net_name) {
        net_names.push_back(net_name);
        return static_cast<int>(net_names.size()) - 1;
    }
    size_t netCount() const { return net_names.size(); }
    double area() const;
};

// Reads structural Verilog with cell instances (one flat module; assigns between nets and
// constants are allowed). Every cell must exist in the library.
CellNetlist readCellNetlist(const std::string& path, const Library& lib);

// Writes it back as structural Verilog that Yosys, OpenSTA and OpenROAD can read
void writeCellNetlist(std::ostream& out, const CellNetlist& netlist);

// The Boolean function of the netlist as a gate forest (one OUTPUT root per output bit), built
// from the cells' Liberty functions, so it can be simulated and checked for equivalence.
// Flip-flop outputs become free inputs named after their net (combinational check only).
GateForest cellNetlistToForest(const CellNetlist& netlist);
