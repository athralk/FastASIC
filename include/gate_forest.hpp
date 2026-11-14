#pragma once
#include <memory>
#include <vector>
#include <string>

enum class GateType {
    INPUT,     
    OUTPUT,    
    ASSIGN,    
    AND, 
    OR, 
    NOT, 
    NAND, 
    NOR, 
    XOR, 
    XNOR, 
    BUF,
    MUX,
    DFF,       
    CONST0, 
    CONST1
};

struct GateNode : public std::enable_shared_from_this<GateNode> {
    int id;                       
    GateType type;
    std::string name;             
    
    std::vector<std::shared_ptr<GateNode>> inputs;   
    std::vector<std::weak_ptr<GateNode>> outputs;    

    double intrinsic_delay = 1.0; 
    double arrival_time = 0.0;    
    double required_time = 0.0;   
    double slack = 0.0;           

    std::string mapped_cell_name; 
    double cell_area = 0.0;       

    GateNode(int unique_id, GateType t, std::string n) 
        : id(unique_id), type(t), name(std::move(n)) {}

    void addFanin(const std::shared_ptr<GateNode>& fanin_node) {
        inputs.push_back(fanin_node);
        fanin_node->outputs.push_back(shared_from_this());
    }
};

using GateForest = std::vector<std::shared_ptr<GateNode>>;

// A module port as declared in the source, kept so a netlist can be written back
// out with exactly the same interface (vector ports are bit-blasted in the forest)
struct PortInfo {
    std::string name;
    bool is_input = true;
    bool is_vector = false;
    int msb = 0;
    int lsb = 0;
};

// One flip-flop. The netlist is cut at every register so all the combinational passes
// (mapping, optimization, simulation, SAT equivalence) keep working on a DAG:
//   - the register's output Q is an INPUT node named `name`
//   - its next state D is an extra forest root named nextStateName(name)
// The clock and reset are kept here, outside the logic.
struct RegisterBit {
    std::string name;               // "q", "acc[3]", "mem[2][7]"
    std::string clock;              // clock input port bit
    bool negedge = false;           // clocked on the falling edge
    std::string reset;              // asynchronous reset input port bit; empty: none
    bool reset_active_low = true;
    bool reset_value = false;       // value the reset forces
};

// A declared reg (or memory), for writing the netlist back out
struct RegisterDecl {
    std::string name;
    bool is_vector = false;
    int msb = 0, lsb = 0;
    bool is_memory = false;
    int first = 0, last = 0;        // memory word range [first:last]
    bool is_port = false;           // output reg
};

inline std::string nextStateName(const std::string& reg) { return reg + "$next"; }
inline bool isNextStateName(const std::string& name) {
    return name.size() > 5 && name.compare(name.size() - 5, 5, "$next") == 0;
}

// A hard macro instance (an SRAM): a black box. Like a register, it cuts the netlist:
//   - each output pin bit is an INPUT node named macroOutputName(instance, pin)
//   - each input pin bit is a forest root named macroInputName(instance, pin)
struct MacroInstance {
    std::string name;                                          // hierarchical instance name
    std::string cell;                                          // library cell, e.g. RM_IHPSG13_1P_1024x8_c2_bm_bist
    std::vector<std::pair<std::string, std::string>> inputs;   // pin bit ("A_ADDR[3]") -> root name
    std::vector<std::pair<std::string, std::string>> outputs;  // pin bit -> INPUT node name
};

inline std::string macroOutputName(const std::string& inst, const std::string& pin) { return inst + "." + pin; }
inline std::string macroInputName(const std::string& inst, const std::string& pin) { return inst + "." + pin + "$in"; }
inline bool isMacroInputName(const std::string& name) {
    return name.size() > 3 && name.compare(name.size() - 3, 3, "$in") == 0;
}

// A module known only by its ports (a hard macro from a Liberty library)
struct BlackBoxModule {
    std::string name;
    std::vector<PortInfo> ports;   // in the library's pin order
};

struct Design {
    std::string name;
    std::vector<PortInfo> ports;
    GateForest forest;   // OUTPUT roots: output bits ("y[3]"), register next states, macro inputs
    std::vector<RegisterBit> registers;
    std::vector<RegisterDecl> reg_decls;
    std::vector<MacroInstance> macros;
    std::vector<std::string> notes;   // decisions worth reporting (e.g. memories mapped to SRAMs)
};