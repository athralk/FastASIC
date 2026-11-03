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

struct Design {
    std::string name;
    std::vector<PortInfo> ports;
    GateForest forest;   // one OUTPUT root per output bit, named like "y" or "y[3]"
};