#include "../include/cell_library.hpp"
#include "../include/netlist_utils.hpp"
#include <stdexcept>

const CellTiming& cellFor(GateType type) {
    //                          intrinsic (rise, fall) ps    R_eff (rise, fall) kOhm
    static const CellTiming inv_1{
        "sky130_fd_sc_hd__inv_1", 3.7536, {"A"}, {2.302}, "Y", 181.28,
        {{{18.779, 13.315}, {5.6270, 3.3299}}},           // A -> Y
    };
    static const CellTiming nand2_1{
        "sky130_fd_sc_hd__nand2_1", 3.7536, {"A", "B"}, {2.315, 2.324}, "Y", 166.64,
        {{{22.412, 18.420}, {6.1671, 5.0652}},            // A -> Y
         {{29.268, 23.156}, {6.1195, 5.0397}}},           // B -> Y (B is the slower input)
    };

    if (type == GateType::NOT) return inv_1;
    if (type == GateType::NAND) return nand2_1;
    throw std::runtime_error("no standard cell for " + gateTypeName(type) +
                             " gates (timing needs the mapped NAND2 + INV netlist)");
}
