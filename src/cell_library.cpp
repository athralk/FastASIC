#include "../include/cell_library.hpp"
#include "../include/netlist_utils.hpp"
#include <stdexcept>

std::string cornerName(Corner corner) {
    switch (corner) {
    case Corner::FAST: return "fast 1.32V -40C";
    case Corner::SLOW: return "slow 1.08V 125C";
    case Corner::TYP:  break;
    }
    return "typ 1.20V 25C";
}

const CellTiming& cellFor(GateType type, Corner corner) {
    // Input capacitance changes with the corner too, not just the drive strength
    //                          intrinsic (rise, fall) ps    R_eff (rise, fall) kOhm
    static const CellTiming inv_1[] = {
        {"sg13cmos5l_inv_1", 5.4432, {"A"}, {2.867}, "Y", 300.00,         // typ_1p20V_25C
         {{{19.589, 19.701}, {2.7682, 2.2105}}}},                         //   A -> Y
        {"sg13cmos5l_inv_1", 5.4432, {"A"}, {3.058}, "Y", 300.00,         // fast_1p32V_m40C
         {{{15.424, 15.999}, {1.7624, 1.4824}}}},
        {"sg13cmos5l_inv_1", 5.4432, {"A"}, {2.703}, "Y", 300.00,         // slow_1p08V_125C
         {{{26.035, 25.777}, {4.3306, 3.6854}}}},
    };
    static const CellTiming nand2_1[] = {
        {"sg13cmos5l_nand2_1", 7.2576, {"A", "B"}, {2.886, 2.978}, "Y", 300.00,   // typ_1p20V_25C
         {{{21.593, 28.063}, {2.7679, 4.0814}},                                   //   A -> Y
          {{25.111, 32.346}, {2.7683, 4.0796}}}},                                 //   B -> Y (slower input)
        {"sg13cmos5l_nand2_1", 7.2576, {"A", "B"}, {3.073, 3.196}, "Y", 300.00,   // fast_1p32V_m40C
         {{{16.872, 21.698}, {1.7613, 2.8038}},
          {{19.289, 24.280}, {1.7612, 2.8014}}}},
        {"sg13cmos5l_nand2_1", 7.2576, {"A", "B"}, {2.722, 2.787}, "Y", 300.00,   // slow_1p08V_125C
         {{{29.006, 40.169}, {4.3301, 6.7692}},
          {{34.238, 46.941}, {4.3319, 6.7669}}}},
    };

    int index = static_cast<int>(corner);
    if (type == GateType::NOT) return inv_1[index];
    if (type == GateType::NAND) return nand2_1[index];
    throw std::runtime_error("no standard cell for " + gateTypeName(type) +
                             " gates (timing needs the mapped NAND2 + INV netlist)");
}
