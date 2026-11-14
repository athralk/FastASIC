#pragma once
#include <string>
#include <unordered_map>
#include <vector>

// Liberty (.lib) standard-cell library: the timing, capacitance, area and function of every
// cell, as characterized by the foundry. Values are converted to ps and fF when read.

// What a table axis is indexed by (from the lu_table_template's variable_1 / variable_2)
enum class TableVar { NONE, INPUT_TRANSITION, OUTPUT_CAP, CONSTRAINED_TRANSITION, RELATED_TRANSITION };

struct TableArgs {
    double input_transition = 0;         // ps, slew at the arc's input
    double output_cap = 0;               // fF, load on the arc's output net
    double constrained_transition = 0;   // ps, slew at a checked data pin
    double related_transition = 0;       // ps, slew at the clock pin it is checked against
};

// NLDM table: values[i * index2.size() + j] at (index1[i], index2[j])
struct LibTable {
    TableVar var1 = TableVar::NONE, var2 = TableVar::NONE;
    std::vector<double> index1, index2, values;

    bool empty() const { return values.empty(); }
    // Bilinear interpolation inside the table, linear extrapolation outside it (as OpenSTA does)
    double lookup(const TableArgs& args) const;
};

enum class TimingSense { POSITIVE, NEGATIVE, NON };

enum class TimingType {
    COMBINATIONAL,
    RISING_EDGE, FALLING_EDGE,          // clock -> output of a flip-flop
    CLEAR, PRESET,                      // asynchronous reset / set -> output
    SETUP_RISING, SETUP_FALLING, HOLD_RISING, HOLD_FALLING,
    RECOVERY_RISING, RECOVERY_FALLING, REMOVAL_RISING, REMOVAL_FALLING,
    OTHER,                              // min_pulse_width, three-state, ... (not timed)
};

bool isCheck(TimingType type);   // setup / hold / recovery / removal

struct LibArc {
    int from_pin = -1;   // the Liberty related_pin
    int to_pin = -1;     // the pin the timing group belongs to
    TimingSense sense = TimingSense::NON;
    TimingType type = TimingType::COMBINATIONAL;
    std::string when;
    LibTable cell_rise, cell_fall, rise_transition, fall_transition;   // delay arcs
    LibTable rise_constraint, fall_constraint;                         // checks
};

enum class PinDir { INPUT, OUTPUT, INOUT, INTERNAL };

struct LibPin {
    std::string name;
    PinDir dir = PinDir::INPUT;
    double cap = 0, rise_cap = 0, fall_cap = 0;   // fF
    // *_capacitance_range: OpenSTA uses the upper bound for late (max) and the lower bound
    // for early (min) analysis; equal to rise_cap / fall_cap when the library gives no range
    double rise_cap_min = 0, rise_cap_max = 0, fall_cap_min = 0, fall_cap_max = 0;
    double max_cap = 0, max_transition = 0;       // fF, ps (0: not given)
    bool is_clock = false;
    std::string function;                         // Boolean function of an output, e.g. "!(A*B)"
    std::string three_state;                      // tri-state enable condition (ebufn, einvn), empty for normal outputs
};

// bus(A_ADDR) { bus_type : A_9_0; ... }: one LibPin per bit, named A_ADDR[i]
struct LibBus {
    std::string name;
    int from = 0, to = 0;    // bit_from / bit_to of the bus type ([9:0]: 9, 0)
    std::vector<int> pins;   // pin indices in from..to order (most significant first for [9:0])
};

struct LibCell {
    std::string name;
    double area = 0;          // um^2
    double leakage = 0;       // in the library's leakage_power_unit
    std::vector<LibPin> pins;
    std::vector<LibArc> arcs;
    bool is_sequential = false;   // has an ff or latch group
    bool is_latch = false;
    std::string ff_clocked_on, ff_next_state, ff_clear, ff_preset;
    std::vector<LibBus> buses;
    bool is_macro = false;        // a hard macro (memory() group): a black box with timing
    // Fraction of a full rail-to-rail transition that this cell's slew numbers measure:
    // (slew_upper - slew_lower threshold) / slew_derate_from_library of its library (0.6 for 20-80%).
    // Slews are converted with it when a net connects cells from different libraries.
    double slew_fraction = 0.6;
    int address_width = 0, word_width = 0;

    int pinIndex(const std::string& name) const;
    const LibBus* findBus(const std::string& name) const;
    int outputPin() const;   // the first output pin, or -1
};

struct Library {
    std::string name;
    std::string path;
    double nom_voltage = 0, nom_temperature = 0;
    std::string leakage_unit;
    double time_unit_ps = 1000;   // the library's time unit (SDC files use it too)
    double slew_fraction = 0.6;   // see LibCell::slew_fraction; SDC slews are in this library's terms
    double cap_unit_ff = 1000;    // the library's capacitance unit
    std::vector<LibCell> cells;
    std::unordered_map<std::string, int> cell_index;

    const LibCell* find(const std::string& name) const;
};

// Reads a Liberty file. Throws std::runtime_error with the line number on syntax errors.
Library readLiberty(const std::string& path);
