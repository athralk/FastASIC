#pragma once
// Floorplan: die and core outline, placement rows, routing tracks, I/O pins, macro placement and
// the power grid (PDN). The result is written as DEF (def.hpp) for placement and routing.
#include "cell_netlist.hpp"
#include "lef.hpp"
#include <string>
#include <vector>

struct FloorplanOptions {
    double utilization = 0.5;     // standard-cell area / core area
    double aspect = 1.0;          // core height / width
    double core_margin = 10.0;    // um between the die edge and the core (I/O pins and routing go here)
    double die_width = 0, die_height = 0;   // um; both set: a fixed die, overriding utilization
    double macro_halo = 10.0;     // um kept free of standard cells around each macro
    std::string site = "CoreSite";
    std::string tracks_file;      // LibreLane tracks.info ("layer X|Y offset pitch"); empty: from the LEF
    std::string pin_hlayer = "Metal3", pin_vlayer = "Metal2";   // pins on left/right, top/bottom edges
    std::string tt_template;      // IHP Tiny Tapeout tile DEF (die, signal pins and supply names)
    bool pdn = true;
    std::string rail_layer = "Metal1", strap_layer = "Metal4", top_layer = "TopMetal1";
    double rail_width = 0.44;     // um, Metal1 follow-pin rails along the rows
    double strap_width = 2.2, strap_pitch = 75.0;   // um, vertical Metal4 straps
    // Width of the rail-to-strap via stacks: narrower than the strap, so their Metal2/Metal3 pads
    // leave more of the tracks under each strap free for signal wiring (two cuts per layer still fit)
    double rail_via_width = 0.96;
    double top_width = 3.2, top_pitch = 75.0;       // um, horizontal TopMetal1 straps
    std::string tie_lo = "sg13cmos5l_tielo", tie_hi = "sg13cmos5l_tiehi";
};

enum class Orient { N, S, FN, FS };
const char* orientName(Orient o);

struct FpRow {
    std::string name;
    Dbu x = 0, y = 0;     // origin of the first site
    int count = 0;        // sites
    Orient orient = Orient::N;
};

struct FpTracks {
    int layer = -1;
    bool x = true;        // TRACKS X: vertical lines at x = start + k * step
    Dbu start = 0, step = 0;
    int count = 0;
};

struct FpPin {
    int port_bit = -1;    // index into CellNetlist::port_bits
    int layer = -1;
    Dbu x = 0, y = 0;     // DEF placement point (on the die edge)
    Rect shape;           // relative to (x, y)
    char side = 'L';      // L R B T
};

struct FpComponent {
    int instance = -1;    // index into CellNetlist::instances
    const LefMacro* macro = nullptr;
    bool placed = false, fixed = false;
    Dbu x = 0, y = 0;
    Orient orient = Orient::N;
};

// A special (power) wire segment, or a via when `via` >= 0
struct FpSpecialWire {
    int net = 0;          // 0 VDD, 1 VSS
    int layer = -1;
    Dbu width = 0;
    Dbu x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    int via = -1;         // index into Floorplan::vias: a via placed at (x0, y0)
    bool followpin = false;
};

// A filler cell (no netlist instance): fills a gap in a row so N-wells and implants stay continuous
struct FpFiller {
    const LefMacro* macro = nullptr;
    Dbu x = 0, y = 0;
    Orient orient = Orient::N;
};

struct FpVia {
    std::string name;
    std::vector<LefShape> shapes;   // relative to the via centre
};

struct Floorplan {
    std::string design;
    int dbu = 1000;
    Rect die, core;
    std::string supplies[2] = {"VDD", "VSS"};
    bool tt_tile = false;
    const LefSite* site = nullptr;
    std::vector<FpRow> rows;
    std::vector<FpTracks> tracks;
    std::vector<FpPin> pins;
    std::vector<FpComponent> components;   // one per netlist instance, same order
    std::vector<FpFiller> fillers;
    std::vector<FpVia> vias;
    std::vector<FpSpecialWire> special;
    std::vector<Rect> macro_blockages;     // macro outline plus halo
    std::vector<std::string> notes;
    double cell_area = 0, macro_area = 0;  // um^2
    int tie_cells = 0;
};

// Replaces constant connections (1'b0 / 1'b1 on cell pins and output ports) with tie cells,
// one per load, as OpenROAD's insert_tiecells does. Returns the number of tie cells added.
int insertTieCells(CellNetlist& netlist, const Library& lib, const std::string& tie_lo, const std::string& tie_hi);

Floorplan buildFloorplan(const CellNetlist& netlist, const Lef& lef, const FloorplanOptions& options);

// Fills every gap in every row with filler cells (widest first). Returns the number added.
int insertFillers(Floorplan& fp, const Lef& lef, const std::vector<std::string>& filler_cells);

// Consistency checks (pins on tracks and inside the die, rows inside the core and clear of
// macros, ...). Returns the problems found; empty when the floorplan is sound.
std::vector<std::string> checkFloorplan(const Floorplan& fp, const CellNetlist& netlist, const Lef& lef);
