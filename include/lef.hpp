#pragma once
// LEF (Library Exchange Format): the physical view of a technology and its cells.
//  - tech LEF: metal and cut layers (direction, pitch, width, spacing, R/C) and vias
//  - cell LEF: each cell's outline, pin shapes and obstructions
// All coordinates are integer database units (DBU, 1000 per micron for IHP).
#include "liberty.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

using Dbu = int64_t;

struct Rect {
    Dbu x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    Dbu width() const { return x1 - x0; }
    Dbu height() const { return y1 - y0; }
    bool overlaps(const Rect& o) const { return x0 < o.x1 && o.x0 < x1 && y0 < o.y1 && o.y0 < y1; }
    bool contains(const Rect& o) const { return x0 <= o.x0 && y0 <= o.y0 && o.x1 <= x1 && o.y1 <= y1; }
};

enum class LayerType { ROUTING, CUT, MASTERSLICE, OTHER };

struct LefLayer {
    std::string name;
    LayerType type = LayerType::OTHER;
    bool horizontal = true;              // preferred direction (routing layers)
    Dbu pitch_x = 0, pitch_y = 0;        // PITCH x [y]
    Dbu offset_x = 0, offset_y = 0;      // OFFSET x [y]
    Dbu width = 0;                       // minimum (default) width; cut size for cut layers
    Dbu spacing = 0;                     // minimum spacing (first entry of the spacing table)
    double min_area = 0;                 // um^2
    // SPACINGTABLE PARALLELRUNLENGTH: spacing[w][p] for wire width >= widths[w], run length >= prl[p]
    std::vector<Dbu> prl, table_widths;
    std::vector<std::vector<Dbu>> table;
    Dbu enclosure_below[2] = {0, 0}, enclosure_above[2] = {0, 0};   // cut layers
    double res_per_sq = 0;               // ohm/sq (routing) or ohm per cut (cut layers)
    double cap_per_area = 0;             // pF/um^2
    double edge_cap = 0;                 // pF/um
    int routing_index = -1;              // 0 for the lowest routing layer, -1 for others

    Dbu prefPitch() const { return horizontal ? pitch_y : pitch_x; }
    Dbu prefOffset() const { return horizontal ? offset_y : offset_x; }
};

struct LefShape {
    int layer = -1;   // index into Lef::layers
    Rect rect;
};

struct LefVia {
    std::string name;
    bool is_default = false;
    std::vector<LefShape> shapes;
};

struct LefSite {
    std::string name;
    Dbu width = 0, height = 0;
};

enum class PinUse { SIGNAL, CLOCK, POWER, GROUND, ANALOG, OTHER };

struct LefPin {
    std::string name;
    PinDir dir = PinDir::INPUT;
    PinUse use = PinUse::SIGNAL;
    std::vector<LefShape> shapes;   // all PORT shapes, relative to the macro's origin
    bool isSupply() const { return use == PinUse::POWER || use == PinUse::GROUND; }
};

enum class MacroClass { CORE, BLOCK, PAD, ENDCAP, COVER, OTHER };

struct LefMacro {
    std::string name;
    MacroClass cls = MacroClass::CORE;
    std::string class_sub;          // e.g. TIEHIGH, SPACER, ANTENNACELL
    std::string site;
    Dbu width = 0, height = 0;
    std::vector<LefPin> pins;
    std::vector<LefShape> obs;

    int pinIndex(const std::string& n) const;
};

struct Lef {
    int dbu = 1000;                  // database units per micron
    Dbu manufacturing_grid = 5;      // DBU
    std::vector<LefLayer> layers;
    std::vector<LefVia> vias;
    std::vector<LefSite> sites;
    std::vector<LefMacro> macros;
    std::vector<std::string> warnings;

    const LefLayer* findLayer(const std::string& n) const;
    int layerIndex(const std::string& n) const;
    const LefMacro* findMacro(const std::string& n) const;
    const LefSite* findSite(const std::string& n) const;
    // Routing layers from the bottom up
    std::vector<int> routingLayers() const;
    Dbu toDbu(double microns) const;
    double toMicrons(Dbu d) const { return static_cast<double>(d) / dbu; }

    std::unordered_map<std::string, int> layer_index, macro_index, site_index;
};

// Reads one LEF file and merges it into `lef` (call once per file: tech LEF first)
void readLef(const std::string& path, Lef& lef);
void parseLef(const std::string& text, const std::string& source, Lef& lef);

// A Library of the macros' signal pins, so cell netlists can be read without Liberty.
// Bus pins (A_ADDR[0], A_ADDR[1], ...) are grouped into LibBus entries.
Library libraryFromLef(const Lef& lef);
