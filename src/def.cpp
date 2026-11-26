// DEF 5.8 writer
#include "../include/def.hpp"
#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace {

// DEF names may not contain spaces or ';' and '/' is the hierarchy divider: escape them
std::string defName(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '/' || c == '#' || c == ';' || c == '*' || c == '"' || c == '(' || c == ')' || c == ' ' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

} // namespace

void writeDef(std::ostream& out, const Floorplan& fp, const CellNetlist& nl, const Lef& lef, const DrouteResult* routes) {
    out << "VERSION 5.8 ;\n"
        << "DIVIDERCHAR \"/\" ;\n"
        << "BUSBITCHARS \"[]\" ;\n"
        << "DESIGN " << defName(fp.design.empty() ? "top" : fp.design) << " ;\n"
        << "UNITS DISTANCE MICRONS " << fp.dbu << " ;\n\n"
        << "DIEAREA ( " << fp.die.x0 << " " << fp.die.y0 << " ) ( " << fp.die.x1 << " " << fp.die.y1 << " ) ;\n\n";

    for (const FpRow& r : fp.rows) {
        out << "ROW " << r.name << " " << fp.site->name << " " << r.x << " " << r.y << " " << orientName(r.orient) << " DO " << r.count
            << " BY 1 STEP " << fp.site->width << " 0 ;\n";
    }
    out << "\n";
    for (const FpTracks& t : fp.tracks) {
        out << "TRACKS " << (t.x ? "X " : "Y ") << t.start << " DO " << t.count << " STEP " << t.step << " LAYER " << lef.layers[t.layer].name << " ;\n";
    }
    out << "\n";

    // Routed wires wider than the layer's default width: one non-default rule for every routed net
    bool ndr = false;
    if (routes) {
        for (size_t l = 0; l < routes->wire_width.size(); ++l)
            if (routes->wire_layer[l] >= 0 && routes->wire_width[l] != lef.layers[routes->wire_layer[l]].width) ndr = true;
    }
    if (ndr) {
        // every routing layer must be listed (OpenROAD's reader rejects the rule otherwise):
        // the routed layers at their drawn width, the others at the default width
        out << "NONDEFAULTRULES 1 ;\n  - FASTPNR_PAD_WIDTH";
        for (int layer : lef.routingLayers()) {
            Dbu w = lef.layers[layer].width;
            for (size_t l = 0; l < routes->wire_width.size(); ++l)
                if (routes->wire_layer[l] == layer && routes->wire_width[l] > 0) w = routes->wire_width[l];
            out << "\n    + LAYER " << lef.layers[layer].name << " WIDTH " << w;
        }
        out << " ;\nEND NONDEFAULTRULES\n\n";
    }
    if (!fp.vias.empty()) {
        out << "VIAS " << fp.vias.size() << " ;\n";
        for (const FpVia& v : fp.vias) {
            out << "  - " << v.name;
            for (const LefShape& s : v.shapes) {
                out << "\n    + RECT " << lef.layers[s.layer].name << " ( " << s.rect.x0 << " " << s.rect.y0 << " ) ( " << s.rect.x1 << " " << s.rect.y1 << " )";
            }
            out << " ;\n";
        }
        out << "END VIAS\n\n";
    }

    out << "COMPONENTS " << fp.components.size() + fp.fillers.size() << " ;\n";
    for (const FpComponent& c : fp.components) {
        out << "  - " << defName(nl.instances[c.instance].name) << " " << c.macro->name;
        if (c.placed) out << " + " << (c.fixed ? "FIXED" : "PLACED") << " ( " << c.x << " " << c.y << " ) " << orientName(c.orient);
        out << " ;\n";
    }
    for (size_t k = 0; k < fp.fillers.size(); ++k) {
        const FpFiller& f = fp.fillers[k];
        out << "  - FILLER_" << k << " " << f.macro->name << " + SOURCE DIST + PLACED ( " << f.x << " " << f.y << " ) " << orientName(f.orient) << " ;\n";
    }
    out << "END COMPONENTS\n\n";

    // Power pins: one on the first top-layer strap of each supply net, so the grid has named ports
    std::vector<const FpSpecialWire*> power_pin(2, nullptr);
    int top_layer = -1;
    for (const FpSpecialWire& w : fp.special) if (w.via < 0) top_layer = std::max(top_layer, w.layer);
    for (const FpSpecialWire& w : fp.special) {
        if (w.via < 0 && w.layer == top_layer && !power_pin[w.net]) power_pin[w.net] = &w;
    }
    size_t npower = (power_pin[0] ? 1 : 0) + (power_pin[1] ? 1 : 0);
    out << "PINS " << fp.pins.size() + npower << " ;\n";
    for (int net = 0; net < 2; ++net) {
        const FpSpecialWire* w = power_pin[net];
        if (!w) continue;
        out << "  - " << fp.supplies[net] << " + NET " << fp.supplies[net] << " + SPECIAL + DIRECTION INOUT + USE " << (net == 0 ? "POWER" : "GROUND");
        if (fp.tt_tile) {
            for (const FpSpecialWire& stripe : fp.special) {
                if (stripe.net != net || stripe.layer != top_layer || stripe.via >= 0) continue;
                Dbu half = stripe.width / 2;
                out << "\n    + PORT + LAYER " << lef.layers[top_layer].name << " ( " << -half << " 0 ) ( " << half << " " << stripe.y1 - stripe.y0
                    << " ) + FIXED ( " << stripe.x0 << " " << stripe.y0 << " ) N";
            }
        } else {
            Dbu len = std::min<Dbu>(std::max(std::llabs(w->x1 - w->x0), std::llabs(w->y1 - w->y0)), 10 * w->width);
            bool hor = w->y0 == w->y1;
            Dbu cx = hor ? std::min(w->x0, w->x1) + len / 2 : w->x0, cy = hor ? w->y0 : std::min(w->y0, w->y1) + len / 2;
            Dbu hx = hor ? len / 2 : w->width / 2, hy = hor ? w->width / 2 : len / 2;
            out << "\n    + PORT + LAYER " << lef.layers[w->layer].name << " ( " << -hx << " " << -hy << " ) ( " << hx << " " << hy << " )"
                << "\n    + FIXED ( " << cx << " " << cy << " ) N";
        }
        out << " ;\n";
    }
    for (const FpPin& p : fp.pins) {
        const auto& pb = nl.port_bits[p.port_bit];
        out << "  - " << defName(pb.name) << " + NET " << defName(nl.net_names[pb.net]) << " + DIRECTION " << (pb.is_input ? "INPUT" : "OUTPUT")
            << " + USE SIGNAL\n"
            << "    + PORT + LAYER " << lef.layers[p.layer].name << " ( " << p.shape.x0 << " " << p.shape.y0 << " ) ( " << p.shape.x1 << " " << p.shape.y1
            << " )\n"
            << (fp.tt_tile ? "    + FIXED ( " : "    + PLACED ( ") << p.x << " " << p.y << " ) N ;\n";
    }
    out << "END PINS\n\n";

    // Power: global connections to every supply pin name, plus the routed grid
    std::vector<std::vector<std::string>> supply_pins(2);
    supply_pins[0].push_back("VDD");
    supply_pins[1].push_back("VSS");
    for (const LefMacro& m : lef.macros) {
        for (const LefPin& p : m.pins) {
            if (!p.isSupply()) continue;
            auto& v = supply_pins[p.use == PinUse::POWER ? 0 : 1];
            bool used = false;
            for (const auto& c : fp.components) used |= c.macro == &m;
            if (used && std::find(v.begin(), v.end(), p.name) == v.end()) v.push_back(p.name);
        }
    }
    out << "SPECIALNETS 2 ;\n";
    for (int net = 0; net < 2; ++net) {
        out << "  - " << fp.supplies[net];
        for (const auto& name : supply_pins[net]) out << " ( * " << defName(name) << " )";
        out << " + USE " << (net == 0 ? "POWER" : "GROUND");
        bool first = true;
        for (const FpSpecialWire& w : fp.special) {
            if (w.net != net) continue;
            out << "\n    " << (first ? "+ ROUTED " : "NEW ") << lef.layers[w.layer].name << " ";
            first = false;
            if (w.via >= 0) {
                out << "0 + SHAPE STRIPE ( " << w.x0 << " " << w.y0 << " ) " << fp.vias[w.via].name;
            } else {
                out << w.width << " + SHAPE " << (w.followpin ? "FOLLOWPIN" : "STRIPE") << " ( " << w.x0 << " " << w.y0 << " ) ( " << w.x1 << " " << w.y1
                    << " )";
            }
        }
        out << " ;\n";
    }
    out << "END SPECIALNETS\n\n";

    // Signal nets: every instance pin and port on each net (constant nets are not written)
    std::vector<std::vector<std::pair<int, int>>> conns(nl.netCount());   // (instance or -1-port_bit, pin)
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] >= 0) conns[inst.pin_net[k]].push_back({static_cast<int>(i), static_cast<int>(k)});
        }
    }
    for (size_t b = 0; b < nl.port_bits.size(); ++b) {
        if (nl.port_bits[b].net >= 0) conns[nl.port_bits[b].net].push_back({-1 - static_cast<int>(b), 0});
    }
    size_t count = 0;
    for (size_t n = 2; n < conns.size(); ++n) count += !conns[n].empty();
    std::map<int, size_t> route_of;
    if (routes) {
        for (size_t k = 0; k < routes->nets.size(); ++k) route_of[routes->nets[k].netlist_net] = k;
    }
    out << "NETS " << count << " ;\n";
    for (size_t n = 2; n < conns.size(); ++n) {
        if (conns[n].empty()) continue;
        out << "  - " << defName(nl.net_names[n]);
        int col = 0;
        for (auto [who, pin] : conns[n]) {
            if (col++ % 4 == 0) out << "\n   ";
            if (who < 0) out << " ( PIN " << defName(nl.port_bits[-1 - who].name) << " )";
            else out << " ( " << defName(nl.instances[who].name) << " " << nl.instances[who].cell->pins[pin].name << " )";
        }
        out << " + USE SIGNAL";
        auto r = route_of.find(static_cast<int>(n));
        if (r != route_of.end() && ndr) out << " + NONDEFAULTRULE FASTPNR_PAD_WIDTH";
        if (r != route_of.end()) {
            const DrouteNet& dn = routes->nets[r->second];
            bool first = true;
            auto lead = [&](int layer) {
                out << "\n    " << (first ? "+ ROUTED " : "NEW ") << lef.layers[layer].name << " ";
                first = false;
            };
            for (const DrouteSegment& s : dn.wires) {
                lead(s.layer);
                out << "( " << s.x0 << " " << s.y0 << " ) ( " << (s.x1 == s.x0 ? std::string("*") : std::to_string(s.x1)) << " "
                    << (s.y1 == s.y0 ? std::string("*") : std::to_string(s.y1)) << " )";
            }
            for (const DrouteVia& v : dn.vias) {
                lead(v.bottom);
                out << "( " << v.x << " " << v.y << " ) " << v.name;
            }
            for (const auto& [layer, r] : dn.patches) {
                lead(layer);
                out << "( " << r.x0 << " " << r.y0 << " ) RECT ( 0 0 " << r.x1 - r.x0 << " " << r.y1 - r.y0 << " )";
            }
        }
        out << " ;\n";
    }
    out << "END NETS\n\nEND DESIGN\n";
}
