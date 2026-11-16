// LEF reader: technology layers, vias, sites and cell macros
#include "../include/lef.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {

struct Token {
    std::string text;
    int line;
};

std::vector<Token> tokenize(const std::string& s) {
    std::vector<Token> out;
    int line = 1;
    size_t i = 0;
    while (i < s.size()) {
        char c = s[i];
        if (c == '\n') { line++; i++; continue; }
        if (std::isspace(static_cast<unsigned char>(c))) { i++; continue; }
        if (c == '#') {
            while (i < s.size() && s[i] != '\n') i++;
            continue;
        }
        if (c == '"') {
            size_t j = i + 1;
            while (j < s.size() && s[j] != '"') {
                if (s[j] == '\n') line++;
                j++;
            }
            out.push_back({s.substr(i, j + 1 - i), line});
            i = j + 1;
            continue;
        }
        if (c == ';') { out.push_back({";", line}); i++; continue; }
        size_t j = i;
        while (j < s.size() && !std::isspace(static_cast<unsigned char>(s[j])) && s[j] != ';') j++;
        out.push_back({s.substr(i, j - i), line});
        i = j;
    }
    return out;
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

class Parser {
public:
    Parser(const std::string& text, const std::string& source, Lef& lef) : toks(tokenize(text)), src(source), lef(lef) {}

    void run() {
        while (pos < toks.size()) {
            std::string k = upper(next());
            if (k == "UNITS") units();
            else if (k == "MANUFACTURINGGRID") {
                lef.manufacturing_grid = lef.toDbu(number());
                endStatement();
            } else if (k == "LAYER") layer();
            else if (k == "VIA") via();
            else if (k == "SITE") site();
            else if (k == "MACRO") macro();
            else if (k == "VIARULE" || k == "NONDEFAULTRULE") skipBlock(next());
            else if (k == "PROPERTYDEFINITIONS" || k == "SPACING" || k == "BEGINEXT") skipBlock(k == "BEGINEXT" ? "" : k);
            else if (k == "END") {
                if (pos < toks.size() && upper(toks[pos].text) == "LIBRARY") break;
            } else endStatement();
        }
    }

private:
    std::vector<Token> toks;
    std::string src;
    Lef& lef;
    size_t pos = 0;

    [[noreturn]] void error(const std::string& m) const {
        int line = pos < toks.size() ? toks[pos].line : (toks.empty() ? 0 : toks.back().line);
        throw std::runtime_error(src + ":" + std::to_string(line) + ": " + m);
    }
    const std::string& next() {
        if (pos >= toks.size()) error("unexpected end of file");
        return toks[pos++].text;
    }
    const std::string& peek() const {
        static const std::string empty;
        return pos < toks.size() ? toks[pos].text : empty;
    }
    double number() {
        const std::string& t = next();
        try {
            size_t used = 0;
            double v = std::stod(t, &used);
            if (used != t.size()) error("expected a number, got '" + t + "'");
            return v;
        } catch (const std::logic_error&) {
            error("expected a number, got '" + t + "'");
        }
    }
    bool isNumber(const std::string& t) const {
        if (t.empty()) return false;
        char c = t[0];
        return std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.';
    }
    // The rest of a statement, up to and including ';'
    std::vector<std::string> statement() {
        std::vector<std::string> out;
        while (true) {
            const std::string& t = next();
            if (t == ";") return out;
            out.push_back(t);
        }
    }
    void endStatement() { statement(); }
    // Skips to "END name" (or a bare END when name is empty)
    void skipBlock(const std::string& name) {
        while (pos < toks.size()) {
            if (upper(toks[pos].text) == "END") {
                pos++;
                if (name.empty()) return;
                if (pos < toks.size() && toks[pos].text == name) { pos++; return; }
                if (pos < toks.size() && upper(toks[pos].text) == upper(name)) { pos++; return; }
                continue;
            }
            pos++;
        }
        error("missing END " + name);
    }
    void expectEnd(const std::string& name) {
        const std::string& t = next();
        if (t != name) error("expected END " + name + ", got END " + t);
    }
    Dbu dbu(double v) const { return lef.toDbu(v); }
    double num(const std::vector<std::string>& st, size_t k) const {
        if (k >= st.size()) throw std::runtime_error(src + ": statement too short");
        return std::stod(st[k]);
    }

    void units() {
        while (true) {
            std::string k = upper(next());
            if (k == "END") { next(); return; }
            std::vector<std::string> st = statement();
            if (k == "DATABASE" && st.size() >= 2 && upper(st[0]) == "MICRONS") {
                int d = static_cast<int>(std::lround(std::stod(st[1])));
                if (!lef.layers.empty() || !lef.macros.empty()) {
                    if (d != lef.dbu) lef.warnings.push_back(src + ": DATABASE MICRONS " + std::to_string(d) + " differs from the tech LEF");
                } else lef.dbu = d;
            }
        }
    }

    void layer() {
        std::string name = next();
        LefLayer L;
        L.name = name;
        while (true) {
            std::string k = upper(next());
            if (k == "END") { expectEnd(name); break; }
            if (k == "PROPERTY" || k == "ANTENNACUMDIFFAREARATIO" || k == "ANTENNADIFFAREARATIO") {
                endStatement();
                continue;
            }
            std::vector<std::string> st = statement();
            if (k == "TYPE" && !st.empty()) {
                std::string t = upper(st[0]);
                L.type = t == "ROUTING" ? LayerType::ROUTING : t == "CUT" ? LayerType::CUT : t == "MASTERSLICE" ? LayerType::MASTERSLICE : LayerType::OTHER;
            } else if (k == "DIRECTION" && !st.empty()) {
                L.horizontal = upper(st[0]) == "HORIZONTAL";
            } else if (k == "PITCH" && !st.empty()) {
                L.pitch_x = dbu(num(st, 0));
                L.pitch_y = st.size() > 1 ? dbu(num(st, 1)) : L.pitch_x;
            } else if (k == "OFFSET" && !st.empty()) {
                L.offset_x = dbu(num(st, 0));
                L.offset_y = st.size() > 1 ? dbu(num(st, 1)) : L.offset_x;
            } else if (k == "WIDTH" && !st.empty()) {
                L.width = dbu(num(st, 0));
            } else if (k == "SPACING" && !st.empty()) {
                // Plain "SPACING s ;" is the minimum; qualified forms (ADJACENTCUTS, ENDOFLINE, ...) are
                // stricter special cases kept for the detailed router later
                Dbu s = dbu(num(st, 0));
                if (st.size() == 1 && (L.spacing == 0 || s < L.spacing)) L.spacing = s;
                else if (L.spacing == 0) L.spacing = s;
            } else if (k == "SPACINGTABLE") {
                // SPACINGTABLE PARALLELRUNLENGTH p0 p1 ... WIDTH w0 s00 s01 ... WIDTH w1 ...
                size_t i = 0;
                if (i < st.size() && upper(st[i]) == "PARALLELRUNLENGTH") {
                    i++;
                    while (i < st.size() && upper(st[i]) != "WIDTH") L.prl.push_back(dbu(num(st, i++)));
                    while (i < st.size() && upper(st[i]) == "WIDTH") {
                        i++;
                        L.table_widths.push_back(dbu(num(st, i++)));
                        std::vector<Dbu> row;
                        for (size_t p = 0; p < L.prl.size(); ++p) row.push_back(dbu(num(st, i++)));
                        L.table.push_back(row);
                    }
                    if (!L.table.empty() && !L.table[0].empty()) L.spacing = L.table[0][0];
                }
            } else if (k == "AREA" && !st.empty()) {
                L.min_area = num(st, 0);
            } else if (k == "ENCLOSURE" && st.size() >= 2) {
                size_t i = 0;
                Dbu* e = nullptr;
                std::string side = upper(st[0]);
                if (side == "BELOW") { e = L.enclosure_below; i = 1; }
                else if (side == "ABOVE") { e = L.enclosure_above; i = 1; }
                if (i + 1 < st.size()) {
                    Dbu a = dbu(num(st, i)), b = dbu(num(st, i + 1));
                    if (e) { e[0] = a; e[1] = b; }
                    else {
                        L.enclosure_below[0] = L.enclosure_above[0] = a;
                        L.enclosure_below[1] = L.enclosure_above[1] = b;
                    }
                }
            } else if (k == "RESISTANCE" && !st.empty()) {
                L.res_per_sq = upper(st[0]) == "RPERSQ" ? num(st, 1) : num(st, 0);
            } else if (k == "CAPACITANCE" && st.size() >= 2) {
                L.cap_per_area = num(st, 1);
            } else if (k == "EDGECAPACITANCE" && !st.empty()) {
                L.edge_cap = num(st, 0);
            }
        }
        if (lef.layer_index.count(name)) {
            lef.warnings.push_back(src + ": layer " + name + " defined twice; the first definition is kept");
            return;
        }
        if (L.type == LayerType::ROUTING) {
            int n = 0;
            for (const auto& other : lef.layers) n += other.type == LayerType::ROUTING;
            L.routing_index = n;
        }
        lef.layer_index[name] = static_cast<int>(lef.layers.size());
        lef.layers.push_back(L);
    }

    // RECT [MASK n] x0 y0 x1 y1 ;   POLYGON x y x y ... ;  (polygons are kept as their bounding box)
    bool shape(const std::string& k, int layer, std::vector<LefShape>& into, Dbu ox, Dbu oy) {
        if (k != "RECT" && k != "POLYGON") return false;
        std::vector<std::string> st = statement();
        size_t i = 0;
        if (i < st.size() && upper(st[i]) == "MASK") i += 2;
        if (i < st.size() && upper(st[i]) == "ITERATE") error("RECT ITERATE is not supported");
        std::vector<double> v;
        for (; i < st.size(); ++i) {
            if (st[i] == "(" || st[i] == ")") continue;
            v.push_back(std::stod(st[i]));
        }
        if (v.size() < 4 || v.size() % 2) error(k + " needs coordinate pairs");
        Rect r{dbu(v[0]), dbu(v[1]), dbu(v[0]), dbu(v[1])};
        for (size_t j = 0; j + 1 < v.size(); j += 2) {
            Dbu x = dbu(v[j]), y = dbu(v[j + 1]);
            r.x0 = std::min(r.x0, x); r.x1 = std::max(r.x1, x);
            r.y0 = std::min(r.y0, y); r.y1 = std::max(r.y1, y);
        }
        r.x0 += ox; r.x1 += ox; r.y0 += oy; r.y1 += oy;
        if (layer >= 0) into.push_back({layer, r});
        return true;
    }

    int layerRef(const std::string& name) {
        int idx = lef.layerIndex(name);
        if (idx < 0) {
            std::string w = src + ": unknown layer " + name + " (its shapes are ignored)";
            if (std::find(lef.warnings.begin(), lef.warnings.end(), w) == lef.warnings.end()) lef.warnings.push_back(w);
        }
        return idx;
    }

    void via() {
        LefVia v;
        v.name = next();
        if (upper(peek()) == "DEFAULT") { v.is_default = true; next(); }
        int layer = -1;
        while (true) {
            std::string k = upper(next());
            if (k == "END") { expectEnd(v.name); break; }
            if (k == "LAYER") {
                std::vector<std::string> st = statement();
                layer = st.empty() ? -1 : layerRef(st[0]);
            } else if (!shape(k, layer, v.shapes, 0, 0)) endStatement();
        }
        lef.vias.push_back(v);
    }

    void site() {
        LefSite s;
        s.name = next();
        while (true) {
            std::string k = upper(next());
            if (k == "END") { expectEnd(s.name); break; }
            std::vector<std::string> st = statement();
            if (k == "SIZE" && st.size() >= 3) {
                s.width = dbu(num(st, 0));
                s.height = dbu(num(st, 2));
            }
        }
        if (!lef.site_index.count(s.name)) {
            lef.site_index[s.name] = static_cast<int>(lef.sites.size());
            lef.sites.push_back(s);
        }
    }

    // Shapes of a PORT or OBS group up to its END
    void shapes(std::vector<LefShape>& into, Dbu ox, Dbu oy) {
        int layer = -1;
        while (true) {
            std::string k = upper(next());
            if (k == "END") return;
            if (k == "LAYER") {
                std::vector<std::string> st = statement();
                layer = st.empty() ? -1 : layerRef(st[0]);
            } else if (!shape(k, layer, into, ox, oy)) endStatement();
        }
    }

    void pin(LefMacro& m, Dbu ox, Dbu oy) {
        LefPin p;
        p.name = next();
        while (true) {
            std::string k = upper(next());
            if (k == "END") { expectEnd(p.name); break; }
            if (k == "PORT") { shapes(p.shapes, ox, oy); continue; }
            std::vector<std::string> st = statement();
            if (k == "DIRECTION" && !st.empty()) {
                std::string d = upper(st[0]);
                p.dir = d == "OUTPUT" ? PinDir::OUTPUT : d == "INOUT" ? PinDir::INOUT : d == "FEEDTHRU" ? PinDir::INOUT : PinDir::INPUT;
            } else if (k == "USE" && !st.empty()) {
                std::string u = upper(st[0]);
                p.use = u == "POWER" ? PinUse::POWER : u == "GROUND" ? PinUse::GROUND : u == "CLOCK" ? PinUse::CLOCK
                      : u == "ANALOG" ? PinUse::ANALOG : u == "SIGNAL" ? PinUse::SIGNAL : PinUse::OTHER;
            }
        }
        m.pins.push_back(p);
    }

    void macro() {
        LefMacro m;
        m.name = next();
        Dbu ox = 0, oy = 0;
        while (true) {
            std::string k = upper(next());
            if (k == "END") { expectEnd(m.name); break; }
            if (k == "PIN") { pin(m, ox, oy); continue; }
            if (k == "OBS") { shapes(m.obs, ox, oy); continue; }
            std::vector<std::string> st = statement();
            if (k == "CLASS" && !st.empty()) {
                std::string c = upper(st[0]);
                m.cls = c == "CORE" ? MacroClass::CORE : c == "BLOCK" ? MacroClass::BLOCK : c == "PAD" ? MacroClass::PAD
                      : c == "ENDCAP" ? MacroClass::ENDCAP : c == "COVER" ? MacroClass::COVER : MacroClass::OTHER;
                if (st.size() > 1) m.class_sub = upper(st[1]);
            } else if (k == "ORIGIN" && st.size() >= 2) {
                // Shapes are given relative to ORIGIN; store them relative to the lower-left corner
                ox = dbu(num(st, 0));
                oy = dbu(num(st, 1));
            } else if (k == "SIZE" && st.size() >= 3) {
                m.width = dbu(num(st, 0));
                m.height = dbu(num(st, 2));
            } else if (k == "SITE" && !st.empty()) {
                m.site = st[0];
            }
        }
        auto it = lef.macro_index.find(m.name);
        if (it != lef.macro_index.end()) {
            lef.macros[it->second] = m;   // a later file overrides
            return;
        }
        lef.macro_index[m.name] = static_cast<int>(lef.macros.size());
        lef.macros.push_back(m);
    }
};

} // namespace

int LefMacro::pinIndex(const std::string& n) const {
    for (size_t i = 0; i < pins.size(); ++i) {
        if (pins[i].name == n) return static_cast<int>(i);
    }
    return -1;
}

const LefLayer* Lef::findLayer(const std::string& n) const {
    int i = layerIndex(n);
    return i < 0 ? nullptr : &layers[i];
}

int Lef::layerIndex(const std::string& n) const {
    auto it = layer_index.find(n);
    return it == layer_index.end() ? -1 : it->second;
}

const LefMacro* Lef::findMacro(const std::string& n) const {
    auto it = macro_index.find(n);
    return it == macro_index.end() ? nullptr : &macros[it->second];
}

const LefSite* Lef::findSite(const std::string& n) const {
    auto it = site_index.find(n);
    return it == site_index.end() ? nullptr : &sites[it->second];
}

std::vector<int> Lef::routingLayers() const {
    std::vector<int> out;
    for (size_t i = 0; i < layers.size(); ++i) {
        if (layers[i].type == LayerType::ROUTING) out.push_back(static_cast<int>(i));
    }
    return out;
}

Dbu Lef::toDbu(double microns) const { return static_cast<Dbu>(std::llround(microns * dbu)); }

void parseLef(const std::string& text, const std::string& source, Lef& lef) {
    Parser(text, source, lef).run();
    // Cut layers without a WIDTH (IHP's Via1..Via3): the cut size of their first via definition
    for (size_t i = 0; i < lef.layers.size(); ++i) {
        LefLayer& L = lef.layers[i];
        if (L.type != LayerType::CUT || L.width > 0) continue;
        for (const LefVia& v : lef.vias) {
            for (const LefShape& s : v.shapes) {
                if (s.layer == static_cast<int>(i) && L.width == 0) L.width = std::min(s.rect.width(), s.rect.height());
            }
        }
    }
}

void readLef(const std::string& path, Lef& lef) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    parseLef(ss.str(), path, lef);
}

Library libraryFromLef(const Lef& lef) {
    Library lib;
    lib.name = "LEF";
    for (const LefMacro& m : lef.macros) {
        LibCell cell;
        cell.name = m.name;
        cell.area = lef.toMicrons(m.width) * lef.toMicrons(m.height);
        cell.is_macro = m.cls == MacroClass::BLOCK;
        // name[i] pins become buses, most significant bit first (as the Liberty reader stores them)
        std::map<std::string, std::vector<std::pair<int, int>>> buses;   // base -> (bit, pin index)
        for (const LefPin& p : m.pins) {
            if (p.isSupply()) continue;
            LibPin lp;
            lp.name = p.name;
            lp.dir = p.dir;
            lp.is_clock = p.use == PinUse::CLOCK;
            int index = static_cast<int>(cell.pins.size());
            cell.pins.push_back(lp);
            size_t open = p.name.find('[');
            if (open != std::string::npos && p.name.back() == ']') {
                try {
                    int bit = std::stoi(p.name.substr(open + 1, p.name.size() - open - 2));
                    buses[p.name.substr(0, open)].push_back({bit, index});
                } catch (const std::logic_error&) {
                }
            }
        }
        for (auto& [base, bits] : buses) {
            std::sort(bits.begin(), bits.end(), [](auto a, auto b) { return a.first > b.first; });
            LibBus bus;
            bus.name = base;
            bus.from = bits.front().first;
            bus.to = bits.back().first;
            for (auto& b : bits) bus.pins.push_back(b.second);
            cell.buses.push_back(bus);
        }
        lib.cell_index[cell.name] = static_cast<int>(lib.cells.size());
        lib.cells.push_back(std::move(cell));
    }
    return lib;
}
