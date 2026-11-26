// Floorplan: die/core sizing, rows, tracks, I/O pins, macro placement and the power grid
#include "../include/floorplan.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

const char* orientName(Orient o) {
    switch (o) {
    case Orient::N: return "N";
    case Orient::S: return "S";
    case Orient::FN: return "FN";
    case Orient::FS: return "FS";
    }
    return "N";
}

namespace {

Dbu roundUp(Dbu v, Dbu grid) { return grid <= 0 ? v : ((v + grid - 1) / grid) * grid; }
Dbu roundDown(Dbu v, Dbu grid) { return grid <= 0 ? v : (v / grid) * grid; }
// Nearest point of the grid origin + k * step
Dbu snapNearest(Dbu v, Dbu origin, Dbu step) {
    if (step <= 0) return v;
    double k = std::round(static_cast<double>(v - origin) / step);
    return origin + static_cast<Dbu>(k) * step;
}

struct Interval {
    Dbu lo, hi;
};

// [lo, hi] minus the blocked intervals
std::vector<Interval> subtract(Dbu lo, Dbu hi, std::vector<Interval> blocked) {
    std::sort(blocked.begin(), blocked.end(), [](const Interval& a, const Interval& b) { return a.lo < b.lo; });
    std::vector<Interval> out;
    Dbu cur = lo;
    for (const Interval& b : blocked) {
        if (b.hi <= cur) continue;
        if (b.lo >= hi) break;
        if (b.lo > cur) out.push_back({cur, b.lo});
        cur = std::max(cur, b.hi);
    }
    if (cur < hi) out.push_back({cur, hi});
    return out;
}

std::vector<Interval> merge(std::vector<Interval> v) {
    std::sort(v.begin(), v.end(), [](const Interval& a, const Interval& b) { return a.lo < b.lo; });
    std::vector<Interval> out;
    for (const Interval& i : v) {
        if (!out.empty() && i.lo <= out.back().hi) out.back().hi = std::max(out.back().hi, i.hi);
        else out.push_back(i);
    }
    return out;
}

// A via filling a w x h overlap: metal pads on every metal layer from `lower` to `upper` and an
// array of cuts on each cut layer, kept inside the enclosure rules. Returns false if no cut fits.
bool makeViaStack(const Lef& lef, int lower, int upper, Dbu w, Dbu h, const std::string& name, FpVia& via) {
    via.name = name;
    via.shapes.clear();
    Rect pad{-w / 2, -h / 2, w - w / 2, h - h / 2};
    for (int i = lower; i <= upper; ++i) {
        const LefLayer& L = lef.layers[i];
        if (L.type == LayerType::ROUTING) {
            via.shapes.push_back({i, pad});
            continue;
        }
        if (L.type != LayerType::CUT) continue;
        Dbu c = L.width, s = L.spacing;
        if (c <= 0) return false;
        Dbu m = std::max({L.enclosure_below[0], L.enclosure_below[1], L.enclosure_above[0], L.enclosure_above[1]});
        Dbu nx = (w - 2 * m + s) / (c + s), ny = (h - 2 * m + s) / (c + s);
        if (nx < 1 || ny < 1) return false;
        Dbu span_x = nx * c + (nx - 1) * s, span_y = ny * c + (ny - 1) * s;
        Dbu x0 = roundDown(-span_x / 2, lef.manufacturing_grid), y0 = roundDown(-span_y / 2, lef.manufacturing_grid);
        for (Dbu iy = 0; iy < ny; ++iy) {
            for (Dbu ix = 0; ix < nx; ++ix) {
                Dbu x = x0 + ix * (c + s), y = y0 + iy * (c + s);
                via.shapes.push_back({i, {x, y, x + c, y + c}});
            }
        }
    }
    return true;
}

struct TrackSpec {
    std::string layer;
    bool x;
    double offset, pitch;
};

std::vector<TrackSpec> readTracksFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open tracks file " + path);
    std::vector<TrackSpec> out;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        TrackSpec t;
        std::string dir;
        if (!(ss >> t.layer >> dir >> t.offset >> t.pitch)) continue;
        t.x = dir == "X" || dir == "x";
        out.push_back(t);
    }
    return out;
}

// The digital TT template has one unrotated rectangular port per signal, with an
// empty special net for each supply. Reject other pin formats rather than moving them.
void readTileInterface(const std::string& path, Floorplan& fp, const CellNetlist& nl, const Lef& lef) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open tile template " + path);
    auto expect = [&](const std::string& want) {
        std::string got;
        if (!(in >> got) || got != want)
            throw std::runtime_error(path + ": expected " + want + ", got " + got);
    };
    auto point = [&]() {
        Dbu x, y;
        expect("(");
        if (!(in >> x >> y)) throw std::runtime_error(path + ": malformed coordinates");
        expect(")");
        return std::make_pair(x, y);
    };
    std::map<std::string, int> bits;
    for (size_t i = 0; i < nl.port_bits.size(); ++i) bits.emplace(nl.port_bits[i].name, i);
    bool units = false, supplies[2] = {};
    std::string key;
    while (in >> key) {
        if (key == "UNITS") {
            expect("DISTANCE"); expect("MICRONS"); expect(std::to_string(lef.dbu)); expect(";");
            units = true;
        } else if (key == "DIEAREA") {
            auto [x0, y0] = point();
            auto [x1, y1] = point();
            expect(";");
            if (x0 || y0 || x1 <= 0 || y1 <= 0) throw std::runtime_error(path + ": unsupported die outline");
            fp.die = {x0, y0, x1, y1};
        } else if (key == "PINS") {
            expect(std::to_string(bits.size())); expect(";");
            while (!bits.empty()) {
                std::string name, layer, placement;
                expect("-"); in >> name;
                auto bit = bits.find(name);
                if (bit == bits.end()) throw std::runtime_error(path + ": unexpected pin " + name);
                FpPin p;
                p.port_bit = bit->second;
                expect("+"); expect("NET"); expect(name); expect("+"); expect("DIRECTION");
                expect(nl.port_bits[p.port_bit].is_input ? "INPUT" : "OUTPUT");
                expect("+"); expect("USE"); expect("SIGNAL"); expect("+"); expect("PORT");
                expect("+"); expect("LAYER"); in >> layer;
                p.layer = lef.layerIndex(layer);
                if (p.layer < 0) throw std::runtime_error(path + ": unknown pin layer " + layer);
                auto [x0, y0] = point();
                auto [x1, y1] = point();
                if (x1 <= x0 || y1 <= y0) throw std::runtime_error(path + ": empty pin " + name);
                p.shape = {x0, y0, x1, y1};
                expect("+"); in >> placement;
                if (placement != "PLACED" && placement != "FIXED") throw std::runtime_error(path + ": unplaced pin " + name);
                std::tie(p.x, p.y) = point();
                expect("N"); expect(";");
                if (p.y + y1 == fp.die.y1) p.side = 'T';
                else if (p.y + y0 == fp.die.y0) p.side = 'B';
                else if (p.x + x1 == fp.die.x1) p.side = 'R';
                else if (p.x + x0 == fp.die.x0) p.side = 'L';
                else throw std::runtime_error(path + ": expected a tile-edge pin for " + name);
                fp.pins.push_back(p);
                bits.erase(bit);
            }
            expect("END"); expect("PINS");
        } else if (key == "SPECIALNETS") {
            expect("2"); expect(";");
            for (int k = 0; k < 2; ++k) {
                std::string name, use;
                expect("-"); in >> name; expect("+"); expect("USE"); in >> use; expect(";");
                int net = use == "POWER" ? 0 : 1;
                if ((use != "POWER" && use != "GROUND") || supplies[net]) throw std::runtime_error(path + ": invalid supplies");
                fp.supplies[net] = name;
                supplies[net] = true;
            }
            expect("END"); expect("SPECIALNETS");
        } else {
            std::string rest;
            std::getline(in, rest);
        }
    }
    if (!units || !bits.empty() || !supplies[0] || !supplies[1] || fp.supplies[0] == fp.supplies[1] || fp.die.width() <= 0)
        throw std::runtime_error(path + ": incomplete tile interface");
}

} // namespace

int insertTieCells(CellNetlist& nl, const Library& lib, const std::string& tie_lo, const std::string& tie_hi) {
    const LibCell* cells[2] = {lib.find(tie_lo), lib.find(tie_hi)};
    int added = 0;
    auto tie = [&](int value) {
        const LibCell* cell = cells[value];
        if (!cell || cell->outputPin() < 0) throw std::runtime_error("tie cell " + (value ? tie_hi : tie_lo) + " not found in the LEF");
        std::string base = value ? "tie_hi_" : "tie_lo_";
        int net = nl.addNet(base + "net" + std::to_string(added));
        CellNetlist::Instance inst;
        inst.name = base + std::to_string(added);
        inst.cell = cell;
        inst.pin_net.assign(cell->pins.size(), -1);
        inst.pin_net[cell->outputPin()] = net;
        nl.instances.push_back(inst);
        added++;
        return net;
    };
    size_t original = nl.instances.size();
    for (size_t i = 0; i < original; ++i) {
        for (size_t k = 0; k < nl.instances[i].pin_net.size(); ++k) {
            int n = nl.instances[i].pin_net[k];
            if (n != CellNetlist::kConst0 && n != CellNetlist::kConst1) continue;
            if (nl.instances[i].cell->pins[k].dir == PinDir::OUTPUT) continue;
            int net = tie(n == CellNetlist::kConst1);
            nl.instances[i].pin_net[k] = net;
        }
    }
    for (auto& pb : nl.port_bits) {
        if (pb.is_input) continue;
        if (pb.net == CellNetlist::kConst0 || pb.net == CellNetlist::kConst1) pb.net = tie(pb.net == CellNetlist::kConst1);
    }
    return added;
}

Floorplan buildFloorplan(const CellNetlist& nl, const Lef& lef, const FloorplanOptions& opt) {
    Floorplan fp;
    fp.design = nl.name;
    fp.dbu = lef.dbu;
    fp.site = lef.findSite(opt.site);
    if (!fp.site) throw std::runtime_error("site " + opt.site + " is not in the LEF");
    const Dbu sw = fp.site->width, sh = fp.site->height, grid = lef.manufacturing_grid;
    auto um = [&](double v) { return lef.toDbu(v); };

    fp.tt_tile = !opt.tt_template.empty();
    if (fp.tt_tile) readTileInterface(opt.tt_template, fp, nl, lef);

    // Components and areas
    std::vector<int> macro_instances;
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        const LefMacro* m = lef.findMacro(inst.cell->name);
        if (!m) throw std::runtime_error("cell " + inst.cell->name + " (instance " + inst.name + ") has no LEF macro");
        FpComponent c;
        c.instance = static_cast<int>(i);
        c.macro = m;
        double a = lef.toMicrons(m->width) * lef.toMicrons(m->height);
        if (m->cls == MacroClass::BLOCK) {
            fp.macro_area += a;
            macro_instances.push_back(static_cast<int>(i));
        } else {
            fp.cell_area += a;
        }
        fp.components.push_back(c);
    }

    // y grid for the core origin: the rail layer's track pitch, so rows sit on its tracks
    Dbu ygrid = sh;
    if (const LefLayer* rail = lef.findLayer(opt.rail_layer)) {
        if (rail->prefPitch() > 0 && sh % rail->prefPitch() == 0) ygrid = rail->prefPitch();
    }
    const Dbu halo = um(opt.macro_halo);
    const Dbu margin = um(opt.core_margin);

    // Minimum height of a band of rows next to a macro: room for a VDD + VSS pair of top straps
    const Dbu strip = opt.pdn ? roundUp(2 * um(opt.top_width) + um(2.0) + um(opt.rail_width), sh) : sh;
    // A column of rows is kept beside the macro shelves. The rows then surround the macros (OpenROAD
    // takes the rows' outline as the core), and power straps run past the macros, keeping the grid
    // above and below them one connected net.
    const Dbu column = macro_instances.empty() ? 0 : roundUp(um(20.0), sw);

    // Macro shelves: packed left to right from the top of the core, a new shelf when one is full
    auto packMacros = [&](Dbu core_w, std::vector<std::pair<Dbu, Dbu>>* at) {
        Dbu x = 0, shelf_y = 0, shelf_h = 0;
        for (int idx : macro_instances) {
            const LefMacro* m = fp.components[idx].macro;
            Dbu fw = m->width + 2 * halo, fh = m->height + 2 * halo;
            if (x > 0 && x + fw > core_w) {
                shelf_y += shelf_h;
                x = 0;
                shelf_h = 0;
            }
            if (at) at->push_back({x + halo, shelf_y + halo + m->height});   // left, top measured down from core top
            x += fw;
            shelf_h = std::max(shelf_h, fh);
        }
        return shelf_y + shelf_h;
    };

    Dbu core_w, core_h;
    if (fp.tt_tile || (opt.die_width > 0 && opt.die_height > 0)) {
        if (!fp.tt_tile) fp.die = {0, 0, roundDown(um(opt.die_width), grid), roundDown(um(opt.die_height), grid)};
        else if ((opt.die_width && um(opt.die_width) != fp.die.width()) || (opt.die_height && um(opt.die_height) != fp.die.height()))
            throw std::runtime_error("requested die size differs from tile template");
        fp.core.x0 = roundUp(margin, sw);
        fp.core.y0 = roundUp(margin, ygrid);
        core_w = roundDown(fp.die.x1 - margin - fp.core.x0, sw);
        core_h = roundDown(fp.die.y1 - margin - fp.core.y0, sh);
        if (core_w <= 0 || core_h <= 0) throw std::runtime_error("the die is smaller than twice the core margin");
    } else {
        if (opt.utilization <= 0 || opt.utilization > 1) throw std::runtime_error("utilization must be in (0, 1]");
        double halo_um = opt.macro_halo, macro_fp = 0, widest = 0;
        for (int idx : macro_instances) {
            const LefMacro* m = fp.components[idx].macro;
            double w = lef.toMicrons(m->width) + 2 * halo_um, h = lef.toMicrons(m->height) + 2 * halo_um;
            macro_fp += w * h;
            widest = std::max(widest, w);
        }
        double area = fp.cell_area / opt.utilization + macro_fp;
        double w = std::sqrt(area / opt.aspect);
        w = std::max(w, widest + lef.toMicrons(column));
        double h = area / w;
        // Never smaller than the I/O pins need (each group spread over two edges, every other
        // track of the coarser pin layer) nor than a few rows of cells
        size_t n_in = 0, n_out = 0;
        for (const auto& pb : nl.port_bits) (pb.is_input ? n_in : n_out)++;
        Dbu pin_pitch = 0;
        for (const std::string& name : {opt.pin_hlayer, opt.pin_vlayer}) {
            if (const LefLayer* L = lef.findLayer(name)) pin_pitch = std::max(pin_pitch, L->prefPitch());
        }
        Dbu min_side = std::max<Dbu>(um(10.0), static_cast<Dbu>((std::max(n_in, n_out) + 1) / 2 + 2) * 2 * pin_pitch);
        core_w = roundUp(std::max(um(w), min_side), sw);
        core_h = roundUp(std::max(um(h), min_side), sh);
        // Room for the macro shelves plus at least two rows of cells
        Dbu shelves = packMacros(core_w - column, nullptr);
        if (!macro_instances.empty() && shelves + strip > core_h) core_h = roundUp(shelves + strip, sh);
        if (core_h < 2 * sh) core_h = 2 * sh;
        fp.core.x0 = roundUp(margin, sw);
        fp.core.y0 = roundUp(margin, ygrid);
        fp.die = {0, 0, roundUp(fp.core.x0 + core_w + margin, grid), roundUp(fp.core.y0 + core_h + margin, grid)};
    }
    fp.core.x1 = fp.core.x0 + core_w;
    fp.core.y1 = fp.core.y0 + core_h;

    // Macros: fixed, site and row aligned, inside the core
    {
        std::vector<std::pair<Dbu, Dbu>> at;
        Dbu shelves = packMacros(core_w - column, &at);
        if (shelves > core_h) throw std::runtime_error("the macros do not fit in the core");
        for (size_t k = 0; k < macro_instances.size(); ++k) {
            FpComponent& c = fp.components[macro_instances[k]];
            Dbu x = fp.core.x0 + roundUp(at[k].first, sw);
            Dbu top = fp.core.y1 - at[k].second;
            Dbu y = fp.core.y0 + roundDown(top - fp.core.y0, sh);
            if (x + c.macro->width > fp.core.x1) throw std::runtime_error("macro " + nl.instances[c.instance].name + " does not fit in the core width");
            c.x = x;
            c.y = y;
            c.placed = c.fixed = true;
            c.orient = Orient::N;
            Rect b{x - halo, y - halo, x + c.macro->width + halo, y + c.macro->height + halo};
            b.x0 = std::max(b.x0, fp.core.x0); b.y0 = std::max(b.y0, fp.core.y0);
            b.x1 = std::min(b.x1, fp.core.x1); b.y1 = std::min(b.y1, fp.core.y1);
            fp.macro_blockages.push_back(b);
        }
    }

    // Rows, alternately N and FS (flipped rows share their power rails), cut around macros
    int nrows = static_cast<int>(core_h / sh);
    for (int r = 0; r < nrows; ++r) {
        Dbu y = fp.core.y0 + r * sh;
        std::vector<Interval> blocked;
        for (const Rect& b : fp.macro_blockages) {
            if (b.y0 < y + sh && y < b.y1) blocked.push_back({b.x0, b.x1});
        }
        int seg = 0;
        for (const Interval& iv : subtract(fp.core.x0, fp.core.x1, blocked)) {
            Dbu x0 = fp.core.x0 + roundUp(iv.lo - fp.core.x0, sw);
            Dbu x1 = fp.core.x0 + roundDown(iv.hi - fp.core.x0, sw);
            if (x1 - x0 < sw) continue;
            FpRow row;
            row.name = "ROW_" + std::to_string(r) + (seg ? "_" + std::to_string(seg) : "");
            row.x = x0;
            row.y = y;
            row.count = static_cast<int>((x1 - x0) / sw);
            row.orient = r % 2 ? Orient::FS : Orient::N;
            fp.rows.push_back(row);
            seg++;
        }
    }

    // Routing tracks over the whole die
    std::vector<TrackSpec> specs;
    if (!opt.tracks_file.empty()) specs = readTracksFile(opt.tracks_file);
    else {
        for (int li : lef.routingLayers()) {
            const LefLayer& L = lef.layers[li];
            specs.push_back({L.name, true, lef.toMicrons(L.offset_x), lef.toMicrons(L.prefPitch())});
            specs.push_back({L.name, false, lef.toMicrons(L.offset_y), lef.toMicrons(L.prefPitch())});
        }
    }
    for (const TrackSpec& s : specs) {
        int li = lef.layerIndex(s.layer);
        if (li < 0) throw std::runtime_error("tracks for unknown layer " + s.layer);
        FpTracks t;
        t.layer = li;
        t.x = s.x;
        t.step = um(s.pitch);
        Dbu lo = s.x ? fp.die.x0 : fp.die.y0, hi = s.x ? fp.die.x1 : fp.die.y1;
        t.start = lo + um(s.offset);
        if (t.step <= 0 || t.start > hi) continue;
        t.count = static_cast<int>((hi - t.start) / t.step) + 1;
        fp.tracks.push_back(t);
    }

    // I/O pins on the die edge: inputs on the left and bottom, outputs on the right and top,
    // each on a routing track of the edge's pin layer, spread evenly
    if (!fp.tt_tile) {
        int hl = lef.layerIndex(opt.pin_hlayer), vl = lef.layerIndex(opt.pin_vlayer);
        if (hl < 0 || vl < 0) throw std::runtime_error("pin layers " + opt.pin_hlayer + "/" + opt.pin_vlayer + " are not in the LEF");
        auto slots = [&](int layer, bool x_tracks, Dbu lo, Dbu hi) {
            std::vector<Dbu> out;
            for (const FpTracks& t : fp.tracks) {
                if (t.layer != layer || t.x != x_tracks) continue;
                for (int k = 0; k < t.count; ++k) {
                    Dbu p = t.start + k * t.step;
                    if (p >= lo && p <= hi) out.push_back(p);
                }
            }
            std::sort(out.begin(), out.end());
            out.erase(std::unique(out.begin(), out.end()), out.end());
            return out;
        };
        // Keep pins off the die corners: within the core span on each edge
        std::vector<Dbu> lr = slots(hl, false, fp.core.y0, fp.core.y1);
        std::vector<Dbu> bt = slots(vl, true, fp.core.x0, fp.core.x1);
        std::vector<int> inputs, outputs;
        for (size_t i = 0; i < nl.port_bits.size(); ++i) (nl.port_bits[i].is_input ? inputs : outputs).push_back(static_cast<int>(i));
        auto pinShape = [&](int layer, char side) {
            const LefLayer& L = lef.layers[layer];
            Dbu w = roundUp(L.width, 2 * grid);
            Dbu len = std::max(um(1.0), roundUp(static_cast<Dbu>(std::ceil(L.min_area * lef.dbu * lef.dbu / std::max<Dbu>(w, 1))), grid));
            switch (side) {
            case 'L': return Rect{0, -w / 2, len, w / 2};
            case 'R': return Rect{-len, -w / 2, 0, w / 2};
            case 'B': return Rect{-w / 2, 0, w / 2, len};
            default: return Rect{-w / 2, -len, w / 2, 0};
            }
        };
        // Places `bits` on one edge; every other track when there is room, else every track
        auto place = [&](const std::vector<int>& bits, char side) {
            if (bits.empty()) return;
            const std::vector<Dbu>& all = side == 'L' || side == 'R' ? lr : bt;
            std::vector<Dbu> s;
            for (size_t k = 0; k < all.size(); k += 2) s.push_back(all[k]);
            if (s.size() < bits.size()) s = all;
            if (s.size() < bits.size())
                throw std::runtime_error("not enough room for " + std::to_string(bits.size()) + " pins on one die edge; use a larger die or a lower utilization");
            int layer = side == 'L' || side == 'R' ? hl : vl;
            for (size_t k = 0; k < bits.size(); ++k) {
                size_t slot = static_cast<size_t>((k + 0.5) * s.size() / bits.size());
                FpPin p;
                p.port_bit = bits[k];
                p.layer = layer;
                p.side = side;
                p.shape = pinShape(layer, side);
                if (side == 'L') { p.x = fp.die.x0; p.y = s[slot]; }
                else if (side == 'R') { p.x = fp.die.x1; p.y = s[slot]; }
                else if (side == 'B') { p.x = s[slot]; p.y = fp.die.y0; }
                else { p.x = s[slot]; p.y = fp.die.y1; }
                fp.pins.push_back(p);
            }
        };
        // Split each group over its two edges in proportion to the slots there
        auto split = [&](const std::vector<int>& bits, char a, char b) {
            size_t ca = (a == 'L' || a == 'R' ? lr : bt).size(), cb = (b == 'L' || b == 'R' ? lr : bt).size();
            size_t na = ca + cb ? (bits.size() * ca + (ca + cb) / 2) / (ca + cb) : bits.size();
            na = std::min(na, bits.size());
            place(std::vector<int>(bits.begin(), bits.begin() + na), a);
            place(std::vector<int>(bits.begin() + na, bits.end()), b);
        };
        split(inputs, 'L', 'B');
        split(outputs, 'R', 'T');
    }

    // Power grid: Metal1 rails along every row boundary, vertical straps, horizontal top straps,
    // via stacks where the same net crosses, and top-strap vias onto the macros' own power pins
    if (fp.tt_tile && !macro_instances.empty())
        throw std::runtime_error("the Metal4 tile grid does not support macro power pins yet");
    if (opt.pdn) {
        int rl = lef.layerIndex(opt.rail_layer), sl = lef.layerIndex(opt.strap_layer), tl = fp.tt_tile ? sl : lef.layerIndex(opt.top_layer);
        if (rl < 0 || sl < 0 || tl < 0 || !(rl < sl && (fp.tt_tile || sl < tl))) {
            if (fp.tt_tile) throw std::runtime_error("the Metal4 tile power grid requires the Metal1 and Metal4 routing layers");
            fp.notes.push_back("power grid skipped: layers " + opt.rail_layer + " < " + opt.strap_layer + " < " + opt.top_layer + " not found in the LEF");
        } else {
            const Dbu rw = roundUp(um(opt.rail_width), 2 * grid), stw = roundUp(um(opt.strap_width), 2 * grid), tw = roundUp(um(opt.top_width), 2 * grid);
            // Rails: boundary k is VSS when even (N rows have VSS at the bottom), VDD when odd
            struct Rail { int net; Dbu y; Interval x; };
            std::vector<Rail> rails;
            for (int k = 0; k <= nrows; ++k) {
                Dbu y = fp.core.y0 + k * sh;
                std::vector<Interval> xs;
                for (const FpRow& r : fp.rows) {
                    if (r.y == y || r.y + sh == y) xs.push_back({r.x, r.x + r.count * sw});
                }
                for (const Interval& iv : merge(xs)) rails.push_back({k % 2 ? 0 : 1, y, iv});
            }
            for (const Rail& r : rails) {
                FpSpecialWire w;
                w.net = r.net; w.layer = rl; w.width = rw; w.followpin = true;
                w.x0 = r.x.lo; w.x1 = r.x.hi; w.y0 = w.y1 = r.y;
                fp.special.push_back(w);
            }
            // Vertical straps, VDD and VSS interleaved, centred on the strap layer's tracks and cut
            // around the macros
            const LefLayer& SL = lef.layers[sl];
            const LefLayer& TL = lef.layers[tl];
            // Wide-wire spacing: the largest entry of the spacing table
            Dbu strap_gap = SL.spacing;
            for (const auto& row : SL.table) for (Dbu v : row) strap_gap = std::max(strap_gap, v);
            const Dbu top_gap = std::max(TL.spacing, grid);
            Dbu pitch = std::min(um(opt.strap_pitch), core_w);
            if (pitch <= 0) throw std::runtime_error("power strap pitch must be positive");
            struct Strap { int net; Dbu c; Interval span; };
            std::vector<Strap> straps;
            auto freeSpans = [&](Dbu c) {
                std::vector<Interval> blocked;
                for (const Rect& b : fp.macro_blockages) {
                    if (b.x0 < c + stw / 2 && c - stw / 2 < b.x1) blocked.push_back({b.y0, b.y1});
                }
                // TT supply ports must reach within 10 um of both die edges.
                Dbu lo = fp.core.y0 - rw / 2, hi = fp.core.y1 + rw / 2;
                if (fp.tt_tile) {
                    lo = std::min(lo, um(5.0));
                    hi = std::max(hi, fp.die.y1 - um(5.0));
                }
                return subtract(lo, hi, blocked);
            };
            for (Dbu base = fp.core.x0 + pitch / 4; base < fp.core.x1; base += pitch) {
                for (int net = 0; net < 2; ++net) {
                    Dbu c = snapNearest(base + net * (pitch / 2), fp.die.x0 + SL.offset_x, SL.prefPitch());
                    if (c - stw / 2 < fp.core.x0 || c + stw / 2 > fp.core.x1) continue;
                    for (const Interval& iv : freeSpans(c)) {
                        if (iv.hi - iv.lo >= sh) straps.push_back({net, c, iv});
                    }
                }
            }
            auto railHasStrap = [&](const Rail& r) {
                for (const Strap& s : straps) {
                    if (s.net == r.net && s.c - stw / 2 >= r.x.lo && s.c + stw / 2 <= r.x.hi && r.y - rw / 2 >= s.span.lo && r.y + rw / 2 <= s.span.hi) return true;
                }
                return false;
            };
            // Repair: a rail no strap reaches (a narrow row segment beside a macro) gets its own strap
            int unpowered = 0;
            for (const Rail& r : rails) {
                if (railHasStrap(r)) continue;
                bool done = false;
                Dbu mid = snapNearest((r.x.lo + r.x.hi) / 2, fp.die.x0 + SL.offset_x, SL.prefPitch());
                for (int k = 0; k < 64 && !done; ++k) {
                    Dbu c = mid + ((k % 2) ? -1 : 1) * ((k + 1) / 2) * SL.prefPitch();
                    if (c - stw / 2 < r.x.lo || c + stw / 2 > r.x.hi) continue;
                    for (const Interval& iv : freeSpans(c)) {
                        if (r.y - rw / 2 < iv.lo || r.y + rw / 2 > iv.hi) continue;
                        bool clash = false;
                        for (const Strap& s : straps) {
                            if (std::llabs(s.c - c) < stw + strap_gap && s.span.lo < iv.hi && iv.lo < s.span.hi) clash = true;
                        }
                        if (!clash) {
                            straps.push_back({r.net, c, iv});
                            done = true;
                        }
                        break;
                    }
                }
                if (!done) unpowered++;
            }
            if (unpowered) fp.notes.push_back(std::to_string(unpowered) + " rail segments have no room for a power strap");
            for (const Strap& s : straps) {
                FpSpecialWire w;
                w.net = s.net; w.layer = sl; w.width = stw;
                w.x0 = w.x1 = s.c; w.y0 = s.span.lo; w.y1 = s.span.hi;
                fp.special.push_back(w);
            }
            // Horizontal top straps across the whole core (over the macros too)
            Dbu tpitch = std::min(um(opt.top_pitch), core_h);
            if (!fp.tt_tile && tpitch <= 0) throw std::runtime_error("top strap pitch must be positive");
            struct Top { int net; Dbu c; };
            std::vector<Top> tops;
            for (Dbu base = fp.core.y0 + tpitch / 4; !fp.tt_tile && base < fp.core.y1; base += tpitch) {
                for (int net = 0; net < 2; ++net) {
                    Dbu c = roundDown(base + net * (tpitch / 2), grid);
                    if (c - tw / 2 < fp.core.y0 || c + tw / 2 > fp.core.y1) continue;
                    tops.push_back({net, c});
                }
            }
            // Repair: a strap segment no top strap crosses (a short one under a macro) gets one
            int floating = 0;
            if (!fp.tt_tile) for (const Strap& s : straps) {
                bool reached = false;
                for (const Top& t : tops) reached |= t.net == s.net && t.c - tw / 2 >= s.span.lo && t.c + tw / 2 <= s.span.hi;
                if (reached) continue;
                bool done = false;
                // Lowest free spot first, so the other net still fits in the same band
                Dbu lo = std::max(s.span.lo, fp.core.y0), hi = std::min(s.span.hi, fp.core.y1);
                for (Dbu y = roundUp(lo + tw / 2, grid); y + tw / 2 <= hi && !done; y += roundUp(tw / 8, grid)) {
                    bool clash = false;
                    for (const Top& t : tops) clash |= std::llabs(t.c - y) < tw + top_gap;
                    if (!clash) {
                        tops.push_back({s.net, y});
                        done = true;
                    }
                }
                if (!done) floating++;
            }
            if (floating) fp.notes.push_back(std::to_string(floating) + " power straps have no room for a top strap");
            for (const Top& t : tops) {
                FpSpecialWire w;
                w.net = t.net; w.layer = tl; w.width = tw;
                w.x0 = fp.core.x0; w.x1 = fp.core.x1; w.y0 = w.y1 = t.c;
                fp.special.push_back(w);
            }
            // Via definitions, one cut layer each (a stack is several vias at one point, as
            // OpenROAD's pdngen writes them), deduplicated by layer and size
            std::map<std::tuple<int, Dbu, Dbu>, int> via_ids;
            auto singleVia = [&](int lower, int upper, Dbu w, Dbu h) {
                auto key = std::make_tuple(lower, w, h);
                auto it = via_ids.find(key);
                if (it != via_ids.end()) return it->second;
                FpVia v;
                std::string name = "pdn_" + lef.layers[lower].name + "_" + lef.layers[upper].name + "_" + std::to_string(w) + "x" + std::to_string(h);
                int id = -1;
                if (makeViaStack(lef, lower, upper, w, h, name, v)) {
                    id = static_cast<int>(fp.vias.size());
                    fp.vias.push_back(v);
                }
                via_ids[key] = id;
                return id;
            };
            // (via, bottom layer) for every routing-layer pair from lower to upper; empty if any is impossible
            auto viaFor = [&](int lower, int upper, Dbu w, Dbu h) {
                std::vector<std::pair<int, int>> stack;
                int below = lower;
                for (int i = lower + 1; i <= upper; ++i) {
                    if (lef.layers[i].type != LayerType::ROUTING) continue;
                    int id = singleVia(below, i, w, h);
                    if (id < 0) return std::vector<std::pair<int, int>>{};
                    stack.push_back({id, below});
                    below = i;
                }
                return stack;
            };
            auto addVia = [&](int net, const std::vector<std::pair<int, int>>& stack, Dbu x, Dbu y) {
                for (auto [id, lower] : stack) {
                    FpSpecialWire w;
                    w.net = net; w.layer = lower; w.via = id; w.x0 = w.x1 = x; w.y0 = w.y1 = y;
                    fp.special.push_back(w);
                }
            };
            int missing = 0;
            for (const Rail& r : rails) {
                for (const Strap& s : straps) {
                    if (s.net != r.net || s.c - stw / 2 < r.x.lo || s.c + stw / 2 > r.x.hi) continue;
                    if (r.y - rw / 2 < s.span.lo || r.y + rw / 2 > s.span.hi) continue;
                    auto stack = viaFor(rl, sl, std::min(stw, roundUp(um(opt.rail_via_width), 2 * grid)), rw);
                    if (stack.empty()) { missing++; continue; }
                    addVia(r.net, stack, s.c, r.y);
                }
            }
            for (const Top& t : tops) {
                for (const Strap& s : straps) {
                    if (s.net != t.net || t.c - tw / 2 < s.span.lo || t.c + tw / 2 > s.span.hi) continue;
                    auto stack = viaFor(sl, tl, stw, tw);
                    if (stack.empty()) { missing++; continue; }
                    addVia(t.net, stack, s.c, t.c);
                }
            }
            // Macro power pins on the strap layer, under a top strap of the same net
            int macro_vias = 0;
            for (int idx : macro_instances) {
                const FpComponent& c = fp.components[idx];
                for (const LefPin& p : c.macro->pins) {
                    if (!p.isSupply()) continue;
                    int net = p.use == PinUse::POWER ? 0 : 1;
                    for (const LefShape& s : p.shapes) {
                        if (s.layer != sl) continue;
                        Rect r{c.x + s.rect.x0, c.y + s.rect.y0, c.x + s.rect.x1, c.y + s.rect.y1};
                        for (const Top& t : tops) {
                            if (t.net != net || t.c - tw / 2 < r.y0 || t.c + tw / 2 > r.y1) continue;
                            Dbu w = roundDown(r.width(), 2 * grid);
                            auto stack = viaFor(sl, tl, w, tw);
                            if (stack.empty()) { missing++; continue; }
                            addVia(net, stack, r.x0 + r.width() / 2, t.c);
                            macro_vias++;
                        }
                    }
                }
            }
            if (fp.tt_tile && (missing || unpowered || straps.empty())) throw std::runtime_error("incomplete Metal4 tile power grid");
            if (missing) fp.notes.push_back(std::to_string(missing) + " power-grid crossings too small for a via");
            if (!macro_instances.empty() && macro_vias == 0) fp.notes.push_back("no top strap reaches a macro power pin; lower --top-pitch");
        }
    }
    return fp;
}

int insertFillers(Floorplan& fp, const Lef& lef, const std::vector<std::string>& names) {
    std::vector<const LefMacro*> fills;
    for (const auto& n : names) if (const LefMacro* m = lef.findMacro(n)) fills.push_back(m);
    std::sort(fills.begin(), fills.end(), [](const LefMacro* a, const LefMacro* b) { return a->width > b->width; });
    if (fills.empty()) return 0;
    fp.fillers.clear();
    const Dbu sw = fp.site->width;
    std::map<Dbu, std::vector<std::pair<Dbu, Dbu>>> used;   // row y -> occupied x intervals
    for (const FpComponent& c : fp.components) {
        if (c.placed && !c.fixed) used[c.y].push_back({c.x, c.x + c.macro->width});
    }
    for (const FpRow& r : fp.rows) {
        auto& cells = used[r.y];
        std::sort(cells.begin(), cells.end());
        Dbu x = r.x, end = r.x + static_cast<Dbu>(r.count) * sw;
        auto fill = [&](Dbu from, Dbu to) {
            for (const LefMacro* m : fills) {
                while (from + m->width <= to) {
                    fp.fillers.push_back({m, from, r.y, r.orient});
                    from += m->width;
                }
            }
        };
        for (const auto& [a, b] : cells) {
            if (b <= r.x || a >= end) continue;
            if (a > x) fill(x, a);
            x = std::max(x, b);
        }
        if (x < end) fill(x, end);
    }
    return static_cast<int>(fp.fillers.size());
}

std::vector<std::string> checkFloorplan(const Floorplan& fp, const CellNetlist& nl, const Lef& lef) {
    std::vector<std::string> bad;
    const Dbu sw = fp.site->width, sh = fp.site->height;
    if (!fp.die.contains(fp.core)) bad.push_back("core is not inside the die");
    double row_area = 0;
    for (const FpRow& r : fp.rows) {
        Rect rr{r.x, r.y, r.x + r.count * sw, r.y + sh};
        row_area += lef.toMicrons(rr.width()) * lef.toMicrons(rr.height());
        if (!fp.core.contains(rr)) bad.push_back("row " + r.name + " leaves the core");
        if ((r.x - fp.core.x0) % sw) bad.push_back("row " + r.name + " is off the site grid");
        for (const Rect& b : fp.macro_blockages) {
            if (b.overlaps(rr)) bad.push_back("row " + r.name + " overlaps a macro");
        }
    }
    if (fp.cell_area > row_area + 1e-9) bad.push_back("the standard cells do not fit in the rows");
    // Pins: inside the die, on a track of their layer, no two on the same spot
    std::map<std::tuple<int, Dbu, Dbu>, int> seen;
    for (const FpPin& p : fp.pins) {
        const std::string& name = nl.port_bits[p.port_bit].name;
        Rect r{p.x + p.shape.x0, p.y + p.shape.y0, p.x + p.shape.x1, p.y + p.shape.y1};
        if (!fp.die.contains(r)) bad.push_back("pin " + name + " leaves the die");
        bool vertical_edge = p.side == 'L' || p.side == 'R';
        Dbu pos = vertical_edge ? p.y : p.x;
        bool on_track = false;
        for (const FpTracks& t : fp.tracks) {
            if (t.layer != p.layer || t.x == vertical_edge) continue;
            if (pos >= t.start && (pos - t.start) % t.step == 0 && (pos - t.start) / t.step < t.count) on_track = true;
        }
        if (!on_track) bad.push_back("pin " + name + " is not on a routing track");
        if (seen[{p.layer, p.x, p.y}]++) bad.push_back("pin " + name + " shares its spot with another pin");
    }
    if (fp.pins.size() != nl.port_bits.size()) bad.push_back("not every port has a pin");
    // Macros: inside the core and apart
    for (size_t i = 0; i < fp.components.size(); ++i) {
        const FpComponent& a = fp.components[i];
        if (!a.placed) continue;
        Rect ra{a.x, a.y, a.x + a.macro->width, a.y + a.macro->height};
        if (!fp.core.contains(ra)) bad.push_back("macro " + nl.instances[a.instance].name + " leaves the core");
        for (size_t j = i + 1; j < fp.components.size(); ++j) {
            const FpComponent& b = fp.components[j];
            if (b.placed && ra.overlaps({b.x, b.y, b.x + b.macro->width, b.y + b.macro->height}))
                bad.push_back("macros " + nl.instances[a.instance].name + " and " + nl.instances[b.instance].name + " overlap");
        }
    }
    // No constant connections may remain (DEF has no 1'b0 / 1'b1)
    for (const auto& inst : nl.instances) {
        for (int n : inst.pin_net) {
            if (n == CellNetlist::kConst0 || n == CellNetlist::kConst1) {
                bad.push_back("instance " + inst.name + " is tied to a constant without a tie cell");
                break;
            }
        }
    }
    return bad;
}
