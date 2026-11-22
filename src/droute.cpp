// Detailed router: track grid, pin access, negotiated A* routing inside the global-route guides
#include "../include/droute.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace {

double msSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

using Node = uint32_t;
constexpr Node kNone = 0xffffffffu;

struct Grid {
    std::vector<Dbu> xs, ys;          // node coordinates: vertical-layer tracks, horizontal-layer tracks
    int nx = 0, ny = 0, nl = 0;
    std::vector<int> lef;             // grid layer -> LEF layer (0 is Metal1, pin access only)
    std::vector<char> hor;
    Node id(int i, int j, int l) const { return static_cast<Node>((static_cast<size_t>(l) * ny + j) * nx + i); }
    int I(Node n) const { return static_cast<int>(n % nx); }
    int J(Node n) const { return static_cast<int>((n / nx) % ny); }
    int Lr(Node n) const { return static_cast<int>(n / (static_cast<size_t>(nx) * ny)); }
    size_t size() const { return static_cast<size_t>(nx) * ny * nl; }
    // index range [lo, hi] of coordinates inside [a, b]
    static void range(const std::vector<Dbu>& v, Dbu a, Dbu b, int& lo, int& hi) {
        lo = static_cast<int>(std::lower_bound(v.begin(), v.end(), a) - v.begin());
        hi = static_cast<int>(std::upper_bound(v.begin(), v.end(), b) - v.begin()) - 1;
    }
};

void orientRect(Orient o, Dbu w, Dbu h, Rect& r) {
    if (o == Orient::FS || o == Orient::S) { Dbu a = h - r.y1, b = h - r.y0; r.y0 = a; r.y1 = b; }
    if (o == Orient::FN || o == Orient::S) { Dbu a = w - r.x1, b = w - r.x0; r.x0 = a; r.x1 = b; }
}

// Is `r` covered by the union of `rs`?
bool covered(const Rect& r, const std::vector<Rect>& rs) {
    std::vector<Rect> parts;
    for (const Rect& q : rs) {
        Rect c{std::max(r.x0, q.x0), std::max(r.y0, q.y0), std::min(r.x1, q.x1), std::min(r.y1, q.y1)};
        if (c.x0 < c.x1 && c.y0 < c.y1) parts.push_back(c);
    }
    if (parts.empty()) return false;
    std::vector<Dbu> xs{r.x0, r.x1}, ys{r.y0, r.y1};
    for (const Rect& p : parts) { xs.push_back(p.x0); xs.push_back(p.x1); ys.push_back(p.y0); ys.push_back(p.y1); }
    std::sort(xs.begin(), xs.end()); xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::sort(ys.begin(), ys.end()); ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
    for (size_t a = 0; a + 1 < xs.size(); ++a) {
        for (size_t b = 0; b + 1 < ys.size(); ++b) {
            Dbu cx = (xs[a] + xs[a + 1]) / 2, cy = (ys[b] + ys[b + 1]) / 2;
            bool in = false;
            for (const Rect& p : parts) in |= p.x0 <= cx && cx <= p.x1 && p.y0 <= cy && cy <= p.y1;
            if (!in) return false;
        }
    }
    return true;
}

// The PDK via between two metal layers whose pads lie along each layer's preferred direction
struct ViaPick {
    std::string name;
    Rect bottom, top;   // pads, relative to the via centre
};

bool pickVia(const Lef& lef, int lower, int upper, bool want_bottom_horizontal, bool want_top_horizontal, ViaPick& out) {
    bool found = false;
    for (const LefVia& v : lef.vias) {
        if (!v.is_default) continue;
        Rect b{}, t{};
        bool hb = false, ht = false, cut = false;
        int cuts = 0;
        for (const LefShape& s : v.shapes) {
            if (s.layer == lower) { b = s.rect; hb = true; }
            else if (s.layer == upper) { t = s.rect; ht = true; }
            else if (s.layer > lower && s.layer < upper) { cut = true; cuts++; }
            else { cut = false; hb = false; break; }
        }
        if (!hb || !ht || !cut || cuts != 1) continue;
        bool bh = b.width() > b.height(), th = t.width() > t.height();
        if (bh != want_bottom_horizontal || th != want_top_horizontal) continue;
        // the smallest pads of that orientation (the plain variant, not the "_s" long ones)
        if (!found || b.width() * b.height() + t.width() * t.height() < out.bottom.width() * out.bottom.height() + out.top.width() * out.top.height()) {
            out = {v.name, b, t};
            found = true;
        }
    }
    return found;
}

struct Pin {
    std::vector<Node> terms;     // grid nodes that connect to the pin
    double cx = 0, cy = 0;       // centre, for ordering
};

struct RNet {
    int netlist_net;
    std::vector<Pin> pins;
    std::vector<int> pin_order;  // Prim pin ordering, precomputed once at setup
    int gx0, gy0, gx1, gy1;      // bounding box in nodes
    std::vector<Node> used;      // nodes the routing occupies
    std::vector<Node> partners;  // nodes reserved next to its off-grid vias (not metal)
    std::vector<std::pair<Node, Node>> edges;
    bool ok = true;
    bool has_escape = false;
};

struct AstarItem {
    float f, d;
    Node n;
};

// Reusable 4-ary min-heap: better cache locality than binary std::priority_queue and zero per-search allocations
struct FastMinHeap {
    std::vector<AstarItem> h;
    void clear() { h.clear(); }
    bool empty() const { return h.empty(); }
    static bool less(const AstarItem& a, const AstarItem& b) {
        return a.f < b.f || (a.f == b.f && a.d > b.d);
    }
    void push(const AstarItem& it) {
        size_t i = h.size();
        h.push_back(it);
        while (i > 0) {
            size_t p = (i - 1) >> 2;
            if (!less(it, h[p])) break;
            h[i] = h[p];
            i = p;
        }
        h[i] = it;
    }
    AstarItem pop() {
        AstarItem top = h[0];
        AstarItem last = h.back();
        h.pop_back();
        const size_t sz = h.size();
        if (sz > 0) {
            size_t i = 0;
            for (;;) {
                size_t c = (i << 2) + 1;
                if (c >= sz) break;
                size_t best = c;
                size_t end = std::min(sz, c + 4);
                for (size_t k = c + 1; k < end; ++k) {
                    if (less(h[k], h[best])) best = k;
                }
                if (!less(h[best], last)) break;
                h[i] = h[best];
                i = best;
            }
            h[i] = last;
        }
        return top;
    }
};

} // namespace

DrouteResult detailedRoute(const Floorplan& fp, const CellNetlist& nl, const Lef& lef, const GrouteResult& gr, const DrouteOptions& opt) {
    DrouteResult res;
    auto t0 = std::chrono::steady_clock::now();
    // ------------------------------------------------ grid
    Grid G;
    std::vector<int> routing = lef.routingLayers();
    if (routing.empty()) throw std::runtime_error("no routing layers");
    G.lef.push_back(routing[0]);   // Metal1: pin access
    for (const auto& l : gr.layers) G.lef.push_back(lef.layerIndex(l.name));
    G.nl = static_cast<int>(G.lef.size());
    for (int l : G.lef) G.hor.push_back(lef.layers[l].horizontal);
    int vref = -1, href = -1;
    for (int l = 1; l < G.nl; ++l) {
        if (!G.hor[l] && vref < 0) vref = G.lef[l];
        if (G.hor[l] && href < 0) href = G.lef[l];
    }
    if (vref < 0 || href < 0) throw std::runtime_error("detailed routing needs a horizontal and a vertical layer");
    for (const FpTracks& t : fp.tracks) {
        if (t.layer == vref && t.x) for (int k = 0; k < t.count; ++k) G.xs.push_back(t.start + k * t.step);
        if (t.layer == href && !t.x) for (int k = 0; k < t.count; ++k) G.ys.push_back(t.start + k * t.step);
    }
    std::sort(G.xs.begin(), G.xs.end()); G.xs.erase(std::unique(G.xs.begin(), G.xs.end()), G.xs.end());
    std::sort(G.ys.begin(), G.ys.end()); G.ys.erase(std::unique(G.ys.begin(), G.ys.end()), G.ys.end());
    G.nx = static_cast<int>(G.xs.size());
    G.ny = static_cast<int>(G.ys.size());
    if (G.nx < 2 || G.ny < 2) throw std::runtime_error("no routing tracks");
    if (G.size() >= kNone) throw std::runtime_error("design too large for the detailed-routing grid");
    const size_t N = G.size();

    // Vias between consecutive grid layers
    std::vector<ViaPick> via(G.nl);
    for (int l = 0; l + 1 < G.nl; ++l) {
        if (!pickVia(lef, G.lef[l], G.lef[l + 1], G.hor[l], G.hor[l + 1], via[l]))
            throw std::runtime_error("no via from " + lef.layers[G.lef[l]].name + " to " + lef.layers[G.lef[l + 1]].name + " with pads along the tracks");
    }

    // Metal1 -> Metal2 also with a vertical Metal1 pad (Via1_YY), for narrow vertical pins
    ViaPick via1v;
    bool have_via1v = pickVia(lef, G.lef[0], G.lef[1], false, G.hor[1], via1v);

    // Drawn wire width per layer: as wide as the via pads across the track (Via2/Via3 bottom pads
    // are 0.21 um against a 0.20 um minimum wire), so wire and pads never form 5 nm steps
    res.wire_width.assign(G.nl, 0);
    res.wire_layer.assign(G.nl, -1);
    for (int l = 1; l < G.nl; ++l) {
        Dbu w = lef.layers[G.lef[l]].width;
        for (int k : {l - 1, l}) {
            if (k < 0 || k + 1 >= G.nl) continue;
            const Rect& r = k == l - 1 ? via[k].top : via[k].bottom;
            w = std::max(w, G.hor[l] ? r.height() : r.width());
        }
        res.wire_width[l] = w;
        res.wire_layer[l] = G.lef[l];
    }
    // ------------------------------------------------ obstacles
    // blocked: no net may use the node; pin_owner: only that net may (its pin)
    std::vector<char> blocked(N, 0);
    std::vector<int> pin_owner(N, -1);
    for (int i = 0; i < G.nx; ++i) for (int j = 0; j < G.ny; ++j) blocked[G.id(i, j, 0)] = 1;   // Metal1: only pin access
    // Envelope of a node's metal (wire or via pad) on each layer, and the spacing kept to obstacles
    std::vector<Dbu> half_cross(G.nl), half_along(G.nl), space(G.nl);
    for (int l = 1; l < G.nl; ++l) {
        const LefLayer& L = lef.layers[G.lef[l]];
        Dbu c = L.width / 2, a = L.width / 2;
        for (int k : {l - 1, l}) {
            if (k < 0 || k + 1 >= G.nl) continue;
            const Rect& pad = k == l - 1 ? via[k].top : via[k].bottom;
            Dbu pc = G.hor[l] ? pad.height() / 2 : pad.width() / 2, pa = G.hor[l] ? pad.width() / 2 : pad.height() / 2;
            c = std::max(c, pc);
            a = std::max(a, pa);
        }
        Dbu step = G.hor[l] ? (G.xs[1] - G.xs[0]) : (G.ys[1] - G.ys[0]);
        half_cross[l] = c;
        half_along[l] = std::max(a, step / 2);
        Dbu s = L.spacing;
        for (size_t w = 0; w < L.table.size() && w < 2; ++w)
            for (size_t p = 0; p < L.table[w].size() && p < 2; ++p) s = std::max(s, L.table[w][p]);
        space[l] = s;
    }
    auto layerOf = [&](int lef_layer) {
        for (int l = 1; l < G.nl; ++l) if (G.lef[l] == lef_layer) return l;
        return -1;
    };
    // Nodes whose envelope (plus spacing) touches r on grid layer l
    auto nodesNear = [&](int l, const Rect& r, auto&& fn) {
        Dbu ex = (G.hor[l] ? half_along[l] : half_cross[l]) + space[l];
        Dbu ey = (G.hor[l] ? half_cross[l] : half_along[l]) + space[l];
        int i0, i1, j0, j1;
        Grid::range(G.xs, r.x0 - ex + 1, r.x1 + ex - 1, i0, i1);
        Grid::range(G.ys, r.y0 - ey + 1, r.y1 + ey - 1, j0, j1);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) fn(G.id(i, j, l));
    };
    std::vector<std::vector<Rect>> obstacles(G.nl);   // per grid layer, for exact spacing checks
    auto block = [&](int lef_layer, const Rect& r) {
        int l = layerOf(lef_layer);
        if (l < 0) return;
        obstacles[l].push_back(r);
        nodesNear(l, r, [&](Node n) { blocked[n] = 1; });
    };
    for (const FpSpecialWire& w : fp.special) {
        if (w.via >= 0) {
            for (const LefShape& s : fp.vias[w.via].shapes) block(s.layer, {w.x0 + s.rect.x0, w.y0 + s.rect.y0, w.x0 + s.rect.x1, w.y0 + s.rect.y1});
            continue;
        }
        if (w.y0 == w.y1) block(w.layer, {std::min(w.x0, w.x1), w.y0 - w.width / 2, std::max(w.x0, w.x1), w.y0 + w.width / 2});
        else block(w.layer, {w.x0 - w.width / 2, std::min(w.y0, w.y1), w.x0 + w.width / 2, std::max(w.y0, w.y1)});
    }
    auto placed = [&](const FpComponent& c, const LefShape& s) {
        Rect r = s.rect;
        orientRect(c.orient, c.macro->width, c.macro->height, r);
        return Rect{c.x + r.x0, c.y + r.y0, c.x + r.x1, c.y + r.y1};
    };
    // Macro pins on the first vertical layer at the macro's bottom edge (IHP SRAMs: 0.26 um squares
    // 0.51 um apart, off the 0.48 um tracks) are reached by an escape instead of the grid
    auto escapeShape = [&](const FpComponent& c, const Rect& r, int l) {
        return c.macro->cls == MacroClass::BLOCK && c.orient == Orient::N && l == 1 && !G.hor[1] && r.y0 <= c.y + lef.manufacturing_grid;
    };
    // Pins block nearby nodes only for other nets: those nodes are reserved for the pin's net
    // (netlist numbering here; remapped to router indices below). Near pins of two nets: blocked.
    auto reserve = [&](int lef_layer, const Rect& r, int net) {
        int l = layerOf(lef_layer);
        if (l < 0) return;
        if (net < 2) { block(lef_layer, r); return; }
        nodesNear(l, r, [&](Node n) {
            if (blocked[n]) return;
            if (pin_owner[n] >= 0 && pin_owner[n] != net) { blocked[n] = 1; pin_owner[n] = -1; return; }
            pin_owner[n] = net;
        });
    };
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        for (const LefShape& s : c.macro->obs) block(s.layer, placed(c, s));
    }
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        const auto& inst = nl.instances[c.instance];
        for (const LefPin& p : c.macro->pins) {
            int k = inst.cell->pinIndex(p.name);
            int net = k >= 0 ? inst.pin_net[k] : -1;
            for (const LefShape& s : p.shapes) {
                Rect r = placed(c, s);
                if (escapeShape(c, r, layerOf(s.layer))) block(s.layer, r);
                else reserve(s.layer, r, net);
            }
        }
    }
    for (const FpPin& p : fp.pins) reserve(p.layer, {p.x + p.shape.x0, p.y + p.shape.y0, p.x + p.shape.x1, p.y + p.shape.y1}, nl.port_bits[p.port_bit].net);

    // ------------------------------------------------ pins: access points and terminals
    // Standard cells: Via1 landings whose cut lies inside the pin and whose Metal1 pad keeps the
    // Metal1 spacing to every other shape of the cell. Computed once per (cell master,
    // orientation), relative to the cell origin. Each point records which Via1 it uses.
    struct Access {
        Dbu dx, dy;
        int kind;          // 0: via[0] (horizontal Metal1 pad), 1: via1v (vertical)
        bool off = false;  // not on a node: joined to node (tx, nearest row) by a Metal2 patch
        int quality = 3;   // 3 pad inside the pin, 2 cut inside, 1 pad overlapping (see below)
        Dbu tx = 0;        // the Metal2 track it joins (equals dx for on-grid points)
    };
    const LefLayer& M1 = lef.layers[G.lef[0]];
    Rect cut1{};
    for (const LefVia& v : lef.vias) {
        if (v.name != via[0].name) continue;
        for (const LefShape& sh : v.shapes) if (sh.layer > G.lef[0] && sh.layer < G.lef[1]) cut1 = sh.rect;
    }
    auto rectDist = [](const Rect& a, const Rect& b) {
        double dx = std::max<Dbu>(0, std::max(a.x0 - b.x1, b.x0 - a.x1)), dy = std::max<Dbu>(0, std::max(a.y0 - b.y1, b.y0 - a.y1));
        return std::sqrt(dx * dx + dy * dy);
    };
    std::map<std::pair<const LefMacro*, int>, std::vector<std::vector<Access>>> access_cache;
    auto accessOf = [&](const FpComponent& c, const LibCell* cell) -> const std::vector<std::vector<Access>>& {
        auto key = std::make_pair(c.macro, static_cast<int>(c.orient));
        auto it = access_cache.find(key);
        if (it != access_cache.end()) return it->second;
        std::vector<std::vector<Access>> out(cell->pins.size());
        Dbu sx = G.xs[1] - G.xs[0], sy = G.ys[1] - G.ys[0];
        // every Metal1 shape of the cell, with the pin it belongs to (-1: obstruction)
        std::vector<std::pair<Rect, int>> shapes;
        for (size_t p = 0; p < c.macro->pins.size(); ++p) {
            for (const LefShape& sh : c.macro->pins[p].shapes) {
                if (sh.layer != G.lef[0]) continue;
                Rect r = sh.rect;
                orientRect(c.orient, c.macro->width, c.macro->height, r);
                shapes.push_back({r, static_cast<int>(p)});
            }
        }
        for (const LefShape& sh : c.macro->obs) {
            if (sh.layer != G.lef[0]) continue;
            Rect r = sh.rect;
            orientRect(c.orient, c.macro->width, c.macro->height, r);
            shapes.push_back({r, -1});
        }
        for (size_t k = 0; k < cell->pins.size(); ++k) {
            int p = c.macro->pinIndex(cell->pins[k].name);
            if (p < 0) continue;
            std::vector<Rect> mine;
            for (auto& [r, owner] : shapes) if (owner == p) mine.push_back(r);
            for (Dbu y = 0; y <= c.macro->height; y += sy) {
                for (Dbu x = 0; x <= c.macro->width; x += sx) {
                    Rect cut{x + cut1.x0, y + cut1.y0, x + cut1.x1, y + cut1.y1};
                    bool cut_inside = covered(cut, mine);
                    // Quality: 3 pad inside the pin, 2 cut inside, 1 pad overlapping the pin by at
                    // least the Metal1 width (the pad bridges to it); the pad must keep the Metal1
                    // spacing to every other shape of the cell unless it adds no metal
                    int best = -1, best_q = 0;
                    for (int kind = 0; kind < (have_via1v ? 2 : 1); ++kind) {
                        const Rect& pr = kind ? via1v.bottom : via[0].bottom;
                        Rect pad{x + pr.x0, y + pr.y0, x + pr.x1, y + pr.y1};
                        int q = 0;
                        if (covered(pad, mine)) q = 3;
                        else if (cut_inside) q = 2;
                        else {
                            for (const Rect& r : mine) {
                                Dbu ow = std::min(pad.x1, r.x1) - std::max(pad.x0, r.x0), oh = std::min(pad.y1, r.y1) - std::max(pad.y0, r.y0);
                                if (ow >= M1.width && oh >= M1.width) q = 1;
                            }
                        }
                        if (q == 0) continue;
                        if (q < 3) {
                            // the added metal keeps its distance from other shapes, and from parts
                            // of the same pin it does not touch (no notch)
                            bool ok = pad.x0 >= 0 && pad.x1 <= c.macro->width;
                            for (auto& [r, owner] : shapes) {
                                double d = rectDist(pad, r);
                                if (owner == p ? (d > 0 && d < M1.spacing) : d < M1.spacing) { ok = false; break; }
                            }
                            if (!ok) continue;
                        }
                        if (q > best_q) { best = kind; best_q = q; }
                    }
                    if (best >= 0) out[k].push_back({x, y, best, false, best_q, x});
                }
            }
            // Pads that stick out of the pin leave notches: they are the last resort. Keep the
            // grid points whose pad lies inside the pin; else try vias between grid rows (pad
            // inside the pin, joined by a Metal2 stub); only then the others.
            std::vector<Access> inside, rest;
            for (const Access& a : out[k]) (a.quality == 3 ? inside : rest).push_back(a);
            out[k] = inside;
            if (out[k].empty() && std::getenv("FASTPNR_TRACE_ACCESS")) {
                fprintf(stderr, "  offgrid search %s/%s orient %d:", c.macro->name.c_str(), cell->pins[k].name.c_str(), (int)c.orient);
                for (const Rect& r : mine) fprintf(stderr, " [%lld %lld %lld %lld]", (long long)r.x0, (long long)r.y0, (long long)r.x1, (long long)r.y1);
                fprintf(stderr, "\n");
            }
            // Off the grid: the via's pad must lie inside the pin; the via may sit between two
            // rows and up to `max_off` beside a Metal2 track (a Metal2 patch then joins its pad to
            // the track's node). max_off keeps the patch the Metal2 spacing away from the
            // neighbouring tracks. On-track positions are preferred.
            if (out[k].empty()) {
                const LefLayer& M2 = lef.layers[G.lef[1]];
                Dbu half_pad = std::max(via[0].top.width(), have_via1v ? via1v.top.width() : 0) / 2;
                Dbu max_off = std::max<Dbu>(0, sx - M2.spacing - 2 * half_pad - 2 * lef.manufacturing_grid);
                Dbu g = lef.manufacturing_grid;
                for (Dbu off_limit : {Dbu(0), max_off}) {
                    for (Dbu tx = 0; tx <= c.macro->width; tx += sx) {
                        bool done = false;
                        for (const Rect& r : mine) {
                            for (int kind : {1, 0}) {
                                if (kind == 1 && !have_via1v) continue;
                                const Rect& pr = kind ? via1v.bottom : via[0].bottom;
                                Dbu xlo = std::max(r.x0 - pr.x0, tx - off_limit), xhi = std::min(r.x1 - pr.x1, tx + off_limit);
                                Dbu ylo = r.y0 - pr.y0, yhi = r.y1 - pr.y1;
                                if (xlo > xhi || ylo > yhi) continue;
                                Dbu x = std::clamp(tx, xlo, xhi);
                                x = (x / g) * g;
                                if (x < xlo) x += g;
                                if (x > xhi) continue;
                                // nearest grid row inside the range, else the closest point to one
                                Dbu y = std::clamp(((ylo + yhi) / 2 + sy / 2) / sy * sy, ylo, yhi);
                                y = (y / g) * g;
                                if (y < ylo) y += g;
                                if (y > yhi || y < 0 || y > c.macro->height) continue;
                                if (std::getenv("FASTPNR_TRACE_ACCESS"))
                                    fprintf(stderr, "  offgrid %s/%s x=%lld (track %lld) y=%lld kind %d\n", c.macro->name.c_str(), cell->pins[k].name.c_str(), (long long)x,
                                            (long long)tx, (long long)y, kind);
                                out[k].push_back({x, y, kind, x != tx || y % sy != 0, 3, tx});
                                done = true;
                                break;
                            }
                            if (done) break;
                        }
                    }
                    if (!out[k].empty()) break;
                }
            }
            if (out[k].empty()) out[k] = rest;
        }
        // A compatible set across the cell's pins: two pins' Via1s on one Metal2 track must be at
        // least a pad height plus the Metal2 spacing apart. Pins with the fewest options choose
        // first; a pin left with nothing keeps its candidates (the conflict is then unavoidable).
        {
            const LefLayer& M2 = lef.layers[G.lef[1]];
            Dbu need = std::max(via[0].top.height(), have_via1v ? via1v.top.height() : 0) + M2.spacing;
            auto clash = [&](const Access& a, const Access& b) { return a.tx == b.tx && std::llabs(a.dy - b.dy) < need; };
            std::vector<size_t> pins_order;
            for (size_t k = 0; k < out.size(); ++k) if (!out[k].empty()) pins_order.push_back(k);
            std::stable_sort(pins_order.begin(), pins_order.end(), [&](size_t a, size_t b) { return out[a].size() < out[b].size(); });
            std::vector<std::vector<Access>> kept(out.size());
            for (size_t k : pins_order) {
                for (const Access& a : out[k]) {
                    bool ok = true;
                    for (size_t j = 0; j < kept.size() && ok; ++j) {
                        if (j == k) continue;
                        for (const Access& b : kept[j]) if (clash(a, b)) { ok = false; break; }
                    }
                    if (ok) kept[k].push_back(a);
                }
                if (kept[k].empty()) kept[k] = out[k];
            }
            out = kept;
        }
        if (std::getenv("FASTPNR_TRACE")) {
            fprintf(stderr, "access %s orient %d:", c.macro->name.c_str(), static_cast<int>(c.orient));
            for (size_t k = 0; k < out.size(); ++k) fprintf(stderr, " %s=%zu", cell->pins[k].name.c_str(), out[k].size());
            fprintf(stderr, "\n");
        }
        return access_cache.emplace(key, std::move(out)).first->second;
    };
    std::unordered_map<Node, int> access_kind;   // Metal1 node -> which Via1
    struct OffGrid {
        Dbu x, y;   // the via
        int kind;   // which Via1
    };
    std::unordered_map<Node, OffGrid> offgrid;   // Metal2 terminal -> its off-grid Via1
    std::unordered_map<Node, Node> partner_of;   // Metal2 terminal -> node its via pad reaches towards
    auto nodeAt = [&](Dbu x, Dbu y, int l) {
        auto ix = std::lower_bound(G.xs.begin(), G.xs.end(), x);
        auto iy = std::lower_bound(G.ys.begin(), G.ys.end(), y);
        if (ix == G.xs.end() || *ix != x || iy == G.ys.end() || *iy != y) return kNone;
        return G.id(static_cast<int>(ix - G.xs.begin()), static_cast<int>(iy - G.ys.begin()), l);
    };
    std::vector<std::vector<Pin>> pins_of(nl.netCount());
    std::vector<int> missing_pins(nl.netCount(), 0);
    auto missingAccess = [&](int net) {
        ++res.pins_without_access;
        ++missing_pins[net];
    };
    int owner_clash = 0;
    auto claim = [&](Node n, int net) {
        if (pin_owner[n] >= 0 && pin_owner[n] != net) { owner_clash++; return false; }
        pin_owner[n] = net;
        return true;
    };
    // Escapes: a Metal2 stub straight down from the pin, a Via2 on a Metal3 row below the macro, a
    // Metal3 patch to the nearest node (the terminal). Neighbouring pins take different rows
    // ("levels"), assigned left to right so that no two nets' Metal3 comes within spacing.
    struct Escape {
        Dbu px, top, yv;            // stub x, stub top (pin centre), via row
        std::vector<Node> extra;    // other Metal3 nodes the patch covers (metal of this net)
    };
    std::unordered_map<Node, Escape> escape_of;
    std::map<std::pair<size_t, size_t>, std::vector<Node>> escape_terms;   // (component, pin) -> terminal
    int escapes_failed = 0;
    {
        const ViaPick& v2 = via[1];
        const LefLayer& M3 = lef.layers[G.lef[2]];
        const Dbu half = v2.top.width() / 2, s3 = M3.spacing;
        const Dbu clear = 500;   // first row at least this far below the macro edge
        const Dbu stub_keep = v2.bottom.width() / 2 + lef.layers[G.lef[1]].width / 2 + space[1];
        for (size_t ci = 0; ci < fp.components.size(); ++ci) {
            const FpComponent& c = fp.components[ci];
            if (!c.placed || c.macro->cls != MacroClass::BLOCK) continue;
            const auto& inst = nl.instances[c.instance];
            struct Want { Dbu px, top; size_t k; int net; };
            std::vector<Want> wants;
            for (size_t k = 0; k < inst.pin_net.size(); ++k) {
                int net = inst.pin_net[k];
                int lp = c.macro->pinIndex(inst.cell->pins[k].name);
                if (net < 2 || lp < 0) continue;
                for (const LefShape& sh : c.macro->pins[lp].shapes) {
                    Rect r = placed(c, sh);
                    if (!escapeShape(c, r, layerOf(sh.layer))) continue;
                    wants.push_back({(r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2, k, net});
                    break;
                }
            }
            std::sort(wants.begin(), wants.end(), [](const Want& a, const Want& b) { return a.px < b.px; });
            int jtop = static_cast<int>(std::upper_bound(G.ys.begin(), G.ys.end(), c.y - clear) - G.ys.begin()) - 1;
            std::vector<std::vector<std::tuple<Dbu, Dbu, int>>> levels(24);   // (lo, hi, net) per level
            for (const Want& w : wants) {
                int xi = static_cast<int>(std::lower_bound(G.xs.begin(), G.xs.end(), w.px) - G.xs.begin());
                if (xi >= G.nx || (xi > 0 && w.px - G.xs[xi - 1] < G.xs[xi] - w.px)) xi--;
                Dbu xn = G.xs[xi];
                Dbu lo = std::min(w.px, xn) - half, hi = std::max(w.px, xn) + half;
                bool done = false;
                for (int L = 0; L < static_cast<int>(levels.size()) && !done; ++L) {
                    int j = jtop - L;
                    if (j < 0) break;
                    bool clash = false;
                    for (auto& [a, b, n] : levels[L]) clash |= n != w.net && a < hi + s3 && lo - s3 < b;
                    if (clash) continue;
                    // Metal3 nodes the patch covers or comes within spacing of: this net's metal
                    std::vector<Node> covered;
                    bool ok = true;
                    for (int i = std::max(0, xi - 2); i <= std::min(G.nx - 1, xi + 2); ++i) {
                        Dbu d = std::max<Dbu>(0, std::max(lo - G.xs[i], G.xs[i] - hi));
                        if (d >= half + s3) continue;
                        Node n = G.id(i, j, 2);
                        if ((blocked[n] && pin_owner[n] != w.net) || (pin_owner[n] >= 0 && pin_owner[n] != w.net)) { ok = false; break; }
                        covered.push_back(n);
                    }
                    if (!ok) continue;
                    Node term = G.id(xi, j, 2);
                    for (Node n : covered) claim(n, w.net);
                    Escape e{w.px, w.top, G.ys[j], {}};
                    for (Node n : covered) if (n != term) e.extra.push_back(n);
                    escape_of[term] = e;
                    escape_terms[{ci, w.k}].push_back(term);
                    levels[L].push_back({lo, hi, w.net});
                    // the stub blocks nearby Metal2 nodes for everyone
                    int i0, i1, j0, j1;
                    Grid::range(G.xs, w.px - stub_keep + 1, w.px + stub_keep - 1, i0, i1);
                    Grid::range(G.ys, G.ys[j] - half_along[1] - space[1], c.y, j0, j1);
                    for (int jj = j0; jj <= j1; ++jj) for (int i = i0; i <= i1; ++i) { blocked[G.id(i, jj, 1)] = 1; pin_owner[G.id(i, jj, 1)] = -1; }
                    done = true;
                }
                if (!done) escapes_failed++;
            }
        }
    }
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        const auto& inst = nl.instances[c.instance];
        bool is_block = c.macro->cls == MacroClass::BLOCK;
        const auto* acc = is_block ? nullptr : &accessOf(c, inst.cell);
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int net = inst.pin_net[k];
            if (net < 2) continue;
            Pin pin;
            int lp = c.macro->pinIndex(inst.cell->pins[k].name);
            if (lp < 0) { missingAccess(net); continue; }
            double sx = 0, sy = 0;
            int ns = 0;
            size_t ci = static_cast<size_t>(&c - fp.components.data());
            auto esc = is_block ? escape_terms.find({ci, k}) : escape_terms.end();
            if (esc != escape_terms.end()) pin.terms = esc->second;
            for (const LefShape& s : c.macro->pins[lp].shapes) {
                Rect r = placed(c, s);
                sx += (r.x0 + r.x1) / 2.0; sy += (r.y0 + r.y1) / 2.0; ns++;
                if (is_block && escapeShape(c, r, layerOf(s.layer))) continue;
                if (is_block) {
                    int l = layerOf(s.layer);
                    if (l < 0) continue;
                    int i0, i1, j0, j1;
                    Grid::range(G.xs, r.x0, r.x1, i0, i1);
                    Grid::range(G.ys, r.y0, r.y1, j0, j1);
                    for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                        Node n = G.id(i, j, l);
                        if (claim(n, net)) pin.terms.push_back(n);
                    }
                }
            }
            if (ns) { pin.cx = sx / ns; pin.cy = sy / ns; }
            if (!is_block) {
                for (const Access& a : (*acc)[k]) {
                    if (a.off) {
                        // terminal: the Metal2 node of track tx nearest to the via. When the via sits
                        // between two rows, the node on the other side is reserved for this net too
                        // (the via pad reaches towards it).
                        Dbu ax = c.x + a.dx, ay = c.y + a.dy, tx = c.x + a.tx;
                        auto ix = std::lower_bound(G.xs.begin(), G.xs.end(), tx);
                        if (ix == G.xs.end() || *ix != tx) continue;
                        int i = static_cast<int>(ix - G.xs.begin());
                        int j = static_cast<int>(std::upper_bound(G.ys.begin(), G.ys.end(), ay) - G.ys.begin()) - 1;
                        if (j < 0 || j >= G.ny) continue;
                        bool on_row = G.ys[j] == ay;
                        if (!on_row && j + 1 >= G.ny) continue;
                        int near = on_row || ay - G.ys[j] <= G.ys[j + 1] - ay ? j : j + 1, other = near == j ? j + 1 : j;
                        // attach on the side away from the cell's other access points
                        auto nextToOther = [&](int jj) {
                            for (size_t q = 0; q < acc->size(); ++q) {
                                if (q == k) continue;
                                for (const Access& b : (*acc)[q]) {
                                    if (c.x + b.tx != tx) continue;
                                    if (std::llabs(c.y + b.dy - G.ys[jj]) <= G.ys[1] - G.ys[0]) return true;
                                }
                            }
                            return false;
                        };
                        if (!on_row && nextToOther(near) && !nextToOther(other)) std::swap(near, other);
                        // strict: the partner node is free and reserved for this net; soft: another
                        // pin reserved it too, so it only counts as used once this access is taken
                        auto tryPair = [&](int tj, int pj, bool soft) {
                            Node tn = G.id(i, tj, 1);
                            if (blocked[tn] || (pin_owner[tn] >= 0 && pin_owner[tn] != net)) return false;
                            Node pn = kNone;
                            if (pj >= 0) {
                                pn = G.id(i, pj, 1);
                                if (blocked[pn] && pin_owner[pn] < 0) {
                                    // the node is blocked by an obstacle: fine if the via pad itself
                                    // keeps the spacing to every obstacle (no other net can use it)
                                    const ViaPick& vp = a.kind ? via1v : via[0];
                                    Rect pad{ax + vp.top.x0, ay + vp.top.y0, ax + vp.top.x1, ay + vp.top.y1};
                                    for (const Rect& o : obstacles[1]) {
                                        Dbu dx = std::max<Dbu>(0, std::max(pad.x0 - o.x1, o.x0 - pad.x1)), dy = std::max<Dbu>(0, std::max(pad.y0 - o.y1, o.y0 - pad.y1));
                                        if (dx * dx + dy * dy < space[1] * space[1]) return false;
                                    }
                                    pn = kNone;
                                } else {
                                    bool taken = pin_owner[pn] >= 0 && pin_owner[pn] != net;
                                    if (taken && !soft) return false;
                                    if (!taken) claim(pn, net);
                                }
                            }
                            claim(tn, net);
                            pin.terms.push_back(tn);
                            offgrid[tn] = {ax, ay, a.kind};
                            if (pn != kNone) partner_of[tn] = pn;
                            return true;
                        };
                        if (std::getenv("FASTPNR_TRACE_ACCESS")) {
                            for (int jj : {near, other}) {
                                Node q = G.id(i, jj, 1);
                                fprintf(stderr, "  claim %s/%s node y=%.3f blocked=%d owner=%d (net %d) partner_of=%d\n", inst.name.c_str(), inst.cell->pins[k].name.c_str(),
                                        G.ys[jj] / (double)lef.dbu, blocked[q], pin_owner[q], net, partner_of.count(q) ? 1 : 0);
                            }
                        }
                        if (on_row) tryPair(near, -1, false);
                        else if (!tryPair(near, other, false) && !tryPair(other, near, false) && !tryPair(near, other, true)) tryPair(other, near, true);
                        continue;
                    }
                    Node n = nodeAt(c.x + a.dx, c.y + a.dy, 0);
                    if (n == kNone) continue;
                    Node up = n + static_cast<Node>(static_cast<size_t>(G.nx) * G.ny);
                    if (blocked[up]) continue;
                    if (claim(n, net)) {
                        pin.terms.push_back(n);
                        access_kind[n] = a.kind;
                    }
                }
            }
            if (pin.terms.empty()) {
                missingAccess(net);
                if (std::getenv("FASTPNR_TRACE")) fprintf(stderr, "no access: %s/%s (%s, orient %s, %zu candidates in the cell)\n", inst.name.c_str(),
                                                          inst.cell->pins[k].name.c_str(), c.macro->name.c_str(), orientName(c.orient), acc ? (*acc)[k].size() : 0);
                continue;
            }
            pins_of[net].push_back(std::move(pin));
        }
    }
    for (const FpPin& p : fp.pins) {
        int net = nl.port_bits[p.port_bit].net;
        if (net < 2) continue;
        int l = layerOf(p.layer);
        if (l < 0) { missingAccess(net); continue; }
        Rect r{p.x + p.shape.x0, p.y + p.shape.y0, p.x + p.shape.x1, p.y + p.shape.y1};
        Pin pin;
        pin.cx = (r.x0 + r.x1) / 2.0;
        pin.cy = (r.y0 + r.y1) / 2.0;
        int i0, i1, j0, j1;
        Grid::range(G.xs, r.x0, r.x1, i0, i1);
        Grid::range(G.ys, r.y0, r.y1, j0, j1);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            Node n = G.id(i, j, l);
            if (claim(n, net)) pin.terms.push_back(n);
        }
        if (pin.terms.empty()) { missingAccess(net); continue; }
        pins_of[net].push_back(std::move(pin));
    }
    if (escapes_failed) res.notes.push_back(std::to_string(escapes_failed) + " macro pins found no free escape row");
    if (owner_clash) res.notes.push_back(std::to_string(owner_clash) + " pin nodes claimed by two nets");

    std::vector<RNet> nets;
    for (size_t n = 2; n < pins_of.size(); ++n) {
        if (pins_of[n].size() < 2) {
            // A net may have too few accessible terminals precisely because it is open.
            if (pins_of[n].size() + missing_pins[n] >= 2) ++res.unrouted;
            continue;
        }
        RNet r;
        r.netlist_net = static_cast<int>(n);
        r.pins = std::move(pins_of[n]);
        r.gx0 = r.gy0 = 1 << 30;
        r.gx1 = r.gy1 = -1;
        for (const Pin& p : r.pins) for (Node t : p.terms) {
            r.gx0 = std::min(r.gx0, G.I(t)); r.gx1 = std::max(r.gx1, G.I(t));
            r.gy0 = std::min(r.gy0, G.J(t)); r.gy1 = std::max(r.gy1, G.J(t));
            if (escape_of.count(t)) r.has_escape = true;
        }
        // Precompute Prim pin ordering once per net (pin positions never change across negotiation rounds)
        r.pin_order.reserve(r.pins.size());
        r.pin_order.push_back(0);
        std::vector<char> taken(r.pins.size(), 0);
        taken[0] = 1;
        // Only the last addition can lower an unvisited pin's distance to the tree.
        std::vector<double> nearest(r.pins.size(), 1e300);
        for (size_t k = 1; k < r.pins.size(); ++k) {
            const Pin& last = r.pins[r.pin_order.back()];
            int best = -1;
            double bd = 1e300;
            for (size_t q = 0; q < r.pins.size(); ++q) {
                if (taken[q]) continue;
                double d = std::fabs(r.pins[q].cx - last.cx) + std::fabs(r.pins[q].cy - last.cy);
                nearest[q] = std::min(nearest[q], d);
                if (nearest[q] < bd) { bd = nearest[q]; best = static_cast<int>(q); }
            }
            taken[best] = 1;
            r.pin_order.push_back(best);
        }
        nets.push_back(std::move(r));
    }
    // pin_owner now holds router net indices; pins of nets that are not routed stay off-limits
    {
        std::vector<int> id_of(nl.netCount(), -2);
        for (size_t k = 0; k < nets.size(); ++k) id_of[nets[k].netlist_net] = static_cast<int>(k);
        for (size_t n = 0; n < N; ++n) {
            if (pin_owner[n] >= 0) {
                int id = id_of[pin_owner[n]];
                pin_owner[n] = id;
                if (id < 0) blocked[n] = 1;
            }
        }
    }
    // Flat dense lookup tables replacing hash maps in inner routing & conflict loops
    std::vector<Node> partner_node(N, kNone);
    for (const auto& [k, v] : partner_of) partner_node[k] = v;
    std::vector<int> escape_idx(N, -1);
    std::vector<std::vector<Node>> escape_extras;
    escape_extras.reserve(escape_of.size());
    for (const auto& [k, e] : escape_of) {
        escape_idx[k] = static_cast<int>(escape_extras.size());
        escape_extras.push_back(e.extra);
    }
    std::vector<char> is_pad_node(N, 0);
    for (const auto& [k, _] : offgrid) is_pad_node[k] = 1;
    for (const auto& [k, _] : escape_of) is_pad_node[k] = 1;

    // Guides per netlist net, as gcell rectangles
    std::vector<int> guide_of(nl.netCount(), -1);
    for (size_t k = 0; k < gr.guides.size(); ++k) guide_of[gr.guides[k].first] = static_cast<int>(k);
    res.ms_setup = msSince(t0);

    // ------------------------------------------------ routing
    auto t1 = std::chrono::steady_clock::now();
    std::vector<int> occ(N, 0), who(N, -1), pocc(N, 0), pwho(N, -1);
    std::vector<float> hist(N, 0);
    const int ggx = std::max(1, gr.gx), ggy = std::max(1, gr.gy);
    const size_t plane = static_cast<size_t>(G.nx) * G.ny;
    const uint32_t plane32 = static_cast<uint32_t>(plane);
    const uint32_t nx32 = static_cast<uint32_t>(G.nx);

    // Per-thread search workspace (compact uint16_t stamps keep memory low for multi-thread & 10-way parallel runs)
    struct SearchCtx {
        std::vector<float> dist;
        std::vector<Node> parent;
        std::vector<uint16_t> seen, mark, target;
        std::vector<int> in_guide, in_layer_guide;
        FastMinHeap pq;
        std::vector<Node> path, tree;
        uint16_t search = 0, mark_stamp = 0;
        int guide_stamp = 0;
        long stat_pops = 0, stat_searches = 0, stat_fallbacks = 0;

        void init(size_t N, size_t gsize, size_t glsize) {
            dist.assign(N, 0.0f);
            parent.assign(N, kNone);
            seen.assign(N, 0);
            mark.assign(N, 0);
            target.assign(N, 0);
            in_guide.assign(gsize, -1);
            in_layer_guide.assign(glsize, -1);
            pq.h.reserve(4096);
            path.reserve(512);
            tree.reserve(512);
        }
        uint16_t nextSearch() {
            if (++search == 65535) {
                std::fill(seen.begin(), seen.end(), 0);
                std::fill(target.begin(), target.end(), 0);
                search = 1;
            }
            return search;
        }
        uint16_t nextMark() {
            if (++mark_stamp == 65535) {
                std::fill(mark.begin(), mark.end(), 0);
                mark_stamp = 1;
            }
            return mark_stamp;
        }
    };

    const int n_threads = std::max(1, opt.threads);
    std::vector<SearchCtx> ctxs;
    auto ensureWorkers = [&](int count) {
        while (static_cast<int>(ctxs.size()) < count) {
            ctxs.emplace_back();
            ctxs.back().init(N, static_cast<size_t>(ggx) * ggy, static_cast<size_t>(ggx) * ggy * G.nl);
        }
    };
    ensureWorkers(1);

    const bool trace_search = std::getenv("FASTPNR_TRACE_SEARCH") != nullptr;
    double pres = 1.0;
    const Dbu sx = G.xs[1] - G.xs[0], sy = G.ys[1] - G.ys[0];
    const double unit = static_cast<double>(std::min(sx, sy));
    // gcell of every (i, j), precomputed: the search asks for it on every step
    std::vector<int> gcell2d(plane);
    for (int j = 0; j < G.ny; ++j) {
        int gy = std::clamp(static_cast<int>(G.ys[j] / std::max<Dbu>(gr.gcell, 1)), 0, ggy - 1);
        for (int i = 0; i < G.nx; ++i) {
            int gx = std::clamp(static_cast<int>(G.xs[i] / std::max<Dbu>(gr.gcell, 1)), 0, ggx - 1);
            gcell2d[static_cast<size_t>(j) * G.nx + i] = gy * ggx + gx;
        }
    }
    auto alongNeighbours = [&](Node n, Node out[2]) {
        int l = G.Lr(n), i = G.I(n), j = G.J(n);
        int c = 0;
        if (l == 0) return 0;
        if (G.hor[l]) {
            if (i > 0) out[c++] = G.id(i - 1, j, l);
            if (i + 1 < G.nx) out[c++] = G.id(i + 1, j, l);
        } else {
            if (j > 0) out[c++] = G.id(i, j - 1, l);
            if (j + 1 < G.ny) out[c++] = G.id(i, j + 1, l);
        }
        return c;
    };
    auto usable = [&](Node n, int net) { return !blocked[n] ? (pin_owner[n] < 0 || pin_owner[n] == net) : pin_owner[n] == net; };
    // via cost between layer l and l + 1; going up to the top layer can cost more
    std::vector<double> via_cost_up(G.nl, opt.via_cost * unit);
    if (G.nl >= 2) via_cost_up[G.nl - 2] = opt.via_cost * opt.top_via_factor * unit;
    // cost of entering node n = (i, j, l); the caller knows the coordinates (no divisions here)
    auto nodeCostAt = [&](const SearchCtx& ctx, Node n, int i, int j, int l, int net, double base, bool all_guides) {
        double conflict = occ[n] > 0 && who[n] != net ? occ[n] : 0;
        if (pocc[n] > 0 && pwho[n] != net) conflict += pocc[n];
        if (l > 0) {
            if (G.hor[l]) {
                if (i > 0 && occ[n - 1] > 0 && who[n - 1] != net) conflict += 1;
                if (i + 1 < G.nx && occ[n + 1] > 0 && who[n + 1] != net) conflict += 1;
            } else {
                if (j > 0 && occ[n - G.nx] > 0 && who[n - G.nx] != net) conflict += 1;
                if (j + 1 < G.ny && occ[n + G.nx] > 0 && who[n + G.nx] != net) conflict += 1;
            }
        }
        // other nets' pin access points under a wire make them harder to reach
        if (l == 1) {
            int o = pin_owner[n - plane32];
            if (o >= 0 && o != net) conflict += opt.access_penalty;
        }
        double g = 1.0;
        if (!all_guides) {
            const size_t gc = gcell2d[static_cast<size_t>(j) * G.nx + i];
            g = ctx.in_layer_guide[gc * G.nl + l] == ctx.guide_stamp ? 1.0
              : ctx.in_guide[gc] == ctx.guide_stamp                  ? opt.other_layer
                                                                     : opt.outside_guide;
        }
        return base * g * (1.0 + hist[n]) * (1.0 + pres * conflict);
    };
    auto occupy = [&](SearchCtx& ctx, RNet& net, int id, Node n) {
        if (ctx.mark[n] == ctx.mark_stamp) return;
        ctx.mark[n] = ctx.mark_stamp;
        net.used.push_back(n);
        occ[n]++;
        who[n] = id;
        int ei = escape_idx[n];
        if (ei >= 0) {
            for (Node x : escape_extras[ei]) {
                net.partners.push_back(x);
                pocc[x]++;
                pwho[x] = id;
            }
        }
        Node pn = partner_node[n];
        if (pn != kNone) {
            net.partners.push_back(pn);
            pocc[pn]++;
            pwho[pn] = id;
        }
    };
    // A* from the tree (seen as sources) to any terminal of pin `p`
    auto connect = [&](SearchCtx& ctx, RNet& net, int id, const std::vector<Node>& tree, const Pin& p,
                       int i0, int j0, int i1, int j1, bool all_guides) {
        const uint16_t cur_search = ctx.nextSearch();
        int tx0 = 1 << 30, ty0 = 1 << 30, tx1 = -1, ty1 = -1;
        for (Node t : p.terms) {
            ctx.target[t] = cur_search;
            tx0 = std::min(tx0, G.I(t)); tx1 = std::max(tx1, G.I(t));
            ty0 = std::min(ty0, G.J(t)); ty1 = std::max(ty1, G.J(t));
        }
        auto hAt = [&](int i, int j) {
            double dx = i < tx0 ? G.xs[tx0] - G.xs[i] : i > tx1 ? G.xs[i] - G.xs[tx1] : 0;
            double dy = j < ty0 ? G.ys[ty0] - G.ys[j] : j > ty1 ? G.ys[j] - G.ys[ty1] : 0;
            return opt.astar_weight * (dx + dy);
        };
        auto h = [&](Node n) { return hAt(G.I(n), G.J(n)); };
        ctx.pq.clear();
        for (Node s : tree) {
            ctx.seen[s] = cur_search;
            ctx.dist[s] = 0.0f;
            ctx.parent[s] = kNone;
            ctx.pq.push({static_cast<float>(h(s)), 0.0f, s});
        }
        Node found = kNone;
        const Node layer_size = plane32;
        while (!ctx.pq.empty()) {
            AstarItem top = ctx.pq.pop();
            Node n = top.n;
            ctx.stat_pops++;
            if (top.d > ctx.dist[n]) continue;   // stale
            if (ctx.target[n] == cur_search) { found = n; break; }
            const uint32_t l = n / plane32;
            const uint32_t r2 = n - l * plane32;
            const uint32_t ju = r2 / nx32;
            const int i = static_cast<int>(r2 - ju * nx32);
            const int j = static_cast<int>(ju);
            const int li = static_cast<int>(l);
            struct Nb { Node m; int i, j, l; double base; };
            Nb nb[6];
            int k = 0;
            if (li > 0) {
                if (G.hor[li]) {
                    if (i > i0) nb[k++] = {n - 1, i - 1, j, li, static_cast<double>(G.xs[i] - G.xs[i - 1])};
                    if (i < i1) nb[k++] = {n + 1, i + 1, j, li, static_cast<double>(G.xs[i + 1] - G.xs[i])};
                } else {
                    if (j > j0) nb[k++] = {n - nx32, i, j - 1, li, static_cast<double>(G.ys[j] - G.ys[j - 1])};
                    if (j < j1) nb[k++] = {n + nx32, i, j + 1, li, static_cast<double>(G.ys[j + 1] - G.ys[j])};
                }
                if (li > 1 || usable(n - layer_size, id)) nb[k++] = {n - layer_size, i, j, li - 1, via_cost_up[li - 1]};
                if (opt.wrong_way > 0) {
                    if (G.hor[li]) {
                        if (j > j0) nb[k++] = {n - nx32, i, j - 1, li, opt.wrong_way * static_cast<double>(G.ys[j] - G.ys[j - 1])};
                        if (j < j1) nb[k++] = {n + nx32, i, j + 1, li, opt.wrong_way * static_cast<double>(G.ys[j + 1] - G.ys[j])};
                    } else {
                        if (i > i0) nb[k++] = {n - 1, i - 1, j, li, opt.wrong_way * static_cast<double>(G.xs[i] - G.xs[i - 1])};
                        if (i < i1) nb[k++] = {n + 1, i + 1, j, li, opt.wrong_way * static_cast<double>(G.xs[i + 1] - G.xs[i])};
                    }
                }
            }
            if (li + 1 < G.nl) nb[k++] = {n + layer_size, i, j, li + 1, via_cost_up[li]};
            for (int q = 0; q < k; ++q) {
                const Nb& e = nb[q];
                Node m = e.m;
                if (!usable(m, id)) continue;
                if (e.l == 0 && ctx.target[m] != cur_search) continue;   // Metal1 only at this pin's access points
                float c = static_cast<float>(ctx.dist[n] + nodeCostAt(ctx, m, e.i, e.j, e.l, id, e.base, all_guides));
                if (ctx.seen[m] == cur_search && ctx.dist[m] <= c) continue;
                ctx.seen[m] = cur_search;
                ctx.dist[m] = c;
                ctx.parent[m] = n;
                ctx.pq.push({c + static_cast<float>(hAt(e.i, e.j)), c, m});
            }
        }
        if (found == kNone) {
            if (trace_search) {
                int reached = 0;
                double best_h = 1e300;
                for (size_t q = 0; q < N; ++q) if (ctx.seen[q] == cur_search) { reached++; best_h = std::min(best_h, h(static_cast<Node>(q))); }
                fprintf(stderr, "   search failed: %d nodes reached, closest %.2f um from the pin, window %d..%d x %d..%d\n", reached, best_h / lef.dbu, i0, i1, j0, j1);
            }
            return false;
        }
        ctx.path.clear();
        for (Node n = found; n != kNone; n = ctx.parent[n]) ctx.path.push_back(n);
        for (size_t q = 0; q + 1 < ctx.path.size(); ++q) net.edges.push_back({ctx.path[q + 1], ctx.path[q]});
        for (Node n : ctx.path) occupy(ctx, net, id, n);
        return true;
    };
    auto ripUp = [&](RNet& net) {
        for (Node n : net.used) occ[n]--;
        for (Node n : net.partners) pocc[n]--;
        net.partners.clear();
        net.used.clear();
        net.edges.clear();
    };
    auto routeNet = [&](SearchCtx& ctx, int id, int margin, bool allow_fallback) -> bool {
        RNet& net = nets[id];
        ctx.nextMark();
        bool all_guides = (guide_of[net.netlist_net] < 0);
        if (!all_guides) {
            ctx.guide_stamp++;
            for (const auto& g : gr.guides[guide_of[net.netlist_net]].second) {
                int x0 = static_cast<int>(g.rect.x0 / gr.gcell) - 1, x1 = static_cast<int>((g.rect.x1 - 1) / gr.gcell) + 1;
                int y0 = static_cast<int>(g.rect.y0 / gr.gcell) - 1, y1 = static_cast<int>((g.rect.y1 - 1) / gr.gcell) + 1;
                for (int y = std::max(0, y0); y <= std::min(ggy - 1, y1); ++y)
                    for (int x = std::max(0, x0); x <= std::min(ggx - 1, x1); ++x) ctx.in_guide[static_cast<size_t>(y) * ggx + x] = ctx.guide_stamp;
                int gl = layerOf(g.layer);
                if (gl < 0) continue;
                int ax0 = static_cast<int>(g.rect.x0 / gr.gcell), ax1 = static_cast<int>((g.rect.x1 - 1) / gr.gcell);
                int ay0 = static_cast<int>(g.rect.y0 / gr.gcell), ay1 = static_cast<int>((g.rect.y1 - 1) / gr.gcell);
                for (int y = std::max(0, ay0); y <= std::min(ggy - 1, ay1); ++y)
                    for (int x = std::max(0, ax0); x <= std::min(ggx - 1, ax1); ++x)
                        ctx.in_layer_guide[(static_cast<size_t>(y) * ggx + x) * G.nl + gl] = ctx.guide_stamp;
            }
        }
        ctx.tree = net.pins[net.pin_order[0]].terms;
        net.ok = true;
        int mi = std::max(1, static_cast<int>(margin * gr.gcell / sx)), mj = std::max(1, static_cast<int>(margin * gr.gcell / sy));
        int i0 = std::max(0, net.gx0 - mi), i1 = std::min(G.nx - 1, net.gx1 + mi);
        int j0 = std::max(0, net.gy0 - mj), j1 = std::min(G.ny - 1, net.gy1 + mj);
        const int loc_mi = std::max(mi, static_cast<int>(2 * gr.gcell / sx));
        const int loc_mj = std::max(mj, static_cast<int>(2 * gr.gcell / sy));
        for (size_t k = 1; k < net.pin_order.size(); ++k) {
            const Pin& p = net.pins[net.pin_order[k]];
            bool ok = false;
            // Fast local pin-to-nearest-tree window first for multi-pin nets (avoids scanning huge net bboxes)
            if (net.pin_order.size() > 2 && !p.terms.empty() && !ctx.tree.empty()) {
                int pi = G.I(p.terms[0]), pj = G.J(p.terms[0]);
                int best_ti = G.I(ctx.tree[0]), best_tj = G.J(ctx.tree[0]);
                int best_d = std::abs(pi - best_ti) + std::abs(pj - best_tj);
                for (Node tn : ctx.tree) {
                    int ti = G.I(tn), tj = G.J(tn);
                    int d = std::abs(pi - ti) + std::abs(pj - tj);
                    if (d < best_d) { best_d = d; best_ti = ti; best_tj = tj; }
                }
                int li0 = std::max(i0, std::min(pi, best_ti) - loc_mi);
                int li1 = std::min(i1, std::max(pi, best_ti) + loc_mi);
                int lj0 = std::max(j0, std::min(pj, best_tj) - loc_mj);
                int lj1 = std::min(j1, std::max(pj, best_tj) + loc_mj);
                if ((li1 - li0 + 1) * (lj1 - lj0 + 1) < (i1 - i0 + 1) * (j1 - j0 + 1)) {
                    ok = connect(ctx, net, id, ctx.tree, p, li0, lj0, li1, lj1, all_guides);
                    ctx.stat_searches++;
                }
            }
            if (!ok) {
                ok = connect(ctx, net, id, ctx.tree, p, i0, j0, i1, j1, all_guides);
                ctx.stat_searches++;
            }
            if (!ok) {
                if (!allow_fallback) {
                    ripUp(net);
                    return false;
                }
                ctx.stat_fallbacks++;
                ok = connect(ctx, net, id, ctx.tree, p, 0, 0, G.nx - 1, G.ny - 1, all_guides);
            }
            if (!ok) { net.ok = false; continue; }
            ctx.tree = net.used;
        }
        return true;
    };

    // Route a list of nets: serially when n_threads == 1, or in disjoint spatial waves when n_threads > 1
    auto routeBatch = [&](const std::vector<int>& batch, int margin) {
        if (n_threads <= 1 || batch.size() < 4) {
            for (int id : batch) routeNet(ctxs[0], id, margin, true);
            return;
        }
        int mi = std::max(1, static_cast<int>(margin * gr.gcell / sx));
        int mj = std::max(1, static_cast<int>(margin * gr.gcell / sy));
        struct Box { int x0, y0, x1, y1, id; };
        std::vector<std::vector<Box>> waves;
        std::vector<int> serial_only;
        const long full_area = static_cast<long>(G.nx) * G.ny;
        for (int id : batch) {
            const RNet& net = nets[id];
            int x0 = std::max(0, net.gx0 - mi) - 2, x1 = std::min(G.nx - 1, net.gx1 + mi) + 2;
            int y0 = std::max(0, net.gy0 - mj) - 2, y1 = std::min(G.ny - 1, net.gy1 + mj) + 2;
            long area = static_cast<long>(x1 - x0 + 1) * (y1 - y0 + 1);
            if (net.has_escape || area * 3 > full_area) {
                serial_only.push_back(id);
                continue;
            }
            Box b{x0, y0, x1, y1, id};
            bool placed = false;
            for (auto& w : waves) {
                bool clash = false;
                for (const Box& o : w) {
                    if (!(b.x1 < o.x0 || o.x1 < b.x0 || b.y1 < o.y0 || o.y1 < b.y0)) { clash = true; break; }
                }
                if (!clash) { w.push_back(b); placed = true; break; }
            }
            if (!placed) waves.push_back({b});
        }
        for (const auto& w : waves) {
            if (w.size() < 4) {
                for (const Box& b : w) routeNet(ctxs[0], b.id, margin, true);
                continue;
            }
            int workers = std::min<int>(n_threads, static_cast<int>(w.size()));
            ensureWorkers(workers);
            std::atomic<size_t> next_idx{0};
            // Retry in wave order, independently of which worker finished first.
            std::vector<char> deferred(w.size(), 0);
            std::vector<std::thread> pool;
            pool.reserve(static_cast<size_t>(workers));
            for (int t = 0; t < workers; ++t) {
                pool.emplace_back([&, t]() {
                    for (;;) {
                        size_t idx = next_idx.fetch_add(1, std::memory_order_relaxed);
                        if (idx >= w.size()) break;
                        int id = w[idx].id;
                        if (!routeNet(ctxs[t], id, margin, false)) {
                            deferred[idx] = 1;
                        }
                    }
                });
            }
            for (auto& th : pool) th.join();
            for (size_t idx = 0; idx < w.size(); ++idx) {
                if (deferred[idx]) routeNet(ctxs[0], w[idx].id, margin, true);
            }
        }
        for (int id : serial_only) routeNet(ctxs[0], id, margin, true);
    };

    std::vector<int> order(nets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        long ha = (nets[a].gx1 - nets[a].gx0) + (nets[a].gy1 - nets[a].gy0), hb = (nets[b].gx1 - nets[b].gx0) + (nets[b].gy1 - nets[b].gy0);
        return ha != hb ? ha < hb : a < b;
    });
    routeBatch(order, 1);

    // Conflicts: a node used by two nets, or two nets on adjacent nodes of one track
    std::vector<char> pad(N, 0);
    std::vector<Dbu> pad_half(G.nl, 0), wire_half(G.nl, 0), exact_space(G.nl, 0);
    for (int l = 1; l < G.nl; ++l) {
        const LefLayer& L = lef.layers[G.lef[l]];
        wire_half[l] = std::max(L.width, res.wire_width[l]) / 2;
        exact_space[l] = L.spacing;
        for (int k : {l - 1, l}) {
            if (k < 0 || k + 1 >= G.nl) continue;
            const Rect& r = k == l - 1 ? via[k].top : via[k].bottom;
            pad_half[l] = std::max(pad_half[l], (G.hor[l] ? r.width() : r.height()) / 2);
        }
        if (l == 1 && have_via1v) pad_half[l] = std::max(pad_half[l], (G.hor[l] ? via1v.top.width() : via1v.top.height()) / 2);
    }
    std::vector<char> multi(N, 0);
    std::vector<int> pown(nl.netCount() > 0 ? N : 0, -1);
    auto conflicts = [&](std::vector<char>* bad_net) {
        std::fill(who.begin(), who.end(), -1);
        std::fill(multi.begin(), multi.end(), 0);
        for (size_t id = 0; id < nets.size(); ++id) {
            for (Node n : nets[id].used) {
                if (who[n] == -1) who[n] = static_cast<int>(id);
                else if (who[n] != static_cast<int>(id)) multi[n] = 1;
            }
        }
        long count = 0;
        // a reserved partner node that another net uses (or reserves) is a conflict for both
        std::fill(pown.begin(), pown.end(), -1);
        for (size_t id = 0; id < nets.size(); ++id) {
            for (Node n : nets[id].partners) {
                bool c = (who[n] >= 0 && who[n] != static_cast<int>(id)) || (pown[n] >= 0 && pown[n] != static_cast<int>(id));
                if (pown[n] >= 0 && pown[n] != static_cast<int>(id) && bad_net) (*bad_net)[pown[n]] = 1;
                pown[n] = static_cast<int>(id);
                if (c) {
                    count++;
                    if (bad_net) {
                        (*bad_net)[id] = 1;
                        if (who[n] >= 0) (*bad_net)[who[n]] = 1;
                        hist[n] += 1.0f;
                    }
                }
            }
        }
        // Certification using precomputed dense is_pad_node table (zero hash lookups)
        std::memcpy(pad.data(), is_pad_node.data(), N);
        for (size_t id = 0; id < nets.size(); ++id) {
            for (auto [a, b] : nets[id].edges) {
                if (G.Lr(a) != G.Lr(b)) { pad[a] = 1; pad[b] = 1; }
            }
        }
        auto legalPair = [&](Node a, Node b) {
            int l = G.Lr(a);
            if (l == 0) return false;
            Dbu step = G.hor[l] ? std::llabs(G.xs[G.I(a)] - G.xs[G.I(b)]) : std::llabs(G.ys[G.J(a)] - G.ys[G.J(b)]);
            Dbu gap = step - (pad[a] ? pad_half[l] : wire_half[l]) - (pad[b] ? pad_half[l] : wire_half[l]);
            return gap >= exact_space[l];
        };
        for (size_t id = 0; id < nets.size(); ++id) {
            for (Node n : nets[id].used) {
                bool c = multi[n];
                Node nb[2];
                int k = alongNeighbours(n, nb);
                for (int q = 0; q < k; ++q) c |= who[nb[q]] >= 0 && who[nb[q]] != static_cast<int>(id) && !legalPair(n, nb[q]);
                if (c) {
                    count++;
                    hist[n] += bad_net ? 1.0f : 0.0f;
                    if (bad_net) (*bad_net)[id] = 1;
                }
            }
        }
        return count;
    };
    const bool trace = std::getenv("FASTPNR_TRACE") != nullptr;
    auto sumStats = [&]() {
        long s = 0, f = 0, p = 0;
        for (const auto& c : ctxs) { s += c.stat_searches; f += c.stat_fallbacks; p += c.stat_pops; }
        return std::tuple<long, long, long>{s, f, p};
    };
    long best = conflicts(nullptr);
    if (trace) {
        auto [ss, sf, sp] = sumStats();
        fprintf(stderr, "dr initial conflicts %ld after %.0f ms (%ld searches, %ld fallbacks, %ld nodes expanded)\n", best, msSince(t1), ss, sf, sp);
    }
    int stall = 0;
    std::vector<int> victim_list;
    for (int it = 1; it <= opt.max_iterations && best > 0; ++it) {
        res.iterations = it;
        std::vector<char> bad(nets.size(), 0);
        conflicts(&bad);
        pres *= 1.5;
        if (stall >= 3) {
            std::vector<char> core = bad;
            for (size_t id = 0; id < nets.size(); ++id) {
                if (!core[id]) continue;
                for (Node n : nets[id].used) {
                    if (occ[n] < 2) {
                        Node nb[2];
                        int k = alongNeighbours(n, nb);
                        bool hot = false;
                        for (int q = 0; q < k; ++q) hot |= who[nb[q]] >= 0 && who[nb[q]] != static_cast<int>(id);
                        if (!hot) continue;
                    }
                    const int rad = std::min(5, 3 + 2 * ((stall - 3) / 3));
                    int ci = G.I(n), cj = G.J(n);
                    for (int l = 1; l < G.nl; ++l)
                        for (int dj = -rad; dj <= rad; ++dj)
                            for (int di = -rad; di <= rad; ++di) {
                                int i = ci + di, j = cj + dj;
                                if (i < 0 || j < 0 || i >= G.nx || j >= G.ny) continue;
                                Node m = G.id(i, j, l);
                                int w = who[m];
                                if (w >= 0) bad[w] = 1;
                                hist[m] += 0.25f;
                            }
                }
            }
        }
        victim_list.clear();
        for (int id : order) {
            if (!bad[id]) continue;
            ripUp(nets[id]);
            victim_list.push_back(id);
        }
        routeBatch(victim_list, 1 + it / 4);
        long c = conflicts(nullptr);
        if (trace) {
            auto [ss, sf, sp] = sumStats();
            fprintf(stderr, "dr iter %d victims %zu conflicts %ld at %.0f ms (%ld searches, %ld fallbacks, %ld nodes expanded)\n", it, victim_list.size(), c,
                    msSince(t1), ss, sf, sp);
        }
        if (c < best && !(victim_list.size() <= 30 && best - c <= 4 && c > 4)) { best = c; stall = 0; }
        else { if (c < best) best = c; if (++stall >= opt.patience) break; }
    }
    res.conflicts = conflicts(nullptr);
    res.ms_route = msSince(t1);

    // ------------------------------------------------ finishing: gaps, minimum area, output
    auto t2 = std::chrono::steady_clock::now();
    std::vector<int> mark(N, 0);
    int stamp = 0;
    auto occupyFinish = [&](RNet& net, int id, Node n) {
        if (mark[n] == stamp) return;
        mark[n] = stamp;
        net.used.push_back(n);
        occ[n]++;
        who[n] = id;
    };
    for (size_t id = 0; id < nets.size(); ++id) {
        RNet& net = nets[id];
        if (!net.ok) res.unrouted++;
        stamp++;
        for (Node n : net.used) mark[n] = stamp;
        // edges as a set
        std::map<std::pair<Node, Node>, char> eset;
        for (auto [a, b] : net.edges) eset[{std::min(a, b), std::max(a, b)}] = 1;
        // same-net nodes next to each other on a track are joined (no notch between them)
        for (Node n : net.used) {
            Node nb[2];
            int k = alongNeighbours(n, nb);
            for (int q = 0; q < k; ++q) if (mark[nb[q]] == stamp) eset[{std::min(n, nb[q]), std::max(n, nb[q])}] = 1;
        }
        if (std::getenv("FASTPNR_TRACE_NET") && nl.net_names[net.netlist_net] == std::getenv("FASTPNR_TRACE_NET")) {
            for (Node n : net.used) if (G.Lr(n) == 1) fprintf(stderr, "net node M2 (%.2f, %.2f) mark=%d\n", G.xs[G.I(n)] / 1000.0, G.ys[G.J(n)] / 1000.0, mark[n] == stamp);
            for (auto& [e, _] : eset) fprintf(stderr, "edge L%d (%.2f,%.2f)-L%d (%.2f,%.2f)\n", G.Lr(e.first), G.xs[G.I(e.first)] / 1000.0, G.ys[G.J(e.first)] / 1000.0, G.Lr(e.second),
                                              G.xs[G.I(e.second)] / 1000.0, G.ys[G.J(e.second)] / 1000.0);
        }
        // runs per track and layer
        auto runsOf = [&]() {
            std::map<std::pair<int, int>, std::vector<int>> pos;   // (layer, track) -> positions
            for (Node n : net.used) {
                int l = G.Lr(n);
                if (l == 0) continue;
                pos[{l, G.hor[l] ? G.J(n) : G.I(n)}].push_back(G.hor[l] ? G.I(n) : G.J(n));
            }
            std::vector<std::tuple<int, int, int, int>> runs;   // layer, track, lo, hi
            for (auto& [key, v] : pos) {
                std::sort(v.begin(), v.end());
                v.erase(std::unique(v.begin(), v.end()), v.end());
                int lo = v[0], prev = v[0];
                for (size_t q = 1; q <= v.size(); ++q) {
                    bool joined = false;
                    if (q < v.size() && v[q] == prev + 1) {
                        auto [l, t] = key;
                        Node a = G.hor[l] ? G.id(prev, t, l) : G.id(t, prev, l), b = G.hor[l] ? G.id(prev + 1, t, l) : G.id(t, prev + 1, l);
                        joined = eset.count({std::min(a, b), std::max(a, b)}) > 0;
                    }
                    if (q < v.size() && joined) { prev = v[q]; continue; }
                    runs.push_back({key.first, key.second, lo, prev});
                    if (q < v.size()) lo = prev = v[q];
                }
            }
            return runs;
        };
        // minimum area: extend short runs along their track into free nodes
        for (const auto& run : runsOf()) {
            const int l = std::get<0>(run), t = std::get<1>(run);
            int lo = std::get<2>(run), hi = std::get<3>(run);
            const LefLayer& L = lef.layers[G.lef[l]];
            if (L.min_area <= 0) continue;
            auto coord = [&](int p) { return static_cast<double>(G.hor[l] ? G.xs[p] : G.ys[p]); };
            auto nodeOf = [&](int p) { return G.hor[l] ? G.id(p, t, l) : G.id(t, p, l); };
            const int limit = G.hor[l] ? G.nx : G.ny;
            double need = L.min_area * lef.dbu * lef.dbu / L.width;   // DBU of length
            auto length = [&]() { return coord(hi) - coord(lo) + static_cast<double>(L.width); };   // DEF extends each end by half the width
            if (length() >= need) continue;
            auto freeFor = [&](int p) {
                if (p < 0 || p >= limit) return false;
                Node n = nodeOf(p);
                if (!usable(n, static_cast<int>(id)) || (occ[n] > 0 && who[n] != static_cast<int>(id))) return false;
                Node nb[2];
                int k = alongNeighbours(n, nb);
                for (int q = 0; q < k; ++q) if (occ[nb[q]] > 0 && who[nb[q]] != static_cast<int>(id)) return false;
                return true;
            };
            bool grew = true;
            while (length() < need && grew) {
                grew = false;
                for (int side : {1, -1}) {
                    if (length() >= need) break;
                    int p = side > 0 ? hi + 1 : lo - 1;
                    if (!freeFor(p)) continue;
                    Node a = nodeOf(side > 0 ? hi : lo), b = nodeOf(p);
                    net.edges.push_back({a, b});
                    eset[{std::min(a, b), std::max(a, b)}] = 1;
                    occupyFinish(net, static_cast<int>(id), b);
                    if (side > 0) hi = p; else lo = p;
                    grew = true;
                }
            }
            if (length() >= need) res.min_area_fixes++;
            else res.min_area_left++;
        }
        // output
        DrouteNet out;
        out.netlist_net = net.netlist_net;
        for (auto [l, t, lo, hi] : runsOf()) {
            if (lo == hi) continue;
            DrouteSegment s;
            s.layer = G.lef[l];
            if (G.hor[l]) { s.x0 = G.xs[lo]; s.x1 = G.xs[hi]; s.y0 = s.y1 = G.ys[t]; }
            else { s.y0 = G.ys[lo]; s.y1 = G.ys[hi]; s.x0 = s.x1 = G.xs[t]; }
            res.wirelength_um += static_cast<double>((s.x1 - s.x0) + (s.y1 - s.y0)) / lef.dbu;
            out.wires.push_back(s);
        }
        for (auto& [e, _] : eset) {
            auto [a, b] = e;
            if (G.Lr(a) == G.Lr(b)) {
                int l = G.Lr(a);
                bool across = G.hor[l] ? G.I(a) == G.I(b) : G.J(a) == G.J(b);   // a wrong-way jog
                if (l > 0 && across) {
                    DrouteSegment sg{G.lef[l], G.xs[G.I(a)], G.ys[G.J(a)], G.xs[G.I(b)], G.ys[G.J(b)]};
                    if (sg.x0 > sg.x1) std::swap(sg.x0, sg.x1);
                    if (sg.y0 > sg.y1) std::swap(sg.y0, sg.y1);
                    out.wires.push_back(sg);
                    res.wirelength_um += static_cast<double>((sg.x1 - sg.x0) + (sg.y1 - sg.y0)) / lef.dbu;
                }
                continue;
            }
            int l = std::min(G.Lr(a), G.Lr(b));
            const std::string& name = l == 0 && access_kind.count(std::min(a, b)) && access_kind[std::min(a, b)] == 1 ? via1v.name : via[l].name;
            out.vias.push_back({name, G.lef[l], G.xs[G.I(a)], G.ys[G.J(a)]});
        }
        // macro escapes: the Metal2 stub, the Via2 and the Metal3 patch to the terminal
        for (const Pin& p : net.pins) {
            for (Node t : p.terms) {
                auto it = escape_of.find(t);
                if (it == escape_of.end() || mark[t] != stamp) continue;
                const Escape& e = it->second;
                out.wires.push_back({G.lef[1], e.px, e.yv, e.px, e.top});
                out.vias.push_back({via[1].name, G.lef[1], e.px, e.yv});
                Dbu x = G.xs[G.I(t)];
                Rect a{e.px + via[1].top.x0, e.yv + via[1].top.y0, e.px + via[1].top.x1, e.yv + via[1].top.y1};
                Rect b{x + via[1].top.x0, e.yv + via[1].top.y0, x + via[1].top.x1, e.yv + via[1].top.y1};
                out.patches.push_back({G.lef[2], {std::min(a.x0, b.x0), a.y0, std::max(a.x1, b.x1), a.y1}});
                res.wirelength_um += static_cast<double>(e.top - e.yv + std::llabs(e.px - x)) / lef.dbu;
                break;
            }
        }
        // off-grid pin access: the Via1 and the Metal2 patch joining it to its node
        for (const Pin& p : net.pins) {
            for (Node t : p.terms) {
                auto it = offgrid.find(t);
                if (it == offgrid.end() || mark[t] != stamp) continue;
                Dbu x = G.xs[G.I(t)], y = G.ys[G.J(t)];
                const OffGrid& o = it->second;
                const ViaPick& vp = o.kind ? via1v : via[0];
                out.vias.push_back({vp.name, G.lef[0], o.x, o.y});
                // Metal2 patch over the via's pad and the node (pad-sized there too): no notch
                Rect a{o.x + vp.top.x0, o.y + vp.top.y0, o.x + vp.top.x1, o.y + vp.top.y1};
                Rect b{x + vp.top.x0, y + vp.top.y0, x + vp.top.x1, y + vp.top.y1};
                out.patches.push_back({G.lef[1], {std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1)}});
                res.wirelength_um += static_cast<double>(std::llabs(o.y - y) + std::llabs(o.x - x)) / lef.dbu;
                break;
            }
        }
        // Notches: two via pads on one track closer than the spacing, with a narrower wire
        // between them, are bridged by a patch as wide as the pads
        {
            std::map<std::pair<int, Dbu>, std::vector<std::pair<Dbu, Rect>>> pads;   // (grid layer, track) -> (position, pad)
            auto nearestTrack = [&](const std::vector<Dbu>& tr, Dbu c) {
                auto it = std::lower_bound(tr.begin(), tr.end(), c);
                if (it == tr.end() || (it != tr.begin() && c - *(it - 1) < *it - c)) --it;
                return *it;
            };
            auto addPad = [&](int l, Dbu x, Dbu y, const Rect& pr) {
                Rect r{x + pr.x0, y + pr.y0, x + pr.x1, y + pr.y1};
                if (G.hor[l]) pads[{l, nearestTrack(G.ys, y)}].push_back({r.x0, r});
                else pads[{l, nearestTrack(G.xs, x)}].push_back({r.y0, r});
            };
            // off-grid via patches already placed: filed under the track they join
            for (const auto& [layer, r] : std::vector<std::pair<int, Rect>>(out.patches)) {
                int l = -1;
                for (int q = 1; q < G.nl; ++q) if (G.lef[q] == layer) l = q;
                if (l < 0) continue;
                Dbu c = G.hor[l] ? (r.y0 + r.y1) / 2 : (r.x0 + r.x1) / 2;
                const std::vector<Dbu>& tr = G.hor[l] ? G.ys : G.xs;
                auto it = std::lower_bound(tr.begin(), tr.end(), c);
                if (it == tr.end() || (it != tr.begin() && c - *(it - 1) < *it - c)) --it;
                if (G.hor[l]) pads[{l, *it}].push_back({r.x0, r});
                else pads[{l, *it}].push_back({r.y0, r});
            }
            // wrong-way jog ends: a wire-wide piece on the track the jog leaves, so a pocket between
            // the jog and a nearby pad or off-track patch of the same net is bridged too (wires are
            // drawn as wide as the pads, so these pieces add no step of their own)
            for (auto& [e, _] : eset) {
                auto [a, b] = e;
                if (G.Lr(a) != G.Lr(b) || G.Lr(a) == 0) continue;
                int l = G.Lr(a);
                if (!(G.hor[l] ? G.I(a) == G.I(b) : G.J(a) == G.J(b))) continue;
                for (Node n : {a, b}) {
                    Dbu x = G.xs[G.I(n)], y = G.ys[G.J(n)], w = wire_half[l];
                    Rect r{x - w, y - w, x + w, y + w};
                    if (G.hor[l]) pads[{l, y}].push_back({r.x0, r});
                    else pads[{l, x}].push_back({r.y0, r});
                }
            }
            for (const DrouteVia& v : out.vias) {
                int lb = -1;
                for (int l = 0; l < G.nl; ++l) if (G.lef[l] == v.bottom) lb = l;
                if (lb < 0) continue;
                const ViaPick& vp = v.name == via1v.name ? via1v : via[lb];
                if (lb >= 1) addPad(lb, v.x, v.y, vp.bottom);
                if (lb + 1 < G.nl) addPad(lb + 1, v.x, v.y, vp.top);
            }
            for (auto& [key, list] : pads) {
                int l = key.first;
                std::sort(list.begin(), list.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                const LefLayer& L = lef.layers[G.lef[l]];
                Rect reach = list[0].second;   // the piece reaching furthest along the track so far
                for (size_t q = 0; q + 1 < list.size(); ++q) {
                    const Rect& cur = list[q].second;
                    if ((G.hor[l] ? cur.x1 : cur.y1) > (G.hor[l] ? reach.x1 : reach.y1)) reach = cur;
                    const Rect& a = reach;
                    const Rect& b = list[q + 1].second;
                    Dbu gap = G.hor[l] ? b.x0 - a.x1 : b.y0 - a.y1;
                    if (gap <= 0 || gap >= L.spacing) continue;
                    // start the bridge where the earliest of the pieces ending near `reach` ends,
                    // so a wider piece that stops just short leaves no sliver either
                    Dbu from = G.hor[l] ? a.x1 : a.y1;
                    Dbu cross0 = G.hor[l] ? std::min(a.y0, b.y0) : std::min(a.x0, b.x0), cross1 = G.hor[l] ? std::max(a.y1, b.y1) : std::max(a.x1, b.x1);
                    for (size_t r = 0; r <= q; ++r) {
                        const Rect& e = list[r].second;
                        Dbu end = G.hor[l] ? e.x1 : e.y1;
                        if (end < from && end > from - L.spacing) {
                            from = end;
                            cross0 = std::min(cross0, G.hor[l] ? e.y0 : e.x0);
                            cross1 = std::max(cross1, G.hor[l] ? e.y1 : e.x1);
                        }
                    }
                    Rect patch = G.hor[l] ? Rect{from, cross0, b.x0, cross1} : Rect{cross0, from, cross1, b.y0};
                    out.patches.push_back({G.lef[l], patch});
                }
            }
        }
        res.vias += static_cast<long>(out.vias.size());
        // connectivity: every pin reached by one component
        std::unordered_map<Node, Node> parent_of;
        std::function<Node(Node)> find = [&](Node x) {
            auto it = parent_of.find(x);
            if (it == parent_of.end() || it->second == x) return x;
            Node r = find(it->second);
            parent_of[x] = r;
            return r;
        };
        for (Node n : net.used) parent_of[n] = n;
        for (auto& [e, _] : eset) parent_of[find(e.first)] = find(e.second);
        Node root = kNone;
        bool connected = true;
        for (const Pin& p : net.pins) {
            Node r = kNone;
            for (Node tnode : p.terms) if (mark[tnode] == stamp) { r = find(tnode); break; }
            if (r == kNone) { connected = false; break; }
            if (root == kNone) root = r;
            else if (find(r) != find(root)) { connected = false; break; }
        }
        if ((!connected || missing_pins[net.netlist_net]) && net.ok) res.unrouted++;
        if (trace && !net.ok) {
            for (const Pin& p : net.pins) {
                Node t = p.terms[0];
                Node below = G.Lr(t) > 0 && G.J(t) > 0 ? t - G.nx : kNone;
                fprintf(stderr, "   pin at (%.3f, %.3f) %zu terms, first L%d (%.3f, %.3f) blocked=%d owner=%d; node below blocked=%d owner=%d\n", p.cx / lef.dbu,
                        p.cy / lef.dbu, p.terms.size(), G.Lr(t), G.xs[G.I(t)] / (double)lef.dbu, G.ys[G.J(t)] / (double)lef.dbu, blocked[t], pin_owner[t],
                        below == kNone ? -1 : blocked[below], below == kNone ? -9 : pin_owner[below]);
            }
        }
        if (trace && (!connected || missing_pins[net.netlist_net] || !net.ok))
            fprintf(stderr, "dr bad net %s: %zu pins, routed=%d connected=%d missing_pin=%d\n", nl.net_names[net.netlist_net].c_str(), net.pins.size(), net.ok,
                    connected, missing_pins[net.netlist_net]);
        res.nets.push_back(std::move(out));
    }
    res.conflicts = conflicts(nullptr);
    if (trace) {
        for (size_t id = 0; id < nets.size(); ++id) {
            for (Node n : nets[id].used) {
                Node nb[2];
                int k = alongNeighbours(n, nb);
                for (int q = 0; q < k; ++q) {
                    if (who[nb[q]] >= 0 && who[nb[q]] != static_cast<int>(id) && nb[q] > n) {
                        Node below_a = G.Lr(n) == 1 ? n - static_cast<Node>(static_cast<size_t>(G.nx) * G.ny) : kNone;
                        Node below_b = G.Lr(nb[q]) == 1 ? nb[q] - static_cast<Node>(static_cast<size_t>(G.nx) * G.ny) : kNone;
                        if (std::getenv("FASTPNR_TRACE_AROUND")) {
                            Node b = nb[q];
                            for (int dj = -1; dj <= 3; ++dj) for (int l = 1; l < G.nl; ++l) {
                                int j = G.J(b) + dj;
                                if (j < 0 || j >= G.ny) continue;
                                Node m = G.id(G.I(b), j, l);
                                fprintf(stderr, "    around L%d y=%.2f: blocked=%d owner=%d who=%d occ=%d\n", l, G.ys[j] / (double)lef.dbu, blocked[m], pin_owner[m], who[m], occ[m]);
                            }
                        }
                        fprintf(stderr, "conflict L%d (%.2f, %.2f) %s / %s; access below: %d %d\n", G.Lr(n), G.xs[G.I(n)] / (double)lef.dbu, G.ys[G.J(n)] / (double)lef.dbu,
                                nl.net_names[nets[id].netlist_net].c_str(), nl.net_names[nets[who[nb[q]]].netlist_net].c_str(),
                                below_a != kNone && pin_owner[below_a] == static_cast<int>(id), below_b != kNone && pin_owner[below_b] == who[nb[q]]);
                    }
                }
            }
        }
    }
    res.ms_finish = msSince(t2);
    return res;
}
