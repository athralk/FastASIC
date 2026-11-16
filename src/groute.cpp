// Global router: gcell capacity from real tracks, pattern routing, negotiated rip-up and reroute
// with A* maze routing, then layer assignment
#include "../include/groute.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace {

double msSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

struct Grid {
    int gx = 1, gy = 1;
    Dbu g = 1, ox = 0, oy = 0, x1 = 0, y1 = 0;
    Dbu cx0(int x) const { return ox + x * g; }
    Dbu cx1(int x) const { return x == gx - 1 ? x1 : ox + (x + 1) * g; }
    Dbu cy0(int y) const { return oy + y * g; }
    Dbu cy1(int y) const { return y == gy - 1 ? y1 : oy + (y + 1) * g; }
    double midX(int x) const { return (cx0(x) + cx1(x)) / 2.0; }
    double midY(int y) const { return (cy0(y) + cy1(y)) / 2.0; }
    int cellX(double x) const { return std::clamp(static_cast<int>(std::floor((x - ox) / static_cast<double>(g))), 0, gx - 1); }
    int cellY(double y) const { return std::clamp(static_cast<int>(std::floor((y - oy) / static_cast<double>(g))), 0, gy - 1); }
    int id(int x, int y) const { return y * gx + x; }
    int nh() const { return (gx - 1) * gy; }
    int nv() const { return gx * (gy - 1); }
    int hEdge(int x, int y) const { return y * (gx - 1) + x; }        // (x,y)-(x+1,y)
    int vEdge(int x, int y) const { return nh() + y * gx + x; }       // (x,y)-(x,y+1)
    bool horizontal(int e) const { return e < nh(); }
    // Edge between two adjacent cells
    int edge(int a, int b) const {
        int ax = a % gx, ay = a / gx, bx = b % gx, by = b / gx;
        if (ay == by) return hEdge(std::min(ax, bx), ay);
        return vEdge(ax, std::min(ay, by));
    }
    double length(int e) const {
        if (horizontal(e)) {
            int y = e / (gx - 1), x = e % (gx - 1);
            (void)y;
            return midX(x + 1) - midX(x);
        }
        int k = e - nh();
        int y = k / gx;
        return midY(y + 1) - midY(y);
    }
};

void orientOffset(Orient o, double w, double h, double& ox, double& oy) {
    if (o == Orient::FS || o == Orient::S) oy = h - oy;
    if (o == Orient::FN || o == Orient::S) ox = w - ox;
}

struct PinNode {
    int cell;    // gcell
    int layer;   // routing index of the pin's lowest shape layer
};

struct Net {
    int netlist_net;
    std::vector<PinNode> pins;
    std::vector<int> cells;   // distinct pin gcells
    int bx0, by0, bx1, by1;   // bounding box in gcells
    std::vector<int> edges;   // routed 2D edges
};

} // namespace

GrouteResult globalRoute(const Floorplan& fp, const CellNetlist& nl, const Lef& lef, const GrouteOptions& opt) {
    GrouteResult res;
    auto t0 = std::chrono::steady_clock::now();
    // ------------------------------------------------ layers and grid
    std::vector<int> routing = lef.routingLayers();
    int lmin = lef.layerIndex(opt.min_layer), lmax = lef.layerIndex(opt.max_layer);
    if (lmin < 0 || lmax < 0) throw std::runtime_error("routing layers " + opt.min_layer + "-" + opt.max_layer + " are not in the LEF");
    std::vector<int> layers;   // LEF layer indices used for routing, bottom up
    for (int l : routing) {
        if (l >= lmin && l <= lmax) layers.push_back(l);
    }
    if (layers.empty()) throw std::runtime_error("no routing layers between " + opt.min_layer + " and " + opt.max_layer);
    Dbu pitch = 0;
    for (int l : routing) {
        if (lef.layers[l].horizontal && lef.layers[l].routing_index > 0) { pitch = lef.layers[l].prefPitch(); break; }
    }
    if (pitch <= 0) pitch = lef.layers[layers[0]].prefPitch();
    Grid G;
    G.g = pitch * opt.gcell_tracks;
    G.ox = fp.die.x0;
    G.oy = fp.die.y0;
    G.x1 = fp.die.x1;
    G.y1 = fp.die.y1;
    G.gx = std::max<int>(1, static_cast<int>((fp.die.x1 - fp.die.x0) / G.g));
    G.gy = std::max<int>(1, static_cast<int>((fp.die.y1 - fp.die.y0) / G.g));
    res.gx = G.gx;
    res.gy = G.gy;
    res.gcell = G.g;
    const int NE = G.nh() + G.nv();
    const int NL = static_cast<int>(layers.size());

    // ------------------------------------------------ blockages per layer
    std::vector<std::vector<Rect>> block(NL);
    auto layerSlot = [&](int lef_layer) {
        for (int k = 0; k < NL; ++k) if (layers[k] == lef_layer) return k;
        return -1;
    };
    for (const FpSpecialWire& w : fp.special) {
        if (w.via >= 0) {
            for (const LefShape& s : fp.vias[w.via].shapes) {
                int k = layerSlot(s.layer);
                if (k >= 0) block[k].push_back({w.x0 + s.rect.x0, w.y0 + s.rect.y0, w.x0 + s.rect.x1, w.y0 + s.rect.y1});
            }
            continue;
        }
        int k = layerSlot(w.layer);
        if (k < 0) continue;
        if (w.y0 == w.y1) block[k].push_back({std::min(w.x0, w.x1), w.y0 - w.width / 2, std::max(w.x0, w.x1), w.y0 + w.width / 2});
        else block[k].push_back({w.x0 - w.width / 2, std::min(w.y0, w.y1), w.x0 + w.width / 2, std::max(w.y0, w.y1)});
    }
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        auto addShape = [&](const LefShape& s) {
            int k = layerSlot(s.layer);
            if (k < 0) return;
            double ax = s.rect.x0, ay = s.rect.y0, bx = s.rect.x1, by = s.rect.y1;
            orientOffset(c.orient, static_cast<double>(c.macro->width), static_cast<double>(c.macro->height), ax, ay);
            orientOffset(c.orient, static_cast<double>(c.macro->width), static_cast<double>(c.macro->height), bx, by);
            block[k].push_back({c.x + static_cast<Dbu>(std::min(ax, bx)), c.y + static_cast<Dbu>(std::min(ay, by)), c.x + static_cast<Dbu>(std::max(ax, bx)),
                                c.y + static_cast<Dbu>(std::max(ay, by))});
        };
        for (const LefShape& s : c.macro->obs) addShape(s);
        for (const LefPin& p : c.macro->pins) for (const LefShape& s : p.shapes) addShape(s);
    }

    // ------------------------------------------------ capacity per layer and edge
    std::vector<std::vector<int>> cap(NL, std::vector<int>(NE, 0)), orig(NL, std::vector<int>(NE, 0));
    for (int k = 0; k < NL; ++k) {
        const LefLayer& L = lef.layers[layers[k]];
        bool hor = L.horizontal;
        // the layer's preferred-direction tracks
        std::vector<Dbu> tracks;
        for (const FpTracks& t : fp.tracks) {
            if (t.layer != layers[k] || t.x == hor) continue;   // horizontal wires run on Y tracks
            for (int i = 0; i < t.count; ++i) tracks.push_back(t.start + i * t.step);
        }
        std::sort(tracks.begin(), tracks.end());
        tracks.erase(std::unique(tracks.begin(), tracks.end()), tracks.end());
        // blocked intervals along each track
        std::vector<std::vector<std::pair<Dbu, Dbu>>> blocked(tracks.size());
        Dbu halo = L.spacing + L.width / 2;
        for (const Rect& r : block[k]) {
            Dbu lo = (hor ? r.y0 : r.x0) - halo, hi = (hor ? r.y1 : r.x1) + halo;
            auto it = std::upper_bound(tracks.begin(), tracks.end(), lo);
            for (; it != tracks.end() && *it < hi; ++it) {
                blocked[it - tracks.begin()].push_back({hor ? r.x0 : r.y0, hor ? r.x1 : r.y1});
            }
        }
        for (auto& b : blocked) std::sort(b.begin(), b.end());
        for (size_t t = 0; t < tracks.size(); ++t) {
            Dbu pos = tracks[t];
            int row = hor ? G.cellY(static_cast<double>(pos)) : G.cellX(static_cast<double>(pos));
            int along = hor ? G.gx : G.gy;
            const auto& b = blocked[t];
            size_t bi = 0;
            for (int a = 0; a + 1 < along; ++a) {
                // A track is usable across this edge when it is free where it crosses the
                // boundary between the two gcells
                double bnd = hor ? static_cast<double>(G.cx1(a)) : static_cast<double>(G.cy1(a));
                double s0 = bnd - L.width / 2.0 - L.spacing, s1 = bnd + L.width / 2.0 + L.spacing;
                int e = hor ? G.hEdge(a, row) : G.vEdge(row, a);
                orig[k][e]++;
                while (bi < b.size() && b[bi].second <= s0) bi++;
                bool free = true;
                for (size_t q = bi; q < b.size() && b[q].first < s1; ++q) {
                    if (b[q].second > s0) { free = false; break; }
                }
                if (free) cap[k][e]++;
            }
        }
        for (int e = 0; e < NE; ++e) cap[k][e] = static_cast<int>(std::floor(cap[k][e] * (1.0 - opt.adjustment) + 1e-9));
    }
    std::vector<int> cap2(NE, 0);
    for (int k = 0; k < NL; ++k) for (int e = 0; e < NE; ++e) cap2[e] += cap[k][e];
    res.ms_capacity = msSince(t0);

    // ------------------------------------------------ nets
    auto t1 = std::chrono::steady_clock::now();
    std::unordered_map<const LibCell*, std::vector<std::pair<std::pair<double, double>, int>>> pin_cache;   // offset, routing index
    auto pinInfo = [&](const LibCell* cell, const LefMacro* m) -> const std::vector<std::pair<std::pair<double, double>, int>>& {
        auto it = pin_cache.find(cell);
        if (it != pin_cache.end()) return it->second;
        std::vector<std::pair<std::pair<double, double>, int>> v(cell->pins.size(), {{m->width / 2.0, m->height / 2.0}, 0});
        for (size_t k = 0; k < cell->pins.size(); ++k) {
            int p = m->pinIndex(cell->pins[k].name);
            if (p < 0 || m->pins[p].shapes.empty()) continue;
            double sx = 0, sy = 0;
            int low = 1 << 20;
            for (const LefShape& s : m->pins[p].shapes) {
                sx += (s.rect.x0 + s.rect.x1) / 2.0;
                sy += (s.rect.y0 + s.rect.y1) / 2.0;
                low = std::min(low, lef.layers[s.layer].routing_index);
            }
            v[k] = {{sx / m->pins[p].shapes.size(), sy / m->pins[p].shapes.size()}, std::max(low, 0)};
        }
        return pin_cache.emplace(cell, std::move(v)).first->second;
    };
    std::vector<std::vector<PinNode>> by_net(nl.netCount());
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        const auto& inst = nl.instances[c.instance];
        const auto& info = pinInfo(inst.cell, c.macro);
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int net = inst.pin_net[k];
            if (net < 2) continue;
            double ox = info[k].first.first, oy = info[k].first.second;
            orientOffset(c.orient, static_cast<double>(c.macro->width), static_cast<double>(c.macro->height), ox, oy);
            by_net[net].push_back({G.id(G.cellX(c.x + ox), G.cellY(c.y + oy)), info[k].second});
        }
    }
    for (const FpPin& p : fp.pins) {
        int net = nl.port_bits[p.port_bit].net;
        if (net < 2) continue;
        double x = p.x + (p.shape.x0 + p.shape.x1) / 2.0, y = p.y + (p.shape.y0 + p.shape.y1) / 2.0;
        by_net[net].push_back({G.id(G.cellX(x), G.cellY(y)), lef.layers[p.layer].routing_index});
    }
    std::vector<Net> nets;
    for (size_t n = 2; n < by_net.size(); ++n) {
        if (by_net[n].size() < 2) continue;
        Net net;
        net.netlist_net = static_cast<int>(n);
        net.pins = by_net[n];
        for (const PinNode& p : net.pins) net.cells.push_back(p.cell);
        std::sort(net.cells.begin(), net.cells.end());
        net.cells.erase(std::unique(net.cells.begin(), net.cells.end()), net.cells.end());
        net.bx0 = net.by0 = 1 << 30;
        net.bx1 = net.by1 = -1;
        for (int c : net.cells) {
            net.bx0 = std::min(net.bx0, c % G.gx); net.bx1 = std::max(net.bx1, c % G.gx);
            net.by0 = std::min(net.by0, c / G.gx); net.by1 = std::max(net.by1, c / G.gx);
        }
        if (net.cells.size() == 1) {
            res.local_nets++;
            continue;
        }
        nets.push_back(std::move(net));
    }
    res.routed_nets = static_cast<int>(nets.size());
    std::vector<int> order(nets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        int ha = (nets[a].bx1 - nets[a].bx0) + (nets[a].by1 - nets[a].by0), hb = (nets[b].bx1 - nets[b].bx0) + (nets[b].by1 - nets[b].by0);
        return ha != hb ? ha < hb : a < b;
    });

    // ------------------------------------------------ 2D routing
    std::vector<int> use(NE, 0);
    std::vector<double> hist(NE, 0);
    double pres = 0.5;
    auto edgeCost = [&](int e) {
        double over = use[e] + 1 - cap2[e];
        double p = over > 0 ? 1 + pres * over : 1;
        return (1 + hist[e]) * p;
    };
    // tree membership, stamped per net
    std::vector<int> in_tree(static_cast<size_t>(G.gx) * G.gy, -1);
    int tree_stamp = 0;
    std::vector<int> tree;
    auto addEdge = [&](Net& net, int a, int b) {
        int e = G.edge(a, b);
        net.edges.push_back(e);
        use[e]++;
    };
    auto addCell = [&](int c) {
        if (in_tree[c] != tree_stamp) { in_tree[c] = tree_stamp; tree.push_back(c); }
    };
    // Cells of an L route a -> corner -> b (corner = (bx, ay) when x_first)
    auto lPath = [&](int a, int b, bool x_first, std::vector<int>& path) {
        path.clear();
        int ax = a % G.gx, ay = a / G.gx, bx = b % G.gx, by = b / G.gx;
        int x = ax, y = ay;
        path.push_back(G.id(x, y));
        auto stepX = [&]() { while (x != bx) { x += bx > x ? 1 : -1; path.push_back(G.id(x, y)); } };
        auto stepY = [&]() { while (y != by) { y += by > y ? 1 : -1; path.push_back(G.id(x, y)); } };
        if (x_first) { stepX(); stepY(); } else { stepY(); stepX(); }
    };
    auto pathCost = [&](const std::vector<int>& path) {
        double c = 0;
        for (size_t k = 0; k + 1 < path.size(); ++k) c += edgeCost(G.edge(path[k], path[k + 1]));
        int ax = path.front() % G.gx, ay = path.front() / G.gx, bx = path.back() % G.gx, by = path.back() / G.gx;
        if (ax != bx && ay != by) c += opt.via_cost;
        return c;
    };
    // A path may run along edges the net already uses: those are not counted twice
    std::vector<int> edge_mark(NE, -1);
    auto commitPath = [&](Net& net, const std::vector<int>& path) {
        for (size_t k = 0; k + 1 < path.size(); ++k) {
            int e = G.edge(path[k], path[k + 1]);
            if (edge_mark[e] == tree_stamp) continue;
            edge_mark[e] = tree_stamp;
            addEdge(net, path[k], path[k + 1]);
        }
        for (int c : path) addCell(c);
    };
    // Prim order: each next pin is the one closest to the tree so far
    auto primOrder = [&](const Net& net) {
        std::vector<int> out;
        std::vector<int> rest = net.cells;
        out.push_back(rest.front());
        rest.erase(rest.begin());
        std::vector<int> dist(rest.size());
        auto md = [&](int a, int b) { return std::abs(a % G.gx - b % G.gx) + std::abs(a / G.gx - b / G.gx); };
        for (size_t k = 0; k < rest.size(); ++k) dist[k] = md(rest[k], out[0]);
        while (!rest.empty()) {
            size_t best = 0;
            for (size_t k = 1; k < rest.size(); ++k) if (dist[k] < dist[best]) best = k;
            int c = rest[best];
            out.push_back(c);
            rest[best] = rest.back(); rest.pop_back();
            dist[best] = dist.back(); dist.pop_back();
            for (size_t k = 0; k < rest.size(); ++k) dist[k] = std::min(dist[k], md(rest[k], c));
        }
        return out;
    };
    std::vector<int> path, path2;
    auto patternRoute = [&](Net& net) {
        tree_stamp++;
        tree.clear();
        std::vector<int> pins = primOrder(net);
        addCell(pins[0]);
        for (size_t k = 1; k < pins.size(); ++k) {
            int p = pins[k];
            if (in_tree[p] == tree_stamp) continue;
            int best = tree[0], bd = 1 << 30;
            for (int t : tree) {
                int d = std::abs(t % G.gx - p % G.gx) + std::abs(t / G.gx - p / G.gx);
                if (d < bd) { bd = d; best = t; }
            }
            lPath(best, p, true, path);
            lPath(best, p, false, path2);
            commitPath(net, pathCost(path) <= pathCost(path2) ? path : path2);
        }
    };
    for (int i : order) patternRoute(nets[i]);
    auto totalOverflow = [&]() {
        long o = 0;
        for (int e = 0; e < NE; ++e) o += std::max(0, use[e] - cap2[e]);
        return o;
    };
    res.ms_pattern = msSince(t1);

    // ------------------------------------------------ negotiated rip-up and reroute (A* maze)
    auto t2 = std::chrono::steady_clock::now();
    const size_t NC = static_cast<size_t>(G.gx) * G.gy;
    std::vector<double> dist(NC * 2, 0);
    std::vector<int> parent(NC * 2, -1), seen(NC * 2, -1);
    int search = 0;
    auto mazeConnect = [&](Net& net, int target, int x0, int y0, int x1, int y1) {
        // states: cell * 2 + (0 arrived horizontally, 1 vertically); tree cells are sources
        search++;
        using Item = std::pair<double, int>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
        int tx = target % G.gx, ty = target / G.gx;
        auto h = [&](int c) { return static_cast<double>(std::abs(c % G.gx - tx) + std::abs(c / G.gx - ty)); };
        for (int c : tree) {
            int cx = c % G.gx, cy = c / G.gx;
            if (cx < x0 || cx > x1 || cy < y0 || cy > y1) continue;   // seeds: tree cells inside the window
            for (int d = 0; d < 2; ++d) {
                int s = c * 2 + d;
                seen[s] = search;
                dist[s] = 0;
                parent[s] = -1;
                pq.push({h(c), s});
            }
        }
        int found = -1;
        while (!pq.empty()) {
            auto [f, s] = pq.top();
            pq.pop();
            int c = s / 2, d = s % 2;
            if (f - h(c) > dist[s] + 1e-9) continue;
            if (c == target) { found = s; break; }
            int cx = c % G.gx, cy = c / G.gx;
            const int nx[4] = {cx + 1, cx - 1, cx, cx};
            const int ny[4] = {cy, cy, cy + 1, cy - 1};
            for (int k = 0; k < 4; ++k) {
                if (nx[k] < x0 || nx[k] > x1 || ny[k] < y0 || ny[k] > y1) continue;
                int nc = G.id(nx[k], ny[k]);
                int nd = k < 2 ? 0 : 1;
                double cost = dist[s] + edgeCost(G.edge(c, nc)) + (parent[s] >= 0 && nd != d ? opt.via_cost : 0);
                int ns = nc * 2 + nd;
                if (seen[ns] == search && dist[ns] <= cost) continue;
                seen[ns] = search;
                dist[ns] = cost;
                parent[ns] = s;
                pq.push({cost + h(nc), ns});
            }
        }
        if (found < 0) return false;
        path.clear();
        for (int s = found; s >= 0; s = parent[s]) path.push_back(s / 2);
        std::reverse(path.begin(), path.end());
        commitPath(net, path);
        return true;
    };
    auto ripUp = [&](Net& net) {
        for (int e : net.edges) use[e]--;
        net.edges.clear();
    };
    auto mazeRoute = [&](Net& net, int margin) {
        tree_stamp++;
        tree.clear();
        std::vector<int> pins = primOrder(net);
        addCell(pins[0]);
        int x0 = std::max(0, net.bx0 - margin), y0 = std::max(0, net.by0 - margin);
        int x1 = std::min(G.gx - 1, net.bx1 + margin), y1 = std::min(G.gy - 1, net.by1 + margin);
        for (size_t k = 1; k < pins.size(); ++k) {
            int p = pins[k];
            if (in_tree[p] == tree_stamp) continue;
            // First a local search: the box spanned by the pin and its nearest tree cell, plus a
            // margin; then the net's whole window; then the whole grid
            int near = tree[0], nd = 1 << 30;
            for (int t : tree) {
                int d = std::abs(t % G.gx - p % G.gx) + std::abs(t / G.gx - p / G.gx);
                if (d < nd) { nd = d; near = t; }
            }
            int lx0 = std::max(x0, std::min(p % G.gx, near % G.gx) - margin), lx1 = std::min(x1, std::max(p % G.gx, near % G.gx) + margin);
            int ly0 = std::max(y0, std::min(p / G.gx, near / G.gx) - margin), ly1 = std::min(y1, std::max(p / G.gx, near / G.gx) + margin);
            if (mazeConnect(net, p, lx0, ly0, lx1, ly1)) continue;
            if (!mazeConnect(net, p, x0, y0, x1, y1)) mazeConnect(net, p, 0, 0, G.gx - 1, G.gy - 1);
        }
    };
    long best = totalOverflow();
    int stall = 0;
    const bool trace = std::getenv("FASTPNR_TRACE") != nullptr;
    if (trace) fprintf(stderr, "gr pattern overflow %ld\n", best);
    for (int it = 1; it <= opt.max_iterations && best > 0; ++it) {
        res.iterations = it;
        std::vector<char> over(NE, 0);
        for (int e = 0; e < NE; ++e) {
            if (use[e] > cap2[e]) {
                over[e] = 1;
                hist[e] += 1.0;
            }
        }
        pres = 0.5 * std::pow(1.5, it);
        std::vector<int> victims;
        for (int i : order) {
            for (int e : nets[i].edges) {
                if (over[e]) { victims.push_back(i); break; }
            }
        }
        int margin = 2 + it / 2;
        for (int i : victims) {
            ripUp(nets[i]);
            mazeRoute(nets[i], margin);
        }
        long o = totalOverflow();
        if (trace) fprintf(stderr, "gr iter %d victims %zu overflow %ld\n", it, victims.size(), o);
        if (o < best) { best = o; stall = 0; }
        else if (++stall >= 10) break;
    }
    res.overflow_2d = totalOverflow();
    res.ms_negotiation = msSince(t2);

    // ------------------------------------------------ layer assignment
    auto t3 = std::chrono::steady_clock::now();
    std::vector<std::vector<int>> luse(NL, std::vector<int>(NE, 0));
    std::vector<int> hlayers, vlayers;
    for (int k = 0; k < NL; ++k) (lef.layers[layers[k]].horizontal ? hlayers : vlayers).push_back(k);
    std::vector<int> mark(NE, -1), node_lo(NC, 1 << 20), node_hi(NC, -1), node_stamp(NC, -1);
    int stamp = 0;
    for (int i : order) {
        Net& net = nets[i];
        stamp++;
        // adjacency of the net's tree (edges may repeat after reroutes: keep each once)
        std::vector<int> edges;
        for (int e : net.edges) {
            if (mark[e] != stamp) { mark[e] = stamp; edges.push_back(e); }
        }
        std::unordered_map<int, std::vector<int>> adj;   // cell -> incident edges
        auto ends = [&](int e, int& a, int& b) {
            if (G.horizontal(e)) { int y = e / (G.gx - 1), x = e % (G.gx - 1); a = G.id(x, y); b = G.id(x + 1, y); }
            else { int k = e - G.nh(); int y = k / G.gx, x = k % G.gx; a = G.id(x, y); b = G.id(x, y + 1); }
        };
        for (int e : edges) {
            int a, b;
            ends(e, a, b);
            adj[a].push_back(e);
            adj[b].push_back(e);
        }
        std::vector<char> is_pin_cell;
        std::unordered_map<int, int> pin_lo;
        for (const PinNode& p : net.pins) {
            auto it = pin_lo.find(p.cell);
            if (it == pin_lo.end()) pin_lo[p.cell] = p.layer;
            else it->second = std::min(it->second, p.layer);
        }
        auto touch = [&](int c, int ri) {
            if (node_stamp[c] != stamp) { node_stamp[c] = stamp; node_lo[c] = 1 << 20; node_hi[c] = -1; }
            node_lo[c] = std::min(node_lo[c], ri);
            node_hi[c] = std::max(node_hi[c], ri);
        };
        std::vector<GrouteResult::Guide> guides;
        std::unordered_map<int, char> done;
        for (int e0 : edges) {
            if (done.count(e0)) continue;
            // grow a straight run through cells with exactly two collinear tree edges and no pin
            bool hor = G.horizontal(e0);
            std::vector<int> run{e0};
            done[e0] = 1;
            for (int side = 0; side < 2; ++side) {
                int a, b;
                ends(e0, a, b);
                int cur = side ? b : a;
                int last = e0;
                while (true) {
                    const auto& inc = adj[cur];
                    if (inc.size() != 2 || pin_lo.count(cur)) break;
                    int nxt = inc[0] == last ? inc[1] : inc[0];
                    if (G.horizontal(nxt) != hor || done.count(nxt)) break;
                    run.push_back(nxt);
                    done[nxt] = 1;
                    int p, q;
                    ends(nxt, p, q);
                    cur = p == cur ? q : p;
                    last = nxt;
                }
            }
            const std::vector<int>& cands = hor ? hlayers : vlayers;
            if (cands.empty()) continue;
            // Best single layer for the whole run; if every layer overflows somewhere along it,
            // the run is split and each edge takes its own least-used layer
            auto runCost = [&](int k, const std::vector<int>& es) {
                double c = 0;
                for (int e : es) {
                    int over = luse[k][e] + 1 - cap[k][e];
                    c += over > 0 ? 1000.0 * over : static_cast<double>(luse[k][e]) / std::max(1, cap[k][e]);
                }
                return c + 0.01 * k;   // ties: the lower layer
            };
            auto assign = [&](const std::vector<int>& es) {
                int best_k = cands[0];
                double best_cost = 1e300;
                for (int k : cands) {
                    double c = runCost(k, es);
                    if (c < best_cost) { best_cost = c; best_k = k; }
                }
                return std::make_pair(best_k, best_cost);
            };
            std::vector<std::vector<int>> pieces;
            if (assign(run).second >= 1000.0 && run.size() > 1) {
                for (int e : run) pieces.push_back({e});
            } else {
                pieces.push_back(run);
            }
            for (const auto& piece : pieces) {
                int best_k = assign(piece).first;
                int ri = lef.layers[layers[best_k]].routing_index;
                int cx0 = 1 << 30, cy0 = 1 << 30, cx1 = -1, cy1 = -1;
                for (int e : piece) {
                    luse[best_k][e]++;
                    int a, b;
                    ends(e, a, b);
                    touch(a, ri);
                    touch(b, ri);
                    for (int c : {a, b}) {
                        cx0 = std::min(cx0, c % G.gx); cx1 = std::max(cx1, c % G.gx);
                        cy0 = std::min(cy0, c / G.gx); cy1 = std::max(cy1, c / G.gx);
                    }
                }
                guides.push_back({layers[best_k], {G.cx0(cx0), G.cy0(cy0), G.cx1(cx1), G.cy1(cy1)}});
            }
        }
        for (auto& [c, lo] : pin_lo) touch(c, lo);
        // vias: at every node, from its lowest to its highest layer
        std::vector<int> nodes;
        for (auto& [c, inc] : adj) nodes.push_back(c);
        for (auto& [c, lo] : pin_lo) if (!adj.count(c)) nodes.push_back(c);
        for (int c : nodes) {
            if (node_stamp[c] != stamp || node_hi[c] < 0) continue;
            res.vias += node_hi[c] - node_lo[c];
            for (int ri = node_lo[c]; ri <= node_hi[c]; ++ri) {
                if (ri >= static_cast<int>(routing.size())) break;
                Rect r{G.cx0(c % G.gx), G.cy0(c / G.gx), G.cx1(c % G.gx), G.cy1(c / G.gx)};
                guides.push_back({routing[ri], r});
            }
        }
        for (int e : edges) res.wirelength_um += G.length(e) / lef.dbu;
        res.guides.push_back({net.netlist_net, std::move(guides)});
    }
    for (int k = 0; k < NL; ++k) {
        GrouteLayerReport r;
        r.name = lef.layers[layers[k]].name;
        r.horizontal = lef.layers[layers[k]].horizontal;
        for (int e = 0; e < NE; ++e) {
            if (G.horizontal(e) != r.horizontal) continue;
            r.resource_original += orig[k][e];
            r.resource += cap[k][e];
            r.demand += luse[k][e];
            int o = luse[k][e] - cap[k][e];
            if (o > 0) { r.overflow += o; r.max_overflow = std::max(r.max_overflow, o); }
        }
        res.layers.push_back(r);
    }
    res.ms_layers = msSince(t3);
    return res;
}

void writeGuides(std::ostream& out, const GrouteResult& res, const CellNetlist& nl, const Lef& lef) {
    for (const auto& [net, guides] : res.guides) {
        out << nl.net_names[net] << "\n(\n";
        for (const auto& g : guides) out << g.rect.x0 << " " << g.rect.y0 << " " << g.rect.x1 << " " << g.rect.y1 << " " << lef.layers[g.layer].name << "\n";
        out << ")\n";
    }
}
