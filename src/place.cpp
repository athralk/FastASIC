// Placement: SimPL-style global placement, Abacus legalization, greedy detailed placement
//
// Global placement alternates two bounds on the final placement:
//  - lower bound: minimise quadratic wirelength (bound-to-bound net model, conjugate gradient,
//    matrix-free) with every cell pulled towards its spread position by an anchor;
//  - upper bound: spread the lower-bound placement so that no region holds more cell area than
//    target_density times its free row area (look-ahead legalization: recursive bisection of
//    each overfilled region by capacity).
// The anchors get stronger every iteration until the two bounds agree.
#include "../include/place.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace {

double msSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// Pin offset from the cell's lower-left corner in N orientation: average of the shape centres
struct PinOffsets {
    std::unordered_map<const LibCell*, std::vector<std::pair<double, double>>> cache;
    const std::vector<std::pair<double, double>>& of(const LibCell* cell, const LefMacro* m) {
        auto it = cache.find(cell);
        if (it != cache.end()) return it->second;
        std::vector<std::pair<double, double>> v(cell->pins.size(), {m->width / 2.0, m->height / 2.0});
        for (size_t k = 0; k < cell->pins.size(); ++k) {
            int p = m->pinIndex(cell->pins[k].name);
            if (p < 0 || m->pins[p].shapes.empty()) continue;
            double sx = 0, sy = 0;
            for (const LefShape& s : m->pins[p].shapes) {
                sx += (s.rect.x0 + s.rect.x1) / 2.0;
                sy += (s.rect.y0 + s.rect.y1) / 2.0;
            }
            v[k] = {sx / m->pins[p].shapes.size(), sy / m->pins[p].shapes.size()};
        }
        return cache.emplace(cell, std::move(v)).first->second;
    }
};

void orientOffset(Orient o, double w, double h, double& ox, double& oy) {
    if (o == Orient::FS || o == Orient::S) oy = h - oy;
    if (o == Orient::FN || o == Orient::S) ox = w - ox;
}

// ---------------------------------------------------------------- placement model (DBU doubles)
struct Pin {
    int cell;          // movable cell, or -1 for a fixed pin
    double px, py;     // offset from the cell's lower-left (N), or the fixed absolute position
};

struct Model {
    std::vector<int> comp;          // movable cell -> Floorplan component
    std::vector<double> w, h;       // cell size
    std::vector<double> x, y;       // cell centre
    std::vector<char> flip;         // FS row: pin y offsets mirrored
    std::vector<std::vector<Pin>> nets;
    std::vector<std::vector<int>> cell_nets;
    double pinX(const Pin& p) const { return p.cell < 0 ? p.px : x[p.cell] - w[p.cell] / 2 + p.px; }
    double pinY(const Pin& p) const {
        if (p.cell < 0) return p.py;
        double oy = flip[p.cell] ? h[p.cell] - p.py : p.py;
        return y[p.cell] - h[p.cell] / 2 + oy;
    }
    double netHpwl(int n) const {
        const auto& pins = nets[n];
        if (pins.size() < 2) return 0;
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
        for (const Pin& p : pins) {
            double px = pinX(p), py = pinY(p);
            x0 = std::min(x0, px); x1 = std::max(x1, px);
            y0 = std::min(y0, py); y1 = std::max(y1, py);
        }
        return (x1 - x0) + (y1 - y0);
    }
    double hpwl() const {
        double t = 0;
        for (size_t n = 0; n < nets.size(); ++n) t += netHpwl(static_cast<int>(n));
        return t;
    }
};

Model buildModel(const Floorplan& fp, const CellNetlist& nl, PinOffsets& offsets) {
    Model m;
    std::vector<int> movable(fp.components.size(), -1);
    for (size_t i = 0; i < fp.components.size(); ++i) {
        const FpComponent& c = fp.components[i];
        if (c.fixed) continue;
        movable[i] = static_cast<int>(m.comp.size());
        m.comp.push_back(static_cast<int>(i));
        m.w.push_back(static_cast<double>(c.macro->width));
        m.h.push_back(static_cast<double>(c.macro->height));
    }
    size_t n = m.comp.size();
    m.x.assign(n, 0);
    m.y.assign(n, 0);
    m.flip.assign(n, 0);
    m.cell_nets.resize(n);
    std::vector<std::vector<Pin>> by_net(nl.netCount());
    for (size_t i = 0; i < fp.components.size(); ++i) {
        const FpComponent& c = fp.components[i];
        const auto& inst = nl.instances[c.instance];
        const auto& off = offsets.of(inst.cell, c.macro);
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int net = inst.pin_net[k];
            if (net < 2) continue;
            if (movable[i] >= 0) {
                by_net[net].push_back({movable[i], off[k].first, off[k].second});
            } else {
                double ox = off[k].first, oy = off[k].second;
                orientOffset(c.orient, static_cast<double>(c.macro->width), static_cast<double>(c.macro->height), ox, oy);
                by_net[net].push_back({-1, c.x + ox, c.y + oy});
            }
        }
    }
    for (const FpPin& p : fp.pins) {
        int net = nl.port_bits[p.port_bit].net;
        if (net < 2) continue;
        by_net[net].push_back({-1, p.x + (p.shape.x0 + p.shape.x1) / 2.0, p.y + (p.shape.y0 + p.shape.y1) / 2.0});
    }
    for (auto& pins : by_net) {
        if (pins.size() < 2) continue;
        bool any_movable = false;
        for (const Pin& p : pins) any_movable |= p.cell >= 0;
        if (!any_movable) continue;
        int id = static_cast<int>(m.nets.size());
        for (const Pin& p : pins) {
            if (p.cell >= 0 && (m.cell_nets[p.cell].empty() || m.cell_nets[p.cell].back() != id)) m.cell_nets[p.cell].push_back(id);
        }
        m.nets.push_back(std::move(pins));
    }
    return m;
}

// ---------------------------------------------------------------- quadratic solve (one axis)
struct Edge {
    int i, j;
    double w;
};

// Minimises sum w (pin_a - pin_b)^2 over the bound-to-bound net model, plus anchors, along one axis.
void solveAxis(Model& m, bool xaxis, const std::vector<double>& anchor, const std::vector<double>& anchor_w, int max_degree, double eps) {
    size_t n = m.comp.size();
    std::vector<double>& v = xaxis ? m.x : m.y;
    std::vector<double> diag(n, 0), b(n, 0);
    std::vector<Edge> edges;
    edges.reserve(m.nets.size() * 4);
    std::vector<double> pos;
    for (const auto& pins : m.nets) {
        size_t P = pins.size();
        if (P < 2 || static_cast<int>(P) > max_degree) continue;
        pos.resize(P);
        size_t lo = 0, hi = 0;
        for (size_t k = 0; k < P; ++k) {
            pos[k] = xaxis ? m.pinX(pins[k]) : m.pinY(pins[k]);
            if (pos[k] < pos[lo]) lo = k;
            if (pos[k] > pos[hi]) hi = k;
        }
        if (lo == hi) hi = lo == 0 ? 1 : 0;
        double base = 2.0 / (P - 1);
        auto add = [&](size_t a, size_t c) {
            const Pin& pa = pins[a];
            const Pin& pc = pins[c];
            if (pa.cell < 0 && pc.cell < 0) return;
            if (pa.cell >= 0 && pa.cell == pc.cell) return;
            double w = base / std::max(std::fabs(pos[a] - pos[c]), eps);
            // offsets of the pins from their cell centres
            double oa = pa.cell >= 0 ? pos[a] - v[pa.cell] : 0, oc = pc.cell >= 0 ? pos[c] - v[pc.cell] : 0;
            if (pa.cell >= 0 && pc.cell >= 0) {
                edges.push_back({pa.cell, pc.cell, w});
                diag[pa.cell] += w;
                diag[pc.cell] += w;
                b[pa.cell] -= w * (oa - oc);
                b[pc.cell] += w * (oa - oc);
            } else if (pa.cell >= 0) {
                diag[pa.cell] += w;
                b[pa.cell] += w * (pos[c] - oa);
            } else {
                diag[pc.cell] += w;
                b[pc.cell] += w * (pos[a] - oc);
            }
        };
        add(lo, hi);
        for (size_t k = 0; k < P; ++k) {
            if (k == lo || k == hi) continue;
            add(k, lo);
            add(k, hi);
        }
    }
    double avg = 0;
    for (double d : diag) avg += d;
    avg = n ? avg / n : 1;
    if (avg <= 0) avg = 1;
    for (size_t i = 0; i < n; ++i) {
        double wa = anchor_w.empty() ? 0 : anchor_w[i];
        wa += 1e-6 * avg;   // keeps the system positive definite (cells with no nets)
        diag[i] += wa;
        b[i] += wa * (anchor.empty() ? v[i] : anchor[i]);
    }
    // Conjugate gradient with a Jacobi preconditioner, warm-started from the current positions
    auto apply = [&](const std::vector<double>& in, std::vector<double>& out) {
        for (size_t i = 0; i < n; ++i) out[i] = diag[i] * in[i];
        for (const Edge& e : edges) {
            out[e.i] -= e.w * in[e.j];
            out[e.j] -= e.w * in[e.i];
        }
    };
    std::vector<double> r(n), z(n), p(n), q(n), inv(n);
    for (size_t i = 0; i < n; ++i) inv[i] = 1.0 / diag[i];
    apply(v, q);
    double rz = 0, r0 = 0;
    for (size_t i = 0; i < n; ++i) {
        r[i] = b[i] - q[i];
        z[i] = r[i] * inv[i];
        p[i] = z[i];
        rz += r[i] * z[i];
        r0 += r[i] * r[i];
    }
    r0 = std::sqrt(r0);
    // A loose solve is enough: the anchors move every iteration (tuned on mac32 and an 8x copy)
    const double tol = 1e-3;
    const int max_cg = 50;
    for (int it = 0; it < max_cg; ++it) {
        apply(p, q);
        double pq = 0;
        for (size_t i = 0; i < n; ++i) pq += p[i] * q[i];
        if (pq <= 0) break;
        double alpha = rz / pq;
        double rz2 = 0, rn = 0;
        for (size_t i = 0; i < n; ++i) {
            v[i] += alpha * p[i];
            r[i] -= alpha * q[i];
            z[i] = r[i] * inv[i];
            rz2 += r[i] * z[i];
            rn += r[i] * r[i];
        }
        if (std::sqrt(rn) <= tol * r0) break;
        double beta = rz2 / rz;
        rz = rz2;
        for (size_t i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
}

// ---------------------------------------------------------------- spreading (upper bound)
class Spreader {
public:
    Spreader(const Floorplan& fp, const Model& m, double density, double per_bin) : fp(fp), m(m) {
        double area = 0;
        for (size_t i = 0; i < m.comp.size(); ++i) area += m.w[i] * m.h[i];
        double avg = m.comp.empty() ? 1 : area / m.comp.size();
        double core_w = static_cast<double>(fp.core.width()), core_h = static_cast<double>(fp.core.height());
        // About 8 cells per bin, bins at least two rows tall
        double bin = std::sqrt(per_bin * avg / std::max(density, 0.05));
        bin = std::max(bin, fp.site->height * 1.0);
        nx = std::clamp(static_cast<int>(core_w / bin), 1, 256);
        ny = std::clamp(static_cast<int>(core_h / bin), 1, 256);
        bw = core_w / nx;
        bh = core_h / ny;
        cap.assign(static_cast<size_t>(nx) * ny, 0);
        for (const FpRow& r : fp.rows) {
            double rx0 = r.x, rx1 = r.x + static_cast<double>(r.count) * fp.site->width, ry0 = r.y, ry1 = r.y + static_cast<double>(fp.site->height);
            for (int by = binY(ry0); by <= binY(ry1 - 1); ++by) {
                for (int bx = binX(rx0); bx <= binX(rx1 - 1); ++bx) {
                    double ox = std::min(rx1, x0(bx) + bw) - std::max(rx0, x0(bx));
                    double oy = std::min(ry1, y0(by) + bh) - std::max(ry0, y0(by));
                    if (ox > 0 && oy > 0) cap[idx(bx, by)] += ox * oy * density;
                }
            }
        }
        capsum = prefix(cap);
    }

    // Spreads (x, y) in place
    void spread(std::vector<double>& x, std::vector<double>& y) {
        std::vector<double> area(cap.size(), 0);
        std::vector<int> bin_of(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            int b = idx(binX(x[i]), binY(y[i]));
            bin_of[i] = b;
            area[b] += m.w[i] * m.h[i];
        }
        std::vector<double> asum = prefix(area);
        auto A = [&](const Box& r) { return sum(asum, r); };
        auto C = [&](const Box& r) { return sum(capsum, r); };
        std::vector<int> over;
        for (size_t b = 0; b < area.size(); ++b) {
            if (area[b] > cap[b] + 1e-9) over.push_back(static_cast<int>(b));
        }
        std::sort(over.begin(), over.end(), [&](int a, int b) { return area[a] - cap[a] > area[b] - cap[b]; });
        std::vector<Box> boxes;
        std::vector<char> covered(cap.size(), 0);
        for (int b : over) {
            if (covered[b]) continue;
            Box r{b % nx, b / nx, b % nx, b / nx};
            while (true) {
                while (A(r) > C(r) && !(r.x0 == 0 && r.y0 == 0 && r.x1 == nx - 1 && r.y1 == ny - 1)) {
                    r.x0 = std::max(0, r.x0 - 1); r.y0 = std::max(0, r.y0 - 1);
                    r.x1 = std::min(nx - 1, r.x1 + 1); r.y1 = std::min(ny - 1, r.y1 + 1);
                }
                bool merged = false;
                for (size_t k = 0; k < boxes.size(); ++k) {
                    if (overlaps(boxes[k], r)) {
                        r = {std::min(r.x0, boxes[k].x0), std::min(r.y0, boxes[k].y0), std::max(r.x1, boxes[k].x1), std::max(r.y1, boxes[k].y1)};
                        boxes.erase(boxes.begin() + k);
                        merged = true;
                        break;
                    }
                }
                if (!merged) break;
            }
            boxes.push_back(r);
            for (int by = r.y0; by <= r.y1; ++by)
                for (int bx = r.x0; bx <= r.x1; ++bx) covered[idx(bx, by)] = 1;
        }
        // Cells of each box, then recursive bisection by capacity
        std::vector<int> box_of_bin(cap.size(), -1);
        for (size_t k = 0; k < boxes.size(); ++k) {
            for (int by = boxes[k].y0; by <= boxes[k].y1; ++by)
                for (int bx = boxes[k].x0; bx <= boxes[k].x1; ++bx) box_of_bin[idx(bx, by)] = static_cast<int>(k);
        }
        std::vector<std::vector<int>> cells(boxes.size());
        for (size_t i = 0; i < x.size(); ++i) {
            int k = box_of_bin[bin_of[i]];
            if (k >= 0) cells[k].push_back(static_cast<int>(i));
        }
        for (size_t k = 0; k < boxes.size(); ++k) bisect(boxes[k], cells[k], x, y);
    }

private:
    struct Box {
        int x0, y0, x1, y1;   // bins, inclusive
    };
    const Floorplan& fp;
    const Model& m;
    int nx = 1, ny = 1;
    double bw = 1, bh = 1;
    std::vector<double> cap, capsum;

    int idx(int bx, int by) const { return by * nx + bx; }
    double x0(int bx) const { return fp.core.x0 + bx * bw; }
    double y0(int by) const { return fp.core.y0 + by * bh; }
    int binX(double x) const { return std::clamp(static_cast<int>((x - fp.core.x0) / bw), 0, nx - 1); }
    int binY(double y) const { return std::clamp(static_cast<int>((y - fp.core.y0) / bh), 0, ny - 1); }
    static bool overlaps(const Box& a, const Box& b) { return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1; }
    std::vector<double> prefix(const std::vector<double>& v) const {
        std::vector<double> s(static_cast<size_t>(nx + 1) * (ny + 1), 0);
        for (int by = 0; by < ny; ++by)
            for (int bx = 0; bx < nx; ++bx)
                s[(by + 1) * (nx + 1) + bx + 1] = v[idx(bx, by)] + s[by * (nx + 1) + bx + 1] + s[(by + 1) * (nx + 1) + bx] - s[by * (nx + 1) + bx];
        return s;
    }
    double sum(const std::vector<double>& s, const Box& r) const {
        auto at = [&](int bx, int by) { return s[by * (nx + 1) + bx]; };
        return at(r.x1 + 1, r.y1 + 1) - at(r.x0, r.y1 + 1) - at(r.x1 + 1, r.y0) + at(r.x0, r.y0);
    }

    void bisect(const Box& r, std::vector<int>& cells, std::vector<double>& x, std::vector<double>& y) {
        if (cells.empty()) return;
        bool can_x = r.x1 > r.x0, can_y = r.y1 > r.y0;
        double C = sum(capsum, r);
        if ((!can_x && !can_y) || C <= 0) {
            leaf(r, cells, x, y);
            return;
        }
        bool cut_x = can_x && (!can_y || (r.x1 - r.x0 + 1) * bw >= (r.y1 - r.y0 + 1) * bh);
        Box a = r, b = r;
        if (cut_x) { a.x1 = (r.x0 + r.x1) / 2; b.x0 = a.x1 + 1; }
        else { a.y1 = (r.y0 + r.y1) / 2; b.y0 = a.y1 + 1; }
        double ca = sum(capsum, a), cb = sum(capsum, b);
        std::vector<double>& key = cut_x ? x : y;
        std::sort(cells.begin(), cells.end(), [&](int p, int q) { return key[p] < key[q]; });
        double total = 0;
        for (int c : cells) total += m.w[c] * m.h[c];
        double target = total * ca / (ca + cb);
        size_t split = 0;
        double acc = 0;
        while (split < cells.size()) {
            double next = acc + m.w[cells[split]] * m.h[cells[split]];
            if (std::fabs(next - target) > std::fabs(acc - target)) break;
            acc = next;
            split++;
        }
        std::vector<int> left(cells.begin(), cells.begin() + split), right(cells.begin() + split, cells.end());
        bisect(a, left, x, y);
        bisect(b, right, x, y);
    }

    // Cells of one region: scaled into the region if they spill out of it, keeping their order
    void leaf(const Box& r, const std::vector<int>& cells, std::vector<double>& x, std::vector<double>& y) {
        double rx0 = x0(r.x0), rx1 = x0(r.x1) + bw, ry0 = y0(r.y0), ry1 = y0(r.y1) + bh;
        auto fit = [&](std::vector<double>& v, double lo, double hi) {
            double mn = 1e300, mx = -1e300;
            for (int c : cells) { mn = std::min(mn, v[c]); mx = std::max(mx, v[c]); }
            if (mn >= lo && mx <= hi) return;
            for (int c : cells) {
                double t = mx > mn ? (v[c] - mn) / (mx - mn) : 0.5;
                v[c] = lo + t * (hi - lo);
            }
        };
        fit(x, rx0, rx1);
        fit(y, ry0, ry1);
    }
};

// ---------------------------------------------------------------- legalization (Abacus)
struct Segment {
    double x0, y;     // DBU
    int sites;
    Orient orient;
    int row;          // row index (by y)
    struct Cluster {
        double e, q;
        int w;        // sites
        int x;        // sites from the segment start
        int first;    // index into cells
    };
    std::vector<Cluster> clusters;
    std::vector<int> cells;   // in x order
    int used = 0;
};

int clusterPos(double e, double q, int w, int sites) {
    double x = q / e;
    x = std::clamp(x, 0.0, static_cast<double>(sites - w));
    return static_cast<int>(std::lround(x));
}

// Position (sites) the cell would get at the end of this segment
int trialPlace(const Segment& s, double t, int wc) {
    double e = 1, q = t;
    int w = wc;
    int x = clusterPos(e, q, w, s.sites);
    for (int k = static_cast<int>(s.clusters.size()) - 1; k >= 0; --k) {
        const auto& c = s.clusters[k];
        if (c.x + c.w <= x) break;
        q = c.q + q - e * c.w;
        e += c.e;
        w += c.w;
        x = clusterPos(e, q, w, s.sites);
    }
    return x + w - wc;
}

void commitPlace(Segment& s, double t, int wc, int cell) {
    s.cells.push_back(cell);
    s.used += wc;
    Segment::Cluster c{1, t, wc, clusterPos(1, t, wc, s.sites), static_cast<int>(s.cells.size()) - 1};
    s.clusters.push_back(c);
    while (s.clusters.size() > 1) {
        auto& cur = s.clusters.back();
        auto& prev = s.clusters[s.clusters.size() - 2];
        if (prev.x + prev.w <= cur.x) break;
        prev.q = prev.q + cur.q - cur.e * prev.w;
        prev.e += cur.e;
        prev.w += cur.w;
        prev.x = clusterPos(prev.e, prev.q, prev.w, s.sites);
        s.clusters.pop_back();
    }
}

} // namespace

// ---------------------------------------------------------------- public API

double placementHpwl(const Floorplan& fp, const CellNetlist& nl, const Lef& lef) {
    PinOffsets offsets;
    std::vector<double> x0(nl.netCount(), 1e300), x1(nl.netCount(), -1e300), y0(nl.netCount(), 1e300), y1(nl.netCount(), -1e300);
    std::vector<int> count(nl.netCount(), 0);
    auto add = [&](int net, double x, double y) {
        x0[net] = std::min(x0[net], x); x1[net] = std::max(x1[net], x);
        y0[net] = std::min(y0[net], y); y1[net] = std::max(y1[net], y);
        count[net]++;
    };
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        const auto& inst = nl.instances[c.instance];
        const auto& off = offsets.of(inst.cell, c.macro);
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int net = inst.pin_net[k];
            if (net < 2) continue;
            double ox = off[k].first, oy = off[k].second;
            orientOffset(c.orient, static_cast<double>(c.macro->width), static_cast<double>(c.macro->height), ox, oy);
            add(net, c.x + ox, c.y + oy);
        }
    }
    for (const FpPin& p : fp.pins) {
        int net = nl.port_bits[p.port_bit].net;
        if (net >= 2) add(net, p.x + (p.shape.x0 + p.shape.x1) / 2.0, p.y + (p.shape.y0 + p.shape.y1) / 2.0);
    }
    double total = 0;
    for (size_t n = 0; n < count.size(); ++n) {
        if (count[n] >= 2) total += (x1[n] - x0[n]) + (y1[n] - y0[n]);
    }
    return total / lef.dbu;
}

PlaceResult placeDesign(Floorplan& fp, const CellNetlist& nl, const Lef& lef, const PlaceOptions& opt) {
    PlaceResult res;
    PinOffsets offsets;
    Model m = buildModel(fp, nl, offsets);
    size_t n = m.comp.size();
    if (n == 0) return res;
    const double sw = fp.site->width, sh = fp.site->height;

    // ------------------------------------------------ global placement
    auto t0 = std::chrono::steady_clock::now();
    {
        double cx = (fp.core.x0 + fp.core.x1) / 2.0, cy = (fp.core.y0 + fp.core.y1) / 2.0;
        uint64_t seed = 0x9E3779B97F4A7C15ull;
        auto rnd = [&]() {
            seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
            return (seed % 1000000) / 1000000.0 - 0.5;
        };
        for (size_t i = 0; i < n; ++i) {
            m.x[i] = cx + rnd() * fp.core.width() * 0.01;
            m.y[i] = cy + rnd() * fp.core.height() * 0.01;
        }
    }
    const double eps = sw;   // B2B: distances below one site count as one site
    for (int k = 0; k < 5; ++k) {
        solveAxis(m, true, {}, {}, opt.max_net_degree, eps);
        solveAxis(m, false, {}, {}, opt.max_net_degree, eps);
    }
    Spreader spreader(fp, m, opt.target_density, 8);
    std::vector<double> ux = m.x, uy = m.y;
    std::vector<double> wx(n), wy(n);
    double best_gap = 1e300;
    int stalled = 0;
    for (int it = 1; it <= opt.max_iterations; ++it) {
        ux = m.x;
        uy = m.y;
        spreader.spread(ux, uy);
        double lower = m.hpwl();
        std::swap(m.x, ux);
        std::swap(m.y, uy);
        double upper = m.hpwl();
        std::swap(m.x, ux);
        std::swap(m.y, uy);
        res.iterations = it;
        double gap = upper > 0 ? (upper - lower) / upper : 0;
        if (gap < best_gap - 0.002) { best_gap = gap; stalled = 0; }
        else if (++stalled >= 4) break;
        if (std::getenv("FASTPNR_TRACE")) fprintf(stderr, "gp %d lower %.0f upper %.0f gap %.3f\n", it, lower / lef.dbu, upper / lef.dbu, gap);
        if (gap < opt.stop_gap) break;
        // Anchors towards the spread positions, stronger each iteration
        double alpha = 0.01 * std::pow(1.3, it);
        for (size_t i = 0; i < n; ++i) {
            wx[i] = alpha / std::max(std::fabs(m.x[i] - ux[i]), eps);
            wy[i] = alpha / std::max(std::fabs(m.y[i] - uy[i]), eps);
        }
        solveAxis(m, true, ux, wx, opt.max_net_degree, eps);
        solveAxis(m, false, uy, wy, opt.max_net_degree, eps);
    }
    // The spread (upper-bound) placement is the global placement result
    m.x = ux;
    m.y = uy;
    res.hpwl_global = m.hpwl() / lef.dbu;
    res.ms_global = msSince(t0);

    // ------------------------------------------------ legalization
    auto t1 = std::chrono::steady_clock::now();
    std::vector<Segment> segs;
    std::map<Dbu, int> row_of_y;
    for (const FpRow& r : fp.rows) row_of_y.emplace(r.y, 0);
    int ri = 0;
    for (auto& [yv, idx] : row_of_y) idx = ri++;
    std::vector<std::vector<int>> segs_in_row(row_of_y.size());
    std::vector<Dbu> row_y(row_of_y.size());
    for (auto& [yv, idx] : row_of_y) row_y[idx] = yv;
    for (const FpRow& r : fp.rows) {
        Segment s;
        s.x0 = static_cast<double>(r.x);
        s.y = static_cast<double>(r.y);
        s.sites = r.count;
        s.orient = r.orient;
        s.row = row_of_y[r.y];
        segs_in_row[s.row].push_back(static_cast<int>(segs.size()));
        segs.push_back(s);
    }
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return m.x[a] - m.w[a] / 2 < m.x[b] - m.w[b] / 2; });
    std::vector<int> seg_of(n, -1);
    std::vector<int> wsites(n);
    for (size_t i = 0; i < n; ++i) wsites[i] = static_cast<int>(std::lround(m.w[i] / sw));
    for (int c : order) {
        double tx = m.x[c] - m.w[c] / 2, ty = m.y[c] - m.h[c] / 2;
        int r0 = static_cast<int>(std::lround((ty - fp.core.y0) / sh));
        r0 = std::clamp(r0, 0, static_cast<int>(row_y.size()) - 1);
        double best = 1e300;
        int best_seg = -1;
        const int nrows = static_cast<int>(row_y.size());
        for (int k = 0; k < nrows; ++k) {
            bool any_close = false;
            for (int r : {r0 + k, r0 - k}) {
                if (r < 0 || r >= nrows || (k == 0 && r != r0)) continue;
                double dy = row_y[r] - ty;
                if (dy * dy >= best) continue;
                any_close = true;
                for (int si : segs_in_row[r]) {
                    const Segment& s = segs[si];
                    if (s.used + wsites[c] > s.sites) continue;
                    int px = trialPlace(s, (tx - s.x0) / sw, wsites[c]);
                    double dx = s.x0 + px * sw - tx;
                    double cost = dx * dx + dy * dy;
                    if (cost < best) {
                        best = cost;
                        best_seg = si;
                    }
                }
            }
            if (!any_close && best_seg >= 0) break;
        }
        if (best_seg < 0) throw std::runtime_error("legalization failed: no row has room for a cell (utilization too high)");
        Segment& s = segs[best_seg];
        commitPlace(s, (tx - s.x0) / sw, wsites[c], c);
        seg_of[c] = best_seg;
    }
    // Final positions from the clusters
    std::vector<int> site_x(n, 0);
    for (Segment& s : segs) {
        for (size_t k = 0; k < s.clusters.size(); ++k) {
            int x = s.clusters[k].x;
            int end = k + 1 < s.clusters.size() ? s.clusters[k + 1].first : static_cast<int>(s.cells.size());
            for (int j = s.clusters[k].first; j < end; ++j) {
                site_x[s.cells[j]] = x;
                x += wsites[s.cells[j]];
            }
        }
    }
    double disp_sum = 0;
    auto applyPos = [&](int c) {
        const Segment& s = segs[seg_of[c]];
        m.x[c] = s.x0 + site_x[c] * sw + m.w[c] / 2;
        m.y[c] = s.y + m.h[c] / 2;
        m.flip[c] = s.orient == Orient::FS;
    };
    for (size_t c = 0; c < n; ++c) {
        double gx = m.x[c], gy = m.y[c];
        applyPos(static_cast<int>(c));
        double d = std::fabs(m.x[c] - gx) + std::fabs(m.y[c] - gy);
        disp_sum += d;
        res.max_displacement = std::max(res.max_displacement, d / lef.dbu);
    }
    res.avg_displacement = disp_sum / n / lef.dbu;
    res.hpwl_legal = m.hpwl() / lef.dbu;
    res.ms_legal = msSince(t1);

    // ------------------------------------------------ detailed placement
    auto t2 = std::chrono::steady_clock::now();
    if (opt.detailed) {
        // Cells of every segment in x order
        for (Segment& s : segs) {
            std::sort(s.cells.begin(), s.cells.end(), [&](int a, int b) { return site_x[a] < site_x[b]; });
        }
        std::vector<int> mark(m.nets.size(), -1);
        int stamp = 0;
        // Moving one cell barely changes a very large net (clock, reset): such nets are not costed
        std::vector<char> big(m.nets.size(), 0);
        for (size_t k = 0; k < m.nets.size(); ++k) big[k] = static_cast<int>(m.nets[k].size()) > opt.max_net_degree;
        std::vector<int> touched;
        auto netsOf = [&](int a, int b) {
            stamp++;
            touched.clear();
            for (int c : {a, b}) {
                if (c < 0) continue;
                for (int nn : m.cell_nets[c]) {
                    if (mark[nn] != stamp && !big[nn]) { mark[nn] = stamp; touched.push_back(nn); }
                }
            }
        };
        auto cost = [&]() {
            double t = 0;
            for (int nn : touched) t += m.netHpwl(nn);
            return t;
        };
        auto indexIn = [&](const Segment& s, int c) {
            auto it = std::lower_bound(s.cells.begin(), s.cells.end(), c, [&](int a, int b) { return site_x[a] < site_x[b]; });
            while (it != s.cells.end() && *it != c) ++it;
            return static_cast<int>(it - s.cells.begin());
        };
        for (int pass = 0; pass < opt.detailed_passes; ++pass) {
            int improved = 0;
            // 1. Move each cell towards its optimal region: into a free gap, or swap with an
            //    equal-width cell there
            for (size_t ci = 0; ci < n; ++ci) {
                int c = static_cast<int>(ci);
                std::vector<double> xs, ys;
                for (int nn : m.cell_nets[c]) {
                    const auto& pins = m.nets[nn];
                    if (static_cast<int>(pins.size()) > opt.max_net_degree) continue;
                    double lx = 1e300, hx = -1e300, ly = 1e300, hy = -1e300;
                    bool any = false;
                    for (const Pin& p : pins) {
                        if (p.cell == c) continue;
                        any = true;
                        double px = m.pinX(p), py = m.pinY(p);
                        lx = std::min(lx, px); hx = std::max(hx, px);
                        ly = std::min(ly, py); hy = std::max(hy, py);
                    }
                    if (!any) continue;
                    xs.push_back(lx); xs.push_back(hx);
                    ys.push_back(ly); ys.push_back(hy);
                }
                if (xs.empty()) continue;
                std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
                std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
                double tx = xs[xs.size() / 2] - m.w[c] / 2, ty = ys[ys.size() / 2] - m.h[c] / 2;
                double cur_x = m.x[c] - m.w[c] / 2, cur_y = m.y[c] - m.h[c] / 2;
                if (std::fabs(tx - cur_x) < 2 * sw && std::fabs(ty - cur_y) < sh / 2) continue;
                int tr = std::clamp(static_cast<int>(std::lround((ty - fp.core.y0) / sh)), 0, static_cast<int>(row_y.size()) - 1);
                bool done = false;
                for (int dr = -1; dr <= 1 && !done; ++dr) {
                    int r = tr + dr;
                    if (r < 0 || r >= static_cast<int>(row_y.size())) continue;
                    for (int si : segs_in_row[r]) {
                        if (done) break;
                        Segment& s = segs[si];
                        int want = static_cast<int>(std::lround((tx - s.x0) / sw));
                        if (want < -wsites[c] || want > s.sites) continue;
                        want = std::clamp(want, 0, s.sites - wsites[c]);
                        // neighbour cells around the wanted spot
                        auto it = std::lower_bound(s.cells.begin(), s.cells.end(), want, [&](int a, int v) { return site_x[a] < v; });
                        int k = static_cast<int>(it - s.cells.begin());
                        // a) free gap containing or next to `want`
                        for (int kk = std::max(0, k - 2); kk <= std::min(static_cast<int>(s.cells.size()), k + 2) && !done; ++kk) {
                            int lo = kk == 0 ? 0 : site_x[s.cells[kk - 1]] + wsites[s.cells[kk - 1]];
                            int hi = kk == static_cast<int>(s.cells.size()) ? s.sites : site_x[s.cells[kk]];
                            if (seg_of[c] == si) {
                                // the cell's own slot counts as free
                                int own = indexIn(s, c);
                                if (kk - 1 == own) lo = own == 0 ? 0 : site_x[s.cells[own - 1]] + wsites[s.cells[own - 1]];
                                if (kk == own) hi = own + 1 < static_cast<int>(s.cells.size()) ? site_x[s.cells[own + 1]] : s.sites;
                            }
                            if (hi - lo < wsites[c]) continue;
                            int px = std::clamp(want, lo, hi - wsites[c]);
                            netsOf(c, -1);
                            double before = cost();
                            int old_seg = seg_of[c], old_x = site_x[c];
                            double ox = m.x[c], oy = m.y[c];
                            char of = m.flip[c];
                            seg_of[c] = si;
                            site_x[c] = px;
                            applyPos(c);
                            if (cost() < before - 1e-9) {
                                Segment& os = segs[old_seg];
                                site_x[c] = old_x;
                                int oi = indexIn(os, c);
                                os.cells.erase(os.cells.begin() + oi);
                                site_x[c] = px;
                                auto ins = std::lower_bound(s.cells.begin(), s.cells.end(), px, [&](int a, int v) { return site_x[a] < v; });
                                s.cells.insert(ins, c);
                                done = true;
                                improved++;
                            } else {
                                seg_of[c] = old_seg;
                                site_x[c] = old_x;
                                m.x[c] = ox; m.y[c] = oy; m.flip[c] = of;
                            }
                        }
                        // b) swap with an equal-width cell near `want`
                        for (int kk = std::max(0, k - 2); kk < std::min(static_cast<int>(s.cells.size()), k + 2) && !done; ++kk) {
                            int d = s.cells[kk];
                            if (d == c || wsites[d] != wsites[c]) continue;
                            netsOf(c, d);
                            double before = cost();
                            std::swap(seg_of[c], seg_of[d]);
                            std::swap(site_x[c], site_x[d]);
                            applyPos(c);
                            applyPos(d);
                            if (cost() < before - 1e-9) {
                                // positions swapped: fix the two segments' cell lists
                                if (seg_of[c] == seg_of[d]) {
                                    Segment& sc = segs[seg_of[c]];
                                    std::sort(sc.cells.begin(), sc.cells.end(), [&](int a2, int b2) { return site_x[a2] < site_x[b2]; });
                                } else {
                                    for (int& v : segs[seg_of[c]].cells) if (v == d) { v = c; break; }
                                    for (int& v : segs[seg_of[d]].cells) if (v == c) { v = d; break; }
                                }
                                done = true;
                                improved++;
                            } else {
                                std::swap(seg_of[c], seg_of[d]);
                                std::swap(site_x[c], site_x[d]);
                                applyPos(c);
                                applyPos(d);
                            }
                        }
                    }
                }
            }
            // 2. Adjacent pairs that touch: try the other order
            for (Segment& s : segs) {
                for (size_t k = 0; k + 1 < s.cells.size(); ++k) {
                    int a = s.cells[k], b = s.cells[k + 1];
                    if (site_x[a] + wsites[a] != site_x[b]) continue;
                    netsOf(a, b);
                    double before = cost();
                    int xa = site_x[a];
                    site_x[b] = xa;
                    site_x[a] = xa + wsites[b];
                    applyPos(a);
                    applyPos(b);
                    if (cost() < before - 1e-9) {
                        std::swap(s.cells[k], s.cells[k + 1]);
                        improved++;
                    } else {
                        site_x[a] = xa;
                        site_x[b] = xa + wsites[a];
                        applyPos(a);
                        applyPos(b);
                    }
                }
            }
            // 3. Order-preserving row re-spacing: every cell is pulled to the middle of its optimal
            //    interval (median of its nets' bounding boxes in x) and the row is re-packed by the
            //    same clustering as Abacus; kept when the row's wirelength drops
            for (size_t si = 0; si < segs.size(); ++si) {
                Segment& s = segs[si];
                if (s.cells.empty()) continue;
                Segment fresh;
                fresh.sites = s.sites;
                std::vector<int> old_x;
                stamp++;
                touched.clear();
                for (int c : s.cells) {
                    old_x.push_back(site_x[c]);
                    std::vector<double> xs;
                    for (int nn : m.cell_nets[c]) {
                        if (big[nn]) continue;
                        if (mark[nn] != stamp) { mark[nn] = stamp; touched.push_back(nn); }
                        const auto& pins = m.nets[nn];
                        if (static_cast<int>(pins.size()) > opt.max_net_degree) continue;
                        double lx = 1e300, hx = -1e300;
                        bool any = false;
                        for (const Pin& p : pins) {
                            if (p.cell == c) continue;
                            any = true;
                            double px = m.pinX(p);
                            lx = std::min(lx, px); hx = std::max(hx, px);
                        }
                        if (any) { xs.push_back(lx); xs.push_back(hx); }
                    }
                    double cur = m.x[c];
                    double target = cur;
                    if (!xs.empty()) {
                        std::sort(xs.begin(), xs.end());
                        double lo = xs[(xs.size() - 1) / 2], hi = xs[xs.size() / 2];
                        target = std::clamp(cur, lo, hi);
                    }
                    commitPlace(fresh, (target - m.w[c] / 2 - s.x0) / sw, wsites[c], c);
                }
                double before = cost();
                std::vector<int> nx(s.cells.size());
                size_t j = 0;
                for (size_t k = 0; k < fresh.clusters.size(); ++k) {
                    int x = fresh.clusters[k].x;
                    int end = k + 1 < fresh.clusters.size() ? fresh.clusters[k + 1].first : static_cast<int>(fresh.cells.size());
                    for (int q = fresh.clusters[k].first; q < end; ++q) {
                        nx[j++] = x;
                        x += wsites[fresh.cells[q]];
                    }
                }
                for (size_t k = 0; k < s.cells.size(); ++k) { site_x[s.cells[k]] = nx[k]; applyPos(s.cells[k]); }
                if (cost() < before - 1e-9) {
                    improved++;
                } else {
                    for (size_t k = 0; k < s.cells.size(); ++k) { site_x[s.cells[k]] = old_x[k]; applyPos(s.cells[k]); }
                }
            }
            if (!improved) break;
        }
    }
    res.ms_detailed = msSince(t2);

    // ------------------------------------------------ write back
    for (size_t c = 0; c < n; ++c) {
        FpComponent& comp = fp.components[m.comp[c]];
        const Segment& s = segs[seg_of[c]];
        comp.x = static_cast<Dbu>(std::llround(s.x0)) + static_cast<Dbu>(site_x[c]) * fp.site->width;
        comp.y = static_cast<Dbu>(std::llround(s.y));
        comp.orient = s.orient;
        comp.placed = true;
    }
    res.hpwl_final = m.hpwl() / lef.dbu;
    return res;
}

std::vector<std::string> checkPlacement(const Floorplan& fp, const CellNetlist& nl, const Lef& lef) {
    (void)lef;
    std::vector<std::string> bad;
    std::map<Dbu, std::vector<const FpRow*>> rows;
    for (const FpRow& r : fp.rows) rows[r.y].push_back(&r);
    std::map<const FpRow*, std::vector<std::pair<Dbu, Dbu>>> in_row;
    for (const FpComponent& c : fp.components) {
        if (c.fixed) continue;
        const std::string& name = nl.instances[c.instance].name;
        if (!c.placed) { bad.push_back(name + " is not placed"); continue; }
        auto it = rows.find(c.y);
        const FpRow* row = nullptr;
        if (it != rows.end()) {
            for (const FpRow* r : it->second) {
                if (c.x >= r->x && c.x + c.macro->width <= r->x + static_cast<Dbu>(r->count) * fp.site->width) row = r;
            }
        }
        if (!row) { bad.push_back(name + " is not inside a row"); continue; }
        if ((c.x - row->x) % fp.site->width) bad.push_back(name + " is off the site grid");
        if (c.orient != row->orient) bad.push_back(name + " does not have its row's orientation");
        in_row[row].push_back({c.x, c.x + c.macro->width});
    }
    for (auto& [row, cells] : in_row) {
        std::sort(cells.begin(), cells.end());
        for (size_t k = 0; k + 1 < cells.size(); ++k) {
            if (cells[k].second > cells[k + 1].first) {
                bad.push_back("cells overlap in row " + row->name);
                break;
            }
        }
    }
    return bad;
}
