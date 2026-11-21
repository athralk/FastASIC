#include "../include/spef.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <set>
#include <tuple>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

std::string unescapeName(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size()) {
            out.push_back(s[++i]);
        } else {
            out.push_back(s[i]);
        }
    }
    return out;
}

std::string spefEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char c : s) {
        if (c == '[' || c == ']' || c == '$' || c == '.' || c == '/') out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

std::string spefPinName(const std::string& inst, const std::string& pin) {
    return spefEscape(inst) + ":" + pin;
}

struct RcEdge {
    int to;
    double r_ohm;
};

// Compute exact Elmore delay (ps) from root node to every reachable node on a spanning tree of an RC graph.
// Units: R in ohm, C in fF => R * C * 1e-3 = ps (since 1 ohm * 1 fF = 1e-15 s = 0.001 ps).
std::vector<double> computeElmoreTreePs(int n_nodes, int root, const std::vector<double>& node_cap_ff,
                                        const std::vector<std::vector<RcEdge>>& adj) {
    std::vector<double> elmore(n_nodes, std::numeric_limits<double>::infinity());
    if (root < 0 || root >= n_nodes) return elmore;
    elmore[root] = 0.0;
    std::vector<int> parent(n_nodes, -1);
    std::vector<double> parent_r(n_nodes, 0.0);
    std::vector<int> order;
    order.reserve(n_nodes);
    std::vector<uint8_t> vis(n_nodes, 0);
    vis[root] = 1;
    order.push_back(root);
    for (size_t head = 0; head < order.size(); ++head) {
        int u = order[head];
        for (const RcEdge& e : adj[u]) {
            if (!vis[e.to]) {
                vis[e.to] = 1;
                parent[e.to] = u;
                parent_r[e.to] = e.r_ohm;
                order.push_back(e.to);
            }
        }
    }
    // Bottom-up downstream capacitance
    std::vector<double> sub_cap = node_cap_ff;
    for (size_t idx = order.size(); idx-- > 1;) {
        int u = order[idx];
        int p = parent[u];
        if (p >= 0) sub_cap[p] += sub_cap[u];
    }
    // Top-down Elmore delay: delay(u) = delay(p) + R(p,u) * C_downstream(u) * 1e-3 ps
    for (size_t idx = 1; idx < order.size(); ++idx) {
        int u = order[idx];
        int p = parent[u];
        elmore[u] = elmore[p] + parent_r[u] * sub_cap[u] * 1e-3;
    }
    return elmore;
}

struct PinLoc {
    std::string spef_name;
    std::string lsta_name;
    bool is_port = false;
    bool is_driver = false;
    Dbu x = 0, y = 0;
    std::vector<LefShape> shapes;
};

} // namespace

std::unordered_map<std::string, double> NetParasitics::elmoreDelays(
    const std::unordered_map<std::string, double>& pin_cap_ff) const {
    std::vector<double> cap = node_cap_ff;
    for (const auto& [pin, c] : pin_cap_ff) {
        auto it = load_nodes.find(pin);
        if (it != load_nodes.end()) cap[it->second] += c;
    }
    std::vector<std::vector<RcEdge>> adj(cap.size());
    for (const auto& r : resistors) {
        adj[r.from].push_back({r.to, r.ohm});
        adj[r.to].push_back({r.from, r.ohm});
    }
    std::unordered_map<std::string, double> delays;
    if (driver_node < 0) return delays;
    auto elmore = computeElmoreTreePs(static_cast<int>(cap.size()), driver_node, cap, adj);
    for (const auto& [pin, n] : load_nodes) {
        if (!std::isfinite(elmore[n])) throw std::runtime_error("SPEF: load pin '" + pin + "' is disconnected from its driver");
        delays[pin] = elmore[n];
    }
    return delays;
}

void writeSpef(std::ostream& out, const Floorplan& fp, const CellNetlist& nl, const Lef& lef,
               const DrouteResult& dr, double cap_scale) {
    // Precompute placed pin coordinates per netlist net
    std::vector<std::vector<PinLoc>> net_pins(nl.netCount());
    for (const FpPin& p : fp.pins) {
        const auto& pb = nl.port_bits[p.port_bit];
        if (pb.net < 2 || static_cast<size_t>(pb.net) >= nl.netCount()) continue;
        Dbu cx = p.x + (p.shape.x0 + p.shape.x1) / 2;
        Dbu cy = p.y + (p.shape.y0 + p.shape.y1) / 2;
        Rect r = p.shape;
        r.x0 += p.x; r.x1 += p.x; r.y0 += p.y; r.y1 += p.y;
        net_pins[pb.net].push_back({spefEscape(pb.name), pb.name, true, pb.is_input, cx, cy, {{p.layer, r}}});
    }
    for (const FpComponent& c : fp.components) {
        if (!c.placed) continue;
        const auto& inst = nl.instances[c.instance];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            int net = inst.pin_net[k];
            if (net < 2 || static_cast<size_t>(net) >= nl.netCount()) continue;
            int lp = c.macro->pinIndex(inst.cell->pins[k].name);
            Dbu cx = c.x + c.macro->width / 2, cy = c.y + c.macro->height / 2;
            std::vector<LefShape> shapes;
            if (lp >= 0) for (const auto& shape : c.macro->pins[lp].shapes) {
                Rect r = shape.rect;
                if (c.orient == Orient::FS || c.orient == Orient::S) {
                    Dbu a = c.macro->height - r.y1, b = c.macro->height - r.y0;
                    r.y0 = a; r.y1 = b;
                }
                if (c.orient == Orient::FN || c.orient == Orient::S) {
                    Dbu a = c.macro->width - r.x1, b = c.macro->width - r.x0;
                    r.x0 = a; r.x1 = b;
                }
                r.x0 += c.x; r.x1 += c.x; r.y0 += c.y; r.y1 += c.y;
                if (shapes.empty()) { cx = (r.x0 + r.x1) / 2; cy = (r.y0 + r.y1) / 2; }
                shapes.push_back({shape.layer, r});
            }
            bool drv = inst.cell->pins[k].dir == PinDir::OUTPUT;
            net_pins[net].push_back({spefPinName(inst.name, inst.cell->pins[k].name),
                                     inst.name + "/" + inst.cell->pins[k].name, false, drv, cx, cy, std::move(shapes)});
        }
    }

    // Via cut resistance lookup from tech LEF (default 20 ohm for IHP single-cut vias)
    std::vector<double> via_res(lef.layers.size(), 20.0);
    for (size_t l = 0; l + 1 < lef.layers.size(); ++l) {
        if (lef.layers[l].type == LayerType::ROUTING) {
            for (size_t k = l + 1; k < lef.layers.size(); ++k) {
                if (lef.layers[k].type == LayerType::CUT) {
                    if (lef.layers[k].res_per_sq > 0) via_res[l] = lef.layers[k].res_per_sq;
                    break;
                }
                if (lef.layers[k].type == LayerType::ROUTING) break;
            }
        }
    }

    // Drawn wire width per LEF layer (from dr.wire_width / dr.wire_layer)
    std::vector<double> drawn_w_um(lef.layers.size(), 0.20);
    for (size_t l = 0; l < lef.layers.size(); ++l) {
        if (lef.layers[l].width > 0) drawn_w_um[l] = lef.toMicrons(lef.layers[l].width);
    }
    for (size_t g = 0; g < dr.wire_layer.size() && g < dr.wire_width.size(); ++g) {
        int ll = dr.wire_layer[g];
        if (ll >= 0 && static_cast<size_t>(ll) < drawn_w_um.size() && dr.wire_width[g] > 0) {
            drawn_w_um[ll] = lef.toMicrons(dr.wire_width[g]);
        }
    }

    out << "*SPEF \"IEEE 1481-1998\"\n"
        << "*DESIGN \"" << nl.name << "\"\n"
        << "*DATE \"FastASIC\"\n"
        << "*VENDOR \"FastASIC\"\n"
        << "*PROGRAM \"fastpnr\"\n"
        << "*VERSION \"1.0\"\n"
        << "*DESIGN_FLOW \"PIN_CAP NONE\"\n"
        << "*DIVIDER /\n"
        << "*DELIMITER :\n"
        << "*BUS_DELIMITER [ ]\n"
        << "*T_UNIT 1 NS\n"
        << "*C_UNIT 1 PF\n"
        << "*R_UNIT 1 OHM\n"
        << "*L_UNIT 1 HENRY\n\n";

    out << std::fixed << std::setprecision(6);

    for (const DrouteNet& dn : dr.nets) {
        int net = dn.netlist_net;
        if (net < 2 || static_cast<size_t>(net) >= nl.netCount()) continue;
        const auto& pins = net_pins[net];
        if (pins.empty()) continue;

        // Quantize wire geometry endpoints into SPEF RC nodes: key = (layer, x, y)
        struct PtKey {
            int l;
            Dbu x, y;
            bool operator==(const PtKey& o) const { return l == o.l && x == o.x && y == o.y; }
        };
        struct PtHash {
            size_t operator()(const PtKey& k) const {
                size_t h = static_cast<size_t>(k.l) * 0x9e3779b97f4a7c15ULL;
                h ^= static_cast<size_t>(k.x) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                h ^= static_cast<size_t>(k.y) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                return h;
            }
        };
        std::unordered_map<PtKey, int, PtHash> pt_id;
        struct NodeInfo {
            int l;
            Dbu x, y;
            double cap_pf = 0.0;
            std::string name;
        };
        std::vector<NodeInfo> nodes;
        std::string net_esc = spefEscape(nl.net_names[net]);

        auto getNode = [&](int l, Dbu x, Dbu y) -> int {
            PtKey k{l, x, y};
            auto it = pt_id.find(k);
            if (it != pt_id.end()) return it->second;
            int id = static_cast<int>(nodes.size());
            pt_id[k] = id;
            nodes.push_back({l, x, y, 0.0, net_esc + ":" + std::to_string(id + 1)});
            return id;
        };

        struct SegRes { int a, b; double r_ohm; };
        std::vector<SegRes> resistors;
        double total_cap_pf = 0.0;

        auto validLayer = [&](int l) { return l >= 0 && static_cast<size_t>(l) < lef.layers.size(); };
        auto inside = [](Dbu x, Dbu y, const Rect& r) {
            return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
        };
        auto onSegment = [](Dbu x, Dbu y, const DrouteSegment& s) {
            return x >= std::min(s.x0, s.x1) && x <= std::max(s.x0, s.x1) &&
                   y >= std::min(s.y0, s.y1) && y <= std::max(s.y0, s.y1) &&
                   (s.x0 == s.x1 ? x == s.x0 : y == s.y0);
        };

        // Register every physical junction before extracting any wire. Merged router
        // runs must be split at branches and vias, not joined later by guessed resistors.
        for (const auto& s : dn.wires) {
            if (!validLayer(s.layer)) continue;
            getNode(s.layer, s.x0, s.y0);
            getNode(s.layer, s.x1, s.y1);
        }
        for (const auto& v : dn.vias) {
            if (!validLayer(v.bottom)) continue;
            int top = v.bottom + 1;
            while (validLayer(top) && lef.layers[top].type != LayerType::ROUTING) ++top;
            if (!validLayer(top)) continue;
            int a = getNode(v.bottom, v.x, v.y), b = getNode(top, v.x, v.y);
            resistors.push_back({a, b, via_res[v.bottom]});
        }
        for (size_t i = 0; i < dn.wires.size(); ++i) {
            const auto& a = dn.wires[i];
            if (!validLayer(a.layer)) continue;
            for (size_t j = i + 1; j < dn.wires.size(); ++j) {
                const auto& b = dn.wires[j];
                if (a.layer != b.layer || (a.x0 == a.x1) == (b.x0 == b.x1)) continue;
                Dbu x = a.x0 == a.x1 ? a.x0 : b.x0;
                Dbu y = a.x0 == a.x1 ? b.y0 : a.y0;
                if (onSegment(x, y, a) && onSegment(x, y, b)) getNode(a.layer, x, y);
            }
        }
        // A pin or patch can meet the middle of a run without a via at that point.
        auto addShapeContacts = [&](int layer, const Rect& r) {
            if (!validLayer(layer)) return;
            for (const auto& s : dn.wires) {
                if (s.layer != layer) continue;
                Dbu x = std::clamp((r.x0 + r.x1) / 2, std::min(s.x0, s.x1), std::max(s.x0, s.x1));
                Dbu y = std::clamp((r.y0 + r.y1) / 2, std::min(s.y0, s.y1), std::max(s.y0, s.y1));
                if (inside(x, y, r)) getNode(layer, x, y);
            }
        };
        for (const auto& p : pins) for (const auto& shape : p.shapes) addShapeContacts(shape.layer, shape.rect);
        for (const auto& [layer, rect] : dn.patches) addShapeContacts(layer, rect);

        std::vector<std::vector<int>> layer_nodes(lef.layers.size());
        for (size_t k = 0; k < nodes.size(); ++k) layer_nodes[nodes[k].l].push_back(static_cast<int>(k));
        // Overlapping collinear runs share each subsegment and must not double-count RC.
        std::set<std::pair<int, int>> wire_edges;
        for (const auto& s : dn.wires) {
            if (!validLayer(s.layer)) continue;
            std::vector<int> cuts;
            for (int k : layer_nodes[s.layer]) if (onSegment(nodes[k].x, nodes[k].y, s)) cuts.push_back(k);
            std::sort(cuts.begin(), cuts.end(), [&](int a, int b) {
                return std::tie(nodes[a].x, nodes[a].y) < std::tie(nodes[b].x, nodes[b].y);
            });
            const LefLayer& L = lef.layers[s.layer];
            double w_um = std::max(drawn_w_um[s.layer], 0.16);
            double r_sq = L.res_per_sq > 0 ? L.res_per_sq : 0.11;
            double c_area = L.cap_per_area > 0 ? L.cap_per_area : 1.8e-5;
            double c_edge = L.edge_cap > 0 ? L.edge_cap : 4.5e-5;
            for (size_t k = 1; k < cuts.size(); ++k) {
                int a = cuts[k - 1], b = cuts[k];
                if (!wire_edges.emplace(std::min(a, b), std::max(a, b)).second) continue;
                double len_um = lef.toMicrons(std::llabs(nodes[a].x - nodes[b].x) + std::llabs(nodes[a].y - nodes[b].y));
                if (len_um <= 0) continue;
                double r = r_sq * len_um / w_um;
                double c_pf = cap_scale * (c_area * len_um * w_um + 2.0 * c_edge * len_um);
                nodes[a].cap_pf += 0.5 * c_pf;
                nodes[b].cap_pf += 0.5 * c_pf;
                total_cap_pf += c_pf;
                resistors.push_back({a, b, r});
            }
        }

        std::vector<int> uf(nodes.size());
        for (size_t i = 0; i < uf.size(); ++i) uf[i] = static_cast<int>(i);
        auto findU = [&](int x) {
            while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; }
            return x;
        };
        for (const auto& r : resistors) uf[findU(r.a)] = findU(r.b);
        auto shapeNodes = [&](int layer, const Rect& r) {
            std::vector<int> contacts;
            if (validLayer(layer)) for (int k : layer_nodes[layer]) {
                if (inside(nodes[k].x, nodes[k].y, r)) contacts.push_back(k);
            }
            return contacts;
        };
        // Bridge only contacts on the actual metal rectangle, on its own layer.
        // Patch resistance is a sheet-resistance estimate; it never repairs an open.
        for (const auto& [layer, rect] : dn.patches) {
            if (!validLayer(layer)) continue;
            auto contacts = shapeNodes(layer, rect);
            if (contacts.empty()) throw std::runtime_error("SPEF: isolated patch on net " + nl.net_names[net]);
            const LefLayer& L = lef.layers[layer];
            double w_um = lef.toMicrons(rect.width()), h_um = lef.toMicrons(rect.height());
            double c_area = L.cap_per_area > 0 ? L.cap_per_area : 1.8e-5;
            double c_edge = L.edge_cap > 0 ? L.edge_cap : 4.5e-5;
            double c_pf = cap_scale * (c_area * w_um * h_um + c_edge * 2.0 * (w_um + h_um));
            total_cap_pf += c_pf;
            int a = contacts.front();
            for (int b : contacts) {
                nodes[b].cap_pf += c_pf / contacts.size();
                if (findU(a) == findU(b)) continue;
                double squares = lef.toMicrons(std::llabs(nodes[a].x - nodes[b].x)) / std::max(h_um, 0.001)
                               + lef.toMicrons(std::llabs(nodes[a].y - nodes[b].y)) / std::max(w_um, 0.001);
                resistors.push_back({a, b, std::max(0.01, (L.res_per_sq > 0 ? L.res_per_sq : 0.11) * squares)});
                uf[findU(b)] = findU(a);
            }
        }

        for (const auto& p : pins) {
            std::vector<int> contacts;
            for (const auto& shape : p.shapes) {
                auto touched = shapeNodes(shape.layer, shape.rect);
                contacts.insert(contacts.end(), touched.begin(), touched.end());
            }
            if (contacts.empty()) throw std::runtime_error("SPEF: no routed access for pin " + p.lsta_name);
            int best = contacts.front();
            for (int k : contacts) {
                Dbu d = std::llabs(nodes[k].x - p.x) + std::llabs(nodes[k].y - p.y);
                Dbu bd = std::llabs(nodes[best].x - p.x) + std::llabs(nodes[best].y - p.y);
                if (d < bd) best = k;
            }
            // Separate access points on the same logical pin are joined by its metal.
            for (int k : contacts) if (findU(k) != findU(best)) {
                resistors.push_back({best, k, 1.0});
                uf[findU(k)] = findU(best);
            }
            int p_id = static_cast<int>(nodes.size());
            nodes.push_back({-1, p.x, p.y, 0.0, p.spef_name});
            resistors.push_back({p_id, best, 1.0});
        }
        for (size_t k = 1; k < uf.size(); ++k) {
            if (findU(static_cast<int>(k)) != findU(0)) {
                throw std::runtime_error("SPEF: disconnected routed geometry on net " + nl.net_names[net]);
            }
        }

        out << "*D_NET " << net_esc << " " << total_cap_pf << "\n";
        out << "*CONN\n";
        for (const PinLoc& p : pins) {
            out << (p.is_port ? "*P " : "*I ") << p.spef_name << " "
                << (p.is_driver ? (p.is_port ? "I" : "O") : (p.is_port ? "O" : "I")) << "\n";
        }
        out << "*CAP\n";
        int cap_idx = 1;
        for (const NodeInfo& ni : nodes) {
            if (ni.cap_pf > 0.0) {
                out << cap_idx++ << " " << ni.name << " " << ni.cap_pf << "\n";
            }
        }
        out << "*RES\n";
        int res_idx = 1;
        for (const SegRes& r : resistors) {
            out << res_idx++ << " " << nodes[r.a].name << " " << nodes[r.b].name << " " << r.r_ohm << "\n";
        }
        out << "*END\n\n";
    }
}

SpefData readSpef(const std::string& path) {
    auto t0 = std::chrono::steady_clock::now();
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open SPEF file: " + path);

    SpefData data;
    double c_to_ff = 1000.0; // default 1 PF -> 1000 fF
    double r_to_ohm = 1.0;   // default 1 OHM -> 1 ohm
    char delim = ':';
    std::unordered_map<std::string, std::string> name_map;

    auto resolve = [&](const std::string& tok) -> std::string {
        if (!tok.empty() && tok[0] == '*') {
            size_t pos = tok.find(delim);
            if (pos == std::string::npos) {
                auto it = name_map.find(tok);
                return it != name_map.end() ? it->second : tok;
            }
            std::string head = tok.substr(0, pos);
            auto it = name_map.find(head);
            std::string base = it != name_map.end() ? it->second : head;
            return base + tok.substr(pos);
        }
        return tok;
    };

    auto toLstaPin = [&](const std::string& raw, bool is_port) -> std::string {
        std::string r = resolve(raw);
        if (is_port) return unescapeName(r);
        size_t pos = r.rfind(delim);
        if (pos == std::string::npos) return unescapeName(r);
        return unescapeName(r.substr(0, pos)) + "/" + unescapeName(r.substr(pos + 1));
    };

    enum Section { TOP, NAME_MAP, D_NET_CONN, D_NET_CAP, D_NET_RES };
    Section sec = TOP;

    std::string cur_net;
    double cur_total_cap_ff = 0.0;
    std::unordered_map<std::string, int> rc_node_id;
    std::vector<double> rc_cap_ff;
    std::vector<SpefResistor> rc_resistors;
    int driver_node = -1;
    std::vector<std::pair<int, std::string>> load_pins; // (rc_node, lsta_pin_name)
    double total_res_ohm = 0.0;

    auto getRcNode = [&](const std::string& name) -> int {
        auto it = rc_node_id.find(name);
        if (it != rc_node_id.end()) return it->second;
        int id = static_cast<int>(rc_cap_ff.size());
        rc_node_id[name] = id;
        rc_cap_ff.push_back(0.0);
        return id;
    };

    auto finishNet = [&]() {
        if (cur_net.empty()) return;
        NetParasitics np;
        np.wire_cap_ff = cur_total_cap_ff;
        np.wire_res_ohm = total_res_ohm;
        // Fallback lumped Elmore delay: 0.5 * R_wire * C_wire * 1e-3 ps
        np.default_elmore_ps = 0.5 * total_res_ohm * cur_total_cap_ff * 1e-3;
        np.driver_node = driver_node;
        np.node_cap_ff = std::move(rc_cap_ff);
        np.resistors = std::move(rc_resistors);
        for (const auto& [nid, pname] : load_pins) np.load_nodes[pname] = nid;
        np.pin_elmore_ps = np.elmoreDelays();
        data.nets[cur_net] = std::move(np);
        data.net_count++;
        cur_net.clear();
        cur_total_cap_ff = 0.0;
        rc_node_id.clear();
        rc_cap_ff.clear();
        rc_resistors.clear();
        driver_node = -1;
        load_pins.clear();
        total_res_ohm = 0.0;
    };

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        if (line[first] == '/' && first + 1 < line.size() && line[first + 1] == '/') continue;
        std::istringstream ss(line);
        std::string tok;
        ss >> tok;
        if (tok.empty()) continue;

        if (tok == "*C_UNIT") {
            double v = 1.0; std::string u;
            if (ss >> v >> u) {
                if (u == "PF") c_to_ff = v * 1000.0;
                else if (u == "FF") c_to_ff = v;
                else if (u == "NF") c_to_ff = v * 1e6;
            }
            continue;
        }
        if (tok == "*R_UNIT") {
            double v = 1.0; std::string u;
            if (ss >> v >> u) {
                if (u == "KOHM") r_to_ohm = v * 1000.0;
                else if (u == "OHM") r_to_ohm = v;
            }
            continue;
        }
        if (tok == "*DELIMITER") {
            std::string d;
            if (ss >> d && !d.empty()) delim = d[0];
            continue;
        }
        if (tok == "*NAME_MAP") { sec = NAME_MAP; continue; }
        if (tok == "*D_NET") {
            finishNet();
            std::string raw_net;
            double cap = 0.0;
            if (ss >> raw_net >> cap) {
                cur_net = unescapeName(resolve(raw_net));
                cur_total_cap_ff = cap * c_to_ff;
            }
            sec = TOP;
            continue;
        }
        if (tok == "*CONN") { sec = D_NET_CONN; continue; }
        if (tok == "*CAP") { sec = D_NET_CAP; continue; }
        if (tok == "*RES") { sec = D_NET_RES; continue; }
        if (tok == "*END") { finishNet(); sec = TOP; continue; }

        if (sec == NAME_MAP && tok[0] == '*') {
            std::string val;
            if (ss >> val) name_map[tok] = val;
        } else if (sec == D_NET_CONN && (tok == "*I" || tok == "*P")) {
            bool is_port = (tok == "*P");
            std::string raw_pin, dir;
            if (ss >> raw_pin >> dir) {
                std::string rpin = resolve(raw_pin);
                int nid = getRcNode(rpin);
                bool is_drv = is_port ? (dir == "I") : (dir == "O");
                if (is_drv && driver_node < 0) driver_node = nid;
                else load_pins.emplace_back(nid, toLstaPin(raw_pin, is_port));
            }
        } else if (sec == D_NET_CAP) {
            std::string n1, n2;
            if (!(ss >> n1)) continue;
            std::string third;
            if (!(ss >> third)) continue;
            // Grounded cap: idx node val; Coupling cap: idx node1 node2 val
            double val = 0.0;
            std::string fourth;
            if (ss >> fourth) {
                try { val = std::stod(fourth) * c_to_ff; } catch (...) { continue; }
                int u = getRcNode(resolve(n1));
                int v = getRcNode(resolve(third));
                rc_cap_ff[u] += val;
                rc_cap_ff[v] += val;
            } else {
                try { val = std::stod(third) * c_to_ff; } catch (...) { continue; }
                int u = getRcNode(resolve(n1));
                rc_cap_ff[u] += val;
            }
        } else if (sec == D_NET_RES) {
            std::string n1, n2;
            double val = 0.0;
            if (ss >> n1 >> n2 >> val) {
                double r_ohm = std::max(0.0, val * r_to_ohm);
                int u = getRcNode(resolve(n1));
                int v = getRcNode(resolve(n2));
                rc_resistors.push_back({u, v, r_ohm});
                total_res_ohm += r_ohm;
            }
        }
    }
    finishNet();
    data.ms_read = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return data;
}
