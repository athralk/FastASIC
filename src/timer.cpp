#include "../include/timer.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr Edge kEdges[] = {RISE, FALL};

enum class NodeKind { PORT_IN, PORT_OUT, PIN_IN, PIN_OUT };

struct Node {
    NodeKind kind;
    int net = -1;
    int inst = -1, pin = -1;   // instance pins
    int port = -1;             // ports
};

struct Arc {
    int from, to;
    const LibArc* arc;
};

struct Candidate {
    int node;
    Edge edge;
    bool max;
    std::string check;
    double arrival, required, slack;
    double clock_time, clock_latency, margin, uncertainty;
};

std::string baseName(const std::string& bit) {
    size_t b = bit.find('[');
    return b == std::string::npos ? bit : bit.substr(0, b);
}

class Timer {
    const CellNetlist& nl;
    const Library& lib;
    const Sdc& sdc;
    TimerOptions options;
    TimingResult res;

    std::vector<Node> nodes;
    std::vector<int> inst_base;
    std::vector<int> net_driver;
    std::vector<std::vector<int>> net_loads;
    std::vector<std::array<double, 4>> net_cap;   // [edge * 2 + mm], mm 0 = max (late), 1 = min (early)
    std::vector<std::vector<Arc>> fanin, fanout;   // cell arcs by output / input node
    std::vector<std::vector<Arc>> checks;          // by data node: from = clock pin node
    std::vector<int> order;
    std::vector<uint8_t> clock_net, fixed;
    std::vector<uint8_t> clock_inverted;   // clock network node sees the inverted clock
    // Each pin's slew is in its own library's terms (slew thresholds / derate). Where a net
    // connects a driver and a load from different libraries the slew is converted by the ratio
    // of their slew fractions, as OpenSTA's DelayCalcBase::thresholdAdjust does; ports use the
    // main (first) library.
    std::vector<double> slew_frac;

    // Timing state, indexed node * 2 + edge
    std::vector<double> arr_max, arr_min, slew_max, slew_min;
    std::vector<int> pred_max, pred_min;

    const SdcClock* clock = nullptr;
    std::vector<uint8_t> is_clock_source;   // by port bit

    bool portMatches(const std::string& pattern, size_t port) const {
        const auto& pb = nl.port_bits[port];
        if (pattern == "@inputs") return pb.is_input;
        if (pattern == "@inputs_no_clocks") return pb.is_input && !is_clock_source[port];
        if (pattern == "@outputs") return !pb.is_input;
        return globMatch(pattern, pb.name) || globMatch(pattern, baseName(pb.name));
    }
    bool anyMatch(const std::vector<std::string>& patterns, size_t port) const {
        for (const auto& p : patterns) {
            if (portMatches(p, port)) return true;
        }
        return false;
    }

    std::string nodeName(int n) const {
        const Node& node = nodes[n];
        if (node.port >= 0) return nl.port_bits[node.port].name;
        const auto& inst = nl.instances[node.inst];
        return inst.name + "/" + inst.cell->pins[node.pin].name;
    }

    void build() {
        size_t ports = nl.port_bits.size();
        is_clock_source.assign(ports, 0);
        if (!sdc.clocks.empty()) {
            clock = &sdc.clocks[0];
            if (sdc.clocks.size() > 1) res.warnings.push_back("only the first clock (" + clock->name + ") is analyzed");
            for (size_t p = 0; p < ports; ++p) {
                if (nl.port_bits[p].is_input && anyMatch(clock->sources, p)) is_clock_source[p] = 1;
            }
        }

        for (size_t p = 0; p < ports; ++p) {
            Node n;
            n.kind = nl.port_bits[p].is_input ? NodeKind::PORT_IN : NodeKind::PORT_OUT;
            n.net = nl.port_bits[p].net;
            n.port = static_cast<int>(p);
            nodes.push_back(n);
        }
        for (size_t i = 0; i < nl.instances.size(); ++i) {
            const auto& inst = nl.instances[i];
            inst_base.push_back(static_cast<int>(nodes.size()));
            for (size_t k = 0; k < inst.cell->pins.size(); ++k) {
                Node n;
                n.kind = inst.cell->pins[k].dir == PinDir::OUTPUT ? NodeKind::PIN_OUT : NodeKind::PIN_IN;
                n.net = inst.pin_net[k];
                n.inst = static_cast<int>(i);
                n.pin = static_cast<int>(k);
                nodes.push_back(n);
            }
            res.area += inst.cell->area;
            res.leakage += inst.cell->leakage;
            if (inst.cell->is_sequential) res.registers++;
        }
        res.cells = nl.instances.size();
        res.nets = nl.netCount();
        slew_frac.assign(nodes.size(), lib.slew_fraction);
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (nodes[n].inst >= 0) slew_frac[n] = nl.instances[nodes[n].inst].cell->slew_fraction;
        }

        size_t n_nodes = nodes.size();
        net_driver.assign(nl.netCount(), -1);
        net_loads.assign(nl.netCount(), {});
        net_cap.assign(nl.netCount(), {0.0, 0.0, 0.0, 0.0});
        fanin.assign(n_nodes, {});
        fanout.assign(n_nodes, {});
        checks.assign(n_nodes, {});

        for (size_t n = 0; n < n_nodes; ++n) {
            const Node& node = nodes[n];
            if (node.net < 0) continue;
            bool drives = node.kind == NodeKind::PORT_IN || node.kind == NodeKind::PIN_OUT;
            if (drives) {
                if (node.net <= CellNetlist::kConst1) continue;
                if (net_driver[node.net] >= 0) {
                    throw std::runtime_error("net '" + nl.net_names[node.net] + "' has several drivers (" +
                                             nodeName(net_driver[node.net]) + ", " + nodeName(static_cast<int>(n)) + ")");
                }
                net_driver[node.net] = static_cast<int>(n);
            } else {
                net_loads[node.net].push_back(static_cast<int>(n));
                if (node.kind == NodeKind::PIN_IN) {
                    const LibPin& pin = nl.instances[node.inst].cell->pins[node.pin];
                    net_cap[node.net][RISE * 2 + 0] += pin.rise_cap_max;
                    net_cap[node.net][RISE * 2 + 1] += pin.rise_cap_min;
                    net_cap[node.net][FALL * 2 + 0] += pin.fall_cap_max;
                    net_cap[node.net][FALL * 2 + 1] += pin.fall_cap_min;
                }
            }
        }
        // External loads on output ports (the last matching set_load wins)
        for (size_t p = 0; p < ports; ++p) {
            if (nl.port_bits[p].is_input) continue;
            double load = 0;
            for (const auto& l : sdc.loads) {
                if (anyMatch(l.ports, p)) load = l.value;
            }
            int net = nl.port_bits[p].net;
            if (net > CellNetlist::kConst1) {
                for (double& c : net_cap[net]) c += load;
            }
        }

        for (size_t i = 0; i < nl.instances.size(); ++i) {
            const auto& inst = nl.instances[i];
            for (const LibArc& arc : inst.cell->arcs) {
                if (inst.pin_net[arc.from_pin] < 0) continue;
                int from = inst_base[i] + arc.from_pin, to = inst_base[i] + arc.to_pin;
                switch (arc.type) {
                case TimingType::CLEAR: case TimingType::PRESET:
                    // Like OpenSTA (sta_preset_clear_arcs_enabled = 0): resets are timed only by
                    // their recovery/removal checks, not as data paths through the register
                    break;
                case TimingType::COMBINATIONAL:
                case TimingType::RISING_EDGE: case TimingType::FALLING_EDGE:
                    fanin[to].push_back({from, to, &arc});
                    fanout[from].push_back({from, to, &arc});
                    break;
                default:
                    if (isCheck(arc.type) && inst.pin_net[arc.to_pin] >= 0) checks[to].push_back({from, to, &arc});
                    break;
                }
            }
        }

        // Topological order of the timing graph (net edges + cell arcs)
        std::vector<int> indegree(n_nodes, 0);
        for (size_t n = 0; n < n_nodes; ++n) {
            indegree[n] += static_cast<int>(fanin[n].size());
            const Node& node = nodes[n];
            bool is_load = node.kind == NodeKind::PIN_IN || node.kind == NodeKind::PORT_OUT;
            if (is_load && node.net > CellNetlist::kConst1 && net_driver[node.net] >= 0) indegree[n]++;
        }
        std::vector<int> queue;
        for (size_t n = 0; n < n_nodes; ++n) {
            if (indegree[n] == 0) queue.push_back(static_cast<int>(n));
        }
        order.reserve(n_nodes);
        for (size_t head = 0; head < queue.size(); ++head) {
            int n = queue[head];
            order.push_back(n);
            const Node& node = nodes[n];
            bool drives = node.kind == NodeKind::PORT_IN || node.kind == NodeKind::PIN_OUT;
            if (drives && node.net > CellNetlist::kConst1 && net_driver[node.net] == n) {
                for (int load : net_loads[node.net]) {
                    if (--indegree[load] == 0) queue.push_back(load);
                }
            }
            for (const Arc& a : fanout[n]) {
                if (--indegree[a.to] == 0) queue.push_back(a.to);
            }
        }
        if (order.size() != n_nodes) {
            for (size_t n = 0; n < n_nodes; ++n) {
                if (indegree[n] > 0) throw std::runtime_error("combinational loop through " + nodeName(static_cast<int>(n)));
            }
        }

        // Clock network: forward from the clock ports through combinational cells to register clock pins
        clock_net.assign(n_nodes, 0);
        clock_inverted.assign(n_nodes, 0);
        std::vector<int> work;
        for (size_t p = 0; p < ports; ++p) {
            if (is_clock_source[p]) {
                clock_net[p] = 1;
                work.push_back(static_cast<int>(p));
            }
        }
        while (!work.empty()) {
            int n = work.back();
            work.pop_back();
            const Node& node = nodes[n];
            bool drives = node.kind == NodeKind::PORT_IN || node.kind == NodeKind::PIN_OUT;
            if (drives && node.net > CellNetlist::kConst1) {
                for (int load : net_loads[node.net]) {
                    if (!clock_net[load]) {
                        clock_net[load] = 1;
                        clock_inverted[load] = clock_inverted[n];
                        work.push_back(load);
                    }
                }
            } else if (node.kind == NodeKind::PIN_IN) {
                for (const Arc& a : fanout[n]) {   // through buffers / inverters / gates, not through registers
                    if (a.arc->type == TimingType::COMBINATIONAL && !clock_net[a.to]) {
                        clock_net[a.to] = 1;
                        clock_inverted[a.to] = clock_inverted[n] ^ (a.arc->sense == TimingSense::NEGATIVE ? 1 : 0);
                        if (a.arc->sense == TimingSense::NON) res.warnings.push_back("clock passes through a non-unate cell (" + nodeName(a.to) + ")");
                        work.push_back(a.to);
                    }
                }
            }
        }
    }

    // Delay and slew through the SDC driving cell of an input port, for output edge `out`
    bool drivingCell(size_t port, Edge out, double load, double& delay, double& slew) const {
        const SdcDrivingCell* drive = nullptr;
        for (const auto& d : sdc.driving_cells) {
            if (anyMatch(d.ports, port)) drive = &d;
        }
        if (!drive) return false;
        const LibCell* cell = lib.find(drive->cell);
        if (!cell) throw std::runtime_error("set_driving_cell: unknown cell " + drive->cell);
        int out_pin = drive->pin.empty() ? cell->outputPin() : cell->pinIndex(drive->pin);
        int from_pin = drive->from_pin.empty() ? -1 : cell->pinIndex(drive->from_pin);
        delay = -kInf;
        slew = -kInf;
        for (const LibArc& arc : cell->arcs) {
            if (arc.to_pin != out_pin || arc.type != TimingType::COMBINATIONAL) continue;
            if (from_pin >= 0 && arc.from_pin != from_pin) continue;
            const LibTable& d = out == RISE ? arc.cell_rise : arc.cell_fall;
            const LibTable& s = out == RISE ? arc.rise_transition : arc.fall_transition;
            if (d.empty()) continue;
            // As OpenSTA / PrimeTime: the port sees the driver's full output slew, but only the
            // load-dependent part of its delay (the intrinsic delay is part of the input delay)
            TableArgs args;
            args.input_transition = 0;
            args.output_cap = 0;
            double unloaded = d.lookup(args);
            args.output_cap = load;
            delay = std::max(delay, d.lookup(args) - unloaded);
            slew = std::max(slew, s.lookup(args));
        }
        if (delay == -kInf) {
            delay = 0;
            slew = 0;
        }
        return true;
    }

    double edgeTime(Edge e) const { return clock ? (e == RISE ? clock->rise : clock->fall) : 0.0; }

    // Ideal time of clock edge e as seen at clock network node n (an inverter swaps the edges)
    double phaseEdge(int n, Edge e) const {
        bool inv = n >= 0 && clock_inverted[n];
        return edgeTime(inv ? (e == RISE ? FALL : RISE) : e);
    }

    // Edge time that launched the path ending at timing index idx: the clock edge at the
    // launching register's clock pin, or the clock's rising edge for input ports
    double launchEdge(int idx, const std::vector<int>& pred) const {
        int launch = launchClockPin(idx, pred);
        if (launch < 0) return edgeTime(RISE);
        return phaseEdge(launch / 2, static_cast<Edge>(launch % 2));
    }

    // Setup captures at the first capture edge after the launch edge, hold checks against the
    // last one at or before it (so rising-to-falling paths get half a cycle)
    double setupEdge(double capture0, double launch) const {
        double t = capture0;
        while (t <= launch + 1e-9) t += clock->period;
        while (t - clock->period > launch + 1e-9) t -= clock->period;
        return t;
    }
    double holdEdge(double capture0, double launch) const {
        double t = capture0;
        while (t > launch + 1e-9) t -= clock->period;
        while (t + clock->period <= launch + 1e-9) t += clock->period;
        return t;
    }

    void initialize() {
        size_t n2 = nodes.size() * 2;
        arr_max.assign(n2, -kInf);
        arr_min.assign(n2, kInf);
        slew_max.assign(n2, 0);
        slew_min.assign(n2, kInf);
        pred_max.assign(n2, -1);
        pred_min.assign(n2, -1);
        fixed.assign(nodes.size(), 0);

        bool propagated = clock && clock->propagated;
        res.propagated_clock = propagated;
        if (clock) {
            res.clock_name = clock->name;
            res.period = clock->period;
        }

        for (size_t n = 0; n < nodes.size(); ++n) {
            if (!clock_net[n]) continue;
            bool source = nodes[n].kind == NodeKind::PORT_IN;
            if (propagated && !source) continue;
            fixed[n] = 1;
            // An ideal clock has the SDC clock transition everywhere; a propagated clock starts
            // with the port's input transition and gets real slews from the tree
            double slew = clock->transition;   // ideal clock slews are used as given, at every clock pin
            if (propagated) {
                slew = 0;
                for (const auto& t : sdc.input_transitions) {
                    if (anyMatch(t.ports, static_cast<size_t>(nodes[n].port))) slew = t.value;
                }
            }
            for (Edge e : kEdges) {
                double t = (propagated ? edgeTime(e) : phaseEdge(static_cast<int>(n), e) + clock->latency);
                set(static_cast<int>(n), e, t, t, slew, slew);
            }
        }

        for (size_t p = 0; p < nl.port_bits.size(); ++p) {
            const auto& pb = nl.port_bits[p];
            if (!pb.is_input || is_clock_source[p] || pb.net <= CellNetlist::kConst1) continue;
            bool has_max = false, has_min = false;
            double d_max = 0, d_min = 0;
            for (const auto& d : sdc.input_delays) {
                if (!anyMatch(d.ports, p)) continue;
                if (d.max) { d_max = d.value; has_max = true; }
                if (d.min) { d_min = d.value; has_min = true; }
            }
            if (!has_max && !has_min) continue;   // unconstrained input, as in OpenSTA
            double transition = 0;
            for (const auto& t : sdc.input_transitions) {
                if (anyMatch(t.ports, p)) transition = t.value;
            }
            fixed[p] = 1;
            for (Edge e : kEdges) {
                double dmax = 0, smax = transition, dmin = 0, smin = transition;
                drivingCell(p, e, net_cap[pb.net][e * 2 + 0], dmax, smax);
                drivingCell(p, e, net_cap[pb.net][e * 2 + 1], dmin, smin);
                double base = edgeTime(RISE);
                set(static_cast<int>(p), e, has_max ? base + d_max + dmax : -kInf, has_min ? base + d_min + dmin : kInf, smax, smin);
            }
        }
    }

    void set(int n, Edge e, double amax, double amin, double smax, double smin) {
        size_t i = static_cast<size_t>(n) * 2 + e;
        arr_max[i] = amax;
        arr_min[i] = amin;
        slew_max[i] = smax;
        slew_min[i] = smin;
    }

    void propagate() {
        for (int n : order) {
            if (fixed[n]) continue;
            const Node& node = nodes[n];
            if (node.kind == NodeKind::PIN_IN || node.kind == NodeKind::PORT_OUT) {
                if (node.net <= CellNetlist::kConst1) continue;
                int d = net_driver[node.net];
                if (d < 0) continue;
                double convert = slew_frac[n] / slew_frac[d];   // 1 unless the libraries differ
                for (Edge e : kEdges) {   // ideal wire: same arrival and slew as the driver
                    size_t i = static_cast<size_t>(n) * 2 + e, j = static_cast<size_t>(d) * 2 + e;
                    arr_max[i] = arr_max[j];
                    arr_min[i] = arr_min[j];
                    slew_max[i] = slew_max[j] * convert;
                    slew_min[i] = slew_min[j] < kInf ? slew_min[j] * convert : kInf;
                    pred_max[i] = pred_min[i] = static_cast<int>(j);
                }
                continue;
            }
            if (node.kind != NodeKind::PIN_OUT) continue;
            std::array<double, 4> cap = node.net > CellNetlist::kConst1 ? net_cap[node.net] : std::array<double, 4>{0.0, 0.0, 0.0, 0.0};
            for (const Arc& a : fanin[n]) {
                if (clock_net[n] && !clock_net[a.from]) continue;   // clock tree: only clock-to-clock arcs
                const LibArc& arc = *a.arc;
                for (Edge in : kEdges) {
                    if (arc.type == TimingType::RISING_EDGE && in != RISE) continue;
                    if (arc.type == TimingType::FALLING_EDGE && in != FALL) continue;
                    size_t fi = static_cast<size_t>(a.from) * 2 + in;
                    bool reach_max = arr_max[fi] > -kInf, reach_min = arr_min[fi] < kInf;
                    if (!reach_max && !reach_min) continue;
                    for (Edge out : kEdges) {
                        if (arc.sense == TimingSense::POSITIVE && out != in) continue;
                        if (arc.sense == TimingSense::NEGATIVE && out == in) continue;
                        const LibTable& dt = out == RISE ? arc.cell_rise : arc.cell_fall;
                        const LibTable& st = out == RISE ? arc.rise_transition : arc.fall_transition;
                        if (dt.empty()) continue;
                        size_t oi = static_cast<size_t>(n) * 2 + out;
                        TableArgs args;
                        if (reach_max) {
                            args.output_cap = cap[out * 2 + 0];
                            args.input_transition = slew_max[fi];
                            double t = arr_max[fi] + dt.lookup(args);
                            if (t > arr_max[oi]) {
                                arr_max[oi] = t;
                                pred_max[oi] = static_cast<int>(fi);
                            }
                            slew_max[oi] = std::max(slew_max[oi], st.lookup(args));
                        }
                        if (reach_min) {
                            args.output_cap = cap[out * 2 + 1];
                            args.input_transition = slew_min[fi];
                            double t = arr_min[fi] + dt.lookup(args);
                            if (t < arr_min[oi]) {
                                arr_min[oi] = t;
                                pred_min[oi] = static_cast<int>(fi);
                            }
                            slew_min[oi] = std::min(slew_min[oi], st.lookup(args));
                        }
                    }
                }
            }
        }
    }

    // The register clock pin that launched the path ending at timing index `idx`, or -1
    int launchClockPin(int idx, const std::vector<int>& pred) const {
        for (int i = idx; i >= 0; i = pred[i]) {
            int n = i / 2;
            if (clock_net[n] && nodes[n].kind == NodeKind::PIN_IN) return i;
        }
        return -1;
    }

    // Clock reconvergence pessimism: launch and capture clock paths share the tree up to the
    // point where they split; a shared buffer cannot be late for one and early for the other,
    // so the late - early difference at that point is credited back (as OpenSTA does)
    double crpr(int launch_idx, int capture_idx) const {
        if (!clock || !clock->propagated || launch_idx < 0) return 0;
        std::vector<int> ancestors;
        for (int i = launch_idx; i >= 0; i = pred_max[i]) ancestors.push_back(i);
        std::sort(ancestors.begin(), ancestors.end());
        for (int i = capture_idx; i >= 0; i = pred_max[i]) {
            if (std::binary_search(ancestors.begin(), ancestors.end(), i)) return arr_max[i] - arr_min[i];
        }
        return 0;
    }

    void checkAll(std::vector<Candidate>& cands) {
        for (size_t n = 0; n < nodes.size(); ++n) {
            for (const Arc& a : checks[n]) {
                const LibArc& arc = *a.arc;
                int c = a.from;
                if (!clock_net[c]) continue;   // checked against a pin that is not a clock
                bool rising = arc.type == TimingType::SETUP_RISING || arc.type == TimingType::HOLD_RISING ||
                              arc.type == TimingType::RECOVERY_RISING || arc.type == TimingType::REMOVAL_RISING;
                Edge ce = rising ? RISE : FALL;
                bool max = arc.type == TimingType::SETUP_RISING || arc.type == TimingType::SETUP_FALLING ||
                           arc.type == TimingType::RECOVERY_RISING || arc.type == TimingType::RECOVERY_FALLING;
                bool async = arc.type == TimingType::RECOVERY_RISING || arc.type == TimingType::RECOVERY_FALLING ||
                             arc.type == TimingType::REMOVAL_RISING || arc.type == TimingType::REMOVAL_FALLING;
                std::string kind = max ? (async ? "recovery" : "setup") : (async ? "removal" : "hold");
                size_t ci = static_cast<size_t>(c) * 2 + ce;
                for (Edge e : kEdges) {
                    const LibTable& table = e == RISE ? arc.rise_constraint : arc.fall_constraint;
                    if (table.empty()) continue;
                    size_t di = n * 2 + e;
                    TableArgs args;
                    if (max) {
                        if (arr_max[di] == -kInf) continue;
                        args.constrained_transition = slew_max[di];
                        args.related_transition = slew_min[ci];
                        double margin = table.lookup(args);
                        double edge0 = phaseEdge(c, ce);
                        double edge = setupEdge(edge0, launchEdge(static_cast<int>(di), pred_max));
                        double capture = arr_min[ci] - edge0 + edge;
                        double credit = crpr(launchClockPin(static_cast<int>(di), pred_max), static_cast<int>(ci));
                        double required = capture - margin - clock->uncertainty_setup + credit;
                        cands.push_back({static_cast<int>(n), e, true, kind, arr_max[di], required, required - arr_max[di],
                                         edge, arr_min[ci] - edge0 + credit, margin, clock->uncertainty_setup});
                    } else {
                        if (arr_min[di] == kInf) continue;
                        args.constrained_transition = slew_min[di];
                        args.related_transition = slew_max[ci];
                        double margin = table.lookup(args);
                        double edge0 = phaseEdge(c, ce);
                        double edge = holdEdge(edge0, launchEdge(static_cast<int>(di), pred_min));
                        double capture = arr_max[ci] - edge0 + edge;
                        double credit = crpr(launchClockPin(static_cast<int>(di), pred_min), static_cast<int>(ci));
                        double required = capture + margin + clock->uncertainty_hold - credit;
                        cands.push_back({static_cast<int>(n), e, false, kind, arr_min[di], required, arr_min[di] - required,
                                         edge, arr_max[ci] - edge0 - credit, margin, clock->uncertainty_hold});
                    }
                }
            }
        }

        // Output ports, against the clock their output delay names (the single clock)
        for (size_t p = 0; p < nl.port_bits.size() && clock; ++p) {
            const auto& pb = nl.port_bits[p];
            if (pb.is_input) continue;
            bool has_max = false, has_min = false;
            double d_max = 0, d_min = 0;
            for (const auto& d : sdc.output_delays) {
                if (!anyMatch(d.ports, p)) continue;
                if (d.max) { d_max = d.value; has_max = true; }
                if (d.min) { d_min = d.value; has_min = true; }
            }
            double latency = clock->propagated ? 0 : clock->latency;
            for (Edge e : kEdges) {
                size_t i = p * 2 + e;
                if (has_max && arr_max[i] > -kInf) {
                    double edge = setupEdge(edgeTime(RISE), launchEdge(static_cast<int>(i), pred_max));
                    double required = edge + latency - d_max - clock->uncertainty_setup;
                    cands.push_back({static_cast<int>(p), e, true, "setup", arr_max[i], required, required - arr_max[i],
                                     edge, latency, d_max, clock->uncertainty_setup});
                }
                if (has_min && arr_min[i] < kInf) {
                    double edge = holdEdge(edgeTime(RISE), launchEdge(static_cast<int>(i), pred_min));
                    double required = edge + latency - d_min + clock->uncertainty_hold;
                    cands.push_back({static_cast<int>(p), e, false, "hold", arr_min[i], required, arr_min[i] - required,
                                     edge, latency, d_min, clock->uncertainty_hold});
                }
            }
        }
    }

    TimingPath trace(const Candidate& c) {
        TimingPath path;
        path.check = c.check;
        path.max = c.max;
        path.arrival = c.arrival;
        path.required = c.required;
        path.slack = c.slack;
        path.clock_time = c.clock_time;
        path.clock_latency = c.clock_latency;
        path.margin = c.margin;
        path.uncertainty = c.uncertainty;

        const auto& arr = c.max ? arr_max : arr_min;
        const auto& slew = c.max ? slew_max : slew_min;
        const auto& pred = c.max ? pred_max : pred_min;
        std::vector<int> chain;
        for (int i = c.node * 2 + c.edge; i >= 0; i = pred[i]) {
            chain.push_back(i);
            if (clock_net[i / 2] && nodes[i / 2].kind == NodeKind::PIN_IN) break;   // a register clock pin
        }
        std::reverse(chain.begin(), chain.end());
        double previous = 0;
        for (size_t k = 0; k < chain.size(); ++k) {
            int i = chain[k];
            int n = i / 2;
            const Node& node = nodes[n];
            PathPoint pt;
            pt.pin = nodeName(n);
            pt.cell = node.inst >= 0 ? nl.instances[node.inst].cell->name : "";
            pt.edge = static_cast<Edge>(i % 2);
            pt.arrival = arr[i];
            pt.slew = slew[i];   // in the pin's own library terms, as OpenSTA reports it
            pt.is_driver = node.kind == NodeKind::PORT_IN || node.kind == NodeKind::PIN_OUT;
            pt.cap = pt.is_driver && node.net > CellNetlist::kConst1 ? net_cap[node.net][pt.edge * 2 + (c.max ? 0 : 1)] : 0;
            pt.delay = k == 0 ? arr[i] : arr[i] - previous;
            previous = arr[i];
            path.points.push_back(pt);
        }
        int start = chain.front() / 2;
        path.startpoint = nodeName(start);
        if (nodes[start].kind == NodeKind::PORT_IN) {
            path.start_kind = "input port clocked by " + res.clock_name;
        } else {
            path.startpoint = nl.instances[nodes[start].inst].name;
            path.start_kind = "rising edge-triggered flip-flop clocked by " + res.clock_name;
        }
        const Node& end = nodes[c.node];
        if (end.kind == NodeKind::PORT_OUT) {
            path.endpoint = nodeName(c.node);
            path.end_kind = "output port clocked by " + res.clock_name;
        } else {
            path.endpoint = nodeName(c.node);
            path.end_kind = (c.check == "recovery" || c.check == "removal") ? c.check + " check against rising-edge clock " + res.clock_name
                                                                            : "rising edge-triggered flip-flop clocked by " + res.clock_name;
        }
        return path;
    }

public:
    Timer(const CellNetlist& n, const Library& l, const Sdc& s, const TimerOptions& o) : nl(n), lib(l), sdc(s), options(o) {}

    TimingResult run() {
        auto t0 = std::chrono::steady_clock::now();
        build();
        auto t1 = std::chrono::steady_clock::now();
        initialize();
        propagate();
        std::vector<Candidate> cands;
        checkAll(cands);

        // Per endpoint pin: worst over its edges and check kinds, for WNS / TNS
        std::map<std::pair<int, bool>, double> worst;
        res.wns_setup = res.wns_hold = kInf;
        for (const Candidate& c : cands) {
            auto key = std::make_pair(c.node, c.max);
            auto it = worst.find(key);
            if (it == worst.end() || c.slack < it->second) worst[key] = c.slack;
            res.endpoints.push_back({nodeName(c.node), c.check, c.max, c.edge, c.arrival, c.required, c.slack});
        }
        res.tns_setup = res.tns_hold = 0;
        for (const auto& [key, slack] : worst) {
            if (key.second) {
                res.setup_endpoints++;
                res.wns_setup = std::min(res.wns_setup, slack);
                if (slack < 0) { res.tns_setup += slack; res.setup_violations++; }
            } else {
                res.hold_endpoints++;
                res.wns_hold = std::min(res.wns_hold, slack);
                if (slack < 0) { res.tns_hold += slack; res.hold_violations++; }
            }
        }
        if (res.setup_endpoints && clock) res.min_period = clock->period - res.wns_setup;

        // Clock skew over register clock pins
        double lo = kInf, hi = -kInf;
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (!checks[n].empty()) {
                for (const Arc& a : checks[n]) {
                    if (!clock_net[a.from]) continue;
                    size_t ci = static_cast<size_t>(a.from) * 2 + RISE;
                    lo = std::min(lo, arr_min[ci]);
                    hi = std::max(hi, arr_max[ci]);
                }
            }
        }
        res.clock_skew = hi > lo ? hi - lo : 0;

        std::stable_sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) { return a.slack < b.slack; });
        std::map<int, bool> seen_max, seen_min;   // one path per endpoint pin
        for (const Candidate& c : cands) {
            auto& seen = c.max ? seen_max : seen_min;
            auto& paths = c.max ? res.setup_paths : res.hold_paths;
            if (static_cast<int>(paths.size()) >= options.paths || seen[c.node]) continue;
            seen[c.node] = true;
            paths.push_back(trace(c));
        }
        auto t2 = std::chrono::steady_clock::now();
        res.ms_build = std::chrono::duration<double, std::milli>(t1 - t0).count();
        res.ms_analyze = std::chrono::duration<double, std::milli>(t2 - t1).count();
        std::sort(res.warnings.begin(), res.warnings.end());
        res.warnings.erase(std::unique(res.warnings.begin(), res.warnings.end()), res.warnings.end());
        return std::move(res);
    }
};

std::string f2(double v) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(2) << v;
    return s.str();
}

} // namespace

TimingResult analyzeNetlistTiming(const CellNetlist& netlist, const Library& lib, const Sdc& sdc, const TimerOptions& options) {
    return Timer(netlist, lib, sdc, options).run();
}

void printTimingPath(std::ostream& out, const TimingPath& p) {
    out << "Startpoint: " << p.startpoint << " (" << p.start_kind << ")\n";
    out << "Endpoint:   " << p.endpoint << " (" << p.end_kind << ")\n";
    out << "Path type:  " << (p.max ? "max" : "min") << " (" << p.check << ")\n\n";
    out << "     Cap     Slew    Delay     Time    Description\n";
    out << "  ------  -------  -------  -------    ----------------------------------------\n";
    for (const PathPoint& pt : p.points) {
        out << std::setw(8) << (pt.is_driver ? f2(pt.cap) : "") << std::setw(9) << f2(pt.slew) << std::setw(9) << f2(pt.delay)
            << std::setw(9) << f2(pt.arrival) << " " << (pt.edge == RISE ? "^" : "v") << "  " << pt.pin
            << (pt.cell.empty() ? (pt.is_driver ? " (in)" : " (out)") : " (" + pt.cell + ")") << "\n";
    }
    out << std::setw(35) << f2(p.arrival) << "    data arrival time\n\n";
    out << std::setw(35) << f2(p.clock_time) << "    clock edge\n";
    out << std::setw(35) << f2(p.clock_latency) << "    clock network delay\n";
    if (p.uncertainty != 0) out << std::setw(35) << f2(p.max ? -p.uncertainty : p.uncertainty) << "    clock uncertainty\n";
    std::string margin_name = p.end_kind.rfind("output", 0) == 0 ? "output external delay" : (p.check + " time");
    double margin = p.end_kind.rfind("output", 0) == 0 ? -p.margin : (p.max ? -p.margin : p.margin);
    out << std::setw(35) << f2(margin) << "    " << margin_name << "\n";
    out << std::setw(35) << f2(p.required) << "    data required time\n";
    out << std::setw(35) << f2(p.slack) << "    slack (" << (p.slack >= 0 ? "MET" : "VIOLATED") << ")\n\n";
}

void printTimingResult(std::ostream& out, const TimingResult& r) {
    out << "Design:   " << r.cells << " cells (" << r.registers << " registers), " << r.nets << " nets, area " << f2(r.area)
        << " um^2, leakage " << f2(r.leakage / 1000) << " nW\n";
    if (r.clock_name.empty()) {
        out << "Clock:    none defined (no paths are constrained)\n";
    } else {
        out << "Clock:    " << r.clock_name << ", period " << f2(r.period) << " ps (" << (r.propagated_clock ? "propagated" : "ideal")
            << "), skew " << f2(r.clock_skew) << " ps\n";
    }
    for (const auto& w : r.warnings) out << "Warning:  " << w << "\n";
    out << "\n";
    for (const auto& p : r.setup_paths) printTimingPath(out, p);
    for (const auto& p : r.hold_paths) printTimingPath(out, p);

    auto wns = [](double v) { return v == std::numeric_limits<double>::infinity() ? std::string("-") : f2(v); };
    out << "Summary (ps)        WNS        TNS   endpoints  violations\n";
    out << "  setup     " << std::setw(11) << wns(r.wns_setup) << std::setw(11) << f2(r.tns_setup) << std::setw(12) << r.setup_endpoints
        << std::setw(12) << r.setup_violations << "\n";
    out << "  hold      " << std::setw(11) << wns(r.wns_hold) << std::setw(11) << f2(r.tns_hold) << std::setw(12) << r.hold_endpoints
        << std::setw(12) << r.hold_violations << "\n";
    if (r.min_period > 0) {
        out << "Minimum period " << f2(r.min_period) << " ps  ->  Fmax " << f2(1e6 / r.min_period) << " MHz\n";
    }
    out << "Runtime: build " << f2(r.ms_build) << " ms, analysis " << f2(r.ms_analyze) << " ms\n";
}

void writeEndpointSlacks(std::ostream& out, const TimingResult& r) {
    out << "# pin check max|min edge arrival required slack (ps)\n";
    for (const auto& e : r.endpoints) {
        out << e.pin << " " << e.check << " " << (e.max ? "max" : "min") << " " << (e.edge == RISE ? "rise" : "fall") << " "
            << f2(e.arrival) << " " << f2(e.required) << " " << f2(e.slack) << "\n";
    }
}
