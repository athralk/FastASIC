#include "spef.hpp"
#include "timer.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void near(double actual, double expected, const std::string& what) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-6) {
        throw std::runtime_error(what + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
    }
}
struct Fixture {
    CellNetlist nl;
    Floorplan fp;
    Lef lef;
    DrouteResult dr;
    explicit Fixture() {
        nl.name = "rc_test";
        int n = nl.addNet("n");
        nl.port_bits = {{"driver", true, n}, {"sink", false, n}};
        FpPin p; p.port_bit = 0; p.layer = 0; fp.pins.push_back(p);
        p.port_bit = 1; p.x = 100000; p.y = 100000; fp.pins.push_back(p);
        LefLayer l; l.name = "M1"; l.type = LayerType::ROUTING; l.width = 1000;
        // 10 ohm/um and 2 fF/um; synthetic geometry has an analytically known RC tree.
        l.res_per_sq = 10; l.cap_per_area = 0.001; l.edge_cap = 0.0005;
        lef.layers.push_back(l);
        DrouteNet dn; dn.netlist_net = n; dr.nets.push_back(dn);
    }
    NetParasitics extract(const std::string& path) const {
        { std::ofstream out(path); writeSpef(out, fp, nl, lef, dr, 1.0); }
        return readSpef(path).nets.at("n");
    }
};
void expectOpen(const Fixture& f, const std::string& path) {
    try { f.extract(path); }
    catch (const std::runtime_error& e) {
        std::string message = e.what();
        if (message.find("disconnected") != std::string::npos || message.find("no routed access") != std::string::npos) return;
        throw;
    }
    throw std::runtime_error("disconnected geometry was silently joined");
}
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string path = argv[1];
    try {
        Fixture f;
        f.dr.nets[0].wires = {{0, 0, 0, 200000, 0}, {0, 100000, 0, 100000, 100000}};
        auto np = f.extract(path);
        near(np.wire_cap_ff, 600, "junction capacitance");
        near(np.pin_elmore_ps.at("sink"), 600.6, "mid-run branch");
        // A 100 fF receiver adds (1 + 1000 + 1000 + 1) ohms * 100 fF.
        near(np.elmoreDelays({{"sink", 100}}).at("sink"), 800.8, "branch receiver load");
        std::reverse(f.dr.nets[0].wires.begin(), f.dr.nets[0].wires.end());
        near(f.extract(path).pin_elmore_ps.at("sink"), 600.6, "segment order independence");
        f.dr.nets[0].wires = {{0, 0, 0, 100000, 0}, {0, 100000, 0, 200000, 0},
                            {0, 100000, 0, 100000, 100000}};
        near(f.extract(path).pin_elmore_ps.at("sink"), 600.6, "explicitly split branch");
        f.dr.nets[0].wires.push_back({0, 400000, 0, 500000, 0});
        expectOpen(f, path);

        Fixture via;
        LefLayer cut; cut.name = "V1"; cut.type = LayerType::CUT; cut.res_per_sq = 20;
        via.lef.layers.push_back(cut);
        auto upper = via.lef.layers[0]; upper.name = "M2"; via.lef.layers.push_back(upper);
        via.fp.pins[1].layer = 2; via.fp.pins[1].y = 0;
        via.dr.nets[0].wires = {{2, 0, 0, 100000, 0}};
        via.dr.nets[0].vias = {{"V1", 0, 0, 0}};
        near(via.extract(path).pin_elmore_ps.at("sink"), 104.2, "pin layer retains via resistance");
        via.dr.nets[0].vias.clear();
        expectOpen(via, path);
        // A via at the middle of a merged trunk also splits that trunk.
        via.fp.pins[1].y = 100000;
        via.dr.nets[0].wires = {{0, 0, 0, 200000, 0}, {2, 100000, 0, 100000, 100000}};
        via.dr.nets[0].vias = {{"V1", 0, 100000, 0}};
        near(via.extract(path).pin_elmore_ps.at("sink"), 604.6, "mid-run via");

        Fixture patch;
        patch.fp.pins[1].x = 101000;
        patch.dr.nets[0].wires = {{0, 0, 0, 100000, 0}, {0, 101000, 0, 101000, 100000}};
        expectOpen(patch, path);
        patch.dr.nets[0].patches = {{0, {99000, -1000, 102000, 1000}}};
        if (!std::isfinite(patch.extract(path).pin_elmore_ps.at("sink"))) {
            throw std::runtime_error("physical metal patch did not connect its contacts");
        }

        // The SPEF has only wire RC. SDC and Liberty loads must participate in
        // the wire delay, including independent rise/fall and early/late values.
        {
            std::ofstream out(path);
            out << "*C_UNIT 1 PF\n*R_UNIT 1 OHM\n*DELIMITER :\n"
                   "*D_NET n 0\n*CONN\n*P driver I\n*P sink O\n*CAP\n*RES\n1 driver sink 1000\n*END\n";
        }
        auto data = readSpef(path);
        Library lib;
        Sdc sdc = parseSdc("create_clock -name vclk -period 10\n"
                           "set_input_delay 0 -clock vclk [all_inputs]\n"
                           "set_output_delay 0 -clock vclk [all_outputs]\n"
                           "set_load 0.1 [all_outputs]\n", 1000, 1000);
        TimerOptions opt; opt.spef = &data;
        auto result = analyzeNetlistTiming(f.nl, lib, sdc, opt);
        if (result.endpoints.size() != 4) throw std::runtime_error("missing port endpoints");
        for (const auto& ep : result.endpoints) near(ep.arrival, 100, "SDC load wire delay");

        {
            std::ofstream out(path);
            out << "*C_UNIT 1 PF\n*R_UNIT 1 OHM\n*DELIMITER :\n"
                   "*D_NET n 0\n*CONN\n*P driver I\n*I u:A I\n*CAP\n*RES\n1 driver u:A 1000\n*END\n";
        }
        data = readSpef(path);
        LibCell cell; cell.name = "buffer";
        LibPin a; a.name = "A"; a.rise_cap_max = 100; a.rise_cap_min = 50;
        a.fall_cap_max = 200; a.fall_cap_min = 80;
        LibPin x; x.name = "X"; x.dir = PinDir::OUTPUT;
        cell.pins = {a, x};
        LibArc arc; arc.from_pin = 0; arc.to_pin = 1; arc.sense = TimingSense::POSITIVE;
        arc.cell_rise.values = arc.cell_fall.values = {0};
        arc.rise_transition.values = arc.fall_transition.values = {0};
        cell.arcs.push_back(arc);
        int out_net = f.nl.addNet("out"); f.nl.port_bits[1].net = out_net;
        f.nl.instances.push_back({"u", &cell, {2, out_net}});
        result = analyzeNetlistTiming(f.nl, lib, sdc, opt);
        if (result.endpoints.size() != 4) throw std::runtime_error("missing cell endpoints");
        for (const auto& ep : result.endpoints) {
            double expected = ep.edge == RISE ? (ep.max ? 100 : 50) : (ep.max ? 200 : 80);
            near(ep.arrival, expected, "Liberty corner load wire delay");
        }
        std::cout << "SPEF geometry and loaded RC checks passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
