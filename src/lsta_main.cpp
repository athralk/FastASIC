// lsta: a light static timing analyser for gate-level netlists (Yosys / OpenLane output)
#include "../include/cell_netlist.hpp"
#include "../include/clock_tree.hpp"
#include "../include/liberty.hpp"
#include "../include/sdc.hpp"
#include "../include/spef.hpp"
#include "../include/timer.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

int usage() {
    std::cerr << "Usage: lsta [options] <netlist.v>\n"
              << "  -lib FILE        Liberty library (repeatable; default: the IHP sg13cmos5l corner below)\n"
              << "  -corner C        typ | fast | slow: sg13cmos5l_stdcell_<corner>.lib from $PDK_ROOT\n"
              << "                   (default ~/ttsetup/pdk)/ihp-sg13cmos5l\n"
              << "  -sdc FILE        timing constraints; without one: clock on port clk (or a virtual clock),\n"
              << "                   input/output delay 0, 6 fF loads, sg13cmos5l_buf_4 input drivers\n"
              << "  -period PS       clock period when there is no SDC (default 10000)\n"
              << "  -paths N         worst paths to report per analysis (default 1)\n"
              << "  -endpoints FILE  write the slack of every endpoint\n"
              << "  -q               summary only, no path reports\n"
              << "  -cts BUF         build a balanced clock buffer tree from cell BUF before timing\n"
              << "  -cts_fanout N    max fanout per clock buffer (default 8)\n"
              << "  -spef FILE       post-route SPEF wire parasitics (wire C + Elmore RC delay)\n"
              << "  -write FILE      write the (clock-tree) netlist as structural Verilog\n";
    return 2;
}

std::string cornerLib(const std::string& corner) {
    const char* root = std::getenv("PDK_ROOT");
    std::string base = root ? root : std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ttsetup/pdk";
    std::string file = corner == "fast" ? "fast_1p32V_m40C" : corner == "slow" ? "slow_1p08V_125C" : "typ_1p20V_25C";
    return base + "/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_" + file + ".lib";
}

double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

int main(int argc, char* argv[]) {
    std::vector<std::string> libs;
    std::string sdc_path, spef_path, netlist_path, endpoints_path, corner = "typ";
    double period = 10000;
    TimerOptions options;
    bool quiet = false;
    std::string cts_buffer, write_path;
    int cts_fanout = 8;
    try {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            bool v = i + 1 < argc;
            if (a == "-lib" && v) libs.push_back(argv[++i]);
            else if (a == "-corner" && v) corner = argv[++i];
            else if (a == "-sdc" && v) sdc_path = argv[++i];
            else if (a == "-period" && v) period = std::stod(argv[++i]);
            else if (a == "-paths" && v) options.paths = std::stoi(argv[++i]);
            else if (a == "-endpoints" && v) endpoints_path = argv[++i];
            else if (a == "-q") quiet = true;
            else if (a == "-cts" && v) cts_buffer = argv[++i];
            else if (a == "-cts_fanout" && v) cts_fanout = std::stoi(argv[++i]);
            else if (a == "-spef" && v) spef_path = argv[++i];
            else if (a == "-write" && v) write_path = argv[++i];
            else if (!a.empty() && a[0] == '-') return usage();
            else netlist_path = a;
        }
        if (netlist_path.empty()) return usage();
        if (libs.empty()) libs.push_back(cornerLib(corner));

        auto t0 = std::chrono::steady_clock::now();
        Library lib = readLiberty(libs[0]);
        for (size_t k = 1; k < libs.size(); ++k) {
            Library more = readLiberty(libs[k]);
            for (auto& cell : more.cells) {
                if (lib.find(cell.name)) continue;
                lib.cell_index[cell.name] = static_cast<int>(lib.cells.size());
                lib.cells.push_back(std::move(cell));
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        CellNetlist netlist = readCellNetlist(netlist_path, lib);
        auto t2 = std::chrono::steady_clock::now();

        Sdc sdc;
        if (!sdc_path.empty()) {
            sdc = readSdc(sdc_path, lib.time_unit_ps, lib.cap_unit_ff);
        } else {
            std::string clk;
            for (const auto& p : netlist.port_bits) {
                if (p.is_input && (p.name == "clk" || p.name == "clock" || p.name == "CLK")) clk = p.name;
            }
            std::ostringstream text;
            text << "create_clock -name clk -period " << period / lib.time_unit_ps << (clk.empty() ? "" : " [get_ports " + clk + "]") << "\n"
                 << "set_input_delay 0 -clock clk [all_inputs -no_clocks]\n"
                 << "set_output_delay 0 -clock clk [all_outputs]\n"
                 << "set_load " << 6.0 / lib.cap_unit_ff << " [all_outputs]\n";
            if (lib.find("sg13cmos5l_buf_4")) text << "set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs -no_clocks]\n";
            sdc = parseSdc(text.str(), lib.time_unit_ps, lib.cap_unit_ff);
        }
        for (const auto& w : sdc.warnings) std::cerr << "SDC warning: " << w << "\n";
        if (!cts_buffer.empty()) {
            if (sdc.clocks.empty() || sdc.clocks[0].sources.empty()) throw std::runtime_error("-cts needs a clock on a port");
            std::string port;
            for (const auto& p : netlist.port_bits) {
                if (p.is_input && (globMatch(sdc.clocks[0].sources[0], p.name))) port = p.name;
            }
            ClockTreeOptions cto;
            cto.buffer = cts_buffer;
            cto.max_fanout = cts_fanout;
            ClockTreeReport ct = insertClockTree(netlist, lib, port, cto);
            std::cout << "Clock tree: " << ct.sinks << " sinks, " << ct.buffers << " x " << cts_buffer << ", " << ct.levels
                      << " levels\n";
        }
        if (!write_path.empty()) {
            std::ofstream out(write_path);
            writeCellNetlist(out, netlist);
        }
        auto t3 = std::chrono::steady_clock::now();
        SpefData spef_data;
        if (!spef_path.empty()) {
            spef_data = readSpef(spef_path);
            options.spef = &spef_data;
        }

        TimingResult result = analyzeNetlistTiming(netlist, lib, sdc, options);
        if (quiet) {
            result.setup_paths.clear();
            result.hold_paths.clear();
        }
        std::cout << "lsta: " << netlist_path << " with " << lib.name
                  << (spef_path.empty() ? "" : " + SPEF " + spef_path + " (" + std::to_string(spef_data.net_count) + " nets)") << "\n";
        printTimingResult(std::cout, result);
        std::cout << std::fixed << std::setprecision(2) << "Load:    liberty " << ms(t0, t1) << " ms, netlist " << ms(t1, t2)
                  << " ms, sdc " << ms(t2, t3) << " ms"
                  << (spef_path.empty() ? "" : ", spef " + std::to_string(static_cast<int>(spef_data.ms_read)) + " ms") << "\n";
        if (!endpoints_path.empty()) {
            std::ofstream out(endpoints_path);
            writeEndpointSlacks(out, result);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "lsta: " << e.what() << "\n";
        return 2;
    }
}
