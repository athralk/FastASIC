// fastpnr: FastASIC's physical design tool: floorplan and place a gate-level netlist, write DEF.
#include "../include/cell_netlist.hpp"
#include "../include/def.hpp"
#include "../include/floorplan.hpp"
#include "../include/droute.hpp"
#include "../include/groute.hpp"
#include "../include/lef.hpp"
#include "../include/place.hpp"
#include "../include/spef.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cctype>
#include <set>
#include <sstream>
#include <sys/stat.h>

namespace {

int usage() {
    std::cerr << "Usage: fastpnr [options] <netlist.v>\n"
              << "  -o FILE          write the floorplan as DEF\n"
              << "  -lef FILE        LEF file (repeatable; default: the IHP sg13cmos5l tech + cell LEF from $PDK_ROOT,\n"
              << "                   plus the SRAM macro LEFs the netlist uses)\n"
              << "  -util U          standard-cell utilization, 0..1 (default 0.5)\n"
              << "  -aspect A        core height / width (default 1)\n"
              << "  -die W H         fixed die size in um (overrides -util)\n"
              << "  -margin UM       die edge to core distance (default 10)\n"
              << "  -halo UM         free space around each macro (default 10)\n"
              << "  -tracks FILE     tracks.info (default: LibreLane's for sg13cmos5l)\n"
              << "  -strap-pitch UM  vertical Metal4 power strap pitch (default 75)\n"
              << "  -top-pitch UM    horizontal TopMetal1 power strap pitch (default 75)\n"
              << "  -no-pdn          no power grid\n"
              << "  -no-place        floorplan only (cells left unplaced)\n"
              << "  -density D       placement target density, 0..1 (default 0.6)\n"
              << "  -no-detailed     skip detailed placement\n"
              << "  -no-route        no global routing\n"
              << "  -droute          detailed routing too (wires and vias go into the DEF)\n"
              << "  -spef FILE       extract post-route RC parasitics as IEEE 1481 SPEF (requires -droute)\n"
              << "  -j N             worker threads for detailed routing (default 1; portable std::thread)\n"
              << "  -guide FILE      write the global route guides (OpenROAD format)\n"
              << "  -adjust A        fraction of tracks held back for detailed routing (default 0.3)\n"
              << "  -gcell N         gcell size in tracks (default 15)\n"
              << "  -layers L1 L2    signal routing layers (default Metal2 Metal4)\n"
              << "  -q               no summary\n";
    return 2;
}

std::string pdkBase() {
    const char* root = std::getenv("PDK_ROOT");
    std::string base = root ? root : std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ttsetup/pdk";
    return base + "/ihp-sg13cmos5l";
}

bool exists(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0;
}

double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

int main(int argc, char* argv[]) {
    std::vector<std::string> lefs;
    std::string netlist_path, out_path, spef_path;
    FloorplanOptions opt;
    bool quiet = false, place = true, route = true, droute = false;
    int threads = 1;
    PlaceOptions popt;
    GrouteOptions gopt;
    std::string guide_path;
    try {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            bool v = i + 1 < argc;
            if (a == "-o" && v) out_path = argv[++i];
            else if (a == "-lef" && v) lefs.push_back(argv[++i]);
            else if (a == "-util" && v) opt.utilization = std::stod(argv[++i]);
            else if (a == "-aspect" && v) opt.aspect = std::stod(argv[++i]);
            else if (a == "-die" && i + 2 < argc) {
                opt.die_width = std::stod(argv[++i]);
                opt.die_height = std::stod(argv[++i]);
            } else if (a == "-margin" && v) opt.core_margin = std::stod(argv[++i]);
            else if (a == "-halo" && v) opt.macro_halo = std::stod(argv[++i]);
            else if (a == "-tracks" && v) opt.tracks_file = argv[++i];
            else if (a == "-strap-pitch" && v) opt.strap_pitch = std::stod(argv[++i]);
            else if (a == "-top-pitch" && v) opt.top_pitch = std::stod(argv[++i]);
            else if (a == "-no-pdn") opt.pdn = false;
            else if (a == "-no-place") place = false;
            else if (a == "-density" && v) popt.target_density = std::stod(argv[++i]);
            else if (a == "-no-detailed") popt.detailed = false;
            else if (a == "-no-route") route = false;
            else if (a == "-droute") droute = true;
            else if (a == "-spef" && v) spef_path = argv[++i];
            else if (a == "-j" && v) threads = std::max(1, std::stoi(argv[++i]));
            else if (a == "-guide" && v) guide_path = argv[++i];
            else if (a == "-adjust" && v) gopt.adjustment = std::stod(argv[++i]);
            else if (a == "-gcell" && v) gopt.gcell_tracks = std::stoi(argv[++i]);
            else if (a == "-layers" && i + 2 < argc) {
                gopt.min_layer = argv[++i];
                gopt.max_layer = argv[++i];
            }
            else if (a == "-q") quiet = true;
            else if (!a.empty() && a[0] == '-') return usage();
            else netlist_path = a;
        }
        if (netlist_path.empty()) return usage();
        if (!spef_path.empty() && (!place || !route || !droute)) {
            std::cerr << "fastpnr: -spef requires -droute with placement and routing enabled\n";
            return 2;
        }

        auto t0 = std::chrono::steady_clock::now();
        std::string base = pdkBase();
        if (lefs.empty()) {
            lefs.push_back(base + "/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef");
            lefs.push_back(base + "/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_stdcell.lef");
            // SRAM macros named in the netlist
            std::ifstream in(netlist_path);
            if (!in) throw std::runtime_error("cannot open " + netlist_path);
            std::stringstream ss;
            ss << in.rdbuf();
            std::string text = ss.str();
            std::set<std::string> macros;
            for (size_t at = text.find("RM_IHPSG13_"); at != std::string::npos; at = text.find("RM_IHPSG13_", at + 1)) {
                size_t end = at;
                while (end < text.size() && (std::isalnum(static_cast<unsigned char>(text[end])) || text[end] == '_')) end++;
                macros.insert(text.substr(at, end - at));
            }
            for (const auto& m : macros) {
                std::string p = base + "/libs.ref/sg13cmos5l_sram/lef/" + m + ".lef";
                if (exists(p)) lefs.push_back(p);
            }
        }
        if (opt.tracks_file.empty()) {
            std::string t = base + "/libs.tech/librelane/sg13cmos5l_stdcell/tracks.info";
            if (exists(t)) opt.tracks_file = t;
        }
        Lef lef;
        for (const auto& f : lefs) readLef(f, lef);
        for (const auto& w : lef.warnings) std::cerr << "LEF warning: " << w << "\n";
        Library lib = libraryFromLef(lef);
        auto t1 = std::chrono::steady_clock::now();

        CellNetlist netlist = readCellNetlist(netlist_path, lib);
        int ties = insertTieCells(netlist, lib, opt.tie_lo, opt.tie_hi);
        auto t2 = std::chrono::steady_clock::now();

        Floorplan fp = buildFloorplan(netlist, lef, opt);
        fp.tie_cells = ties;
        std::vector<std::string> problems = checkFloorplan(fp, netlist, lef);
        auto t3 = std::chrono::steady_clock::now();
        PlaceResult pr;
        if (place && problems.empty()) {
            pr = placeDesign(fp, netlist, lef, popt);
            for (const auto& p : checkPlacement(fp, netlist, lef)) problems.push_back(p);
            insertFillers(fp, lef, {"sg13cmos5l_fill_8", "sg13cmos5l_fill_4", "sg13cmos5l_fill_2", "sg13cmos5l_fill_1"});
        }
        auto t3b = std::chrono::steady_clock::now();
        GrouteResult gr;
        bool routed = route && place && problems.empty();
        if (routed) {
            gr = globalRoute(fp, netlist, lef, gopt);
            if (!guide_path.empty()) {
                std::ofstream g(guide_path);
                if (!g) throw std::runtime_error("cannot write " + guide_path);
                writeGuides(g, gr, netlist, lef);
            }
        }
        auto t3c = std::chrono::steady_clock::now();
        DrouteResult dr;
        bool detailed = routed && droute;
        if (detailed) {
            DrouteOptions dopt;
            dopt.threads = threads;
            if (const char* w = std::getenv("FASTPNR_ASTAR_WEIGHT")) dopt.astar_weight = std::atof(w);
            if (const char* v = std::getenv("FASTPNR_VIA_COST")) dopt.via_cost = std::atof(v);
            if (const char* v = std::getenv("FASTPNR_OTHER_LAYER")) dopt.other_layer = std::atof(v);
            if (const char* v = std::getenv("FASTPNR_ACCESS_PENALTY")) dopt.access_penalty = std::atof(v);
            if (const char* v = std::getenv("FASTPNR_TOP_VIA")) dopt.top_via_factor = std::atof(v);
            if (const char* v = std::getenv("FASTPNR_WRONG_WAY")) dopt.wrong_way = std::atof(v);
            if (const char* v = std::getenv("FASTPNR_THREADS")) dopt.threads = std::max(1, std::atoi(v));
            dr = detailedRoute(fp, netlist, lef, gr, dopt);
            if (dr.conflicts) problems.push_back(std::to_string(dr.conflicts) + " detailed-routing conflicts left (shorts or spacing)");
            if (dr.unrouted) problems.push_back(std::to_string(dr.unrouted) + " nets not fully connected");
        }
        auto t3d = std::chrono::steady_clock::now();

        if (!out_path.empty()) {
            std::ofstream out(out_path);
            if (!out) throw std::runtime_error("cannot write " + out_path);
            writeDef(out, fp, netlist, lef, detailed ? &dr : nullptr);
        }
        if (!spef_path.empty() && detailed) {
            std::ofstream sout(spef_path);
            if (!sout) throw std::runtime_error("cannot write " + spef_path);
            writeSpef(sout, fp, netlist, lef, dr);
        }
        auto t4 = std::chrono::steady_clock::now();

        if (!quiet) {
            auto um = [&](Dbu d) { return lef.toMicrons(d); };
            size_t macros = 0, wires = 0, vias = 0, sites = 0;
            for (const auto& c : fp.components) macros += c.fixed;
            for (const auto& w : fp.special) (w.via >= 0 ? vias : wires)++;
            for (const auto& r : fp.rows) sites += r.count;
            double row_area = sites * um(fp.site->width) * um(fp.site->height);
            std::cout << std::fixed << std::setprecision(2)
                      << "Design      " << netlist.name << ": " << netlist.instances.size() << " cells (" << ties << " tie cells added), "
                      << macros << " macros, " << fp.pins.size() << " pins\n"
                      << "Die         " << um(fp.die.width()) << " x " << um(fp.die.height()) << " um\n"
                      << "Core        " << um(fp.core.width()) << " x " << um(fp.core.height()) << " um at (" << um(fp.core.x0) << ", " << um(fp.core.y0)
                      << ")\n"
                      << "Rows        " << fp.rows.size() << " rows, " << sites << " sites\n"
                      << "Utilization " << std::setprecision(1) << 100.0 * fp.cell_area / std::max(row_area, 1e-9) << "% of the rows ("
                      << std::setprecision(2) << fp.cell_area << " um^2 of cells)\n"
                      << "Tracks      " << fp.tracks.size() << " track sets\n"
                      << "Power grid  " << wires << " wires, " << vias << " vias, " << fp.vias.size() << " via types\n";
            for (const auto& n : fp.notes) std::cout << "Note        " << n << "\n";
            if (place && problems.empty()) {
                std::cout << std::setprecision(0) << "Fillers     " << fp.fillers.size() << " filler cells close the gaps between cells\n";
                std::cout << std::setprecision(0) << "Placement   HPWL " << pr.hpwl_global << " um global (" << pr.iterations << " iterations), " << pr.hpwl_legal
                          << " legalized, " << pr.hpwl_final << " detailed\n"
                          << std::setprecision(2) << "            legalization moved cells " << pr.avg_displacement << " um on average, " << pr.max_displacement
                          << " um at most\n";
            }
            if (routed) {
                std::cout << std::setprecision(2) << "Routing     " << gr.gx << " x " << gr.gy << " gcells of " << lef.toMicrons(gr.gcell) << " um, "
                          << gr.routed_nets << " nets routed (" << gr.local_nets << " inside one gcell), " << gr.iterations << " negotiation rounds\n";
                std::cout << "            Layer      Dir  Resources  Derated   Demand  Usage   Overflow (max)\n";
                long tr = 0, td = 0, to = 0;
                for (const auto& l : gr.layers) {
                    std::cout << "            " << std::left << std::setw(10) << l.name << " " << (l.horizontal ? "H" : "V") << std::right << std::setw(12)
                              << l.resource_original << std::setw(9) << l.resource << std::setw(9) << l.demand << std::setw(6) << std::setprecision(1)
                              << (l.resource ? 100.0 * l.demand / l.resource : 0.0) << "%" << std::setw(9) << l.overflow << " (" << l.max_overflow << ")\n";
                    tr += l.resource; td += l.demand; to += l.overflow;
                }
                std::cout << "            Total            " << std::setw(19) << tr << std::setw(9) << td << std::setw(6) << std::setprecision(1)
                          << (tr ? 100.0 * td / tr : 0.0) << "%" << std::setw(9) << to << "\n"
                          << std::setprecision(0) << "            wirelength " << gr.wirelength_um << " um, " << gr.vias << " vias, 2D overflow " << gr.overflow_2d << "\n";
            }
            if (detailed) {
                std::cout << std::setprecision(0) << "Detailed    " << dr.nets.size() << " nets, " << dr.wirelength_um << " um of wire, " << dr.vias << " vias, "
                          << dr.iterations << " negotiation rounds; " << dr.conflicts << " conflicts, " << dr.unrouted << " unconnected nets, "
                          << dr.pins_without_access << " pins without access, min-area fixes " << dr.min_area_fixes << " (" << dr.min_area_left << " left)\n"
                          << std::setprecision(1) << "            time " << ms(t3c, t3d) << " ms (setup " << dr.ms_setup << ", route " << dr.ms_route << ", finish "
                          << dr.ms_finish << ")\n";
                for (const auto& n : dr.notes) std::cout << "Note        " << n << "\n";
            }
            std::cout << std::setprecision(1) << "Time        " << ms(t0, t4) << " ms (LEF " << ms(t0, t1) << ", netlist " << ms(t1, t2) << ", floorplan "
                      << ms(t2, t3) << ", placement " << ms(t3, t3b) << " [global " << pr.ms_global << ", legal " << pr.ms_legal << ", detailed "
                      << pr.ms_detailed << "], routing " << ms(t3b, t3c) << " [capacity " << gr.ms_capacity << ", pattern " << gr.ms_pattern
                      << ", negotiation " << gr.ms_negotiation << ", layers " << gr.ms_layers << "], DEF " << ms(t3c, t4) << ")\n";
        }
        for (const auto& p : problems) std::cerr << "Check failed: " << p << "\n";
        return problems.empty() ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
