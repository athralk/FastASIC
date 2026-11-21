#include "../include/file_reader.hpp"
#include "../include/preprocessor.hpp"
#include "../include/lexer.hpp"
#include "../include/parser.hpp"
#include "../include/mapper.hpp"
#include "../include/optimizer.hpp"
#include "../include/netlist_utils.hpp"
#include "../include/simulator.hpp"
#include "../include/elaborator.hpp"
#include "../include/verilog_writer.hpp"
#include "../include/sta.hpp"
#include "../include/cec.hpp"
#include "../include/cell_netlist.hpp"
#include "../include/liberty.hpp"
#include "../include/sdc.hpp"
#include "../include/tech_mapper.hpp"
#include "../include/cut_mapper.hpp"
#include "../include/balance.hpp"
#include "../include/clock_tree.hpp"
#include "../include/timer.hpp"
#include <algorithm>
#include <cstdlib>
#include <dirent.h>
#include <map>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

// Exit codes: 0 = success, 1 = designs not equivalent, 2 = usage / parse error
constexpr int kExitNotEquivalent = 1;
constexpr int kExitError = 2;

// Source text -> tokens -> syntax tree -> bit-level gate netlist
std::string topModule;                      // --top
PreprocessorOptions preprocessor;           // -D, -I
std::vector<std::string> macroLibPaths;     // --macro-lib
std::string macroCorner = "typ";            // corner used to find SRAM libraries automatically
std::vector<Library> macroLibraries;        // every hard-macro library loaded so far
bool inferSrams = true;                     // --no-sram-infer turns it off
AdderStyle adderStyle = AdderStyle::RIPPLE; // --adder (default: ripple, Sklansky with --map delay)
bool adderChosen = false;

std::string pdkRoot() {
    const char* root = std::getenv("PDK_ROOT");
    return root ? root : std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ttsetup/pdk";
}

// The ports of a Liberty macro cell; buses become vector ports
BlackBoxModule blackBoxFromCell(const LibCell& cell) {
    BlackBoxModule bb;
    bb.name = cell.name;
    std::vector<int> bus_of(cell.pins.size(), -1);
    for (size_t b = 0; b < cell.buses.size(); ++b) {
        for (int p : cell.buses[b].pins) bus_of[p] = static_cast<int>(b);
    }
    std::vector<bool> done(cell.buses.size(), false);
    for (size_t p = 0; p < cell.pins.size(); ++p) {
        bool input = cell.pins[p].dir != PinDir::OUTPUT;
        if (bus_of[p] < 0) {
            bb.ports.push_back({cell.pins[p].name, input, false, 0, 0});
        } else if (!done[bus_of[p]]) {
            const LibBus& bus = cell.buses[bus_of[p]];
            bb.ports.push_back({bus.name, input, true, bus.from, bus.to});
            done[bus_of[p]] = true;
        }
    }
    return bb;
}

// A module instantiated but defined nowhere: look for <name>_<corner>*.lib among the PDK SRAMs
std::string findMacroLibrary(const std::string& name) {
    std::string dir = pdkRoot() + "/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lib/";
    std::string prefix = name + "_" + macroCorner;
    if (DIR* d = opendir(dir.c_str())) {
        std::string found;
        while (dirent* e = readdir(d)) {
            std::string f = e->d_name;
            if (f.rfind(prefix, 0) == 0 && f.size() > 4 && f.compare(f.size() - 4, 4, ".lib") == 0) found = dir + f;
        }
        closedir(d);
        return found;
    }
    return "";
}

Design parseFile(const std::string& path) {
    VerilogLexer lexer(preprocessVerilog(readVerilogFile(path), path, preprocessor));
    VerilogParser parser(lexer.tokenize());
    std::vector<Module> modules = parser.parseModules();

    // Hard macros: explicit --macro-lib files, plus PDK SRAMs found by name
    std::set<std::string> defined, missing;
    for (const auto& m : modules) defined.insert(m.name);
    for (const auto& m : modules) {
        for (const auto& inst : m.instances) {
            if (!defined.count(inst.module)) missing.insert(inst.module);
        }
    }
    std::vector<std::string> paths = macroLibPaths;
    for (const auto& name : missing) {
        bool known = false;
        for (const auto& lib : macroLibraries) known |= lib.find(name) != nullptr;
        if (known) continue;
        std::string p = findMacroLibrary(name);
        if (!p.empty()) paths.push_back(p);
    }
    for (const auto& p : paths) {
        bool loaded = false;
        for (const auto& lib : macroLibraries) loaded |= lib.path == p;
        if (!loaded) macroLibraries.push_back(readLiberty(p));
    }
    // SRAM inference: only when some module declares a memory, load every PDK SRAM for the corner
    SramInference srams;
    bool has_memory = false;
    for (const auto& m : modules) {
        for (const auto& n : m.nets) has_memory |= n.first != nullptr;
    }
    std::string sram_dir = pdkRoot() + "/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lib/";
    if (inferSrams && has_memory) {
        if (DIR* d = opendir(sram_dir.c_str())) {
            std::vector<std::string> files;
            while (dirent* e = readdir(d)) {
                std::string f = e->d_name;
                if (f.rfind("RM_IHPSG13_", 0) == 0 && f.find("_" + macroCorner + "_") != std::string::npos &&
                    f.size() > 4 && f.compare(f.size() - 4, 4, ".lib") == 0) {
                    files.push_back(sram_dir + f);
                }
            }
            closedir(d);
            std::sort(files.begin(), files.end());
            for (const auto& f : files) {
                bool loaded = false;
                for (const auto& lib : macroLibraries) loaded |= lib.path == f;
                if (!loaded) macroLibraries.push_back(readLiberty(f));
            }
            std::string file = macroCorner == "fast" ? "fast_1p32V_m40C" : macroCorner == "slow" ? "slow_1p08V_125C" : "typ_1p20V_25C";
            Library cells = readLiberty(pdkRoot() + "/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_" + file + ".lib");
            if (const LibCell* ff = cells.find("sg13cmos5l_dfrbpq_1")) srams.flop_area = ff->area;
            srams.enabled = srams.flop_area > 0;
        }
    }
    std::vector<BlackBoxModule> blackboxes;
    for (const auto& lib : macroLibraries) {
        for (const auto& cell : lib.cells) {
            if (!cell.is_macro) continue;
            blackboxes.push_back(blackBoxFromCell(cell));
            if (cell.address_width > 0 && cell.address_width < 31) {
                srams.macros.push_back({cell.name, 1LL << cell.address_width, cell.word_width,
                                        cell.name.find("_2P_") != std::string::npos ? 2 : 1, cell.area});
            }
        }
    }
    return elaborate(modules, topModule, blackboxes, srams, adderStyle);
}

// The netlist is a DAG, so a gate can be reached from several places. Each gate is expanded
// the first time it is printed; later visits just point back to it instead of repeating the subtree.
void printGate(const std::shared_ptr<GateNode>& node, int depth, std::set<const GateNode*>& expanded) {
    if (!node) return;

    std::string indent(depth * 2, ' ');
    std::string typeStr = gateTypeName(node->type);
    if (node->type == GateType::INPUT || node->type == GateType::OUTPUT || node->type == GateType::ASSIGN) {
        typeStr += " (" + node->name + ")";
    }

    if (!node->inputs.empty() && !expanded.insert(node.get()).second) {
        std::cout << indent << "- " << typeStr << " [id: " << node->id << "] (shared, see above)\n";
        return;
    }

    std::cout << indent << "- " << typeStr << " [id: " << node->id << "]\n";
    for (const auto& fanin : node->inputs) {
        printGate(fanin, depth + 1, expanded);
    }
}

void printForest(const std::string& title, const std::string& rootLabel, const GateForest& forest, bool quiet) {
    std::cout << "\n=== " << title << " ===\n";
    std::set<const GateNode*> expanded;
    for (size_t i = 0; i < forest.size() && !quiet; ++i) {
        std::cout << rootLabel << " [" << i << "]:\n";
        printGate(forest[i], 1, expanded);
    }

    int total = 0;
    std::string breakdown;
    for (const auto& [type, count] : gateCounts(forest)) {
        total += count;
        breakdown += (breakdown.empty() ? "" : ", ") + gateTypeName(type) + ": " + std::to_string(count);
    }
    std::cout << "Gates: " << total << (breakdown.empty() ? "" : " (" + breakdown + ")") << "\n";
}

// How equivalence is decided
enum class EquivMethod {
    SAT,    // SAT proof (default): a proof for any input count, and faster than simulation here
    SIM,    // simulation: exhaustive up to 20 inputs, 65,536 random patterns above (not a proof)
    NONE,   // skip internal equivalence check (fast inner-loop exploration mode)
};

bool reportEquivalence(const std::string& label, const GateForest& a, const GateForest& b, EquivMethod method) {
    if (method == EquivMethod::NONE) {
        std::cout << "  " << label << ": SKIPPED (--equiv-method none)\n";
        return true;
    }
    std::cout << "  " << label << ": ";
    if (method == EquivMethod::SAT) {
        CecResult result = proveEquivalence(a, b);
        std::ostringstream ms;
        ms << std::fixed << std::setprecision(1) << result.milliseconds;
        if (!result.equivalent) {
            std::cout << "FAIL (SAT, " << ms.str() << " ms) - " << result.message << "\n";
            return false;
        }
        std::cout << "PASS (SAT proof, " << result.outputs << " outputs: " << result.structural << " structural, "
                  << result.by_sat << " by SAT, " << result.swept << " internal merged, " << result.aig_ands << " AIG ANDs, " << result.sat.conflicts
                  << " conflicts, " << ms.str() << " ms)\n";
        return true;
    }

    EquivalenceResult result = checkEquivalence(a, b);
    if (result.equivalent) {
        std::cout << "PASS (" << (result.exhaustive ? "exhaustive, " : "random, ") << result.patterns << " patterns)\n";
    } else {
        std::cout << "FAIL - " << result.message << "\n";
    }
    return result.equivalent;
}

// "16 flip-flops (posedge clk, async active-low reset rst_n)"
std::string describeRegisters(const Design& d) {
    if (d.registers.empty()) return "";
    std::set<std::string> domains;
    for (const auto& r : d.registers) {
        std::string text = std::string(r.negedge ? "negedge " : "posedge ") + r.clock;
        if (!r.reset.empty()) text += ", async active-" + std::string(r.reset_active_low ? "low" : "high") + " reset " + r.reset;
        domains.insert(text);
    }
    std::string out = std::to_string(d.registers.size()) + " flip-flops (";
    bool first = true;
    for (const auto& t : domains) {
        out += (first ? "" : "; ") + t;
        first = false;
    }
    return out + ")";
}

// The SAT check compares next-state logic with registers matched by name; their clocking and
// reset must match too
std::string registerMismatch(const Design& a, const Design& b) {
    std::map<std::string, const RegisterBit*> rb;
    for (const auto& r : b.registers) rb[r.name] = &r;
    for (const auto& r : a.registers) {
        auto it = rb.find(r.name);
        if (it == rb.end()) return "register '" + r.name + "' exists only in the first design";
        const RegisterBit& o = *it->second;
        if (r.clock != o.clock || r.negedge != o.negedge) return "register '" + r.name + "' is clocked differently";
        if (r.reset != o.reset || (!r.reset.empty() && (r.reset_active_low != o.reset_active_low || r.reset_value != o.reset_value))) {
            return "register '" + r.name + "' has a different asynchronous reset";
        }
        rb.erase(it);
    }
    if (!rb.empty()) return "register '" + rb.begin()->first + "' exists only in the second design";
    std::map<std::string, std::string> ma, mb;
    for (const auto& m : a.macros) ma[m.name] = m.cell;
    for (const auto& m : b.macros) mb[m.name] = m.cell;
    if (ma != mb) return "the designs instantiate different hard macros";
    return "";
}

int usage(const char* program) {
    std::cerr << "Usage: " << program << " [-q|--quiet] [-o netlist.v] [--equiv-method M] [--sta [timing options]] <source.v>\n"
              << "       " << program << " [--equiv-method M] --equiv <first.v> <second.v>\n"
              << "  -q, --quiet           print gate counts and checks only, not the gate trees\n"
              << "  --top MODULE          top of the hierarchy (default: the module no other module instantiates)\n"
              << "  -DNAME[=value], -I DIR  preprocessor defines and `include directories\n"
              << "  --macro-lib FILE      Liberty file of a hard macro (SRAM); PDK SRAMs (RM_IHPSG13_*) are found by name\n"
              << "  --no-sram-infer       keep reg memories as flip-flops (default: big ones become PDK SRAM macros)\n"
              << "  -o FILE               write the optimized NAND + INV netlist as structural Verilog\n"
              << "  --equiv-method sat|sim  sat: SAT proof (default); sim: bit-parallel simulation, exhaustive up to\n"
              << "                        20 inputs, 65,536 random patterns above (a sample, not a proof)\n"
              << "  --map area|delay      map onto real sg13cmos5l cells by tree covering (-o then writes cells)\n"
              << "  --mapper cut|tree     cut: cut enumeration + Boolean matching (default); tree: tree covering\n"
              << "  --adder STYLE         ripple | brent-kung | sklansky | kogge-stone for +, -, comparisons and\n"
              << "                        multipliers (default ripple; sklansky with --map delay)\n"
              << "  --lib FILE            Liberty library for --map / nldm (default: IHP sg13cmos5l at --corner)\n"
              << "  --sta                 static timing analysis of the optimized netlist (IHP sg13cmos5l cells)\n"
              << "Timing options:\n"
              << "  --delay-model rc|unit|nldm  rc: t = R*C_par + R*C_load (default); unit: every cell = 1;\n"
              << "                        nldm: Liberty tables with slew propagation on the mapped cells\n"
              << "  --sdc FILE            constraints for nldm timing (default: from the options below)\n"
              << "  --corner typ|fast|slow  typ 1.20V 25C (sign-off, default), fast 1.32V -40C, slow 1.08V 125C\n"
              << "  --period PS           clock period (default 10000)\n"
              << "  --input-delay PS      input arrival after the clock edge (default 0)\n"
              << "  --output-delay PS     output setup before the next edge (default 0)\n"
              << "  --load FF             capacitance on every primary output (default 6.0, the PDK's LibreLane value)\n";
    return kExitError;
}

// Parses a non-negative number given to a command-line option
double numberArgument(const std::string& option, const std::string& text) {
    try {
        size_t used = 0;
        double value = std::stod(text, &used);
        if (used == text.size() && value >= 0) return value;
    } catch (const std::exception&) {
    }
    throw std::runtime_error(option + " needs a non-negative number, got '" + text + "'");
}

int main(int argc, char* argv[]) {
    bool equivMode = false;
    bool quiet = false;
    bool runSta = false;
    EquivMethod equivMethod = EquivMethod::SAT;
    DelayModel delayModel = DelayModel::RC;
    Corner corner = Corner::TYP;
    TimingConstraints constraints;
    std::string netlistPath, libPath, sdcPath;
    bool mapCells = false, nldm = false, treeMapper = false;
    bool noBufferNets = false, forceBufferNets = false;
    MapGoal mapGoal = MapGoal::AREA;
    std::vector<std::string> files;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            bool hasValue = i + 1 < argc;
            if (arg == "--equiv") equivMode = true;
            else if (arg == "-q" || arg == "--quiet") quiet = true;
            else if (arg == "--sta") runSta = true;
            else if (arg == "-o" && hasValue) netlistPath = argv[++i];
            else if (arg == "--lib" && hasValue) libPath = argv[++i];
            else if (arg == "--top" && hasValue) topModule = argv[++i];
            else if (arg.rfind("-D", 0) == 0 && arg.size() > 2) {
                std::string def = arg.substr(2);
                size_t eq = def.find('=');
                preprocessor.defines[def.substr(0, eq)] = eq == std::string::npos ? "1" : def.substr(eq + 1);
            }
            else if (arg == "-I" && hasValue) preprocessor.include_dirs.push_back(argv[++i]);
            else if (arg == "--macro-lib" && hasValue) macroLibPaths.push_back(argv[++i]);
            else if (arg == "--no-sram-infer") inferSrams = false;
            else if (arg == "--adder" && hasValue) {
                std::string name = argv[++i];
                adderChosen = true;
                if (name == "ripple") adderStyle = AdderStyle::RIPPLE;
                else if (name == "brent-kung") adderStyle = AdderStyle::BRENT_KUNG;
                else if (name == "sklansky") adderStyle = AdderStyle::SKLANSKY;
                else if (name == "kogge-stone") adderStyle = AdderStyle::KOGGE_STONE;
                else return usage(argv[0]);
            }
            else if (arg == "--mapper" && hasValue) {
                std::string name = argv[++i];
                if (name == "tree") treeMapper = true;
                else if (name == "cut") treeMapper = false;
                else return usage(argv[0]);
            }
            else if (arg == "--sdc" && hasValue) sdcPath = argv[++i];
            else if (arg == "--map" && hasValue) {
                std::string name = argv[++i];
                mapCells = true;
                if (name == "area") mapGoal = MapGoal::AREA;
                else if (name == "delay") mapGoal = MapGoal::DELAY;
                else return usage(argv[0]);
            }
            else if (arg == "--equiv-method" && hasValue) {
                std::string name = argv[++i];
                if (name == "sim") equivMethod = EquivMethod::SIM;
                else if (name == "sat") equivMethod = EquivMethod::SAT;
                else if (name == "none") equivMethod = EquivMethod::NONE;
                else return usage(argv[0]);
            }
            else if (arg == "--no-buffer") noBufferNets = true;
            else if (arg == "--buffer-nets") forceBufferNets = true;
            else if (arg == "--delay-model" && hasValue) {
                std::string name = argv[++i];
                if (name == "rc") delayModel = DelayModel::RC;
                else if (name == "unit") delayModel = DelayModel::UNIT;
                else if (name == "nldm") nldm = true;
                else return usage(argv[0]);
            }
            else if (arg == "--corner" && hasValue) {
                std::string name = argv[++i];
                macroCorner = name;
                if (name == "typ") corner = Corner::TYP;
                else if (name == "fast") corner = Corner::FAST;
                else if (name == "slow") corner = Corner::SLOW;
                else return usage(argv[0]);
            }
            else if (arg == "--period" && hasValue) constraints.clock_period_ps = numberArgument(arg, argv[++i]);
            else if (arg == "--input-delay" && hasValue) constraints.input_delay_ps = numberArgument(arg, argv[++i]);
            else if (arg == "--output-delay" && hasValue) constraints.output_delay_ps = numberArgument(arg, argv[++i]);
            else if (arg == "--load" && hasValue) constraints.output_load_ff = numberArgument(arg, argv[++i]);
            else if (!arg.empty() && arg[0] == '-') return usage(argv[0]);
            else files.push_back(arg);
        }
        if (files.size() != (equivMode ? 2u : 1u)) return usage(argv[0]);
        if (!adderChosen && mapCells && mapGoal == MapGoal::DELAY) adderStyle = AdderStyle::SKLANSKY;

        if (equivMode) {
            Design first = parseFile(files[0]);
            Design second = parseFile(files[1]);
            std::cout << "=== Equivalence Check ===\n";
            std::string mismatch = registerMismatch(first, second);
            if (!mismatch.empty()) {
                std::cout << "  " << files[0] << " vs " << files[1] << ": FAIL - " << mismatch << "\n";
                return kExitNotEquivalent;
            }
            bool equivalent = reportEquivalence(files[0] + " vs " + files[1], first.forest, second.forest, equivMethod);
            return equivalent ? 0 : kExitNotEquivalent;
        }

        Design design = parseFile(files[0]);
        GateForest& forest = design.forest;
        printForest("Original AST Gate Forest", "Root", forest, quiet);
        if (!design.registers.empty()) {
            std::cout << "Registers: " << describeRegisters(design) << "; each register's next state is a root named <reg>$next\n";
        }
        for (const auto& note : design.notes) std::cout << "SRAM inference: " << note << "\n";

        // Run Technology Mapping to NAND + Inverter Form
        // Before mapping onto real cells, rebuild associative chains as balanced trees
        // (the equivalence checks below still compare against the unbalanced original)
        GateForest source = mapCells || (nldm && runSta) ? balanceAssociative(forest) : forest;
        GateForest mappedForest = TechnologyMapper::mapToNandInverter(source);
        printForest("Mapped Canonical NAND + Inverter Netlist", "Mapped Root", mappedForest, quiet);

        // Strash, constant propagation and double-inversion removal on the mapped netlist
        GateForest optimizedForest = NetlistOptimizer::optimize(mappedForest);
        printForest("Optimized NAND + Inverter Netlist", "Optimized Root", optimizedForest, quiet);

        // Every transformation must preserve the logic function
        std::cout << "\n=== Equivalence Checks ===\n";
        bool mappedOk = reportEquivalence("original vs mapped", forest, mappedForest, equivMethod);
        bool optimizedOk = reportEquivalence("original vs optimized", forest, optimizedForest, equivMethod);

        bool cellsOk = true;
        if (nldm && runSta) mapCells = true;
        CellNetlist cellNetlist;
        Library lib;
        if (mapCells) {
            if (libPath.empty()) {
                const char* root = std::getenv("PDK_ROOT");
                std::string base = root ? root : std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ttsetup/pdk";
                std::string file = corner == Corner::FAST ? "fast_1p32V_m40C" : corner == Corner::SLOW ? "slow_1p08V_125C" : "typ_1p20V_25C";
                libPath = base + "/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_" + file + ".lib";
            }
            lib = readLiberty(libPath);
            for (const auto& macros : macroLibraries) {   // hard macros join the cell library
                for (const auto& cell : macros.cells) {
                    if (lib.find(cell.name)) continue;
                    lib.cell_index[cell.name] = static_cast<int>(lib.cells.size());
                    lib.cells.push_back(cell);
                }
            }
            MapResult mapped = treeMapper ? mapToCells(design, optimizedForest, lib, mapGoal)
                                          : mapByCuts(design, optimizedForest, lib, mapGoal);
            cellNetlist = std::move(mapped.netlist);
            if (!noBufferNets && (forceBufferNets || mapGoal == MapGoal::DELAY)) {
                std::set<std::string> clk_ports = {"clk", "clock", "CLK"};
                for (const auto& r : design.registers) {
                    if (!r.clock.empty()) clk_ports.insert(r.clock);
                }
                int cts_bufs = 0;
                if (!design.registers.empty() && !design.registers[0].clock.empty() && lib.find("sg13cmos5l_buf_8")) {
                    ClockTreeOptions cto;
                    cto.buffer = "sg13cmos5l_buf_8";
                    cto.max_fanout = 24;
                    try {
                        ClockTreeReport ctr = insertClockTree(cellNetlist, lib, design.registers[0].clock, cto);
                        cts_bufs = ctr.buffers;
                        if (ctr.buffers > 0) mapped.cell_counts[cto.buffer] += ctr.buffers;
                    } catch (const std::exception&) {}
                }
                BufferNetsReport bnr = bufferHighFanoutNets(cellNetlist, lib, 16, "sg13cmos5l_buf_4", clk_ports);
                if (bnr.buffers_added > 0) mapped.cell_counts["sg13cmos5l_buf_4"] += bnr.buffers_added;
                int hold_bufs = repairFastHoldPaths(cellNetlist, lib, 2, 24);
                if (hold_bufs > 0) mapped.cell_counts["sg13cmos5l_dlygate4sd3_1"] += hold_bufs;
                mapped.area = cellNetlist.area();
                if (cts_bufs > 0 || bnr.buffers_added > 0 || hold_bufs > 0) {
                    std::cout << "\n=== Net Buffering & Hold Repair ===\n"
                              << "Clock tree buffers: " << cts_bufs << ", high-fanout nets buffered: " << bnr.buffered_nets
                              << " (+" << bnr.buffers_added << " buf_4), hold buffers: " << hold_bufs << "\n";
                }
            }
            std::cout << "\n=== Technology Mapping (" << lib.name << ", " << (mapGoal == MapGoal::AREA ? "area" : "delay") << ") ===\n";
            int total = 0;
            std::string breakdown;
            for (const auto& [cell, count] : mapped.cell_counts) {
                total += count;
                breakdown += (breakdown.empty() ? "" : ", ") + (cell.rfind("sg13cmos5l_", 0) == 0 ? cell.substr(11) : cell) + ": " + std::to_string(count);
            }
            std::cout << "Cells: " << total << " (" << breakdown << ")\n";
            std::ostringstream area;
            area << std::fixed << std::setprecision(2) << mapped.area;
            std::cout << "Area: " << area.str() << " um^2\n";
            cellsOk = reportEquivalence("original vs cells", forest, cellNetlistToForest(cellNetlist), equivMethod);
        }

        if (!netlistPath.empty()) {
            std::ofstream out(netlistPath);
            if (!out) throw std::runtime_error("cannot write " + netlistPath);
            if (mapCells) writeCellNetlist(out, cellNetlist);
            else writeVerilog(out, {design.name, design.ports, optimizedForest, design.registers, design.reg_decls, design.macros, {}});
            std::cout << "\nWrote " << (mapCells ? "cell" : "optimized") << " netlist to " << netlistPath << "\n";
        }

        if (runSta && nldm) {
            Sdc sdc;
            if (!sdcPath.empty()) {
                sdc = readSdc(sdcPath, lib.time_unit_ps, lib.cap_unit_ff);
            } else {
                // A real clock on the registers' clock port, or a virtual one for combinational designs
                std::ostringstream text;
                std::string inputs = "[all_inputs]";
                text << "create_clock -name vclk -period " << constraints.clock_period_ps / lib.time_unit_ps;   // "vclk": virtual unless bound to a port below
                if (!design.registers.empty()) {
                    text << " [get_ports {" << design.registers[0].clock << "}]";
                    inputs = "[all_inputs -no_clocks]";
                }
                text << "\n"
                     << "set_input_delay " << constraints.input_delay_ps / lib.time_unit_ps << " -clock vclk " << inputs << "\n"
                     << "set_output_delay " << constraints.output_delay_ps / lib.time_unit_ps << " -clock vclk [all_outputs]\n"
                     << "set_load " << constraints.output_load_ff / lib.cap_unit_ff << " [all_outputs]\n";
                if (lib.find("sg13cmos5l_buf_4")) text << "set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X " << inputs << "\n";
                sdc = parseSdc(text.str(), lib.time_unit_ps, lib.cap_unit_ff);
            }
            std::cout << "\n=== Static Timing Analysis (NLDM, " << lib.name << ") ===\n";
            printTimingResult(std::cout, analyzeNetlistTiming(cellNetlist, lib, sdc));
        } else if (runSta) {
            printTimingReport(std::cout, analyzeTiming(optimizedForest, constraints, delayModel, corner));
        }
        return mappedOk && optimizedOk && cellsOk ? 0 : kExitNotEquivalent;
        return mappedOk && optimizedOk ? 0 : kExitNotEquivalent;

    } catch (const std::exception& e) {
        std::cerr << "Compilation error: " << e.what() << "\n";
        return kExitError;
    }
}
