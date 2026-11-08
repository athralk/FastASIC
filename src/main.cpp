#include "../include/file_reader.hpp"
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
#include "../include/balance.hpp"
#include "../include/timer.hpp"
#include <cstdlib>
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
Design parseFile(const std::string& path) {
    VerilogLexer lexer(readVerilogFile(path));
    VerilogParser parser(lexer.tokenize());
    return elaborate(parser.parse());
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
};

bool reportEquivalence(const std::string& label, const GateForest& a, const GateForest& b, EquivMethod method) {
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
                  << result.by_sat << " by SAT, " << result.aig_ands << " AIG ANDs, " << result.sat.conflicts
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

int usage(const char* program) {
    std::cerr << "Usage: " << program << " [-q|--quiet] [-o netlist.v] [--equiv-method M] [--sta [timing options]] <source.v>\n"
              << "       " << program << " [--equiv-method M] --equiv <first.v> <second.v>\n"
              << "  -q, --quiet           print gate counts and checks only, not the gate trees\n"
              << "  -o FILE               write the optimized NAND + INV netlist as structural Verilog\n"
              << "  --equiv-method sat|sim  sat: SAT proof (default); sim: bit-parallel simulation, exhaustive up to\n"
              << "                        20 inputs, 65,536 random patterns above (a sample, not a proof)\n"
              << "  --map area|delay      map onto real sg13cmos5l cells by tree covering (-o then writes cells)\n"
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
    bool mapCells = false, nldm = false;
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
                else return usage(argv[0]);
            }
            else if (arg == "--delay-model" && hasValue) {
                std::string name = argv[++i];
                if (name == "rc") delayModel = DelayModel::RC;
                else if (name == "unit") delayModel = DelayModel::UNIT;
                else if (name == "nldm") nldm = true;
                else return usage(argv[0]);
            }
            else if (arg == "--corner" && hasValue) {
                std::string name = argv[++i];
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

        if (equivMode) {
            Design first = parseFile(files[0]);
            Design second = parseFile(files[1]);
            std::cout << "=== Equivalence Check ===\n";
            bool equivalent = reportEquivalence(files[0] + " vs " + files[1], first.forest, second.forest, equivMethod);
            return equivalent ? 0 : kExitNotEquivalent;
        }

        Design design = parseFile(files[0]);
        GateForest& forest = design.forest;
        printForest("Original AST Gate Forest", "Root", forest, quiet);

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
            MapResult mapped = mapToCells(design, optimizedForest, lib, mapGoal);
            cellNetlist = std::move(mapped.netlist);
            std::cout << "\n=== Technology Mapping (" << lib.name << ", " << (mapGoal == MapGoal::AREA ? "area" : "delay") << ") ===\n";
            int total = 0;
            std::string breakdown;
            for (const auto& [cell, count] : mapped.cell_counts) {
                total += count;
                breakdown += (breakdown.empty() ? "" : ", ") + cell.substr(cell.find('_') + 1) + ": " + std::to_string(count);
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
            else writeVerilog(out, {design.name, design.ports, optimizedForest});
            std::cout << "\nWrote " << (mapCells ? "cell" : "optimized") << " netlist to " << netlistPath << "\n";
        }

        if (runSta && nldm) {
            Sdc sdc;
            if (!sdcPath.empty()) {
                sdc = readSdc(sdcPath, lib.time_unit_ps, lib.cap_unit_ff);
            } else {
                std::ostringstream text;
                text << "create_clock -name vclk -period " << constraints.clock_period_ps / lib.time_unit_ps << "\n"
                     << "set_input_delay " << constraints.input_delay_ps / lib.time_unit_ps << " -clock vclk [all_inputs]\n"
                     << "set_output_delay " << constraints.output_delay_ps / lib.time_unit_ps << " -clock vclk [all_outputs]\n"
                     << "set_load " << constraints.output_load_ff / lib.cap_unit_ff << " [all_outputs]\n";
                if (lib.find("sg13cmos5l_buf_4")) text << "set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs]\n";
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
