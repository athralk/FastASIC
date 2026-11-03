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
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
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

bool reportEquivalence(const std::string& label, const GateForest& a, const GateForest& b) {
    EquivalenceResult result = checkEquivalence(a, b);
    std::cout << "  " << label << ": ";
    if (result.equivalent) {
        std::cout << "PASS (" << (result.exhaustive ? "exhaustive, " : "random, ") << result.patterns << " patterns)\n";
    } else {
        std::cout << "FAIL - " << result.message << "\n";
    }
    return result.equivalent;
}

int usage(const char* program) {
    std::cerr << "Usage: " << program << " [-q|--quiet] [-o netlist.v] [--sta [timing options]] <source.v>\n"
              << "       " << program << " --equiv <first.v> <second.v>\n"
              << "  -q, --quiet           print gate counts and checks only, not the gate trees\n"
              << "  -o FILE               write the optimized NAND + INV netlist as structural Verilog\n"
              << "  --sta                 static timing analysis of the optimized netlist (IHP sg13cmos5l cells)\n"
              << "Timing options:\n"
              << "  --delay-model rc|unit RC: t = R*C_par + R*C_load (default); unit: every cell = 1\n"
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
    DelayModel delayModel = DelayModel::RC;
    Corner corner = Corner::TYP;
    TimingConstraints constraints;
    std::string netlistPath;
    std::vector<std::string> files;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            bool hasValue = i + 1 < argc;
            if (arg == "--equiv") equivMode = true;
            else if (arg == "-q" || arg == "--quiet") quiet = true;
            else if (arg == "--sta") runSta = true;
            else if (arg == "-o" && hasValue) netlistPath = argv[++i];
            else if (arg == "--delay-model" && hasValue) {
                std::string name = argv[++i];
                if (name == "rc") delayModel = DelayModel::RC;
                else if (name == "unit") delayModel = DelayModel::UNIT;
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
            bool equivalent = reportEquivalence(files[0] + " vs " + files[1], first.forest, second.forest);
            return equivalent ? 0 : kExitNotEquivalent;
        }

        Design design = parseFile(files[0]);
        GateForest& forest = design.forest;
        printForest("Original AST Gate Forest", "Root", forest, quiet);

        // Run Technology Mapping to NAND + Inverter Form
        GateForest mappedForest = TechnologyMapper::mapToNandInverter(forest);
        printForest("Mapped Canonical NAND + Inverter Netlist", "Mapped Root", mappedForest, quiet);

        // Strash, constant propagation and double-inversion removal on the mapped netlist
        GateForest optimizedForest = NetlistOptimizer::optimize(mappedForest);
        printForest("Optimized NAND + Inverter Netlist", "Optimized Root", optimizedForest, quiet);

        // Every transformation must preserve the logic function
        std::cout << "\n=== Equivalence Checks ===\n";
        bool mappedOk = reportEquivalence("original vs mapped", forest, mappedForest);
        bool optimizedOk = reportEquivalence("original vs optimized", forest, optimizedForest);

        if (!netlistPath.empty()) {
            std::ofstream out(netlistPath);
            if (!out) throw std::runtime_error("cannot write " + netlistPath);
            writeVerilog(out, {design.name, design.ports, optimizedForest});
            std::cout << "\nWrote optimized netlist to " << netlistPath << "\n";
        }

        if (runSta) {
            printTimingReport(std::cout, analyzeTiming(optimizedForest, constraints, delayModel, corner));
        }
        return mappedOk && optimizedOk ? 0 : kExitNotEquivalent;

    } catch (const std::exception& e) {
        std::cerr << "Compilation error: " << e.what() << "\n";
        return kExitError;
    }
}
