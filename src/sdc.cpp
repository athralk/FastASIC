#include "../include/sdc.hpp"
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <stdexcept>

bool globMatch(const std::string& p, const std::string& t) {
    size_t pi = 0, ti = 0, star = std::string::npos, mark = 0;
    while (ti < t.size()) {
        if (pi < p.size() && (p[pi] == '?' || p[pi] == t[ti])) {
            pi++;
            ti++;
        } else if (pi < p.size() && p[pi] == '*') {
            star = pi++;
            mark = ti;
        } else if (star != std::string::npos) {
            pi = star + 1;
            ti = ++mark;
        } else {
            return false;
        }
    }
    while (pi < p.size() && p[pi] == '*') pi++;
    return pi == p.size();
}

namespace {

// A tiny Tcl: commands are lists of words; [cmd ...] is evaluated, {..} is literal,
// "$name" is substituted. Every value is a string; lists are space-separated.
class Interp {
    double time_scale, cap_scale;
    std::map<std::string, std::string> vars;
    Sdc& sdc;
    int line = 1;

    [[noreturn]] void error(const std::string& m) const { throw std::runtime_error("SDC line " + std::to_string(line) + ": " + m); }

    static std::vector<std::string> splitList(const std::string& s) {
        std::vector<std::string> out;
        std::string cur;
        int depth = 0;
        for (char c : s) {
            if (c == '{') {
                if (depth++ > 0) cur += c;
            } else if (c == '}') {
                if (--depth > 0) cur += c;
            } else if (std::isspace(static_cast<unsigned char>(c)) && depth == 0) {
                if (!cur.empty()) out.push_back(cur);
                cur.clear();
            } else {
                cur += c;
            }
        }
        if (!cur.empty()) out.push_back(cur);
        return out;
    }

    static std::string joinList(const std::vector<std::string>& items) {
        std::string out;
        for (const auto& i : items) out += (out.empty() ? "" : " ") + i;
        return out;
    }

    // Splits a command into words, evaluating [..] and $vars
    std::vector<std::string> words(const std::string& cmd) {
        std::vector<std::string> out;
        size_t i = 0;
        while (i < cmd.size()) {
            while (i < cmd.size() && std::isspace(static_cast<unsigned char>(cmd[i]))) i++;
            if (i >= cmd.size()) break;
            std::string word;
            bool literal = false;
            if (cmd[i] == '{') {
                int depth = 0;
                size_t start = i;
                do {
                    if (cmd[i] == '{') depth++;
                    else if (cmd[i] == '}') depth--;
                    i++;
                } while (i < cmd.size() && depth > 0);
                word = cmd.substr(start + 1, i - start - 2);
                literal = true;
            } else {
                bool quoted = cmd[i] == '"';
                if (quoted) i++;
                while (i < cmd.size() && (quoted ? cmd[i] != '"' : !std::isspace(static_cast<unsigned char>(cmd[i])))) {
                    if (cmd[i] == '[') {
                        int depth = 0;
                        size_t start = i;
                        do {
                            if (cmd[i] == '[') depth++;
                            else if (cmd[i] == ']') depth--;
                            i++;
                        } while (i < cmd.size() && depth > 0);
                        word += evaluate(cmd.substr(start + 1, i - start - 2));
                    } else if (cmd[i] == '$') {
                        i++;
                        std::string name;
                        if (i < cmd.size() && cmd[i] == '{') {
                            size_t end = cmd.find('}', i);
                            name = cmd.substr(i + 1, end - i - 1);
                            i = end + 1;
                        } else {
                            while (i < cmd.size() && (std::isalnum(static_cast<unsigned char>(cmd[i])) || cmd[i] == '_' || cmd[i] == ':' || cmd[i] == '(' || cmd[i] == ')')) {
                                name += cmd[i++];
                            }
                        }
                        auto it = vars.find(name);
                        if (it == vars.end()) error("unknown variable $" + name);
                        word += it->second;
                    } else if (cmd[i] == '\\' && i + 1 < cmd.size()) {
                        word += cmd[i + 1];
                        i += 2;
                    } else {
                        word += cmd[i++];
                    }
                }
                if (quoted && i < cmd.size()) i++;
            }
            (void)literal;
            out.push_back(word);
        }
        return out;
    }

    double num(const std::string& s) const {
        try {
            size_t used = 0;
            double v = std::stod(s, &used);
            if (used == s.size() && std::isfinite(v)) return v;
        } catch (const std::exception&) {
        }
        error("expected a number, got '" + s + "'");
    }

    // [expr ...]: + - * / and parentheses over numbers
    double exprValue(const std::string& e, size_t& i) const {
        auto space = [&] { while (i < e.size() && std::isspace(static_cast<unsigned char>(e[i]))) i++; };
        std::function<double()> sum, product, atom;
        atom = [&]() -> double {
            space();
            if (i < e.size() && e[i] == '(') {
                i++;
                double v = sum();
                space();
                if (i == e.size() || e[i] != ')') error("expected ')' in expr");
                i++;
                return v;
            }
            if (i < e.size() && (e[i] == '-' || e[i] == '+')) {
                bool negative = e[i++] == '-';
                double v = atom();
                return negative ? -v : v;
            }
            size_t used = 0;
            double v;
            try { v = std::stod(e.substr(i), &used); }
            catch (const std::exception&) { error("expected a number in expr"); }
            i += used;
            return v;
        };
        product = [&]() -> double {
            double v = atom();
            for (space(); i < e.size() && (e[i] == '*' || e[i] == '/'); space()) {
                char op = e[i++];
                double r = atom();
                v = op == '*' ? v * r : v / r;
            }
            return v;
        };
        sum = [&]() -> double {
            double v = product();
            for (space(); i < e.size() && (e[i] == '+' || e[i] == '-'); space()) {
                char op = e[i++];
                double r = product();
                v = op == '+' ? v + r : v - r;
            }
            return v;
        };
        double value = sum();
        if (i != e.size() || !std::isfinite(value)) error("invalid arithmetic expression: " + e);
        return value;
    }

    std::string evaluate(const std::string& cmd) {
        std::vector<std::string> w = words(cmd);
        if (w.empty()) return "";
        const std::string& c = w[0];
        auto flags = [&](size_t from) {   // the non-option words
            std::vector<std::string> rest;
            for (size_t k = from; k < w.size(); ++k) {
                if (!w[k].empty() && w[k][0] == '-' && !std::isdigit(static_cast<unsigned char>(w[k].size() > 1 ? w[k][1] : 'x'))) continue;
                for (const auto& item : splitList(w[k])) rest.push_back(item);
            }
            return rest;
        };
        auto has = [&](const char* f) {
            for (const auto& x : w) {
                if (x == f) return true;
            }
            return false;
        };
        auto option = [&](const char* f) -> std::string {
            for (size_t k = 0; k + 1 < w.size(); ++k) {
                if (w[k] == f) return w[k + 1];
            }
            return "";
        };
        // Words that are neither options nor option values
        auto positional = [&](std::initializer_list<const char*> valued) {
            std::vector<std::string> rest;
            for (size_t k = 1; k < w.size(); ++k) {
                bool is_valued = false;
                for (const char* v : valued) is_valued |= w[k] == v;
                if (is_valued) {
                    k++;
                    continue;
                }
                if (w[k].size() > 1 && w[k][0] == '-' && !std::isdigit(static_cast<unsigned char>(w[k][1])) && w[k][1] != '.') continue;
                rest.push_back(w[k]);
            }
            return rest;
        };

        if (c == "set") {
            if (w.size() < 3) error("set needs a name and a value");
            vars[w[1]] = w[2];
            return w[2];
        }
        if (c == "expr") {
            std::string e;
            for (size_t k = 1; k < w.size(); ++k) e += w[k] + " ";
            size_t i = 0;
            std::ostringstream out;
            out << std::setprecision(std::numeric_limits<double>::max_digits10) << exprValue(e, i);
            return out.str();
        }
        if (c == "get_ports" || c == "get_clocks" || c == "get_pins" || c == "get_nets" || c == "get_cells") {
            return joinList(flags(1));
        }
        if (c == "all_inputs") return has("-no_clocks") ? "@inputs_no_clocks" : "@inputs";
        if (c == "all_outputs") return "@outputs";
        if (c == "all_clocks") {
            std::vector<std::string> names;
            for (const auto& k : sdc.clocks) names.push_back(k.name);
            return joinList(names);
        }
        if (c == "current_design") return "@current_design";
        if (c == "set_units" || c == "puts") return "";

        if (c == "set_timing_derate") {
            auto pos = positional({});
            static const std::vector<std::string> options = {"-early", "-late", "-rise", "-fall", "-clock",
                                                             "-data", "-cell_delay", "-net_delay", "-cell_check"};
            bool unsupported = false;
            for (size_t k = 1; k < w.size(); ++k) {
                if (!w[k].empty() && w[k][0] == '-' && std::find(options.begin(), options.end(), w[k]) == options.end())
                    unsupported = true;
            }
            if (unsupported || (pos.size() != 1 && !(pos.size() == 2 && pos[1] == "@current_design"))) {
                sdc.warnings.push_back("unsupported options or object scope on set_timing_derate");
                return "";
            }
            if (has("-early") == has("-late")) error("set_timing_derate needs exactly one of -early and -late");
            double factor = num(pos[0]);
            if (!(factor > 0)) error("set_timing_derate needs a positive factor");
            auto selected = [&](const char* flag, const char* other) { return has(flag) || !has(other); };
            bool explicit_type = has("-cell_delay") || has("-net_delay") || has("-cell_check");
            for (auto type : {DerateType::Cell, DerateType::Net, DerateType::Check}) {
                const char* flag = type == DerateType::Cell ? "-cell_delay" : type == DerateType::Net ? "-net_delay" : "-cell_check";
                if (!has(flag) && (explicit_type || type == DerateType::Check)) continue;
                for (int clk = 0; clk < 2; ++clk) for (int fall = 0; fall < 2; ++fall) for (int late = 0; late < 2; ++late) {
                    if (selected(clk ? "-clock" : "-data", clk ? "-data" : "-clock") &&
                        selected(fall ? "-fall" : "-rise", fall ? "-rise" : "-fall") &&
                        selected(late ? "-late" : "-early", late ? "-early" : "-late"))
                        sdc.derates.factors[static_cast<int>(type)][clk][fall][late] = factor;
                }
            }
            return "";
        }

        if (c == "create_clock") {
            SdcClock clk;
            clk.period = num(option("-period")) * time_scale;
            if (!(clk.period > 0)) error("create_clock needs a positive period");
            std::vector<std::string> objs = positional({"-period", "-name", "-waveform", "-add"});
            for (const auto& o : objs) {
                for (const auto& item : splitList(o)) clk.sources.push_back(item);
            }
            clk.name = option("-name");
            if (clk.name.empty()) clk.name = clk.sources.empty() ? "clk" : clk.sources[0];
            clk.fall = clk.period / 2;
            std::string wave = option("-waveform");
            if (!wave.empty()) {
                auto edges = splitList(wave);
                if (edges.size() >= 2) {
                    clk.rise = num(edges[0]) * time_scale;
                    clk.fall = num(edges[1]) * time_scale;
                }
            }
            for (auto& existing : sdc.clocks) {
                if (existing.name == clk.name) {
                    existing = clk;
                    return clk.name;
                }
            }
            sdc.clocks.push_back(clk);
            return clk.name;
        }

        auto clocksNamed = [&](const std::vector<std::string>& names) {
            std::vector<SdcClock*> out;
            for (auto& k : sdc.clocks) {
                bool match = names.empty();
                for (const auto& n : names) match |= globMatch(n, k.name);
                if (match) out.push_back(&k);
            }
            return out;
        };
        auto listOf = [&](const std::vector<std::string>& objs) {
            std::vector<std::string> out;
            for (const auto& o : objs) {
                for (const auto& item : splitList(o)) out.push_back(item);
            }
            return out;
        };

        if (c == "set_clock_uncertainty") {
            auto pos = positional({});
            if (pos.empty()) error("set_clock_uncertainty needs a value");
            double v = num(pos[0]) * time_scale;
            bool setup = !has("-hold"), hold = !has("-setup");
            for (SdcClock* k : clocksNamed(listOf({pos.begin() + 1, pos.end()}))) {
                if (setup) k->uncertainty_setup = v;
                if (hold) k->uncertainty_hold = v;
            }
            return "";
        }
        if (c == "set_clock_transition" || c == "set_clock_latency") {
            auto pos = positional({});
            if (pos.empty()) error(c + " needs a value");
            double v = num(pos[0]) * time_scale;
            if (c == "set_clock_latency" && has("-source")) sdc.warnings.push_back("set_clock_latency -source treated as network latency");
            for (SdcClock* k : clocksNamed(listOf({pos.begin() + 1, pos.end()}))) {
                if (c == "set_clock_transition") k->transition = v;
                else k->latency += v;
            }
            return "";
        }
        if (c == "set_propagated_clock") {
            for (SdcClock* k : clocksNamed(listOf(positional({})))) k->propagated = true;
            return "";
        }
        if (c == "set_input_delay" || c == "set_output_delay") {
            auto pos = positional({"-clock", "-reference_pin"});
            if (pos.empty()) error(c + " needs a value");
            SdcPortDelay d;
            d.value = num(pos[0]) * time_scale;
            d.clock = option("-clock");
            d.ports = listOf({pos.begin() + 1, pos.end()});
            if (has("-max") && !has("-min")) d.min = false;
            if (has("-min") && !has("-max")) d.max = false;
            (c == "set_input_delay" ? sdc.input_delays : sdc.output_delays).push_back(d);
            return "";
        }
        if (c == "set_load" || c == "set_input_transition") {
            auto pos = positional({});
            if (pos.empty()) error(c + " needs a value");
            SdcPortValue v;
            v.value = num(pos[0]) * (c == "set_load" ? cap_scale : time_scale);
            v.ports = listOf({pos.begin() + 1, pos.end()});
            (c == "set_load" ? sdc.loads : sdc.input_transitions).push_back(v);
            return "";
        }
        if (c == "set_driving_cell") {
            SdcDrivingCell d;
            d.cell = option("-lib_cell");
            d.pin = option("-pin");
            d.from_pin = option("-from_pin");
            d.ports = listOf(positional({"-lib_cell", "-pin", "-from_pin", "-library", "-input_transition_rise", "-input_transition_fall"}));
            if (d.cell.empty()) error("set_driving_cell needs -lib_cell");
            sdc.driving_cells.push_back(d);
            return "";
        }
        sdc.warnings.push_back("ignored: " + c);
        return "";
    }

public:
    Interp(double t, double cap, Sdc& s) : time_scale(t), cap_scale(cap), sdc(s) {}

    void run(const std::string& text) {
        std::string cmd;
        int depth = 0;
        int cmd_line = 1;
        for (size_t i = 0; i <= text.size(); ++i) {
            char ch = i < text.size() ? text[i] : '\n';
            if (ch == '#' && depth == 0 && cmd.find_first_not_of(" \t") == std::string::npos) {
                while (i < text.size() && text[i] != '\n') i++;
                ch = '\n';
            }
            if (ch == '\\' && i + 1 < text.size() && text[i + 1] == '\n') {
                i++;
                line++;
                cmd += ' ';
                continue;
            }
            if (ch == '[' || ch == '{') depth++;
            if (ch == ']' || ch == '}') depth--;
            if ((ch == '\n' || ch == ';') && depth <= 0) {
                if (cmd.find_first_not_of(" \t\r") != std::string::npos) {
                    line = cmd_line;
                    evaluate(cmd);
                }
                cmd.clear();
                if (ch == '\n') line++;
                cmd_line = line;
                continue;
            }
            if (ch == '\n') line++;
            cmd += ch;
        }
    }
};

} // namespace

Sdc parseSdc(const std::string& text, double time_unit_ps, double cap_unit_ff) {
    Sdc sdc;
    Interp(time_unit_ps, cap_unit_ff, sdc).run(text);
    return sdc;
}

Sdc readSdc(const std::string& path, double time_unit_ps, double cap_unit_ff) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open SDC file " + path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return parseSdc(buffer.str(), time_unit_ps, cap_unit_ff);
}
