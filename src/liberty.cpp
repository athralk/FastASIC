#include "../include/liberty.hpp"
#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace {

// ---------------------------------------------------------------------------------------------
// Generic Liberty syntax: groups  name (args) { ... }
//                         simple attributes  name : value ;
//                         complex attributes  name (args) ;
// ---------------------------------------------------------------------------------------------
struct Group {
    std::string_view type;
    std::vector<std::string_view> args;
    std::vector<std::pair<std::string_view, std::string_view>> attrs;
    std::vector<std::pair<std::string_view, std::vector<std::string_view>>> complex;
    std::vector<Group> groups;
    int line = 0;

    std::string_view attr(std::string_view name) const {
        for (const auto& [k, v] : attrs) {
            if (k == name) return v;
        }
        return {};
    }
    const std::vector<std::string_view>* complexAttr(std::string_view name) const {
        for (const auto& [k, v] : complex) {
            if (k == name) return &v;
        }
        return nullptr;
    }
};

class Parser {
    std::string_view text;
    size_t pos = 0;
    int line = 1;
    std::string path;

    [[noreturn]] void error(const std::string& message) const {
        throw std::runtime_error(path + ":" + std::to_string(line) + ": " + message);
    }

    void skipSpace() {
        while (pos < text.size()) {
            char c = text[pos];
            if (c == '\n') {
                line++;
                pos++;
            } else if (c == ' ' || c == '\t' || c == '\r') {
                pos++;
            } else if (c == '\\' && pos + 1 < text.size() && (text[pos + 1] == '\n' || text[pos + 1] == '\r')) {
                pos++;   // line continuation
            } else if (c == '/' && pos + 1 < text.size() && text[pos + 1] == '*') {
                size_t end = text.find("*/", pos + 2);
                if (end == std::string_view::npos) error("unterminated comment");
                line += static_cast<int>(std::count(text.begin() + pos, text.begin() + end, '\n'));
                pos = end + 2;
            } else if (c == '/' && pos + 1 < text.size() && text[pos + 1] == '/') {
                while (pos < text.size() && text[pos] != '\n') pos++;
            } else {
                break;
            }
        }
    }

    static bool isPunct(char c) {
        return c == '(' || c == ')' || c == '{' || c == '}' || c == ':' || c == ';' || c == ',' || c == '"';
    }

    // A word or a quoted string (quotes stripped; may span continued lines)
    std::string_view token() {
        skipSpace();
        if (pos >= text.size()) error("unexpected end of file");
        if (text[pos] == '"') {
            size_t start = ++pos;
            while (pos < text.size() && text[pos] != '"') {
                if (text[pos] == '\n') line++;
                pos++;
            }
            if (pos >= text.size()) error("unterminated string");
            return text.substr(start, pos++ - start);
        }
        size_t start = pos;
        while (pos < text.size() && !isPunct(text[pos]) && !std::isspace(static_cast<unsigned char>(text[pos]))) pos++;
        if (start == pos) error(std::string("unexpected '") + text[pos] + "'");
        return text.substr(start, pos - start);
    }

    char peek() {
        skipSpace();
        return pos < text.size() ? text[pos] : '\0';
    }

    void expect(char c) {
        if (peek() != c) error(std::string("expected '") + c + "'");
        pos++;
    }

    void parseBody(Group& group) {
        while (peek() != '}') {
            int stmt_line = line;
            std::string_view name = token();
            char next = peek();
            if (next == ':') {
                pos++;
                std::string_view value = token();
                // Rare unquoted multi-token values (e.g. expressions) run to the ';'
                while (peek() != ';' && peek() != '}' && peek() != '\0') {
                    std::string_view more = token();
                    value = std::string_view(value.data(), more.data() + more.size() - value.data());
                }
                if (peek() == ';') pos++;
                group.attrs.emplace_back(name, value);
            } else if (next == '(') {
                pos++;
                std::vector<std::string_view> args;
                while (peek() != ')') {
                    args.push_back(token());
                    if (peek() == ',') pos++;
                }
                pos++;
                if (peek() == '{') {
                    pos++;
                    Group child;
                    child.type = name;
                    child.args = std::move(args);
                    child.line = stmt_line;
                    parseBody(child);
                    expect('}');
                    group.groups.push_back(std::move(child));
                } else {
                    if (peek() == ';') pos++;
                    group.complex.emplace_back(name, std::move(args));
                }
            } else {
                error("expected ':' or '(' after '" + std::string(name) + "'");
            }
        }
    }

public:
    Parser(std::string_view t, std::string p) : text(t), path(std::move(p)) {}

    Group parseLibrary() {
        Group root;
        root.type = token();
        if (root.type != "library") error("expected 'library'");
        expect('(');
        while (peek() != ')') {
            root.args.push_back(token());
            if (peek() == ',') pos++;
        }
        pos++;
        expect('{');
        parseBody(root);
        expect('}');
        return root;
    }
};

// ---------------------------------------------------------------------------------------------
// Semantic conversion
// ---------------------------------------------------------------------------------------------
double number(std::string_view s) {
    double v = 0;
    std::from_chars(s.data(), s.data() + s.size(), v);
    return v;
}

// Numbers separated by commas / spaces / line continuations, in one or more strings
std::vector<double> numberList(const std::vector<std::string_view>& args, double scale) {
    std::vector<double> out;
    for (std::string_view arg : args) {
        const char* p = arg.data();
        const char* end = p + arg.size();
        while (p < end) {
            while (p < end && (*p == ',' || *p == ' ' || *p == '\\' || *p == '\n' || *p == '\r' || *p == '\t')) p++;
            if (p >= end) break;
            double v = 0;
            auto [next, ec] = std::from_chars(p, end, v);
            if (ec != std::errc()) throw std::runtime_error("bad number in table: '" + std::string(arg) + "'");
            out.push_back(v * scale);
            p = next;
        }
    }
    return out;
}

// e.g. "1ns" -> 1000 (ps per unit), "100ps" -> 100
double timeScale(std::string_view unit) {
    std::string u(unit);
    double mult = std::strtod(u.c_str(), nullptr);
    if (mult == 0) mult = 1;
    if (u.find("ns") != std::string::npos) return mult * 1000;
    if (u.find("ps") != std::string::npos) return mult;
    if (u.find("us") != std::string::npos) return mult * 1e6;
    throw std::runtime_error("unsupported time_unit '" + u + "'");
}

TableVar tableVar(std::string_view v) {
    if (v == "input_net_transition" || v == "input_transition_time") return TableVar::INPUT_TRANSITION;
    if (v == "total_output_net_capacitance") return TableVar::OUTPUT_CAP;
    if (v == "constrained_pin_transition") return TableVar::CONSTRAINED_TRANSITION;
    if (v == "related_pin_transition") return TableVar::RELATED_TRANSITION;
    return TableVar::NONE;
}

struct Template {
    TableVar var1 = TableVar::NONE, var2 = TableVar::NONE;
    std::vector<double> index1, index2;   // raw (unscaled) values
};

TimingType timingType(std::string_view t) {
    if (t.empty() || t == "combinational") return TimingType::COMBINATIONAL;
    if (t == "rising_edge") return TimingType::RISING_EDGE;
    if (t == "falling_edge") return TimingType::FALLING_EDGE;
    if (t == "clear") return TimingType::CLEAR;
    if (t == "preset") return TimingType::PRESET;
    if (t == "setup_rising") return TimingType::SETUP_RISING;
    if (t == "setup_falling") return TimingType::SETUP_FALLING;
    if (t == "hold_rising") return TimingType::HOLD_RISING;
    if (t == "hold_falling") return TimingType::HOLD_FALLING;
    if (t == "recovery_rising") return TimingType::RECOVERY_RISING;
    if (t == "recovery_falling") return TimingType::RECOVERY_FALLING;
    if (t == "removal_rising") return TimingType::REMOVAL_RISING;
    if (t == "removal_falling") return TimingType::REMOVAL_FALLING;
    return TimingType::OTHER;
}

class Converter {
    double time_scale = 1000;   // ps per library time unit
    double cap_scale = 1000;    // fF per library capacitance unit
    std::unordered_map<std::string, Template> templates;

    double scaleFor(TableVar v) const {
        return v == TableVar::OUTPUT_CAP ? cap_scale : (v == TableVar::NONE ? 1 : time_scale);
    }

    LibTable table(const Group& g, double value_scale) const {
        LibTable t;
        Template tmpl;
        if (!g.args.empty()) {
            auto it = templates.find(std::string(g.args[0]));
            if (it != templates.end()) tmpl = it->second;
        }
        t.var1 = tmpl.var1;
        t.var2 = tmpl.var2;
        auto axis = [&](const char* name, const std::vector<double>& fallback, TableVar var) {
            const auto* own = g.complexAttr(name);
            std::vector<double> raw = own ? numberList(*own, 1) : fallback;
            for (double& x : raw) x *= scaleFor(var);
            return raw;
        };
        t.index1 = axis("index_1", tmpl.index1, t.var1);
        t.index2 = axis("index_2", tmpl.index2, t.var2);
        if (const auto* values = g.complexAttr("values")) t.values = numberList(*values, value_scale);

        size_t expected = std::max<size_t>(1, t.index1.size()) * std::max<size_t>(1, t.index2.size());
        if (t.values.size() != expected) {
            throw std::runtime_error("table at line " + std::to_string(g.line) + " has " + std::to_string(t.values.size()) +
                                     " values, expected " + std::to_string(expected));
        }
        return t;
    }

    void readPin(const Group& g, std::string_view name, LibCell& cell, std::vector<std::pair<const Group*, int>>& timing) {
        LibPin pin;
        pin.name = std::string(name);
        std::string_view dir = g.attr("direction");
        if (dir == "output") pin.dir = PinDir::OUTPUT;
        else if (dir == "inout") pin.dir = PinDir::INOUT;
        else if (dir == "internal") pin.dir = PinDir::INTERNAL;
        pin.cap = number(g.attr("capacitance")) * cap_scale;
        std::string_view rc = g.attr("rise_capacitance"), fc = g.attr("fall_capacitance");
        pin.rise_cap = rc.empty() ? pin.cap : number(rc) * cap_scale;
        pin.fall_cap = fc.empty() ? pin.cap : number(fc) * cap_scale;
        pin.rise_cap_min = pin.rise_cap_max = pin.rise_cap;
        pin.fall_cap_min = pin.fall_cap_max = pin.fall_cap;
        if (const auto* r = g.complexAttr("rise_capacitance_range"); r && r->size() == 2) {
            pin.rise_cap_min = number((*r)[0]) * cap_scale;
            pin.rise_cap_max = number((*r)[1]) * cap_scale;
        }
        if (const auto* f = g.complexAttr("fall_capacitance_range"); f && f->size() == 2) {
            pin.fall_cap_min = number((*f)[0]) * cap_scale;
            pin.fall_cap_max = number((*f)[1]) * cap_scale;
        }
        pin.max_cap = number(g.attr("max_capacitance")) * cap_scale;
        pin.max_transition = number(g.attr("max_transition")) * time_scale;
        pin.is_clock = g.attr("clock") == "true";
        pin.function = std::string(g.attr("function"));
        int index = static_cast<int>(cell.pins.size());
        cell.pins.push_back(pin);
        for (const Group& sub : g.groups) {
            if (sub.type == "timing") timing.emplace_back(&sub, index);
        }
    }

    void readArc(const Group& g, int to_pin, LibCell& cell) {
        std::string related(g.attr("related_pin"));
        std::istringstream names(related);
        std::string from;
        while (names >> from) {   // "A B" means one arc from each
            LibArc arc;
            arc.to_pin = to_pin;
            arc.from_pin = cell.pinIndex(from);
            if (arc.from_pin < 0) continue;
            std::string_view sense = g.attr("timing_sense");
            arc.sense = sense == "positive_unate" ? TimingSense::POSITIVE
                      : sense == "negative_unate" ? TimingSense::NEGATIVE : TimingSense::NON;
            arc.type = timingType(g.attr("timing_type"));
            arc.when = std::string(g.attr("when"));
            for (const Group& t : g.groups) {
                if (t.type == "cell_rise") arc.cell_rise = table(t, time_scale);
                else if (t.type == "cell_fall") arc.cell_fall = table(t, time_scale);
                else if (t.type == "rise_transition") arc.rise_transition = table(t, time_scale);
                else if (t.type == "fall_transition") arc.fall_transition = table(t, time_scale);
                else if (t.type == "rise_constraint") arc.rise_constraint = table(t, time_scale);
                else if (t.type == "fall_constraint") arc.fall_constraint = table(t, time_scale);
            }
            cell.arcs.push_back(std::move(arc));
        }
    }

    LibCell readCell(const Group& g) {
        LibCell cell;
        cell.name = g.args.empty() ? "" : std::string(g.args[0]);
        cell.area = number(g.attr("area"));
        cell.leakage = number(g.attr("cell_leakage_power"));
        std::vector<std::pair<const Group*, int>> timing;
        for (const Group& sub : g.groups) {
            if (sub.type == "pin") {
                for (std::string_view name : sub.args) readPin(sub, name, cell, timing);
            } else if (sub.type == "ff") {
                cell.is_sequential = true;
                cell.ff_clocked_on = std::string(sub.attr("clocked_on"));
                cell.ff_next_state = std::string(sub.attr("next_state"));
                cell.ff_clear = std::string(sub.attr("clear"));
                cell.ff_preset = std::string(sub.attr("preset"));
            } else if (sub.type == "latch") {
                cell.is_sequential = true;
                cell.is_latch = true;
            }
        }
        for (const auto& [group, pin] : timing) readArc(*group, pin, cell);   // pins can be referenced before they appear
        return cell;
    }

public:
    Library convert(const Group& root) {
        Library lib;
        lib.name = root.args.empty() ? "" : std::string(root.args[0]);
        if (std::string_view t = root.attr("time_unit"); !t.empty()) time_scale = timeScale(t);
        if (const auto* cap = root.complexAttr("capacitive_load_unit"); cap && cap->size() == 2) {
            double mult = number((*cap)[0]);
            std::string unit((*cap)[1]);
            std::transform(unit.begin(), unit.end(), unit.begin(), ::tolower);
            cap_scale = mult * (unit == "pf" ? 1000 : unit == "ff" ? 1 : unit == "nf" ? 1e6 : 1000);
        }
        lib.time_unit_ps = time_scale;
        lib.cap_unit_ff = cap_scale;
        lib.nom_voltage = number(root.attr("nom_voltage"));
        lib.nom_temperature = number(root.attr("nom_temperature"));
        lib.leakage_unit = std::string(root.attr("leakage_power_unit"));

        for (const Group& g : root.groups) {
            if (g.type != "lu_table_template" || g.args.empty()) continue;
            Template t;
            t.var1 = tableVar(g.attr("variable_1"));
            t.var2 = tableVar(g.attr("variable_2"));
            if (const auto* i1 = g.complexAttr("index_1")) t.index1 = numberList(*i1, 1);
            if (const auto* i2 = g.complexAttr("index_2")) t.index2 = numberList(*i2, 1);
            templates[std::string(g.args[0])] = t;
        }
        for (const Group& g : root.groups) {
            if (g.type != "cell") continue;
            lib.cell_index[g.args.empty() ? "" : std::string(g.args[0])] = static_cast<int>(lib.cells.size());
            lib.cells.push_back(readCell(g));
        }
        return lib;
    }
};

// Index of the segment [axis[i], axis[i+1]] used for x (clamped, so outside the table the
// end segment is extended), and the fraction t of the way along it
void locate(const std::vector<double>& axis, double x, size_t& i, double& t) {
    if (axis.size() < 2) {
        i = 0;
        t = 0;
        return;
    }
    size_t upper = static_cast<size_t>(std::upper_bound(axis.begin(), axis.end(), x) - axis.begin());
    i = upper == 0 ? 0 : std::min(upper - 1, axis.size() - 2);
    t = (x - axis[i]) / (axis[i + 1] - axis[i]);
}

double axisValue(TableVar var, const TableArgs& a) {
    switch (var) {
    case TableVar::INPUT_TRANSITION: return a.input_transition;
    case TableVar::OUTPUT_CAP: return a.output_cap;
    case TableVar::CONSTRAINED_TRANSITION: return a.constrained_transition;
    case TableVar::RELATED_TRANSITION: return a.related_transition;
    case TableVar::NONE: break;
    }
    return 0;
}

} // namespace

double LibTable::lookup(const TableArgs& args) const {
    if (values.empty()) return 0;
    if (values.size() == 1) return values[0];
    size_t i, j;
    double t, u;
    locate(index1, axisValue(var1, args), i, t);
    locate(index2, axisValue(var2, args), j, u);
    size_t cols = std::max<size_t>(1, index2.size());
    if (index2.size() < 2) return values[i] + t * (values[i + 1] - values[i]);
    if (index1.size() < 2) return values[j] + u * (values[j + 1] - values[j]);
    double v00 = values[i * cols + j], v01 = values[i * cols + j + 1];
    double v10 = values[(i + 1) * cols + j], v11 = values[(i + 1) * cols + j + 1];
    return (1 - t) * (1 - u) * v00 + (1 - t) * u * v01 + t * (1 - u) * v10 + t * u * v11;
}

bool isCheck(TimingType type) {
    switch (type) {
    case TimingType::SETUP_RISING: case TimingType::SETUP_FALLING:
    case TimingType::HOLD_RISING: case TimingType::HOLD_FALLING:
    case TimingType::RECOVERY_RISING: case TimingType::RECOVERY_FALLING:
    case TimingType::REMOVAL_RISING: case TimingType::REMOVAL_FALLING:
        return true;
    default:
        return false;
    }
}

int LibCell::pinIndex(const std::string& pin_name) const {
    for (size_t i = 0; i < pins.size(); ++i) {
        if (pins[i].name == pin_name) return static_cast<int>(i);
    }
    return -1;
}

int LibCell::outputPin() const {
    for (size_t i = 0; i < pins.size(); ++i) {
        if (pins[i].dir == PinDir::OUTPUT) return static_cast<int>(i);
    }
    return -1;
}

const LibCell* Library::find(const std::string& cell_name) const {
    auto it = cell_index.find(cell_name);
    return it == cell_index.end() ? nullptr : &cells[it->second];
}

Library readLiberty(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open Liberty file " + path);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    Group root = Parser(text, path).parseLibrary();
    Library lib = Converter().convert(root);
    lib.path = path;
    return lib;
}
