#include "../include/cell_netlist.hpp"
#include "../include/netlist_utils.hpp"
#include <cctype>
#include <fstream>
#include <functional>
#include <map>
#include <numeric>
#include <sstream>
#include <string_view>
#include <stdexcept>
#include <unordered_map>

double CellNetlist::area() const {
    double total = 0;
    for (const Instance& inst : instances) total += inst.cell->area;
    return total;
}

namespace {

// ---------------------------------------------------------------------------------------------
// Tokenizer for structural Verilog
// ---------------------------------------------------------------------------------------------
struct Tok {
    enum Kind { IDENT, NUMBER, SYMBOL, END } kind;
    std::string_view text;   // into the source buffer, which outlives the parse
    int line;
};

std::vector<Tok> tokenize(const std::string& src, const std::string& path) {
    std::string_view view(src);
    std::vector<Tok> toks;
    size_t i = 0;
    int line = 1;
    auto error = [&](const std::string& m) { throw std::runtime_error(path + ":" + std::to_string(line) + ": " + m); };
    while (i < src.size()) {
        char c = src[i];
        if (c == '\n') { line++; i++; continue; }
        if (std::isspace(static_cast<unsigned char>(c))) { i++; continue; }
        if (c == '/' && i + 1 < src.size() && src[i + 1] == '/') {
            while (i < src.size() && src[i] != '\n') i++;
            continue;
        }
        if (c == '/' && i + 1 < src.size() && src[i + 1] == '*') {
            size_t end = src.find("*/", i + 2);
            if (end == std::string::npos) error("unterminated comment");
            for (size_t k = i; k < end; ++k) line += src[k] == '\n';
            i = end + 2;
            continue;
        }
        if (c == '(' && i + 1 < src.size() && src[i + 1] == '*') {   // (* attribute *)
            size_t end = src.find("*)", i + 2);
            if (end == std::string::npos) error("unterminated attribute");
            i = end + 2;
            continue;
        }
        if (c == '`') {   // `timescale etc.
            while (i < src.size() && src[i] != '\n') i++;
            continue;
        }
        if (c == '\\') {   // escaped identifier: up to whitespace
            size_t start = ++i;
            while (i < src.size() && !std::isspace(static_cast<unsigned char>(src[i]))) i++;
            toks.push_back({Tok::IDENT, view.substr(start, i - start), line});
            continue;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            size_t start = i;
            while (i < src.size() && (std::isalnum(static_cast<unsigned char>(src[i])) || src[i] == '_' || src[i] == '$')) i++;
            toks.push_back({Tok::IDENT, view.substr(start, i - start), line});
            continue;
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '\'') {
            size_t start = i;
            while (i < src.size() && (std::isalnum(static_cast<unsigned char>(src[i])) || src[i] == '\'' || src[i] == '_')) i++;
            toks.push_back({Tok::NUMBER, view.substr(start, i - start), line});
            continue;
        }
        toks.push_back({Tok::SYMBOL, view.substr(i, 1), line});
        i++;
    }
    toks.push_back({Tok::END, "", line});
    return toks;
}

// Bits of a sized or unsized constant, LSB first
std::vector<bool> constantBits(const std::string& text) {
    size_t q = text.find('\'');
    if (q == std::string::npos) {
        unsigned long long v = std::stoull(text);
        std::vector<bool> bits;
        for (int b = 0; b < 32; ++b) bits.push_back((v >> b) & 1);
        return bits;
    }
    int width = q == 0 ? 32 : std::stoi(text.substr(0, q));
    size_t p = q + 1;
    if (p < text.size() && (text[p] == 's' || text[p] == 'S')) p++;
    char base = static_cast<char>(std::tolower(text[p++]));
    int bits_per = base == 'h' ? 4 : base == 'o' ? 3 : base == 'b' ? 1 : 0;
    std::vector<bool> bits;
    std::string digits;
    for (size_t k = p; k < text.size(); ++k) {
        if (text[k] != '_') digits += text[k];
    }
    if (bits_per == 0) {
        unsigned long long v = std::stoull(digits);
        for (int b = 0; b < width; ++b) bits.push_back(b < 64 && ((v >> b) & 1));
        return bits;
    }
    for (auto it = digits.rbegin(); it != digits.rend(); ++it) {
        char d = static_cast<char>(std::tolower(*it));
        if (d == 'x' || d == 'z' || d == '?') throw std::runtime_error("x/z constants are not supported: " + text);
        int v = std::isdigit(static_cast<unsigned char>(d)) ? d - '0' : d - 'a' + 10;
        for (int b = 0; b < bits_per; ++b) bits.push_back((v >> b) & 1);
    }
    bits.resize(width, false);
    return bits;
}

class NetlistParser {
    std::vector<Tok> toks;
    size_t pos = 0;
    std::string path;
    const Library& lib;
    CellNetlist nl;

    struct Signal {
        int msb = 0, lsb = 0;
        bool is_vector = false;
        std::vector<int> nets;   // index k = bit (lsb + k) for msb >= lsb, (lsb - k) otherwise
    };
    std::unordered_map<std::string, Signal> signals;
    std::vector<int> parent;   // union-find over nets (assign aliases)

    [[noreturn]] void error(const std::string& m) const {
        throw std::runtime_error(path + ":" + std::to_string(toks[pos].line) + ": " + m);
    }
    const Tok& peek() const { return toks[pos]; }
    bool is(const char* s) const { return toks[pos].text == s && toks[pos].kind != Tok::END; }
    Tok next() { return toks[pos++]; }
    void expect(const char* s) {
        if (!is(s)) error(std::string("expected '") + s + "', got '" + std::string(peek().text) + "'");
        pos++;
    }
    std::string ident() {
        if (peek().kind != Tok::IDENT) error("expected a name, got '" + std::string(peek().text) + "'");
        return std::string(next().text);
    }
    int number() {
        if (peek().kind != Tok::NUMBER) error("expected a number");
        return std::stoi(std::string(next().text));
    }

    int newNet(const std::string& name) {
        parent.push_back(static_cast<int>(parent.size()));
        return nl.addNet(name);
    }
    int find(int n) {
        while (parent[n] != n) n = parent[n] = parent[parent[n]];
        return n;
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (a < b) std::swap(a, b);   // the smaller index (constants, then ports) represents
        parent[a] = b;
    }

    Signal& declare(const std::string& name, bool is_vector, int msb, int lsb) {
        auto it = signals.find(name);
        if (it != signals.end()) return it->second;   // "input a; wire a;" declares once
        Signal s;
        s.is_vector = is_vector;
        s.msb = msb;
        s.lsb = lsb;
        int width = std::abs(msb - lsb) + 1;
        for (int k = 0; k < width; ++k) {
            int bit = msb >= lsb ? lsb + k : lsb - k;
            int net = newNet(is_vector ? name + "[" + std::to_string(bit) + "]" : name);
            if (is_vector) nl.vector_nets.insert(net);
            s.nets.push_back(net);
        }
        return signals[name] = s;
    }

    Signal& lookup(const std::string& name) {
        auto it = signals.find(name);
        if (it != signals.end()) return it->second;
        return declare(name, false, 0, 0);   // implicit wire
    }

    int bitNet(Signal& s, int bit, const std::string& name) {
        int k = s.msb >= s.lsb ? bit - s.lsb : s.lsb - bit;
        if (k < 0 || k >= static_cast<int>(s.nets.size())) error("bit " + std::to_string(bit) + " of '" + name + "' is out of range");
        return s.nets[k];
    }

    // A connection expression as nets, MSB first
    std::vector<int> expr() {
        if (is("{")) {
            pos++;
            std::vector<int> bits;
            while (!is("}")) {
                std::vector<int> part = expr();
                bits.insert(bits.end(), part.begin(), part.end());
                if (is(",")) pos++;
            }
            pos++;
            return bits;
        }
        if (peek().kind == Tok::NUMBER) {
            std::vector<bool> lsb_first = constantBits(std::string(next().text));
            std::vector<int> bits;
            for (auto it = lsb_first.rbegin(); it != lsb_first.rend(); ++it) bits.push_back(*it ? CellNetlist::kConst1 : CellNetlist::kConst0);
            return bits;
        }
        std::string name = ident();
        Signal& s = lookup(name);
        if (is("[")) {
            pos++;
            int hi = number();
            int lo = hi;
            if (is(":")) {
                pos++;
                lo = number();
            }
            expect("]");
            std::vector<int> bits;
            int step = hi >= lo ? -1 : 1;
            for (int b = hi;; b += step) {
                bits.push_back(bitNet(s, b, name));
                if (b == lo) break;
            }
            return bits;
        }
        std::vector<int> bits(s.nets.rbegin(), s.nets.rend());   // whole signal, MSB first
        return bits;
    }

    void declaration(bool is_port, bool is_input) {
        if (is("wire") || is("reg")) pos++;
        bool is_vector = false;
        int msb = 0, lsb = 0;
        if (is("[")) {
            pos++;
            msb = number();
            expect(":");
            lsb = number();
            expect("]");
            is_vector = true;
        }
        for (;;) {
            std::string name = ident();
            declare(name, is_vector, msb, lsb);
            if (is_port) {
                bool known = false;
                for (auto& p : nl.ports) {
                    if (p.name == name) {
                        p = {name, is_input, is_vector, msb, lsb};
                        known = true;
                    }
                }
                if (!known) nl.ports.push_back({name, is_input, is_vector, msb, lsb});
            }
            if (!is(",")) break;
            // In ANSI headers a comma may be followed by the next direction keyword
            if (toks[pos + 1].text == "input" || toks[pos + 1].text == "output" || toks[pos + 1].text == "inout") break;
            pos++;
        }
    }

    void instance(const std::string& cell_name) {
        const LibCell* cell = lib.find(cell_name);
        if (!cell) error("unknown cell or statement '" + cell_name + "' (not in " + lib.name + ")");
        if (is("#")) error("parameterized instances are not supported");
        CellNetlist::Instance inst;
        inst.name = ident();
        inst.cell = cell;
        inst.pin_net.assign(cell->pins.size(), -1);
        expect("(");
        while (!is(")")) {
            expect(".");
            std::string pin = ident();
            int index = cell->pinIndex(pin);
            const LibBus* bus = index < 0 ? cell->findBus(pin) : nullptr;
            if (index < 0 && !bus) error("cell " + cell_name + " has no pin '" + pin + "'");
            expect("(");
            if (!is(")")) {
                std::vector<int> bits = expr();   // most significant first
                if (bus) {
                    // LSB-aligned onto the bus pins (also listed most significant first)
                    if (bits.size() > bus->pins.size()) error("bus " + inst.name + "/" + pin + " is connected to " + std::to_string(bits.size()) + " bits");
                    size_t offset = bus->pins.size() - bits.size();
                    for (size_t k = 0; k < bits.size(); ++k) inst.pin_net[bus->pins[offset + k]] = bits[k];
                } else {
                    if (bits.size() != 1) error("pin " + inst.name + "/" + pin + " is connected to " + std::to_string(bits.size()) + " bits");
                    inst.pin_net[index] = bits[0];
                }
            }
            expect(")");
            if (is(",")) pos++;
        }
        pos++;
        expect(";");
        nl.instances.push_back(std::move(inst));
    }

    void assign() {
        std::vector<int> lhs = expr();
        expect("=");
        std::vector<int> rhs = expr();
        expect(";");
        // Align LSBs; a narrower right side is zero-extended
        for (size_t k = 0; k < lhs.size(); ++k) {
            int r = k < rhs.size() ? rhs[rhs.size() - 1 - k] : CellNetlist::kConst0;
            unite(lhs[lhs.size() - 1 - k], r);
        }
    }

public:
    NetlistParser(std::vector<Tok> t, std::string p, const Library& l) : toks(std::move(t)), path(std::move(p)), lib(l) {
        parent = {0, 1};
    }

    CellNetlist parse() {
        expect("module");
        nl.name = ident();
        if (is("(")) {
            pos++;
            while (!is(")")) {
                if (is("input") || is("output") || is("inout")) {
                    bool input = next().text == "input";
                    declaration(true, input);
                } else {
                    nl.ports.push_back({ident(), true, false, 0, 0});   // direction comes later
                }
                if (is(",")) pos++;
            }
            pos++;
        }
        expect(";");
        while (!is("endmodule")) {
            if (peek().kind == Tok::END) error("missing endmodule");
            std::string word = ident();
            if (word == "input" || word == "output" || word == "inout") {
                declaration(true, word == "input");
                expect(";");
            } else if (word == "wire") {
                pos--;
                declaration(false, false);
                expect(";");
            } else if (word == "assign") {
                assign();
            } else if (word == "module") {
                error("only one (flattened) module per netlist is supported");
            } else {
                instance(word);
            }
        }
        return finish();
    }

    // Merge aliased nets, prefer port names, renumber densely
    CellNetlist finish() {
        for (const auto& port : nl.ports) {
            Signal& s = lookup(port.name);
            int width = static_cast<int>(s.nets.size());
            for (int k = 0; k < width; ++k) {
                int bit = s.msb >= s.lsb ? s.lsb + k : s.lsb - k;
                nl.port_bits.push_back({s.is_vector ? port.name + "[" + std::to_string(bit) + "]" : port.name, port.is_input, s.nets[k], s.is_vector});
            }
        }
        // Representative names: constants, then input ports, then output ports, then the rest
        std::vector<int> representative(nl.netCount());
        std::vector<int> rank(nl.net_names.size(), 4);
        auto offer = [&](int net, int r) {
            int root = find(net);
            if (r < rank[root]) {
                rank[root] = r;
                representative[root] = net;
            }
        };
        offer(CellNetlist::kConst0, 0);
        offer(CellNetlist::kConst1, 0);
        for (const auto& pb : nl.port_bits) offer(pb.net, pb.is_input ? 1 : 2);
        for (size_t n = 2; n < nl.net_names.size(); ++n) offer(static_cast<int>(n), 3);

        std::vector<int> new_index(nl.net_names.size(), -1);
        std::vector<std::string> names;
        std::unordered_set<int> vector_nets;
        for (size_t n = 0; n < nl.net_names.size(); ++n) {
            int root = find(static_cast<int>(n));
            if (new_index[root] < 0) {
                new_index[root] = static_cast<int>(names.size());
                int rep = representative[root];
                if (nl.vector_nets.count(rep)) vector_nets.insert(new_index[root]);
                names.push_back(nl.net_names[rep]);
            }
        }
        auto remap = [&](int net) { return net < 0 ? -1 : new_index[find(net)]; };
        for (auto& pb : nl.port_bits) pb.net = remap(pb.net);
        for (auto& inst : nl.instances) {
            for (int& n : inst.pin_net) n = remap(n);
        }
        nl.net_names = std::move(names);
        nl.vector_nets = std::move(vector_nets);
        if (new_index[find(0)] != 0 || new_index[find(1)] != 1) error("constant 0 and 1 were tied together");
        return std::move(nl);
    }
};

std::string verilogName(const std::string& name) {
    bool simple = !name.empty() && (std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_');
    for (char c : name) simple &= std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '$';
    return simple ? name : "\\" + name + " ";
}

// ---------------------------------------------------------------------------------------------
// Liberty function strings -> gate forest nodes
//   precedence: ! (and postfix ') > ^ > * & (and juxtaposition) > + |
// ---------------------------------------------------------------------------------------------
class FunctionBuilder {
    const std::string& f;
    size_t i = 0;
    const std::function<std::shared_ptr<GateNode>(const std::string&)>& pin;
    const std::function<std::shared_ptr<GateNode>(GateType)>& make;

    void space() {
        while (i < f.size() && f[i] == ' ') i++;
    }
    std::shared_ptr<GateNode> gate(GateType type, std::shared_ptr<GateNode> a, std::shared_ptr<GateNode> b = nullptr) {
        auto g = make(type);
        g->addFanin(a);
        if (b) g->addFanin(b);
        return g;
    }
    std::shared_ptr<GateNode> primary() {
        space();
        if (i >= f.size()) throw std::runtime_error("bad Liberty function '" + f + "'");
        std::shared_ptr<GateNode> n;
        if (f[i] == '!') {
            i++;
            n = gate(GateType::NOT, primary());
        } else if (f[i] == '(') {
            i++;
            n = orExpr();
            space();
            if (i >= f.size() || f[i] != ')') throw std::runtime_error("bad Liberty function '" + f + "'");
            i++;
        } else if (f[i] == '0' || f[i] == '1') {
            n = make(f[i] == '1' ? GateType::CONST1 : GateType::CONST0);
            i++;
        } else {
            size_t start = i;
            while (i < f.size() && (std::isalnum(static_cast<unsigned char>(f[i])) || f[i] == '_')) i++;
            if (start == i) throw std::runtime_error("bad Liberty function '" + f + "'");
            n = pin(f.substr(start, i - start));
        }
        space();
        while (i < f.size() && f[i] == '\'') {
            i++;
            n = gate(GateType::NOT, n);
            space();
        }
        return n;
    }
    std::shared_ptr<GateNode> xorExpr() {
        auto n = primary();
        for (space(); i < f.size() && f[i] == '^'; space()) {
            i++;
            n = gate(GateType::XOR, n, primary());
        }
        return n;
    }
    std::shared_ptr<GateNode> andExpr() {
        auto n = xorExpr();
        for (;;) {
            space();
            if (i < f.size() && (f[i] == '*' || f[i] == '&')) {
                i++;
                n = gate(GateType::AND, n, xorExpr());
            } else if (i < f.size() && (f[i] == '(' || f[i] == '!' || std::isalnum(static_cast<unsigned char>(f[i])))) {
                n = gate(GateType::AND, n, xorExpr());   // juxtaposition means AND
            } else {
                return n;
            }
        }
    }
    std::shared_ptr<GateNode> orExpr() {
        auto n = andExpr();
        for (space(); i < f.size() && (f[i] == '+' || f[i] == '|'); space()) {
            i++;
            n = gate(GateType::OR, n, andExpr());
        }
        return n;
    }

public:
    FunctionBuilder(const std::string& function, const std::function<std::shared_ptr<GateNode>(const std::string&)>& p,
                    const std::function<std::shared_ptr<GateNode>(GateType)>& m)
        : f(function), pin(p), make(m) {}
    std::shared_ptr<GateNode> build() {
        auto n = orExpr();
        space();
        if (i != f.size()) throw std::runtime_error("bad Liberty function '" + f + "'");
        return n;
    }
};

} // namespace

CellNetlist readCellNetlist(const std::string& path, const Library& lib) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return NetlistParser(tokenize(buffer.str(), path), path, lib).parse();
}

void writeCellNetlist(std::ostream& out, const CellNetlist& nl) {
    out << "// Generated by ASIC_backend\n";
    out << "module " << nl.name << " (";
    for (size_t i = 0; i < nl.ports.size(); ++i) out << (i ? ", " : "") << nl.ports[i].name;
    out << ");\n";
    for (const auto& p : nl.ports) {
        out << "  " << (p.is_input ? "input " : "output ");
        if (p.is_vector) out << "[" << p.msb << ":" << p.lsb << "] ";
        out << p.name << ";\n";
    }
    std::vector<bool> is_port_name(nl.netCount(), false);
    for (const auto& pb : nl.port_bits) {
        if (nl.net_names[pb.net] == pb.name) is_port_name[pb.net] = true;
    }
    for (size_t n = 2; n < nl.netCount(); ++n) {
        if (!is_port_name[n]) out << "  wire " << verilogName(nl.net_names[n]) << ";\n";
    }
    auto ref = [&](int net) {
        if (net == CellNetlist::kConst0) return std::string("1'b0");
        if (net == CellNetlist::kConst1) return std::string("1'b1");
        const std::string& name = nl.net_names[net];
        size_t b = name.find('[');
        if (is_port_name[net] && b != std::string::npos) return name;   // "a[3]" of a declared port vector
        return verilogName(name);
    };
    int dangling = 0;
    for (const auto& inst : nl.instances) {
        out << "  " << inst.cell->name << " " << verilogName(inst.name) << " (";
        bool first = true;
        std::vector<bool> in_bus(inst.pin_net.size(), false);
        for (const LibBus& bus : inst.cell->buses) {
            bool any = false;
            for (int k : bus.pins) {
                in_bus[k] = true;
                any |= inst.pin_net[k] >= 0;
            }
            if (!any) continue;
            out << (first ? "" : ",") << "\n    ." << bus.name << "({";
            for (size_t i = 0; i < bus.pins.size(); ++i) {
                int net = inst.pin_net[bus.pins[i]];
                std::string text = net >= 0 ? ref(net)
                                 : inst.cell->pins[bus.pins[i]].dir == PinDir::OUTPUT ? "" : "1'b0";
                if (text.empty()) {   // an unconnected output bit still needs a net in a concatenation
                    text = "\\_dangling" + std::to_string(dangling++) + "_ ";
                }
                out << (i ? ", " : "") << text;
            }
            out << "})";
            first = false;
        }
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] < 0 || in_bus[k]) continue;
            out << (first ? "" : ",") << "\n    ." << inst.cell->pins[k].name << "(" << ref(inst.pin_net[k]) << ")";
            first = false;
        }
        out << "\n  );\n";
    }
    for (const auto& pb : nl.port_bits) {
        if (!pb.is_input && nl.net_names[pb.net] != pb.name) out << "  assign " << pb.name << " = " << ref(pb.net) << ";\n";
    }
    out << "endmodule\n";
}

GateForest cellNetlistToForest(const CellNetlist& nl) {
    int next_id = 1;
    auto make = [&](GateType type, const std::string& name = "") { return std::make_shared<GateNode>(next_id++, type, name); };

    // Driver of every net: an input port, a constant, or an instance output pin
    std::vector<std::pair<int, int>> driver(nl.netCount(), {-1, -1});   // (instance, pin)
    for (size_t i = 0; i < nl.instances.size(); ++i) {
        const auto& inst = nl.instances[i];
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] >= 0 && inst.cell->pins[k].dir == PinDir::OUTPUT) {
                if (driver[inst.pin_net[k]].first >= 0) {
                    throw std::runtime_error("net '" + nl.net_names[inst.pin_net[k]] + "' has several drivers");
                }
                driver[inst.pin_net[k]] = {static_cast<int>(i), static_cast<int>(k)};
            }
        }
    }
    std::vector<bool> is_input(nl.netCount(), false);
    for (const auto& pb : nl.port_bits) {
        if (pb.is_input) is_input[pb.net] = true;
    }

    std::vector<std::shared_ptr<GateNode>> node(nl.netCount());
    std::vector<int> state(nl.netCount(), 0);   // 1 = being built (loop detection)
    std::function<std::shared_ptr<GateNode>(int)> build = [&](int net) -> std::shared_ptr<GateNode> {
        if (node[net]) return node[net];
        if (state[net] == 1) throw std::runtime_error("combinational loop through net '" + nl.net_names[net] + "'");
        state[net] = 1;
        std::shared_ptr<GateNode> n;
        if (net == CellNetlist::kConst0) n = make(GateType::CONST0);
        else if (net == CellNetlist::kConst1) n = make(GateType::CONST1);
        else if (is_input[net]) n = make(GateType::INPUT, nl.net_names[net]);
        else if (driver[net].first < 0) throw std::runtime_error("net '" + nl.net_names[net] + "' has no driver");
        else {
            const auto& inst = nl.instances[driver[net].first];
            const LibPin& out_pin = inst.cell->pins[driver[net].second];
            if (inst.cell->is_sequential || out_pin.function.empty()) {
                n = make(GateType::INPUT, nl.net_names[net]);   // register output: a free variable
            } else {
                std::function<std::shared_ptr<GateNode>(const std::string&)> pin = [&](const std::string& p) {
                    int k = inst.cell->pinIndex(p);
                    if (k < 0) throw std::runtime_error(inst.cell->name + ": function uses unknown pin '" + p + "'");
                    if (inst.pin_net[k] < 0) throw std::runtime_error(inst.name + "/" + p + " is unconnected");
                    return build(inst.pin_net[k]);
                };
                std::function<std::shared_ptr<GateNode>(GateType)> maker = [&](GateType t) { return make(t); };
                n = FunctionBuilder(out_pin.function, pin, maker).build();
            }
        }
        state[net] = 2;
        return node[net] = n;
    };

    GateForest forest;
    for (const auto& pb : nl.port_bits) {
        if (pb.is_input) continue;
        auto root = make(GateType::OUTPUT, pb.name);
        root->addFanin(build(pb.net));
        forest.push_back(root);
    }
    // Flip-flops: the register output is a free input named after its net (above); its next
    // state, from the Liberty ff group's next_state, becomes a root named nextStateName(net)
    // Hard macros: their input pins are cut points named like the elaborator names them
    for (const auto& inst : nl.instances) {
        if (!inst.cell->is_macro) continue;
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            if (inst.pin_net[k] < 0 || inst.cell->pins[k].dir == PinDir::OUTPUT) continue;
            auto root = make(GateType::OUTPUT, macroInputName(inst.name, inst.cell->pins[k].name));
            root->addFanin(build(inst.pin_net[k]));
            forest.push_back(root);
        }
    }
    for (const auto& inst : nl.instances) {
        if (!inst.cell->is_sequential || inst.cell->is_latch || inst.cell->ff_next_state.empty()) continue;
        for (size_t k = 0; k < inst.pin_net.size(); ++k) {
            const LibPin& pin = inst.cell->pins[k];
            if (pin.dir != PinDir::OUTPUT || inst.pin_net[k] < 0) continue;
            if (pin.function != "IQ" && pin.function != "IQN") continue;
            std::function<std::shared_ptr<GateNode>(const std::string&)> pinNode = [&](const std::string& p) {
                int j = inst.cell->pinIndex(p);
                if (j < 0) throw std::runtime_error(inst.cell->name + ": next_state uses unknown pin '" + p + "'");
                if (inst.pin_net[j] < 0) throw std::runtime_error(inst.name + "/" + p + " is unconnected");
                return build(inst.pin_net[j]);
            };
            std::function<std::shared_ptr<GateNode>(GateType)> maker = [&](GateType t) { return make(t); };
            auto next = FunctionBuilder(inst.cell->ff_next_state, pinNode, maker).build();
            if (pin.function == "IQN") {
                auto inv = make(GateType::NOT);
                inv->addFanin(next);
                next = inv;
            }
            auto root = make(GateType::OUTPUT, nextStateName(nl.net_names[inst.pin_net[k]]));
            root->addFanin(next);
            forest.push_back(root);
        }
    }
    return forest;
}
