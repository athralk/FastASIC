#include "../include/elaborator.hpp"
#include "../include/netlist_utils.hpp"
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <unordered_map>

namespace {

using NodePtr = std::shared_ptr<GateNode>;
using Bits = std::vector<NodePtr>;   // one node per bit, index 0 = least significant bit

constexpr long long kMaxWidth = 1 << 16;

std::string lowercase(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool isArithmeticOrBitwise(const std::string& op) {
    return op == "+" || op == "-" || op == "*" || op == "/" || op == "%" ||
           op == "&" || op == "|" || op == "^" || op == "~^";
}

// A declared net. Bits are stored by position (0 = least significant), which for
// [7:0] is index 0 but for an ascending [0:7] declaration is index 7.
struct Signal {
    NetKind kind = NetKind::WIRE;
    bool is_vector = false;
    long long msb = 0, lsb = 0;
    Bits bits;

    // Position of declared index i, or -1 when it is out of range
    long long position(long long i) const {
        long long p = msb >= lsb ? i - lsb : lsb - i;
        return (p < 0 || p >= static_cast<long long>(bits.size())) ? -1 : p;
    }
    std::string rangeText() const {
        return is_vector ? "[" + std::to_string(msb) + ":" + std::to_string(lsb) + "]" : "";
    }
};

class Elaborator {
    const Module& module;
    int next_id = 0;
    std::unordered_map<std::string, long long> params;
    std::unordered_map<std::string, Signal> signals;
    std::vector<std::string> signal_order;
    NodePtr const0, const1;

    [[noreturn]] void errorAt(size_t line, const std::string& message) const {
        throw std::runtime_error("line " + std::to_string(line) + ": " + message);
    }

    // ---------- Gate construction, with light constant folding so the netlist starts small ----------

    NodePtr constant(bool value) {
        auto& slot = value ? const1 : const0;
        if (!slot) {
            slot = std::make_shared<GateNode>(next_id++, value ? GateType::CONST1 : GateType::CONST0,
                                              value ? "1'b1" : "1'b0");
        }
        return slot;
    }

    static bool isConst(const NodePtr& n, bool value) {
        return n->type == (value ? GateType::CONST1 : GateType::CONST0);
    }

    NodePtr gate(GateType type, const NodePtr& a, const NodePtr& b = nullptr) {
        auto node = std::make_shared<GateNode>(next_id++, type, lowercase(gateTypeName(type)));
        node->addFanin(a);
        if (b) node->addFanin(b);
        return node;
    }

    NodePtr notGate(const NodePtr& a) {
        if (isConst(a, false)) return constant(true);
        if (isConst(a, true)) return constant(false);
        return gate(GateType::NOT, a);
    }

    NodePtr andGate(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false) || isConst(b, false)) return constant(false);
        if (isConst(a, true)) return b;
        if (isConst(b, true)) return a;
        return gate(GateType::AND, a, b);
    }

    NodePtr orGate(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, true) || isConst(b, true)) return constant(true);
        if (isConst(a, false)) return b;
        if (isConst(b, false)) return a;
        return gate(GateType::OR, a, b);
    }

    NodePtr xorGate(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false)) return b;
        if (isConst(b, false)) return a;
        if (isConst(a, true)) return notGate(b);
        if (isConst(b, true)) return notGate(a);
        return gate(GateType::XOR, a, b);
    }

    NodePtr xnorGate(const NodePtr& a, const NodePtr& b) {
        if (a->type == GateType::CONST0 || a->type == GateType::CONST1 ||
            b->type == GateType::CONST0 || b->type == GateType::CONST1) {
            return notGate(xorGate(a, b));
        }
        return gate(GateType::XNOR, a, b);
    }

    // Balanced tree: depth log2(n) instead of n, which matters once these paths are timed
    NodePtr reduce(GateType type, Bits v) {
        while (v.size() > 1) {
            Bits next;
            for (size_t i = 0; i + 1 < v.size(); i += 2) {
                next.push_back(type == GateType::AND ? andGate(v[i], v[i + 1])
                             : type == GateType::OR  ? orGate(v[i], v[i + 1])
                                                     : xorGate(v[i], v[i + 1]));
            }
            if (v.size() % 2) next.push_back(v.back());
            v = std::move(next);
        }
        return v[0];
    }

    // Ripple-carry adder: a + b + carry, as wide as the operands (carry out dropped)
    Bits add(const Bits& a, const Bits& b, NodePtr carry) {
        Bits sum;
        for (size_t i = 0; i < a.size(); ++i) {
            auto p = xorGate(a[i], b[i]);
            sum.push_back(xorGate(p, carry));
            carry = orGate(andGate(a[i], b[i]), andGate(p, carry));
        }
        return sum;
    }

    // Unsigned a >= b: the carry out of a + ~b + 1 (only the carry chain is built)
    NodePtr greaterOrEqual(const Bits& a, const Bits& b) {
        NodePtr carry = constant(true);
        for (size_t i = 0; i < a.size(); ++i) {
            auto nb = notGate(b[i]);
            carry = orGate(andGate(a[i], nb), andGate(xorGate(a[i], nb), carry));
        }
        return carry;
    }

    // Zero-extends or truncates to the requested width
    Bits resize(Bits v, long long width) {
        v.resize(static_cast<size_t>(width), constant(false));
        return v;
    }

    Bits constantBits(long long value, long long width) {
        Bits v;
        for (long long i = 0; i < width; ++i) v.push_back(constant((value >> std::min(i, 63LL)) & 1));
        return v;
    }

    // ---------- Constant expressions (parameters, ranges, indices, shift amounts) ----------

    bool isConstant(const Expr& e) const {
        switch (e.kind) {
        case ExprKind::NUMBER: return true;
        case ExprKind::IDENTIFIER:
            if (!params.count(e.name) && !signals.count(e.name)) errorAt(e.line, "undeclared identifier '" + e.name + "'");
            return params.count(e.name) > 0;
        case ExprKind::UNARY:
        case ExprKind::BINARY:
        case ExprKind::TERNARY:
            return std::all_of(e.operands.begin(), e.operands.end(), [&](const ExprPtr& op) { return isConstant(*op); });
        default:
            return false;
        }
    }

    long long requireConst(const Expr& e, const std::string& message) const {
        if (!isConstant(e)) errorAt(e.line, message);
        return evalConst(e);
    }

    long long evalConst(const Expr& e) const {
        switch (e.kind) {
        case ExprKind::NUMBER: {
            long long v = 0;
            for (size_t i = 0; i < e.bits.size(); ++i) {
                if (!e.bits[i]) continue;
                if (i >= 63) errorAt(e.line, "constant is too large to use here");
                v |= 1LL << i;
            }
            return v;
        }
        case ExprKind::IDENTIFIER:
            return params.at(e.name);
        case ExprKind::UNARY: {
            long long v = evalConst(*e.operands[0]);
            if (e.op == "-") return -v;
            if (e.op == "+") return v;
            if (e.op == "~") return ~v;
            if (e.op == "!") return !v;
            errorAt(e.line, "operator '" + e.op + "' is not supported in constant expressions");
        }
        case ExprKind::BINARY: {
            long long a = evalConst(*e.operands[0]);
            long long b = evalConst(*e.operands[1]);
            const std::string& op = e.op;
            if (op == "+") return a + b;
            if (op == "-") return a - b;
            if (op == "*") return a * b;
            if (op == "/" || op == "%") {
                if (b == 0) errorAt(e.line, "division by zero in constant expression");
                return op == "/" ? a / b : a % b;
            }
            if (op == "<<") return (b < 0 || b > 63) ? 0 : static_cast<long long>(static_cast<unsigned long long>(a) << b);
            if (op == ">>") return (b < 0 || b > 63) ? 0 : static_cast<long long>(static_cast<unsigned long long>(a) >> b);
            if (op == "&") return a & b;
            if (op == "|") return a | b;
            if (op == "^") return a ^ b;
            if (op == "~^") return ~(a ^ b);
            if (op == "==") return a == b;
            if (op == "!=") return a != b;
            if (op == "<") return a < b;
            if (op == "<=") return a <= b;
            if (op == ">") return a > b;
            if (op == ">=") return a >= b;
            if (op == "&&") return a && b;
            if (op == "||") return a || b;
            errorAt(e.line, "operator '" + op + "' is not supported in constant expressions");
        }
        case ExprKind::TERNARY:
            return evalConst(*e.operands[0]) ? evalConst(*e.operands[1]) : evalConst(*e.operands[2]);
        default:
            errorAt(e.line, "expected a constant expression");
        }
    }

    // ---------- Signals and widths ----------

    const Signal& lookupSignal(const std::string& name, size_t line) const {
        auto it = signals.find(name);
        if (it != signals.end()) return it->second;
        if (params.count(name)) errorAt(line, "'" + name + "' is a parameter, not a signal");
        errorAt(line, "undeclared net '" + name + "'");
    }

    long long checkedWidth(long long width, size_t line) const {
        if (width < 1 || width > kMaxWidth) errorAt(line, "expression width " + std::to_string(width) + " is out of range");
        return width;
    }

    long long replicationCount(const Expr& e) const {
        long long count = requireConst(*e.operands[0], "replication count must be a constant");
        if (count < 1) errorAt(e.line, "replication count must be positive");
        return count;
    }

    // The bits a select picks out (existing nodes, no new gates)
    Bits selectBits(const Expr& e) const {
        const Signal& sig = lookupSignal(e.name, e.line);
        auto position = [&](long long index) {
            long long p = sig.position(index);
            if (p < 0) {
                errorAt(e.line, "index " + std::to_string(index) + " is out of range for '" + e.name + sig.rangeText() + "'");
            }
            return p;
        };

        if (e.kind == ExprKind::BIT_SELECT) {
            long long index = requireConst(*e.operands[0], "bit select index must be a constant (variable indexing is not supported yet)");
            return {sig.bits[position(index)]};
        }

        long long high, low;   // positions of the selected range's two ends
        if (e.kind == ExprKind::PART_SELECT) {
            high = position(requireConst(*e.operands[0], "part select bounds must be constants"));
            low = position(requireConst(*e.operands[1], "part select bounds must be constants"));
            if (high < low) {
                errorAt(e.line, "part select of '" + e.name + "' is reversed relative to its declaration " + sig.rangeText());
            }
        } else {
            long long base = requireConst(*e.operands[0], "indexed part select base must be a constant");
            long long width = requireConst(*e.operands[1], "indexed part select width must be a constant");
            if (width < 1) errorAt(e.line, "indexed part select width must be positive");
            long long other = e.op == "+:" ? base + width - 1 : base - width + 1;
            high = std::max(position(base), position(other));
            low = std::min(position(base), position(other));
        }
        return Bits(sig.bits.begin() + low, sig.bits.begin() + high + 1);
    }

    // Self-determined width (Verilog 2005 table 5-22)
    long long selfWidth(const Expr& e) const {
        switch (e.kind) {
        case ExprKind::IDENTIFIER:
            if (params.count(e.name)) return 32;
            return static_cast<long long>(lookupSignal(e.name, e.line).bits.size());
        case ExprKind::NUMBER:
            return static_cast<long long>(e.bits.size());
        case ExprKind::UNARY:
            return (e.op == "~" || e.op == "-" || e.op == "+") ? selfWidth(*e.operands[0]) : 1;
        case ExprKind::BINARY:
            if (isArithmeticOrBitwise(e.op)) return std::max(selfWidth(*e.operands[0]), selfWidth(*e.operands[1]));
            if (e.op == "<<" || e.op == ">>") return selfWidth(*e.operands[0]);
            return 1;   // comparisons and logical operators
        case ExprKind::TERNARY:
            return std::max(selfWidth(*e.operands[1]), selfWidth(*e.operands[2]));
        case ExprKind::CONCAT: {
            long long total = 0;
            for (const auto& op : e.operands) total += selfWidth(*op);
            return checkedWidth(total, e.line);
        }
        case ExprKind::REPLICATE:
            return checkedWidth(replicationCount(e) * selfWidth(*e.operands[1]), e.line);
        default:
            return static_cast<long long>(selectBits(e).size());
        }
    }

    // ---------- Expressions -> gates ----------

    Bits evaluateSelf(const Expr& e) { return evaluate(e, selfWidth(e)); }

    // Builds e at the given context width. Following Verilog, operands of arithmetic and
    // bitwise operators are extended to the context width *before* the operation, so
    // "assign {c, s} = a + b" keeps the carry.
    Bits evaluate(const Expr& e, long long width) {
        switch (e.kind) {
        case ExprKind::IDENTIFIER: {
            auto it = params.find(e.name);
            if (it != params.end()) return constantBits(it->second, width);
            return resize(lookupSignal(e.name, e.line).bits, width);
        }
        case ExprKind::NUMBER: {
            Bits v;
            for (bool b : e.bits) v.push_back(constant(b));
            return resize(v, width);
        }
        case ExprKind::BIT_SELECT:
        case ExprKind::PART_SELECT:
        case ExprKind::INDEXED_PART_SELECT:
            return resize(selectBits(e), width);
        case ExprKind::UNARY:
            return evaluateUnary(e, width);
        case ExprKind::BINARY:
            return evaluateBinary(e, width);
        case ExprKind::TERNARY: {
            // Per-bit 2:1 mux: (s & t) | (~s & f)
            NodePtr s = reduce(GateType::OR, evaluateSelf(*e.operands[0]));
            NodePtr not_s = notGate(s);
            Bits t = evaluate(*e.operands[1], width);
            Bits f = evaluate(*e.operands[2], width);
            Bits out;
            for (long long i = 0; i < width; ++i) {
                out.push_back(t[i] == f[i] ? t[i] : orGate(andGate(s, t[i]), andGate(not_s, f[i])));
            }
            return out;
        }
        case ExprKind::CONCAT: {
            Bits out;   // the last operand holds the least significant bits
            for (auto it = e.operands.rbegin(); it != e.operands.rend(); ++it) {
                Bits part = evaluateSelf(**it);
                out.insert(out.end(), part.begin(), part.end());
            }
            return resize(out, width);
        }
        case ExprKind::REPLICATE: {
            long long count = replicationCount(e);
            Bits inner = evaluateSelf(*e.operands[1]);
            checkedWidth(count * static_cast<long long>(inner.size()), e.line);
            Bits out;
            for (long long i = 0; i < count; ++i) out.insert(out.end(), inner.begin(), inner.end());
            return resize(out, width);
        }
        }
        errorAt(e.line, "unsupported expression");
    }

    Bits evaluateUnary(const Expr& e, long long width) {
        const Expr& operand = *e.operands[0];
        if (e.op == "~") {
            Bits v = evaluate(operand, width);
            for (auto& b : v) b = notGate(b);
            return v;
        }
        if (e.op == "+") return evaluate(operand, width);
        if (e.op == "-") {
            // Two's complement: -x = ~x + 1
            Bits v = evaluate(operand, width);
            for (auto& b : v) b = notGate(b);
            return add(v, constantBits(0, width), constant(true));
        }

        // Logical not and reductions collapse the operand to a single bit
        NodePtr r;
        Bits v = evaluateSelf(operand);
        if (e.op == "!") r = notGate(reduce(GateType::OR, v));
        else if (e.op == "&" || e.op == "~&") r = reduce(GateType::AND, v);
        else if (e.op == "|" || e.op == "~|") r = reduce(GateType::OR, v);
        else if (e.op == "^" || e.op == "~^") r = reduce(GateType::XOR, v);
        else errorAt(e.line, "unsupported unary operator '" + e.op + "'");
        if (e.op == "~&" || e.op == "~|" || e.op == "~^") r = notGate(r);
        return resize({r}, width);
    }

    Bits evaluateBinary(const Expr& e, long long width) {
        const Expr& lhs = *e.operands[0];
        const Expr& rhs = *e.operands[1];
        const std::string& op = e.op;

        if (op == "&" || op == "|" || op == "^" || op == "~^") {
            Bits a = evaluate(lhs, width);
            Bits b = evaluate(rhs, width);
            for (long long i = 0; i < width; ++i) {
                a[i] = op == "&" ? andGate(a[i], b[i])
                     : op == "|" ? orGate(a[i], b[i])
                     : op == "^" ? xorGate(a[i], b[i])
                                 : xnorGate(a[i], b[i]);
            }
            return a;
        }
        if (op == "+") return add(evaluate(lhs, width), evaluate(rhs, width), constant(false));
        if (op == "-") {
            // a - b = a + ~b + 1
            Bits b = evaluate(rhs, width);
            for (auto& bit : b) bit = notGate(bit);
            return add(evaluate(lhs, width), b, constant(true));
        }
        if (op == "*" || op == "/" || op == "%") {
            if (!isConstant(e)) errorAt(e.line, "'" + op + "' is only supported between constants for now");
            return constantBits(evalConst(e), width);
        }
        if (op == "<<" || op == ">>") {
            long long amount = requireConst(rhs, "shift amount must be a constant (variable shifts are not supported yet)");
            if (amount < 0) errorAt(e.line, "shift amount must not be negative");
            Bits a = evaluate(lhs, width);
            Bits out(static_cast<size_t>(width), constant(false));
            for (long long i = 0; i < width; ++i) {
                long long from = op == "<<" ? i - amount : i + amount;
                if (from >= 0 && from < width) out[i] = a[from];
            }
            return out;
        }
        if (op == "&&" || op == "||") {
            NodePtr a = reduce(GateType::OR, evaluateSelf(lhs));
            NodePtr b = reduce(GateType::OR, evaluateSelf(rhs));
            return resize({op == "&&" ? andGate(a, b) : orGate(a, b)}, width);
        }

        // Comparisons: operands are sized to each other, not to the surrounding expression
        long long w = std::max(selfWidth(lhs), selfWidth(rhs));
        Bits a = evaluate(lhs, w);
        Bits b = evaluate(rhs, w);
        NodePtr result;
        if (op == "==" || op == "!=") {
            Bits same;
            for (long long i = 0; i < w; ++i) same.push_back(xnorGate(a[i], b[i]));
            result = reduce(GateType::AND, same);
            if (op == "!=") result = notGate(result);
        } else if (op == ">=") {
            result = greaterOrEqual(a, b);
        } else if (op == "<") {
            result = notGate(greaterOrEqual(a, b));
        } else if (op == "<=") {
            result = greaterOrEqual(b, a);
        } else if (op == ">") {
            result = notGate(greaterOrEqual(b, a));
        } else {
            errorAt(e.line, "unsupported operator '" + op + "'");
        }
        return resize({result}, width);
    }

    // The nodes an assignment drives, least significant first
    Bits lvalueBits(const Expr& e) const {
        switch (e.kind) {
        case ExprKind::IDENTIFIER:
            return lookupSignal(e.name, e.line).bits;
        case ExprKind::BIT_SELECT:
        case ExprKind::PART_SELECT:
        case ExprKind::INDEXED_PART_SELECT:
            return selectBits(e);
        case ExprKind::CONCAT: {
            Bits out;
            for (auto it = e.operands.rbegin(); it != e.operands.rend(); ++it) {
                Bits part = lvalueBits(**it);
                out.insert(out.end(), part.begin(), part.end());
            }
            return out;
        }
        default:
            errorAt(e.line, "invalid assignment target");
        }
    }

    // ---------- Module-level steps ----------

    void declareParams() {
        for (const auto& p : module.params) {
            if (params.count(p.name)) errorAt(p.line, "parameter '" + p.name + "' is already declared");
            params[p.name] = requireConst(*p.value, "value of parameter '" + p.name + "' must be a constant expression");
        }
    }

    void declareNets() {
        for (const auto& decl : module.nets) {
            if (params.count(decl.name)) errorAt(decl.line, "'" + decl.name + "' is already declared as a parameter");

            Signal sig;
            sig.kind = decl.kind;
            sig.is_vector = decl.msb != nullptr;
            if (sig.is_vector) {
                sig.msb = requireConst(*decl.msb, "range of '" + decl.name + "' must be constant");
                sig.lsb = requireConst(*decl.lsb, "range of '" + decl.name + "' must be constant");
            }
            long long width = checkedWidth((sig.msb >= sig.lsb ? sig.msb - sig.lsb : sig.lsb - sig.msb) + 1, decl.line);

            auto it = signals.find(decl.name);
            if (it != signals.end()) {
                // "output [3:0] y; wire [3:0] y;" is legal: the wire declaration only restates the port's net type
                bool is_port = it->second.kind != NetKind::WIRE;
                if (decl.kind == NetKind::WIRE && is_port) {
                    if (sig.is_vector && width != static_cast<long long>(it->second.bits.size())) {
                        errorAt(decl.line, "'" + decl.name + "' is redeclared with a different width");
                    }
                    continue;
                }
                errorAt(decl.line, "'" + decl.name + "' is already declared");
            }

            GateType type = decl.kind == NetKind::INPUT  ? GateType::INPUT
                          : decl.kind == NetKind::OUTPUT ? GateType::OUTPUT
                                                         : GateType::ASSIGN;
            for (long long p = 0; p < width; ++p) {
                long long index = sig.msb >= sig.lsb ? sig.lsb + p : sig.lsb - p;
                std::string bit_name = sig.is_vector ? decl.name + "[" + std::to_string(index) + "]" : decl.name;
                sig.bits.push_back(std::make_shared<GateNode>(next_id++, type, bit_name));
            }
            signals[decl.name] = std::move(sig);
            signal_order.push_back(decl.name);
        }
    }

    void connectAssigns() {
        for (const auto& assign : module.assigns) {
            Bits targets = lvalueBits(*assign.lhs);
            long long width = std::max(static_cast<long long>(targets.size()), selfWidth(*assign.rhs));
            Bits values = evaluate(*assign.rhs, width);
            for (size_t i = 0; i < targets.size(); ++i) {
                const auto& target = targets[i];
                if (target->type == GateType::INPUT) errorAt(assign.line, "cannot assign to input '" + target->name + "'");
                if (!target->inputs.empty()) errorAt(assign.line, "'" + target->name + "' has more than one driver");
                target->addFanin(values[i]);
            }
        }
    }

    void checkDrivers() const {
        for (const auto& port : module.header_ports) {
            auto it = signals.find(port);
            if (it == signals.end() || it->second.kind == NetKind::WIRE) {
                throw std::runtime_error("port '" + port + "' has no input/output declaration");
            }
        }
        for (const auto& name : signal_order) {
            const Signal& sig = signals.at(name);
            for (const auto& bit : sig.bits) {
                if (!bit->inputs.empty()) continue;
                if (sig.kind == NetKind::OUTPUT) {
                    throw std::runtime_error("output '" + bit->name + "' is never assigned");
                }
                if (sig.kind == NetKind::WIRE && !bit->outputs.empty()) {
                    throw std::runtime_error("wire '" + bit->name + "' is used but never assigned");
                }
            }
        }
    }

    PortInfo portInfo(const std::string& name) const {
        const Signal& sig = signals.at(name);
        return {name, sig.kind == NetKind::INPUT, sig.is_vector, static_cast<int>(sig.msb), static_cast<int>(sig.lsb)};
    }

public:
    explicit Elaborator(const Module& m) : module(m) {}

    Design run() {
        declareParams();
        declareNets();
        connectAssigns();
        checkDrivers();

        Design design;
        design.name = module.name;
        if (!module.header_ports.empty()) {
            for (const auto& name : module.header_ports) design.ports.push_back(portInfo(name));
        } else {
            for (const auto& name : signal_order) {
                if (signals.at(name).kind != NetKind::WIRE) design.ports.push_back(portInfo(name));
            }
        }
        for (const auto& name : signal_order) {
            const Signal& sig = signals.at(name);
            if (sig.kind != NetKind::OUTPUT) continue;
            for (const auto& bit : sig.bits) design.forest.push_back(bit);
        }

        topologicalOrder(design.forest);   // throws on combinational loops
        compactFanouts(design.forest);     // folded-away gates leave stale fanout links
        return design;
    }
};

} // namespace

Design elaborate(const Module& module) {
    return Elaborator(module).run();
}
