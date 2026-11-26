#include "../include/elaborator.hpp"
#include "../include/netlist_utils.hpp"
#include "../include/cec.hpp"
#include <sstream>
#include <algorithm>
#include <cctype>
#include <functional>
#include <stdexcept>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>

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
    bool is_reg = false;                  // a reg (kind REG) or an output reg
    bool is_signed = false;
    long long msb = 0, lsb = 0;
    bool is_memory = false;               // reg [msb:lsb] name [first:last]
    long long first = 0, last = 0;
    Bits bits;                            // memories: word position * word width + bit position

    long long wordWidth() const { return (msb >= lsb ? msb - lsb : lsb - msb) + 1; }
    long long wordPosition(long long w) const {
        long long p = first <= last ? w - first : first - w;
        long long words = (first <= last ? last - first : first - last) + 1;
        return (p < 0 || p >= words) ? -1 : p;
    }

    // Position of declared index i, or -1 when it is out of range
    long long position(long long i) const {
        long long p = msb >= lsb ? i - lsb : lsb - i;
        return (p < 0 || p >= wordWidth()) ? -1 : p;
    }
    std::string rangeText() const {
        return is_vector ? "[" + std::to_string(msb) + ":" + std::to_string(lsb) + "]" : "";
    }
};

// State shared by every module instance while the hierarchy is flattened into one netlist
struct Shared {
    std::unordered_map<std::string, const Module*> modules;
    int next_id = 0;
    NodePtr const0, const1;
    struct PendingRegister {
        RegisterBit bit;          // clock / reset names are filled in once they are traced to the top
        NodePtr clock, reset;     // the clock / reset nets as seen by the module that declared the register
        NodePtr next;             // the next-state root
        std::string where;
    };
    std::vector<PendingRegister> registers;
    std::vector<RegisterDecl> reg_decls;              // every reg declared anywhere, hierarchical names
    std::vector<std::string> stack;                   // module path being elaborated (recursion check)
    std::unordered_set<const GateNode*> top_inputs;   // top-level input port bits
    std::unordered_map<std::string, const BlackBoxModule*> blackboxes;
    std::vector<MacroInstance> macros;
    std::vector<NodePtr> macro_roots;                 // the macros' input pins
    const SramInference* srams = nullptr;
    AdderStyle adder = AdderStyle::RIPPLE;
    // A reg memory being turned into an SRAM: its one write and one registered read are recorded
    // during symbolic execution; the macro is chosen once the whole design is elaborated
    struct PendingSram {
        std::string instance, memory, where;
        long long words = 0, width = 0;
        NodePtr clock;
        const SramMacro* one_port = nullptr;   // best fitting macros, if any
        const SramMacro* two_port = nullptr;
        NodePtr wcond, rcond;
        Bits waddr, wdata, raddr;
        Bits q;                                // the read data, driving the read register
    };
    std::vector<PendingSram> pending_srams;
    std::vector<std::string> notes;
};

class Elaborator {
    Module module;                                    // a copy: generate blocks are expanded into it
    Shared& sh;
    std::string prefix;                               // "u_core.u_alu." for an instance, "" at the top
    bool is_top;
    using Overrides = std::unordered_map<std::string, std::function<ExprPtr(long long)>>;
    Overrides param_values;   // parameter values given by the instance
    std::unordered_set<const GateNode*> input_bits;          // this module's input port bits
    std::unordered_map<const Stmt*, std::pair<size_t, bool>> sram_access;   // stmt -> pending SRAM, is write
    NodePtr path_cond;                                       // condition under which the current statement runs
    std::unordered_map<std::string, ExprPtr> params;
    std::unordered_map<std::string, Signal> signals;
    std::vector<std::string> signal_order;
    std::unordered_set<std::string> integers;   // only usable as for-loop variables
    std::unordered_map<std::string, const FunctionDecl*> functions;
    int call_depth = 0;
    int generate_count = 0;

    // Procedural (always block) state
    enum class BlockKind { COMB, SEQ };
    struct Env {
        std::map<std::string, Bits> blocking, nonblocking;    // current values of assigned regs
        std::map<std::string, std::set<long long>> touched;    // bit positions written on any path
    };
    BlockKind block_kind = BlockKind::COMB;
    const std::map<std::string, Bits>* overrides = nullptr;   // blocking values visible to reads

    std::string context() const {
        return is_top ? "" : " (module " + module.name + ", instance " + prefix.substr(0, prefix.size() - 1) + ")";
    }

    [[noreturn]] void errorAt(size_t line, const std::string& message) const {
        throw std::runtime_error("line " + std::to_string(line) + context() + ": " + message);
    }

    // ---------- Gate construction, with light constant folding so the netlist starts small ----------

    NodePtr constant(bool value) {
        auto& slot = value ? sh.const1 : sh.const0;
        if (!slot) {
            slot = std::make_shared<GateNode>(sh.next_id++, value ? GateType::CONST1 : GateType::CONST0,
                                              value ? "1'b1" : "1'b0");
        }
        return slot;
    }

    static bool isConst(const NodePtr& n, bool value) {
        return n->type == (value ? GateType::CONST1 : GateType::CONST0);
    }

    NodePtr gate(GateType type, const NodePtr& a, const NodePtr& b = nullptr) {
        auto node = std::make_shared<GateNode>(sh.next_id++, type, lowercase(gateTypeName(type)));
        node->addFanin(a);
        if (b) node->addFanin(b);
        return node;
    }

    NodePtr notGate(const NodePtr& a) {
        if (isConst(a, false)) return constant(true);
        if (isConst(a, true)) return constant(false);
        if (a->type == GateType::NOT && !a->inputs.empty()) return a->inputs[0];
        auto it = not_memo.find(a.get());
        if (it != not_memo.end()) return it->second;
        NodePtr n = gate(GateType::NOT, a);
        not_memo[a.get()] = n;
        return n;
    }

    NodePtr andGate(const NodePtr& a, const NodePtr& b) {
        if (isConst(a, false) || isConst(b, false)) return constant(false);
        if (isConst(a, true)) return b;
        if (isConst(b, true)) return a;
        if (a == b) return a;
        const GateNode* p0 = std::min(a.get(), b.get());
        const GateNode* p1 = std::max(a.get(), b.get());
        auto it = and_memo.find({p0, p1});
        if (it != and_memo.end()) return it->second;
        NodePtr n = gate(GateType::AND, a, b);
        and_memo[{p0, p1}] = n;
        return n;
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

    // (g, p) o (g', p') = (g | p & g', p & p'): the carry operator of prefix adders
    void combine(NodePtr& g, NodePtr& p, const NodePtr& g_low, const NodePtr& p_low) {
        g = orGate(g, andGate(p, g_low));
        p = andGate(p, p_low);
    }

    // Prefix network over (G, P): afterwards G[i] is the group generate of positions 0..i
    void prefixNetwork(std::vector<NodePtr>& G, std::vector<NodePtr>& P) {
        size_t n = G.size();
        switch (sh.adder) {
        case AdderStyle::KOGGE_STONE:   // log2(n) levels, every position at every level
            for (size_t d = 1; d < n; d *= 2) {
                std::vector<NodePtr> g2 = G, p2 = P;
                for (size_t i = d; i < n; ++i) combine(g2[i], p2[i], G[i - d], P[i - d]);
                G = std::move(g2);
                P = std::move(p2);
            }
            break;
        case AdderStyle::BRENT_KUNG:    // 2 log2(n) levels, about 2n operators
            for (size_t d = 1; d < n; d *= 2) {
                for (size_t i = 2 * d - 1; i < n; i += 2 * d) combine(G[i], P[i], G[i - d], P[i - d]);
            }
            {
                size_t top = 1;   // the largest up-sweep distance; the down-sweep starts at half of it
                while (top * 2 < n) top *= 2;
                for (size_t d = top / 2; d >= 1; d /= 2) {
                    for (size_t i = 3 * d - 1; i < n; i += 2 * d) combine(G[i], P[i], G[i - d], P[i - d]);
                }
            }
            break;
        default:                        // Sklansky: log2(n) levels, divide and conquer
            for (size_t d = 1; d < n; d *= 2) {
                for (size_t i = 0; i < n; ++i) {
                    if (i & d) {
                        size_t j = (i & ~(2 * d - 1)) + d - 1;
                        combine(G[i], P[i], G[j], P[j]);
                    }
                }
            }
            break;
        }
    }

    // a + b + carry, as wide as the operands (carry out dropped): ripple carry (smallest) or a
    // parallel-prefix adder (logarithmic depth), per --adder
    Bits add(const Bits& a, const Bits& b, NodePtr carry) {
        Bits sum;
        if (sh.adder == AdderStyle::RIPPLE || a.size() < 3) {
            for (size_t i = 0; i < a.size(); ++i) {
                auto p = xorGate(a[i], b[i]);
                sum.push_back(xorGate(p, carry));
                carry = orGate(andGate(a[i], b[i]), andGate(p, carry));
            }
            return sum;
        }
        size_t n = a.size();
        std::vector<NodePtr> G(n + 1), P(n + 1), p(n);
        G[0] = carry;   // position 0 is the carry-in
        P[0] = constant(false);
        for (size_t i = 0; i < n; ++i) {
            p[i] = xorGate(a[i], b[i]);
            G[i + 1] = andGate(a[i], b[i]);
            P[i + 1] = p[i];
        }
        G.pop_back();   // the carry out is not needed
        P.pop_back();
        prefixNetwork(G, P);
        for (size_t i = 0; i < n; ++i) sum.push_back(xorGate(p[i], G[i]));
        return sum;
    }

    // Unsigned a >= b: the carry out of a + ~b + 1 (only the carry is built)
    NodePtr greaterOrEqual(const Bits& a, const Bits& b) {
        if (sh.adder == AdderStyle::RIPPLE) {
            NodePtr carry = constant(true);
            for (size_t i = 0; i < a.size(); ++i) {
                auto nb = notGate(b[i]);
                carry = orGate(andGate(a[i], nb), andGate(xorGate(a[i], nb), carry));
            }
            return carry;
        }
        // Balanced reduction of the (g, p) pairs: logarithmic depth
        std::vector<std::pair<NodePtr, NodePtr>> gp = {{constant(true), constant(false)}};
        for (size_t i = 0; i < a.size(); ++i) {
            auto nb = notGate(b[i]);
            gp.push_back({andGate(a[i], nb), xorGate(a[i], nb)});
        }
        while (gp.size() > 1) {
            std::vector<std::pair<NodePtr, NodePtr>> next;
            for (size_t i = 0; i + 1 < gp.size(); i += 2) {
                NodePtr g = gp[i + 1].first, p = gp[i + 1].second;
                combine(g, p, gp[i].first, gp[i].second);
                next.push_back({g, p});
            }
            if (gp.size() % 2) next.push_back(gp.back());
            gp = std::move(next);
        }
        return gp[0].first;
    }

    // Full adder: (sum, carry)
    std::pair<NodePtr, NodePtr> fullAdd(const NodePtr& a, const NodePtr& b, const NodePtr& c) {
        NodePtr p = xorGate(a, b);
        return {xorGate(p, c), orGate(andGate(a, b), andGate(p, c))};
    }

    // Bits of v that matter for its value as a signed number: the top bits that only repeat
    // the one below them (sign or zero extension) are dropped
    static size_t significantBits(const Bits& v) {
        size_t n = v.size();
        while (n > 1 && v[n - 1] == v[n - 2]) n--;
        return n;
    }

    // a * b mod 2^width: radix-4 Booth partial products (half as many rows as bits), the sign
    // extensions replaced by one constant row, a Dadda tree of full / half adders down to two
    // rows, then one carry-propagate adder (--adder). Operand bits that are only extension are
    // not multiplied, so 16 x 16 in a 32-bit context costs a 16 x 16 multiplier.
    Bits multiply(Bits a, Bits b, long long width) {
        size_t la = significantBits(a), lb = significantBits(b);
        if (lb > la) {
            std::swap(a, b);
            std::swap(la, lb);
        }
        std::vector<std::vector<NodePtr>> col(static_cast<size_t>(width));
        auto place = [&](long long c, const NodePtr& bit) {
            if (c < width && !isConst(bit, false)) col[c].push_back(bit);
        };
        auto yb = [&](long long k) -> NodePtr {   // multiplier bit k, sign-extended
            if (k < 0) return constant(false);
            return k < static_cast<long long>(lb) ? b[k] : b[lb - 1];
        };
        auto ab = [&](long long j) -> NodePtr {   // multiplicand bit j, sign-extended
            if (j < 0) return constant(false);
            return j < static_cast<long long>(la) ? a[j] : a[la - 1];
        };
        std::vector<bool> inverted_sign(width, false);
        long long rows = (static_cast<long long>(lb) + 1) / 2;
        for (long long i = 0; i < rows; ++i) {
            NodePtr y2 = yb(2 * i + 1), y1 = yb(2 * i), y0 = yb(2 * i - 1);
            NodePtr neg = y2;
            NodePtr one = xorGate(y1, y0);
            NodePtr two = orGate(andGate(y2, andGate(notGate(y1), notGate(y0))), andGate(notGate(y2), andGate(y1, y0)));
            long long base = 2 * i;
            if (base >= width) break;
            long long rw = static_cast<long long>(la) + 1;   // row bits 0..la, bit la is the sign
            for (long long j = 0; j < rw; ++j) {
                NodePtr pp = xorGate(orGate(andGate(one, ab(j)), andGate(two, ab(j - 1))), neg);
                place(base + j, j == rw - 1 ? notGate(pp) : pp);   // sign bit inverted (see correction)
            }
            place(base, neg);   // + 1 completes the two's complement of a negative digit
            // inverting the sign bit adds 2^(base + la): subtract it again in the constant row
            if (base + static_cast<long long>(la) < width) inverted_sign[base + la] = true;
        }
        // Two's complement of the inverted sign columns at the full product width.
        bool lowest_one_seen = false;
        for (long long c = 0; c < width; ++c) {
            if (inverted_sign[c] != lowest_one_seen) place(c, constant(true));
            lowest_one_seen |= inverted_sign[c];
        }
        // Constant ones in a column: fold pairs into the next column (no adders needed)
        for (long long c = 0; c < width; ++c) {
            int ones = 0;
            std::vector<NodePtr> rest;
            for (const auto& bit : col[c]) {
                if (isConst(bit, true)) ones++;
                else rest.push_back(bit);
            }
            if (c + 1 < width) {
                for (int k = 0; k < ones / 2; ++k) col[c + 1].push_back(constant(true));
            }
            if (ones % 2) rest.push_back(constant(true));
            col[c] = std::move(rest);
        }

        // Dadda: reduce every column to at most d bits, for d = ..., 9, 6, 4, 3, 2
        std::vector<size_t> heights = {2};
        while (heights.back() < 2000) heights.push_back(heights.back() * 3 / 2);
        size_t tallest = 0;
        for (const auto& c : col) tallest = std::max(tallest, c.size());
        for (size_t k = heights.size(); k-- > 0;) {
            size_t d = heights[k];
            if (d >= tallest) continue;
            std::vector<std::vector<NodePtr>> next(col.size());
            for (size_t c = 0; c < col.size(); ++c) {
                std::vector<NodePtr>& avail = col[c];
                size_t taken = 0;
                while (avail.size() - taken + next[c].size() > d && avail.size() - taken >= 2) {
                    bool half = avail.size() - taken + next[c].size() == d + 1 || avail.size() - taken == 2;
                    std::pair<NodePtr, NodePtr> r;
                    if (half) {
                        r = {xorGate(avail[taken], avail[taken + 1]), andGate(avail[taken], avail[taken + 1])};
                        taken += 2;
                    } else {
                        r = fullAdd(avail[taken], avail[taken + 1], avail[taken + 2]);
                        taken += 3;
                    }
                    next[c].push_back(r.first);
                    if (c + 1 < col.size()) next[c + 1].push_back(r.second);
                }
                for (size_t t = taken; t < avail.size(); ++t) next[c].push_back(avail[t]);
            }
            col = std::move(next);
            tallest = d;
        }
        Bits r0, r1;
        for (const auto& c : col) {
            r0.push_back(c.size() > 0 ? c[0] : constant(false));
            r1.push_back(c.size() > 1 ? c[1] : constant(false));
            if (c.size() > 2) throw std::logic_error("Dadda reduction left a column of " + std::to_string(c.size()));
        }
        return add(r0, r1, constant(false));
    }

    // Logarithmic barrel shifter: stage k shifts by 2^k when amount bit k is set
    Bits barrelShift(Bits v, const Bits& amount, bool left, const NodePtr& fill) {
        long long width = static_cast<long long>(v.size());
        NodePtr overflow = constant(false);   // amount >= width: everything shifted out
        for (size_t k = 0; k < amount.size(); ++k) {
            long long d = k < 62 ? (1LL << k) : width;
            if (d >= width) {
                overflow = orGate(overflow, amount[k]);
                continue;
            }
            Bits next(v.size());
            NodePtr s = amount[k], ns = notGate(s);
            for (long long i = 0; i < width; ++i) {
                long long from = left ? i - d : i + d;
                NodePtr moved = from >= 0 && from < width ? v[from] : fill;
                next[i] = moved == v[i] ? moved : orGate(andGate(s, moved), andGate(ns, v[i]));
            }
            v = std::move(next);
        }
        if (!isConst(overflow, false)) {
            for (auto& bit : v) bit = fill == bit ? bit : orGate(andGate(overflow, fill), andGate(notGate(overflow), bit));
        }
        return v;
    }

    // Zero-extends or truncates to the requested width
    Bits resize(Bits v, long long width) {
        v.resize(static_cast<size_t>(width), constant(false));
        return v;
    }

    // Truncates, or extends with the sign bit (signed) or zeros
    Bits extend(Bits v, long long width, bool sign) {
        NodePtr fill = sign && !v.empty() ? v.back() : constant(false);
        v.resize(static_cast<size_t>(width), fill);
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
            if (params.count(e.name)) return true;
            if (integers.count(e.name)) errorAt(e.line, "integer '" + e.name + "' can only be used as a for-loop variable");
            if (!signals.count(e.name)) errorAt(e.line, "undeclared identifier '" + e.name + "'");
            return false;
        case ExprKind::UNARY:
        case ExprKind::BINARY:
        case ExprKind::TERNARY:
        case ExprKind::CONCAT:
        case ExprKind::REPLICATE:
            return std::all_of(e.operands.begin(), e.operands.end(), [&](const ExprPtr& op) { return isConstant(*op); });
        case ExprKind::CALL:
            return (e.name == "$clog2" || e.name == "$signed" || e.name == "$unsigned") && e.operands.size() == 1 &&
                   isConstant(*e.operands[0]);
        default:
            return false;
        }
    }

    long long requireConst(const Expr& e, const std::string& message) {
        if (!isConstant(e)) errorAt(e.line, message);
        return evalConst(e);
    }

    long long evalConst(const Expr& e) {
        // Convert to a host index only at API boundaries that require one.
        Bits v = evaluateSelf(e);
        bool negative = isSigned(e) && isConst(v.back(), true);
        long long value = negative ? -1 : 0;
        for (size_t i = 0; i < v.size(); ++i) {
            if (isConst(v[i], negative)) continue;
            if (i >= 63) errorAt(e.line, "constant is too large to use here");
            value ^= 1LL << i;
        }
        return value;
    }

    ExprPtr constantLiteral(const Expr& e, long long declared_width = 0) {
        if (!isConstant(e)) errorAt(e.line, "parameter value must be a constant expression");
        auto lit = std::make_shared<Expr>();
        lit->kind = ExprKind::NUMBER;
        lit->line = e.line;
        lit->is_signed = !declared_width && isSigned(e);
        Bits v = evaluate(e, std::max(declared_width, selfWidth(e)));
        v.resize(declared_width ? declared_width : v.size());
        for (const auto& bit : v) lit->bits.push_back(isConst(bit, true));
        return lit;
    }

    // ---------- Signals and widths ----------

    const Signal& lookupSignal(const std::string& name, size_t line) const {
        auto it = signals.find(name);
        if (it != signals.end()) return it->second;
        if (params.count(name)) errorAt(line, "'" + name + "' is a parameter, not a signal");
        if (integers.count(name)) errorAt(line, "integer '" + name + "' can only be used as a for-loop variable");
        errorAt(line, "undeclared net '" + name + "'");
    }

    long long checkedWidth(long long width, size_t line) const {
        if (width < 1 || width > kMaxWidth) errorAt(line, "expression width " + std::to_string(width) + " is out of range");
        return width;
    }

    long long replicationCount(const Expr& e) {
        long long count = requireConst(*e.operands[0], "replication count must be a constant");
        if (count < 0) errorAt(e.line, "replication count must be nonnegative");
        return count;
    }

    // Value of a signal as seen right now: inside an always block, after its blocking assignments
    Bits currentBits(const std::string& name, size_t line) const {
        const Bits& declared = lookupSignal(name, line).bits;
        if (overrides) {
            auto it = overrides->find(name);
            if (it != overrides->end()) {
                Bits v = it->second;   // bits not assigned (yet) in this block read the signal itself
                for (size_t i = 0; i < v.size(); ++i) {
                    if (!v[i]) v[i] = declared[i];
                }
                return v;
            }
        }
        return declared;
    }

    static Bits checkedRead(Bits v, const Expr& e) {
        for (const auto& b : v) {
            if (!b) throw std::runtime_error("line " + std::to_string(e.line) + ": '" + e.name +
                                             "' is read before it is assigned on every path");
        }
        return v;
    }

    // Positions (within one word) of a constant bit / part select
    std::pair<long long, long long> staticRange(const Expr& e, const Signal& sig) {
        auto position = [&](long long index) {
            long long p = sig.position(index);
            if (p < 0) {
                errorAt(e.line, "index " + std::to_string(index) + " is out of range for '" + e.name + sig.rangeText() + "'");
            }
            return p;
        };
        if (e.kind == ExprKind::BIT_SELECT) {
            long long p = position(evalConst(*e.operands[0]));
            return {p, p};
        }
        if (e.kind == ExprKind::PART_SELECT) {
            long long high = position(requireConst(*e.operands[0], "part select bounds must be constants"));
            long long low = position(requireConst(*e.operands[1], "part select bounds must be constants"));
            if (high < low) errorAt(e.line, "part select of '" + e.name + "' is reversed relative to its declaration " + sig.rangeText());
            return {low, high};
        }
        long long base = requireConst(*e.operands[0], "indexed part select base must be a constant");
        long long width = requireConst(*e.operands[1], "indexed part select width must be a constant");
        if (width < 1) errorAt(e.line, "indexed part select width must be positive");
        long long other = e.op == "+:" ? base + width - 1 : base - width + 1;
        return {std::min(position(base), position(other)), std::max(position(base), position(other))};
    }

    // Width of a select, without building anything
    long long selectWidth(const Expr& e) {
        const Signal& sig = lookupSignal(e.name, e.line);
        if (sig.is_memory && !e.word) {
            if (e.kind != ExprKind::BIT_SELECT) errorAt(e.line, "select a word of memory '" + e.name + "' first: " + e.name + "[i]");
            return sig.wordWidth();
        }
        if (e.kind == ExprKind::BIT_SELECT) return 1;
        auto [lo, hi] = staticRange(e, sig);
        return hi - lo + 1;
    }

    // idx == value, as one node (constant 0 when value does not fit in idx)
    NodePtr equalsConstant(const Bits& idx, long long value) {
        if (value < 0) return constant(false);
        if (static_cast<size_t>(64) > idx.size() && (value >> idx.size()) != 0) return constant(false);
        Bits terms;
        for (size_t i = 0; i < idx.size(); ++i) {
            bool bit = i < 63 && ((value >> i) & 1);
            terms.push_back(bit ? idx[i] : notGate(idx[i]));
        }
        return terms.empty() ? constant(value == 0) : reduce(GateType::AND, terms);
    }

    struct Candidate {
        long long value;      // index value that selects it
        long long position;   // first bit position in the signal
    };

    // Candidates for an index into a vector (one bit each) or a memory (one word each)
    std::vector<Candidate> candidates(const Signal& sig, bool words) const {
        std::vector<Candidate> out;
        long long lo = words ? std::min(sig.first, sig.last) : std::min(sig.msb, sig.lsb);
        long long hi = words ? std::max(sig.first, sig.last) : std::max(sig.msb, sig.lsb);
        for (long long v = lo; v <= hi; ++v) {
            out.push_back({v, words ? sig.wordPosition(v) * sig.wordWidth() : sig.position(v)});
        }
        return out;
    }

    // src[index] with a variable index: a decoder and an OR of the selected elements
    // (an index outside the declared range reads 0)
    Bits dynamicRead(const Bits& src, const std::vector<Candidate>& cands, long long elem_width, const Bits& idx,
                     const Expr& e) {
        Bits out(static_cast<size_t>(elem_width), constant(false));
        for (const Candidate& c : cands) {
            NodePtr hit = equalsConstant(idx, c.value);
            if (isConst(hit, false)) continue;
            for (long long b = 0; b < elem_width; ++b) {
                const NodePtr& bit = src[static_cast<size_t>(c.position + b)];
                if (!bit) checkedRead({bit}, e);
                out[b] = orGate(out[b], andGate(hit, bit));
            }
        }
        return out;
    }

    // The bits a select reads (may build a decoder for variable indices)
    Bits selectBits(const Expr& e) {
        const Signal& sig = lookupSignal(e.name, e.line);
        const Bits src = currentBits(e.name, e.line);
        long long ww = sig.wordWidth();

        Bits word;   // the word (or whole vector) the inner select applies to
        if (sig.is_memory) {
            const Expr& index = e.word ? *e.word : *e.operands[0];
            if (!e.word && e.kind != ExprKind::BIT_SELECT) errorAt(e.line, "select a word of memory '" + e.name + "' first: " + e.name + "[i]");
            if (isConstant(index)) {
                long long p = sig.wordPosition(evalConst(index));
                if (p < 0) word = Bits(static_cast<size_t>(ww), constant(false));
                else word = checkedRead(Bits(src.begin() + p * ww, src.begin() + (p + 1) * ww), e);
            } else {
                word = dynamicRead(src, candidates(sig, true), ww, evaluateSelf(index), e);
            }
            if (!e.word) return word;
        } else {
            word = src;
        }

        if (e.kind == ExprKind::BIT_SELECT && !isConstant(*e.operands[0])) {
            std::vector<Candidate> cands = candidates(sig, false);
            return dynamicRead(word, cands, 1, evaluateSelf(*e.operands[0]), e);
        }
        auto [lo, hi] = staticRange(e, sig);
        return checkedRead(Bits(word.begin() + lo, word.begin() + hi + 1), e);
    }

    // Self-determined width (Verilog 2005 table 5-22)
    long long selfWidth(const Expr& e) {
        switch (e.kind) {
        case ExprKind::IDENTIFIER: {
            if (params.count(e.name)) return params.at(e.name)->bits.size();
            const Signal& sig = lookupSignal(e.name, e.line);
            if (sig.is_memory) errorAt(e.line, "memory '" + e.name + "' must be indexed: " + e.name + "[i]");
            return static_cast<long long>(sig.bits.size());
        }
        case ExprKind::NUMBER:
            return static_cast<long long>(e.bits.size());
        case ExprKind::UNARY:
            return (e.op == "~" || e.op == "-" || e.op == "+") ? selfWidth(*e.operands[0]) : 1;
        case ExprKind::BINARY:
            if (isArithmeticOrBitwise(e.op)) return std::max(selfWidth(*e.operands[0]), selfWidth(*e.operands[1]));
            if (e.op == "<<" || e.op == ">>" || e.op == "<<<" || e.op == ">>>") return selfWidth(*e.operands[0]);
            return 1;   // comparisons and logical operators
        case ExprKind::TERNARY:
            return std::max(selfWidth(*e.operands[1]), selfWidth(*e.operands[2]));
        case ExprKind::CONCAT: {
            long long total = 0;
            for (const auto& op : e.operands) {
                if (op->kind == ExprKind::REPLICATE && replicationCount(*op) == 0) {
                    selfWidth(*op->operands[1]);
                    continue;
                }
                total += selfWidth(*op);
            }
            return checkedWidth(total, e.line);
        }
        case ExprKind::REPLICATE: {
            long long inner = selfWidth(*e.operands[1]), count = replicationCount(e);
            if (count > kMaxWidth / inner) errorAt(e.line, "replication exceeds maximum width");
            return checkedWidth(count * inner, e.line);
        }
        case ExprKind::CALL:
            if (e.name == "$signed" || e.name == "$unsigned") return selfWidth(*e.operands.at(0));
            if (e.name == "$clog2") return 32;
            return functionWidth(e);
        default:
            return selectWidth(e);
        }
    }

    // Verilog signedness: an expression is signed only if all its operands are
    bool isSigned(const Expr& e) const {
        switch (e.kind) {
        case ExprKind::IDENTIFIER:
            if (params.count(e.name)) return params.at(e.name)->is_signed;
            return lookupSignal(e.name, e.line).is_signed;
        case ExprKind::NUMBER:
            return e.is_signed;
        case ExprKind::BIT_SELECT: {   // a whole memory word keeps the memory's signedness
            const Signal& sig = lookupSignal(e.name, e.line);
            return sig.is_memory && !e.word && sig.is_signed;
        }
        case ExprKind::UNARY:
            return (e.op == "~" || e.op == "-" || e.op == "+") && isSigned(*e.operands[0]);
        case ExprKind::BINARY:
            if (isArithmeticOrBitwise(e.op)) return isSigned(*e.operands[0]) && isSigned(*e.operands[1]);
            if (e.op == "<<" || e.op == ">>" || e.op == "<<<" || e.op == ">>>") return isSigned(*e.operands[0]);
            return false;
        case ExprKind::TERNARY:
            return isSigned(*e.operands[1]) && isSigned(*e.operands[2]);
        case ExprKind::CALL:
            if (e.name == "$signed" || e.name == "$clog2") return true;
            if (e.name == "$unsigned") return false;
            return functionSigned(e);
        default:
            return false;
        }
    }

    // ---------- Expressions -> gates ----------

    Bits evaluateSelf(const Expr& e) { return evaluate(e, selfWidth(e)); }

    // Builds e at the given context width. Following Verilog, operands of arithmetic and
    // bitwise operators are extended to the context width *before* the operation, so
    // "assign {c, s} = a + b" keeps the carry.
    Bits evaluate(const Expr& e, long long width) { return evaluateT(e, width, isSigned(e)); }

    // sx: whether the context-determined expression is signed, i.e. whether its operands are
    // sign-extended to the context width
    Bits evaluateT(const Expr& e, long long width, bool sx) {
        switch (e.kind) {
        case ExprKind::IDENTIFIER: {
            auto it = params.find(e.name);
            if (it != params.end()) return evaluateT(*it->second, width, sx);
            if (lookupSignal(e.name, e.line).is_memory) errorAt(e.line, "memory '" + e.name + "' must be indexed: " + e.name + "[i]");
            return extend(checkedRead(currentBits(e.name, e.line), e), width, sx);
        }
        case ExprKind::NUMBER: {
            if (!e.wild.empty()) errorAt(e.line, "x/z/? digits are only allowed in casez / casex labels");
            Bits v;
            for (bool b : e.bits) v.push_back(constant(b));
            return extend(v, width, sx);
        }
        case ExprKind::BIT_SELECT:
        case ExprKind::PART_SELECT:
        case ExprKind::INDEXED_PART_SELECT:
            return extend(selectBits(e), width, sx);
        case ExprKind::UNARY:
            return evaluateUnary(e, width, sx);
        case ExprKind::BINARY:
            return evaluateBinary(e, width, sx);
        case ExprKind::CALL: {
            if (e.name == "$signed" || e.name == "$unsigned") {
                if (e.operands.size() != 1) errorAt(e.line, e.name + " takes one argument");
                return extend(evaluateSelf(*e.operands[0]), width, sx);
            }
            if (e.name == "$clog2") {
                if (!isConstant(e)) errorAt(e.line, "$clog2 needs a constant argument");
                Bits arg = evaluateSelf(*e.operands[0]);
                // ceil(log2(n)) = bit length of n-1, interpreting the argument as unsigned.
                size_t highest = 0;
                bool lower_one = false;
                for (size_t i = 0; i < arg.size(); ++i) {
                    if (isConst(arg[i], true)) {
                        highest = i + lower_one;
                        lower_one = true;
                    }
                }
                return constantBits(highest, width);
            }
            if (!e.name.empty() && e.name[0] == '$') errorAt(e.line, "system function " + e.name + " is not supported");
            return extend(callFunction(e), width, sx);
        }
        case ExprKind::TERNARY: {
            // Per-bit 2:1 mux: (s & t) | (~s & f)
            NodePtr s = reduce(GateType::OR, evaluateSelf(*e.operands[0]));
            NodePtr not_s = notGate(s);
            Bits t = evaluateT(*e.operands[1], width, sx);
            Bits f = evaluateT(*e.operands[2], width, sx);
            Bits out;
            for (long long i = 0; i < width; ++i) {
                out.push_back(t[i] == f[i] ? t[i] : orGate(andGate(s, t[i]), andGate(not_s, f[i])));
            }
            return out;
        }
        case ExprKind::CONCAT: {
            Bits out;   // the last operand holds the least significant bits
            for (auto it = e.operands.rbegin(); it != e.operands.rend(); ++it) {
                if ((*it)->kind == ExprKind::REPLICATE && replicationCount(**it) == 0) continue;
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

    Bits evaluateUnary(const Expr& e, long long width, bool sx) {
        const Expr& operand = *e.operands[0];
        if (e.op == "~") {
            Bits v = evaluateT(operand, width, sx);
            for (auto& b : v) b = notGate(b);
            return v;
        }
        if (e.op == "+") return evaluateT(operand, width, sx);
        if (e.op == "-") {
            // Two's complement: -x = ~x + 1
            Bits v = evaluateT(operand, width, sx);
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

    Bits divideConstant(const Expr& e, long long width, bool sign) {
        Bits a = evaluateT(*e.operands[0], width, sign);
        Bits b = evaluateT(*e.operands[1], width, sign);
        auto negate = [&](Bits& v) {
            for (auto& bit : v) bit = notGate(bit);
            v = add(v, constantBits(0, width), constant(true));
        };
        bool neg_a = sign && isConst(a.back(), true), neg_b = sign && isConst(b.back(), true);
        if (neg_a) negate(a);
        if (neg_b) negate(b);
        // Subtract from high to low, using only the significant divisor bits.
        long long size = width;
        while (size && isConst(b[size - 1], false)) --size;
        if (!size) errorAt(e.line, "division by zero in constant expression");
        Bits quotient(width, constant(false));
        for (long long offset = width - size; offset >= 0; --offset) {
            long long j = size - 1;
            while (j >= 0 && a[offset + j] == b[j]) --j;
            bool carry_bit = offset + size < width && isConst(a[offset + size], true);
            if (!carry_bit && j >= 0 && isConst(a[offset + j], false)) continue;
            bool borrow = false;
            for (long long k = 0; k < size; ++k) {
                int diff = int(isConst(a[offset + k], true)) - int(isConst(b[k], true)) - borrow;
                a[offset + k] = constant(diff % 2 != 0);
                borrow = diff < 0;
            }
            if (borrow) a[offset + size] = constant(false);
            quotient[offset] = constant(true);
        }
        if (e.op == "%") {
            if (neg_a) negate(a);
            return a;
        }
        if (neg_a != neg_b) negate(quotient);
        return quotient;
    }

    Bits evaluateBinary(const Expr& e, long long width, bool sx) {
        const Expr& lhs = *e.operands[0];
        const Expr& rhs = *e.operands[1];
        const std::string& op = e.op;

        if (op == "&" || op == "|" || op == "^" || op == "~^") {
            Bits a = evaluateT(lhs, width, sx);
            Bits b = evaluateT(rhs, width, sx);
            for (long long i = 0; i < width; ++i) {
                a[i] = op == "&" ? andGate(a[i], b[i])
                     : op == "|" ? orGate(a[i], b[i])
                     : op == "^" ? xorGate(a[i], b[i])
                                 : xnorGate(a[i], b[i]);
            }
            return a;
        }
        if (op == "+") {
            // a + b + c ...: flatten the sum, and use a 1-bit term as the carry-in of the first
            // adder instead of adding it with a second carry chain (what Yosys's alumacc does)
            std::vector<const Expr*> terms;
            std::function<void(const Expr&)> collect = [&](const Expr& t) {
                if (t.kind == ExprKind::BINARY && t.op == "+") {
                    collect(*t.operands[0]);
                    collect(*t.operands[1]);
                } else {
                    terms.push_back(&t);
                }
            };
            collect(e);
            NodePtr carry = constant(false);
            for (size_t k = 0; k < terms.size(); ++k) {
                // a 1-bit term becomes the carry-in, unless it is signed (it would sign-extend)
                if (selfWidth(*terms[k]) == 1 && terms.size() > 1 && !sx) {
                    carry = evaluate(*terms[k], 1)[0];
                    terms.erase(terms.begin() + static_cast<long>(k));
                    break;
                }
            }
            Bits acc = evaluateT(*terms[0], width, sx);
            if (terms.size() == 1) return add(acc, constantBits(0, width), carry);
            for (size_t k = 1; k < terms.size(); ++k) {
                acc = add(acc, evaluateT(*terms[k], width, sx), carry);
                carry = constant(false);
            }
            return acc;
        }
        if (op == "-") {
            // a - b = a + ~b + 1
            Bits b = evaluateT(rhs, width, sx);
            for (auto& bit : b) bit = notGate(bit);
            return add(evaluateT(lhs, width, sx), b, constant(true));
        }
        if (op == "*") {
            return multiply(evaluateT(lhs, width, sx), evaluateT(rhs, width, sx), width);
        }
        if (op == "/" || op == "%") {
            if (!isConstant(e)) errorAt(e.line, "'" + op + "' is only supported between constants (no divider yet)");
            return divideConstant(e, width, sx);
        }
        if (op == "<<" || op == ">>" || op == "<<<" || op == ">>>") {
            Bits a = evaluateT(lhs, width, sx);
            bool left = op == "<<" || op == "<<<";
            NodePtr fill = op == ">>>" && sx ? a.back() : constant(false);   // >>> is arithmetic when signed
            if (isConstant(rhs)) {
                Bits distance = evaluateSelf(rhs);
                long long amount = 0;
                for (auto it = distance.rbegin(); it != distance.rend() && amount < width; ++it)
                    amount = 2 * amount + isConst(*it, true);
                Bits out(static_cast<size_t>(width), fill);
                for (long long i = 0; i < width; ++i) {
                    long long from = left ? i - amount : i + amount;
                    out[i] = from >= 0 && from < width ? a[from] : (left ? constant(false) : fill);
                }
                return out;
            }
            // the amount is self-determined and unsigned
            return barrelShift(a, evaluateT(rhs, selfWidth(rhs), false), left, left ? constant(false) : fill);
        }
        if (op == "&&" || op == "||") {
            NodePtr a = reduce(GateType::OR, evaluateSelf(lhs));
            NodePtr b = reduce(GateType::OR, evaluateSelf(rhs));
            return resize({op == "&&" ? andGate(a, b) : orGate(a, b)}, width);
        }

        // Comparisons: operands are sized to each other, not to the surrounding expression, and
        // compared as signed only if both are signed (then: flip the sign bits, compare unsigned)
        long long w = std::max(selfWidth(lhs), selfWidth(rhs));
        bool csx = isSigned(lhs) && isSigned(rhs);
        Bits a = evaluateT(lhs, w, csx);
        Bits b = evaluateT(rhs, w, csx);
        if (csx && op != "==" && op != "!=") {
            a.back() = notGate(a.back());
            b.back() = notGate(b.back());
        }
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
    Bits lvalueBits(const Expr& e) {
        if (e.kind != ExprKind::CONCAT && e.kind != ExprKind::NUMBER) {
            const Signal& sig = lookupSignal(e.name, e.line);
            if (sig.is_reg) errorAt(e.line, "'" + e.name + "' is a reg: assign it in an always block, not with assign");
            if (e.kind == ExprKind::BIT_SELECT && !isConstant(*e.operands[0])) {
                errorAt(e.line, "assign targets need constant indices");
            }
        }
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

    // ---------- Generate blocks: expanded into ordinary items before elaboration ----------

    struct Subst {
        std::map<std::string, ExprPtr> consts;          // genvars and generate-local parameters
        std::map<std::string, std::string> renames;     // names declared in a generate scope
    };

    ExprPtr numberExpr(long long v, size_t line) const {
        auto e = std::make_shared<Expr>();
        e->kind = ExprKind::NUMBER;
        e->line = line;
        e->is_signed = true;
        for (int i = 0; i < 32; ++i) e->bits.push_back((static_cast<unsigned long long>(v) >> i) & 1);
        return e;
    }

    ExprPtr subst(const ExprPtr& e, const Subst& sb) const {
        if (!e) return e;
        if (e->kind == ExprKind::IDENTIFIER) {
            auto c = sb.consts.find(e->name);
            if (c != sb.consts.end()) {
                auto n = std::make_shared<Expr>(*c->second);
                n->line = e->line;
                return n;
            }
        }
        auto n = std::make_shared<Expr>(*e);
        if (n->kind != ExprKind::CALL && n->kind != ExprKind::NUMBER) {
            auto r = sb.renames.find(n->name);
            if (r != sb.renames.end()) n->name = r->second;
        }
        for (auto& op : n->operands) op = subst(op, sb);
        n->word = subst(n->word, sb);
        return n;
    }

    StmtPtr subst(const StmtPtr& st, const Subst& sb) const {
        if (!st) return st;
        auto n = std::make_shared<Stmt>(*st);
        n->lhs = subst(st->lhs, sb);
        n->rhs = subst(st->rhs, sb);
        n->cond = subst(st->cond, sb);
        for (auto& x : n->stmts) x = subst(x, sb);
        n->then_stmt = subst(st->then_stmt, sb);
        n->else_stmt = subst(st->else_stmt, sb);
        for (auto& item : n->items) {
            for (auto& l : item.labels) l = subst(l, sb);
            item.body = subst(item.body, sb);
        }
        n->init = subst(st->init, sb);
        n->step = subst(st->step, sb);
        n->body = subst(st->body, sb);
        for (auto& a : n->args) a = subst(a, sb);
        return n;
    }

    long long genConst(const ExprPtr& e, const Subst& sb, const char* what) {
        return requireConst(*subst(e, sb), std::string(what) + " must be a constant (parameters and genvars only)");
    }

    void emitGenerateBody(const Module& body, Subst sb, const std::string& scope) {
        for (const auto& n : body.nets) {
            if (n.kind == NetKind::INPUT || n.kind == NetKind::OUTPUT) errorAt(n.line, "ports cannot be declared in a generate block");
            if (!scope.empty()) sb.renames[n.name] = scope + n.name;
        }
        for (const auto& p : body.params) {   // generate-local parameters are constants of this scope
            long long width = p.msb ? declWidth(subst(p.msb, sb), subst(p.lsb, sb), p.line) : 0;
            sb.consts[p.name] = constantLiteral(*subst(p.value, sb), width);
        }
        for (const auto& n : body.nets) {
            NetDecl d = n;
            d.name = scope.empty() ? n.name : scope + n.name;
            d.msb = subst(n.msb, sb);
            d.lsb = subst(n.lsb, sb);
            d.first = subst(n.first, sb);
            d.last = subst(n.last, sb);
            module.nets.push_back(d);
        }
        for (const auto& a : body.assigns) module.assigns.push_back({subst(a.lhs, sb), subst(a.rhs, sb), a.line});
        for (const auto& ab : body.always) {
            AlwaysBlock b = ab;
            for (auto& ev : b.events) {
                auto r = sb.renames.find(ev.signal);
                if (r != sb.renames.end()) ev.signal = r->second;
            }
            b.body = subst(ab.body, sb);
            module.always.push_back(b);
        }
        for (const auto& inst : body.instances) {
            InstanceDecl i = inst;
            i.name = scope + inst.name;
            for (auto& c : i.ports) c.expr = subst(c.expr, sb);
            for (auto& c : i.params) c.expr = subst(c.expr, sb);
            module.instances.push_back(i);
        }
        for (const auto& f : body.functions) module.functions.push_back(f);
        for (const auto& g : body.generates) expandGenerate(g, sb, scope);
    }

    void expandGenerate(const GenerateBlock& g, const Subst& sb, const std::string& scope) {
        auto scoped = [&](const std::string& label) {
            return label.empty() ? scope : scope + label + ".";
        };
        switch (g.kind) {
        case GenerateBlock::FOR: {
            Subst inner = sb;
            long long v = genConst(g.init, sb, "the generate loop start");
            std::string label = g.label.empty() ? "genblk" + std::to_string(++generate_count) : g.label;
            for (int iterations = 0;; ++iterations) {
                inner.consts[g.genvar] = numberExpr(v, g.line);
                if (!genConst(g.cond, inner, "the generate loop condition")) break;
                if (iterations > 65536) errorAt(g.line, "generate loop runs more than 65536 times");
                emitGenerateBody(*g.body, inner, scope + label + "[" + std::to_string(v) + "].");
                v = genConst(g.step, inner, "the generate loop step");
            }
            break;
        }
        case GenerateBlock::IF:
            if (genConst(g.cond, sb, "a generate if condition")) emitGenerateBody(*g.body, sb, scoped(g.label));
            else if (g.else_body) emitGenerateBody(*g.else_body, sb, scoped(g.else_body->name));
            break;
        case GenerateBlock::CASE: {
            ExprPtr selector = constantLiteral(*subst(g.cond, sb));
            const std::pair<std::vector<ExprPtr>, std::shared_ptr<Module>>* chosen = nullptr;
            // Generate expressions are elaborated constants. Compare each item's
            // value with the selector after evaluating both at their own widths.
            for (const auto& item : g.cases) {
                if (item.first.empty()) {
                    if (!chosen) chosen = &item;
                    continue;
                }
                for (const auto& expr : item.first) {
                    ExprPtr label = constantLiteral(*subst(expr, sb));
                    long long width = std::max(selfWidth(*selector), selfWidth(*label));
                    bool sign = isSigned(*selector) && isSigned(*label);
                    if ((!chosen || chosen->first.empty()) &&
                        evaluateT(*selector, width, sign) == evaluateT(*label, width, sign)) chosen = &item;
                }
            }
            if (chosen) emitGenerateBody(*chosen->second, sb, scoped(chosen->second->name));
            break;
        }
        case GenerateBlock::BLOCK:
            emitGenerateBody(*g.body, sb, scoped(g.label));
            break;
        }
    }

    void expandGenerates() {
        std::vector<GenerateBlock> gens = std::move(module.generates);
        module.generates.clear();
        for (const auto& g : gens) expandGenerate(g, Subst{}, "");
        for (const auto& f : module.functions) {
            if (functions.count(f.name)) errorAt(f.line, "'" + f.name + "' is defined twice");
        }
        functions.clear();
        for (const auto& f : module.functions) functions[f.name] = &f;
    }

    void declareParams() {
        for (const auto& p : module.params) {
            if (params.count(p.name)) errorAt(p.line, "parameter '" + p.name + "' is already declared");
            auto it = param_values.find(p.name);
            long long width = p.msb ? declWidth(p.msb, p.lsb, p.line) : 0;
            params[p.name] = it != param_values.end() && !p.is_local
                                 ? it->second(width) : constantLiteral(*p.value, width);
        }
    }

    void declareNets() {
        for (const auto& decl : module.nets) {
            if (params.count(decl.name)) errorAt(decl.line, "'" + decl.name + "' is already declared as a parameter");
            if (decl.kind == NetKind::INTEGER) {
                if (signals.count(decl.name) || !integers.insert(decl.name).second) errorAt(decl.line, "'" + decl.name + "' is already declared");
                continue;
            }

            Signal sig;
            sig.kind = decl.kind;
            sig.is_reg = decl.kind == NetKind::REG || decl.is_reg;
            sig.is_signed = decl.is_signed;
            sig.is_vector = decl.msb != nullptr;
            if (sig.is_vector) {
                sig.msb = requireConst(*decl.msb, "range of '" + decl.name + "' must be constant");
                sig.lsb = requireConst(*decl.lsb, "range of '" + decl.name + "' must be constant");
            }
            long long width = checkedWidth(sig.wordWidth(), decl.line);
            long long words = 1;
            if (decl.first) {
                sig.is_memory = true;
                sig.first = requireConst(*decl.first, "memory range of '" + decl.name + "' must be constant");
                sig.last = requireConst(*decl.last, "memory range of '" + decl.name + "' must be constant");
                words = (sig.first <= sig.last ? sig.last - sig.first : sig.first - sig.last) + 1;
                checkedWidth(words * width, decl.line);
            }

            auto it = signals.find(decl.name);
            if (it != signals.end()) {
                // "output [3:0] y; wire [3:0] y;" (or "reg [3:0] y;") only restates the port's net type
                Signal& existing = it->second;
                bool is_port = existing.kind != NetKind::WIRE && existing.kind != NetKind::REG;
                if ((decl.kind == NetKind::WIRE || decl.kind == NetKind::REG) && is_port && !decl.first) {
                    if (sig.is_vector && width != static_cast<long long>(existing.bits.size())) {
                        errorAt(decl.line, "'" + decl.name + "' is redeclared with a different width");
                    }
                    if (decl.kind == NetKind::REG) {
                        if (existing.kind == NetKind::INPUT) errorAt(decl.line, "input '" + decl.name + "' cannot be a reg");
                        existing.is_reg = true;
                    }
                    existing.is_signed |= decl.is_signed;
                    continue;
                }
                errorAt(decl.line, "'" + decl.name + "' is already declared");
            }

            // Only the top module's ports are real ports; an instance's ports are nets of its parent
            GateType type = !is_top                      ? GateType::ASSIGN
                          : decl.kind == NetKind::INPUT  ? GateType::INPUT
                          : decl.kind == NetKind::OUTPUT ? GateType::OUTPUT
                                                         : GateType::ASSIGN;
            for (long long w = 0; w < words; ++w) {
                long long word_index = sig.first <= sig.last ? sig.first + w : sig.first - w;
                std::string word_name = prefix + decl.name + (sig.is_memory ? "[" + std::to_string(word_index) + "]" : "");
                for (long long p = 0; p < width; ++p) {
                    long long index = sig.msb >= sig.lsb ? sig.lsb + p : sig.lsb - p;
                    std::string bit_name = sig.is_vector ? word_name + "[" + std::to_string(index) + "]" : word_name;
                    sig.bits.push_back(std::make_shared<GateNode>(sh.next_id++, type, bit_name));
                    if (decl.kind == NetKind::INPUT) input_bits.insert(sig.bits.back().get());
                    if (decl.kind == NetKind::INPUT && is_top) sh.top_inputs.insert(sig.bits.back().get());
                }
            }
            signals[decl.name] = std::move(sig);
            signal_order.push_back(decl.name);
        }
    }

    // ---------- Always blocks: symbolic execution ----------

    // What an assigned reg holds before any assignment in this block: its current value in a
    // clocked block (registers hold), nothing in a combinational one (a latch if it stays so)
    Bits initialEntry(const std::string& name, size_t line) {
        const Signal& sig = lookupSignal(name, line);
        if (block_kind == BlockKind::SEQ) return sig.bits;
        return Bits(sig.bits.size(), nullptr);
    }

    struct MuxInfo {
        NodePtr result;  // retain the node used as the key; allocator reuse must not alias old muxes
        NodePtr c, t, f;
    };
    std::unordered_map<const GateNode*, NodePtr> not_memo;
    std::map<std::pair<const GateNode*, const GateNode*>, NodePtr> and_memo;
    std::unordered_map<const GateNode*, MuxInfo> mux_memo;

    NodePtr muxBit(const NodePtr& c, const NodePtr& t, const NodePtr& f) {
        if (t == f) return t;
        if (!t || !f) return nullptr;   // assigned on only one path
        if (isConst(c, true)) return t;
        if (isConst(c, false)) return f;
        // Collapse nested muxes that share a fallback branch (e.g. nested if-enable chains on registers/memories):
        //   c ? (d ? x : f) : f  ==>  (c & d) ? x : f
        //   c ? (d ? f : x) : f  ==>  (c & ~d) ? x : f
        //   c ? t : (d ? x : t)  ==>  (~c & d) ? x : t
        //   c ? t : (d ? t : x)  ==>  (~c & ~d) ? x : t
        auto it_t = mux_memo.find(t.get());
        if (it_t != mux_memo.end()) {
            const MuxInfo mt = it_t->second;
            if (mt.f == f) return muxBit(andGate(c, mt.c), mt.t, f);
            if (mt.t == f) return muxBit(andGate(c, notGate(mt.c)), mt.f, f);
        }
        auto it_f = mux_memo.find(f.get());
        if (it_f != mux_memo.end()) {
            const MuxInfo mf = it_f->second;
            if (mf.f == t) return muxBit(andGate(notGate(c), mf.c), mf.t, t);
            if (mf.t == t) return muxBit(andGate(notGate(c), notGate(mf.c)), mf.f, t);
        }
        NodePtr when_true = andGate(c, t), when_false = andGate(notGate(c), f);
        NodePtr n = orGate(when_true, when_false);
        // Only a newly constructed OR is a mux root. Folding a ? a : 0 to a must
        // not attach a self-referential mux description to that existing input.
        if (n != when_true && n != when_false && n->type == GateType::OR) {
            mux_memo.emplace(n.get(), MuxInfo{n, c, t, f});
        }
        return n;
    }

    // env := c ? t : f, where t and f started as copies of env
    void merge(Env& env, const NodePtr& c, Env& t, Env& f, size_t line) {
        for (auto member : {&Env::blocking, &Env::nonblocking}) {
            std::set<std::string> keys;
            for (const auto& [k, v] : t.*member) keys.insert(k);
            for (const auto& [k, v] : f.*member) keys.insert(k);
            for (const auto& k : keys) {
                auto value = [&](Env& side) -> Bits {
                    auto it = (side.*member).find(k);
                    if (it != (side.*member).end()) return it->second;
                    auto base = (env.*member).find(k);
                    return base != (env.*member).end() ? base->second : initialEntry(k, line);
                };
                Bits bt = value(t), bf = value(f), out(bt.size());
                for (size_t i = 0; i < bt.size(); ++i) out[i] = muxBit(c, bt[i], bf[i]);
                (env.*member)[k] = std::move(out);
            }
        }
        for (Env* side : {&t, &f}) {
            for (const auto& [k, positions] : side->touched) env.touched[k].insert(positions.begin(), positions.end());
        }
    }

    struct Target {
        std::string name;
        std::vector<long long> positions;   // static target bits
        // variable index: candidate element starts and the index value
        std::vector<Candidate> cands;
        long long elem_width = 0, inner_offset = 0;
        Bits index;
        long long width = 0;
    };

    void targets(const Expr& e, std::vector<Target>& out) {
        if (e.kind == ExprKind::CONCAT) {
            for (const auto& part : e.operands) targets(*part, out);   // most significant first
            return;
        }
        if (e.kind == ExprKind::IDENTIFIER && integers.count(e.name)) {
            errorAt(e.line, "integer '" + e.name + "' can only be assigned in a for-loop header");
        }
        if (e.kind != ExprKind::IDENTIFIER && e.kind != ExprKind::BIT_SELECT && e.kind != ExprKind::PART_SELECT &&
            e.kind != ExprKind::INDEXED_PART_SELECT) {
            errorAt(e.line, "invalid assignment target");
        }
        const Signal& sig = lookupSignal(e.name, e.line);
        if (!sig.is_reg) errorAt(e.line, "'" + e.name + "' is a wire: only regs can be assigned in always blocks");
        Target t;
        t.name = e.name;
        long long ww = sig.wordWidth();

        if (e.kind == ExprKind::IDENTIFIER) {
            if (sig.is_memory) errorAt(e.line, "assign memory words one at a time: " + e.name + "[i] <= ...");
            for (long long p = 0; p < static_cast<long long>(sig.bits.size()); ++p) t.positions.push_back(p);
            t.width = static_cast<long long>(sig.bits.size());
            out.push_back(t);
            return;
        }

        long long word_base = 0;   // first bit of the addressed word
        bool dynamic_word = false;
        if (sig.is_memory) {
            const Expr& index = e.word ? *e.word : *e.operands[0];
            if (!e.word && e.kind != ExprKind::BIT_SELECT) errorAt(e.line, "select a word of memory '" + e.name + "' first");
            if (isConstant(index)) {
                long long p = sig.wordPosition(evalConst(index));
                if (p < 0) errorAt(e.line, "memory index is out of range for '" + e.name + "'");
                word_base = p * ww;
            } else {
                dynamic_word = true;
                t.cands = candidates(sig, true);
                t.index = evaluateSelf(index);
            }
            if (!e.word) {   // the whole word
                t.width = ww;
                if (dynamic_word) {
                    t.elem_width = ww;
                } else {
                    for (long long b = 0; b < ww; ++b) t.positions.push_back(word_base + b);
                }
                out.push_back(t);
                return;
            }
        }

        if (e.kind == ExprKind::BIT_SELECT && !isConstant(*e.operands[0])) {
            if (dynamic_word) errorAt(e.line, "a variable word index and a variable bit index together are not supported");
            t.cands = candidates(sig, false);
            for (auto& c : t.cands) c.position += word_base;
            t.index = evaluateSelf(*e.operands[0]);
            t.elem_width = 1;
            t.width = 1;
            out.push_back(t);
            return;
        }
        auto [lo, hi] = staticRange(e, sig);
        t.width = hi - lo + 1;
        if (dynamic_word) {
            t.elem_width = t.width;
            t.inner_offset = lo;
        } else {
            for (long long p = lo; p <= hi; ++p) t.positions.push_back(word_base + p);
        }
        out.push_back(t);
    }

    // ---------- Functions and tasks: inlined by symbolic execution of their bodies ----------

    const FunctionDecl& findFunction(const std::string& name, size_t line, bool task) const {
        auto it = functions.find(name);
        if (it == functions.end()) errorAt(line, std::string(task ? "unknown task '" : "unknown function '") + name + "'");
        if (it->second->is_task != task) errorAt(line, "'" + name + "' is a " + (task ? "function" : "task"));
        return *it->second;
    }

    long long declWidth(const ExprPtr& msb, const ExprPtr& lsb, size_t line) {
        if (!msb) return 1;
        return checkedWidth(std::abs(requireConst(*msb, "range must be constant") - requireConst(*lsb, "range must be constant")) + 1, line);
    }

    long long functionWidth(const Expr& call) {
        const FunctionDecl& f = findFunction(call.name, call.line, false);
        return declWidth(f.msb, f.lsb, f.line);
    }

    bool functionSigned(const Expr& call) const { return findFunction(call.name, call.line, false).is_signed; }

    // Runs a function or task body with its arguments bound; returns the values of `results`
    std::vector<Bits> runSubroutine(const FunctionDecl& f, const std::vector<ExprPtr>& args, size_t line,
                                    const std::vector<std::string>& results) {
        if (++call_depth > 64) errorAt(line, "'" + f.name + "' calls itself too deeply (recursion is not supported)");
        std::vector<const NetDecl*> inputs;
        for (const auto& p : f.ports) {
            if (p.kind == NetKind::INPUT) inputs.push_back(&p);
        }
        size_t expected = f.is_task ? f.ports.size() : inputs.size();
        if (args.size() != expected) {
            errorAt(line, "'" + f.name + "' takes " + std::to_string(expected) + " arguments, got " + std::to_string(args.size()));
        }
        // Arguments are evaluated in the caller's scope, converted like an assignment
        std::unordered_map<std::string, Bits> bound;
        size_t k = 0;
        for (const auto& p : f.ports) {
            const Expr& arg = *args[f.is_task ? &p - &f.ports[0] : k];
            if (p.kind != NetKind::INPUT) continue;
            long long w = declWidth(p.msb, p.lsb, p.line);
            Bits v = evaluate(arg, std::max(w, selfWidth(arg)));
            v.resize(static_cast<size_t>(w));
            bound[p.name] = v;
            k++;
        }

        // Locals shadow module signals for the duration of the call
        struct Binding { std::string name; std::optional<Signal> signal; ExprPtr param; bool integer; };
        std::vector<Binding> saved;
        auto declare = [&](const NetDecl& d, const Bits* value) {
            auto it = signals.find(d.name);
            auto p = params.find(d.name);
            saved.push_back({d.name, it == signals.end() ? std::nullopt : std::optional<Signal>(it->second),
                             p == params.end() ? nullptr : p->second, integers.erase(d.name) != 0});
            if (p != params.end()) params.erase(p);
            if (d.kind == NetKind::INTEGER) {
                integers.insert(d.name);
                signals.erase(d.name);
                return;
            }
            Signal sig;
            sig.kind = NetKind::REG;
            sig.is_reg = true;
            sig.is_signed = d.is_signed;
            sig.is_vector = d.msb != nullptr;
            if (d.msb) {
                sig.msb = requireConst(*d.msb, "range must be constant");
                sig.lsb = requireConst(*d.lsb, "range must be constant");
            }
            long long w = sig.wordWidth();
            if (value) {
                sig.bits = *value;
            } else {
                for (long long i = 0; i < w; ++i) sig.bits.push_back(constant(false));   // unassigned locals read 0
            }
            signals[d.name] = std::move(sig);
        };
        for (const auto& p : f.ports) declare(p, bound.count(p.name) ? &bound[p.name] : nullptr);
        if (!f.is_task) {
            NetDecl ret;
            ret.kind = NetKind::REG;
            ret.name = f.name;
            ret.msb = f.msb;
            ret.lsb = f.lsb;
            ret.is_signed = f.is_signed;
            ret.line = f.line;
            declare(ret, nullptr);
        }
        for (const auto& l : f.locals) declare(l, nullptr);

        Env env;
        if (overrides) env.blocking = *overrides;   // the caller's current values of module regs
        for (const auto& binding : saved) env.blocking.erase(binding.name);
        BlockKind saved_kind = block_kind;
        const std::map<std::string, Bits>* saved_overrides = overrides;
        NodePtr saved_path = path_cond;
        block_kind = BlockKind::COMB;
        overrides = &env.blocking;
        path_cond = constant(true);   // a function's statements run whenever it is called
        exec(*f.body, env);
        block_kind = saved_kind;
        overrides = saved_overrides;
        path_cond = saved_path;

        std::vector<Bits> out;
        for (const auto& name : results) {
            Bits v = currentBitsIn(env, name, line);
            out.push_back(v);
        }
        for (auto it = saved.rbegin(); it != saved.rend(); ++it) {
            signals.erase(it->name);
            integers.erase(it->name);
            if (it->signal) signals[it->name] = *it->signal;
            if (it->integer) integers.insert(it->name);
            if (it->param) params[it->name] = it->param;
        }
        call_depth--;
        return out;
    }

    Bits currentBitsIn(Env& env, const std::string& name, size_t line) const {
        const Bits& declared = lookupSignal(name, line).bits;
        auto it = env.blocking.find(name);
        if (it == env.blocking.end()) return declared;
        Bits v = it->second;
        for (size_t i = 0; i < v.size(); ++i) {
            if (!v[i]) errorAt(line, "'" + name + "' is not assigned on every path through the function or task");
        }
        return v;
    }

    Bits callFunction(const Expr& call) {
        const FunctionDecl& f = findFunction(call.name, call.line, false);
        return runSubroutine(f, call.operands, call.line, {f.name})[0];
    }

    void callTask(const Stmt& s, Env& env) {
        const FunctionDecl& t = findFunction(s.name, s.line, true);
        std::vector<std::string> outs;
        std::vector<const Expr*> targets_of;
        for (size_t k = 0; k < t.ports.size(); ++k) {
            if (t.ports[k].kind == NetKind::OUTPUT) {
                outs.push_back(t.ports[k].name);
                if (k < s.args.size()) targets_of.push_back(s.args[k].get());
            }
        }
        std::vector<Bits> values = runSubroutine(t, s.args, s.line, outs);
        // task outputs are copied to the argument variables like blocking assignments
        for (size_t k = 0; k < values.size() && k < targets_of.size(); ++k) {
            std::vector<Target> ts;
            targets(*targets_of[k], ts);
            bool nonblocking = false;
            auto& map = nonblocking ? env.nonblocking : env.blocking;
            auto& other = nonblocking ? env.blocking : env.nonblocking;
            assignTargets(ts, values[k], map, other, env, s.line);
        }
    }

    void execAssign(const Stmt& s, Env& env) {
        auto access = sram_access.find(&s);
        if (access != sram_access.end()) {   // a memory turned into an SRAM: record, don't execute
            Shared::PendingSram& p = sh.pending_srams[access->second.first];
            if (access->second.second) {
                p.wcond = path_cond;
                p.waddr = evaluateSelf(*s.lhs->operands[0]);
                Bits v = evaluate(*s.rhs, std::max(p.width, selfWidth(*s.rhs)));
                v.resize(static_cast<size_t>(p.width));
                p.wdata = v;
            } else {
                p.rcond = path_cond;
                p.raddr = evaluateSelf(*s.rhs->operands[0]);
            }
            return;
        }
        bool nonblocking = s.kind == StmtKind::NONBLOCKING && block_kind == BlockKind::SEQ;
        auto& map = nonblocking ? env.nonblocking : env.blocking;
        auto& other = nonblocking ? env.blocking : env.nonblocking;

        std::vector<Target> ts;
        targets(*s.lhs, ts);
        long long total = 0;
        for (const auto& t : ts) total += t.width;
        long long width = std::max(total, selfWidth(*s.rhs));
        Bits value = evaluate(*s.rhs, width);
        assignTargets(ts, value, map, other, env, s.line);
    }

    void assignTargets(const std::vector<Target>& ts, Bits value, std::map<std::string, Bits>& map,
                       std::map<std::string, Bits>& other, Env& env, size_t line) {
        long long total = 0;
        for (const auto& t : ts) total += t.width;
        value.resize(static_cast<size_t>(std::max<long long>(total, static_cast<long long>(value.size()))), constant(false));
        Stmt dummy;
        dummy.line = line;
        const Stmt& s = dummy;

        long long next_bit = 0;   // the last target takes the least significant bits
        for (auto it = ts.rbegin(); it != ts.rend(); ++it) {
            const Target& t = *it;
            if (other.count(t.name)) errorAt(s.line, "'" + t.name + "' is assigned with both = and <= in one block");
            auto entry = map.find(t.name);
            if (entry == map.end()) entry = map.emplace(t.name, initialEntry(t.name, s.line)).first;
            Bits& bits = entry->second;
            auto& touched = env.touched[t.name];
            if (t.elem_width == 0) {
                for (long long k = 0; k < static_cast<long long>(t.positions.size()); ++k) {
                    bits[t.positions[k]] = value[next_bit + k];
                    touched.insert(t.positions[k]);
                }
            } else {
                for (const Candidate& c : t.cands) {
                    NodePtr hit = equalsConstant(t.index, c.value);
                    if (isConst(hit, false)) continue;
                    for (long long b = 0; b < t.elem_width; ++b) {
                        long long p = c.position + t.inner_offset + b;
                        bits[p] = muxBit(hit, value[next_bit + b], bits[p]);
                        touched.insert(p);
                    }
                }
            }
            next_bit += t.width;
        }
    }

    NodePtr condition(const Expr& e) { return reduce(GateType::OR, evaluateSelf(e)); }

    void exec(const Stmt& s, Env& env) {
        switch (s.kind) {
        case StmtKind::EMPTY:
            return;
        case StmtKind::BLOCK:
            for (const auto& st : s.stmts) exec(*st, env);
            return;
        case StmtKind::BLOCKING:
        case StmtKind::NONBLOCKING:
            execAssign(s, env);
            return;
        case StmtKind::IF: {
            NodePtr c = condition(*s.cond);
            if (isConst(c, true)) { exec(*s.then_stmt, env); return; }
            if (isConst(c, false)) { if (s.else_stmt) exec(*s.else_stmt, env); return; }
            Env t = env, f = env;
            NodePtr outer = path_cond;
            path_cond = andGate(outer, c);
            exec(*s.then_stmt, t);
            path_cond = andGate(outer, notGate(c));
            if (s.else_stmt) exec(*s.else_stmt, f);
            path_cond = outer;
            merge(env, c, t, f, s.line);
            return;
        }
        case StmtKind::CASE:
            execCase(s, env);
            return;
        case StmtKind::FOR:
            execFor(s, env);
            return;
        case StmtKind::TASK_CALL:
            callTask(s, env);
            return;
        }
    }

    void execCase(const Stmt& s, Env& env) {
        long long ws = selfWidth(*s.cond);
        // Items in matching order; default (wherever it is written) matches last
        std::vector<const CaseItem*> items;
        const CaseItem* fallback = nullptr;
        for (const auto& item : s.items) {
            if (item.labels.empty()) {
                if (fallback) errorAt(item.line, "case has more than one default");
                fallback = &item;
            } else {
                items.push_back(&item);
            }
        }
        // IEEE 1364 §9.5 sizes the selector and all labels as one comparison.
        long long width = ws;
        bool sign = isSigned(*s.cond);
        for (const auto* item : items) for (const auto& label : item->labels) {
            width = std::max(width, selfWidth(*label));
            sign &= isSigned(*label);
        }
        Bits selector = evaluateT(*s.cond, width, sign);
        std::vector<bool> covered(!fallback && ws <= 12 ? 1U << ws : 0, false);
        std::vector<NodePtr> conditions;
        for (const auto* item : items) {
            std::vector<NodePtr> hits;
            for (const auto& label : item->labels) {
                Bits pattern;
                if (label->wild.empty()) {
                    pattern = evaluateT(*label, width, sign);
                } else {
                    if (!s.case_wild)
                        errorAt(label->line, "x/z/? in a case label never matches in synthesis: use casez / casex");
                    if (s.case_wild == 1 && std::find(label->wild.begin(), label->wild.end(),
                                                      LiteralDigit::X) != label->wild.end()) continue;
                    for (size_t j = 0; j < label->bits.size(); ++j)
                        pattern.push_back(label->wild[j] == LiteralDigit::Known ? constant(label->bits[j]) : nullptr);
                    pattern = extend(std::move(pattern), width, sign);
                }
                Bits equal;
                unsigned mask = 0, value = 0;
                bool enumerable = !covered.empty();
                for (long long j = 0; j < width; ++j) {
                    if (!pattern[j]) continue;
                    equal.push_back(xnorGate(selector[j], pattern[j]));
                    if (!enumerable) continue;
                    bool one = isConst(pattern[j], true);
                    if (!one && !isConst(pattern[j], false)) { enumerable = false; continue; }
                    if (isConst(selector[j], true) || isConst(selector[j], false)) {
                        enumerable = selector[j] == pattern[j];
                        continue;
                    }
                    // Higher bits must be known or repeat the selector sign.
                    // Arithmetic selectors may acquire extra bits in this context.
                    if (j >= ws && selector[j] != selector[ws - 1]) { enumerable = false; continue; }
                    unsigned bit = 1U << std::min(j, ws - 1);
                    if ((mask & bit) && bool(value & bit) != one) enumerable = false;
                    mask |= bit;
                    if (one) value |= bit;
                }
                hits.push_back(equal.empty() ? constant(true) : reduce(GateType::AND, equal));
                if (enumerable) {
                    if (mask == covered.size() - 1) covered[value] = true;
                    else for (unsigned v = 0; v < covered.size(); ++v)
                        if ((v & mask) == value) covered[v] = true;
                }
            }
            conditions.push_back(hits.empty() ? constant(false) : reduce(GateType::OR, hits));
        }
        if (!covered.empty() && std::all_of(covered.begin(), covered.end(), [](bool v) { return v; })) {
            fallback = items.back();
            items.pop_back();
        }
        std::function<void(size_t, Env&)> from = [&](size_t k, Env& e) {
            if (k == items.size()) {
                if (fallback) exec(*fallback->body, e);
                return;
            }
            NodePtr c = conditions[k];
            if (isConst(c, true)) { exec(*items[k]->body, e); return; }
            if (isConst(c, false)) { from(k + 1, e); return; }
            Env t = e, f = e;
            NodePtr outer = path_cond;
            path_cond = andGate(outer, c);
            exec(*items[k]->body, t);
            path_cond = andGate(outer, notGate(c));
            from(k + 1, f);
            path_cond = outer;
            merge(e, c, t, f, items[k]->line);
        };
        from(0, env);
    }

    void execFor(const Stmt& s, Env& env) {
        const Expr& var = *s.init->lhs;
        if (var.kind != ExprKind::IDENTIFIER || !integers.count(var.name)) {
            errorAt(s.line, "for-loop variables must be declared as integer");
        }
        if (s.step->lhs->kind != ExprKind::IDENTIFIER || s.step->lhs->name != var.name) {
            errorAt(s.line, "the for-loop step must assign the loop variable '" + var.name + "'");
        }
        if (params.count(var.name)) errorAt(s.line, "nested loops over the same variable '" + var.name + "'");
        params[var.name] = numberExpr(requireConst(*s.init->rhs, "for-loop start value must be a constant"), s.line);
        long long iterations = 0;
        while (requireConst(*s.cond, "for-loop condition must be constant (it can use the loop variable)")) {
            if (++iterations > 65536) errorAt(s.line, "for loop runs more than 65536 times");
            exec(*s.body, env);
            params[var.name] = numberExpr(requireConst(*s.step->rhs, "for-loop step must be a constant expression"), s.line);
        }
        params.erase(var.name);
    }

    // Async reset: "if (!rst_n)" / "if (rst)" at the top of the block, matching the edge
    bool resetTest(const Expr& c, const EdgeEvent& ev, bool& active_low) {
        const Expr* e = &c;
        bool negated = false;
        if (e->kind == ExprKind::UNARY && (e->op == "!" || e->op == "~")) {
            negated = true;
            e = e->operands[0].get();
        }
        if (e->kind == ExprKind::BINARY && (e->op == "==" || e->op == "!=") && e->operands[0]->kind == ExprKind::IDENTIFIER &&
            isConstant(*e->operands[1])) {
            bool zero = evalConst(*e->operands[1]) == 0;
            negated = negated != (zero == (e->op == "=="));
            e = e->operands[0].get();
        }
        if (e->kind != ExprKind::IDENTIFIER || e->name != ev.signal) return false;
        active_low = negated;
        return true;
    }

    NodePtr portBit(const std::string& name, size_t line, const char* what) const {
        const Signal& sig = lookupSignal(name, line);
        if (sig.kind != NetKind::INPUT || sig.bits.size() != 1) errorAt(line, std::string(what) + " '" + name + "' must be a 1-bit input port");
        return sig.bits[0];
    }

    void elaborateAlways(const AlwaysBlock& ab) {
        path_cond = constant(true);
        bool edges = false, levels = false;
        for (const auto& ev : ab.events) (ev.posedge || ev.negedge ? edges : levels) = true;
        if (edges && levels) errorAt(ab.line, "mixing edge (posedge/negedge) and level events in one sensitivity list");

        if (!edges) {   // combinational: always @(*) or @(a or b) (an incomplete list is treated as @*)
            block_kind = BlockKind::COMB;
            Env env;
            overrides = &env.blocking;
            exec(*ab.body, env);
            overrides = nullptr;
            for (const auto& [name, bits] : env.blocking) {
                Signal& sig = signals.at(name);
                for (long long p : env.touched[name]) {
                    if (!bits[p]) {
                        errorAt(ab.line, "latch inferred: '" + sig.bits[p]->name +
                                         "' is not assigned on every path through this always block");
                    }
                    NodePtr target = sig.bits[p];
                    if (!target->inputs.empty() || target->type == GateType::INPUT) {
                        errorAt(ab.line, "'" + target->name + "' is driven by more than one always block");
                    }
                    target->addFanin(bits[p]);
                }
            }
            return;
        }

        // Clocked: one clock, optionally one asynchronous reset tested at the top
        const Stmt* body = ab.body.get();
        const Stmt* reset_body = nullptr;
        RegisterBit proto;
        NodePtr clock_node, reset_node;
        if (ab.events.size() == 1) {
            clock_node = portBit(ab.events[0].signal, ab.line, "clock");
            proto.negedge = ab.events[0].negedge;
        } else if (ab.events.size() == 2) {
            const Stmt* top = body;
            while (top->kind == StmtKind::BLOCK && top->stmts.size() == 1) top = top->stmts[0].get();
            int reset_index = -1;
            bool active_low = false;
            if (top->kind == StmtKind::IF) {
                for (int k = 0; k < 2; ++k) {
                    if (resetTest(*top->cond, ab.events[k], active_low)) reset_index = k;
                }
            }
            if (reset_index < 0) {
                errorAt(ab.line, "with two edges in the sensitivity list the block must start with if (reset) (asynchronous reset)");
            }
            const EdgeEvent& rst = ab.events[reset_index];
            const EdgeEvent& clk = ab.events[1 - reset_index];
            if (active_low != rst.negedge) {
                errorAt(ab.line, rst.negedge ? "negedge " + rst.signal + " is an active-low reset: test it as !" + rst.signal
                                             : "posedge " + rst.signal + " is an active-high reset: test it as " + rst.signal);
            }
            clock_node = portBit(clk.signal, ab.line, "clock");
            proto.negedge = clk.negedge;
            reset_node = portBit(rst.signal, ab.line, "reset");
            proto.reset_active_low = active_low;
            reset_body = top->then_stmt.get();
            body = top->else_stmt.get();
        } else {
            errorAt(ab.line, "only one clock and one asynchronous reset per always block are supported");
        }

        block_kind = BlockKind::SEQ;
        Env env;
        overrides = &env.blocking;
        if (body) exec(*body, env);
        Env reset_env;
        if (reset_body) {
            overrides = &reset_env.blocking;
            exec(*reset_body, reset_env);
        }
        overrides = nullptr;

        std::set<std::string> names;
        for (const auto& [n, v] : env.nonblocking) names.insert(n);
        for (const auto& [n, v] : env.blocking) names.insert(n);
        for (const auto& [n, v] : reset_env.touched) names.insert(n);
        for (const auto& name : names) {
            Signal& sig = signals.at(name);
            const Bits* next = env.nonblocking.count(name) ? &env.nonblocking.at(name)
                             : env.blocking.count(name)    ? &env.blocking.at(name) : nullptr;
            std::set<long long> positions = env.touched[name];
            positions.insert(reset_env.touched[name].begin(), reset_env.touched[name].end());
            for (long long p : positions) {
                RegisterBit reg = proto;
                reg.name = sig.bits[p]->name;
                if (reset_body) {
                    const auto& rt = reset_env.touched[name];
                    if (!rt.count(p)) errorAt(ab.line, "register '" + reg.name + "' is not given a value in the reset branch");
                    const Bits& rv = reset_env.nonblocking.count(name) ? reset_env.nonblocking.at(name) : reset_env.blocking.at(name);
                    if (!isConst(rv[p], false) && !isConst(rv[p], true)) {
                        errorAt(ab.line, "the asynchronous reset value of '" + reg.name + "' must be a constant");
                    }
                    reg.reset_value = isConst(rv[p], true);
                }
                NodePtr target = sig.bits[p];
                NodePtr q;
                if (sig.kind == NetKind::OUTPUT) {
                    if (!target->inputs.empty()) errorAt(ab.line, "'" + reg.name + "' is driven by more than one always block");
                    q = std::make_shared<GateNode>(sh.next_id++, GateType::INPUT, reg.name);
                    target->addFanin(q);
                } else {
                    if (!target->inputs.empty() || target->type == GateType::INPUT) {
                        errorAt(ab.line, "'" + reg.name + "' is driven by more than one always block");
                    }
                    target->type = GateType::INPUT;   // the placeholder becomes the register output
                    q = target;
                }
                auto d = std::make_shared<GateNode>(sh.next_id++, GateType::OUTPUT, nextStateName(reg.name));
                d->addFanin(next ? (*next)[p] : target);
                sh.registers.push_back({reg, clock_node, reset_node, d, "line " + std::to_string(ab.line) + context()});
            }
        }
    }

    void elaborateAlwaysBlocks() {
        for (const auto& ab : module.always) elaborateAlways(ab);
    }

    void connectAssigns() {
        for (const auto& assign : module.assigns) {
            Bits targets = lvalueBits(*assign.lhs);
            long long width = std::max(static_cast<long long>(targets.size()), selfWidth(*assign.rhs));
            Bits values = evaluate(*assign.rhs, width);
            for (size_t i = 0; i < targets.size(); ++i) {
                const auto& target = targets[i];
                if (target->type == GateType::INPUT || input_bits.count(target.get())) {
                    errorAt(assign.line, "cannot assign to input '" + target->name + "'");
                }
                if (!target->inputs.empty()) errorAt(assign.line, "'" + target->name + "' has more than one driver");
                target->addFanin(values[i]);
            }
        }
    }

    void checkDrivers() const {
        for (const auto& port : module.header_ports) {
            auto it = signals.find(port);
            if (it == signals.end() || (it->second.kind != NetKind::INPUT && it->second.kind != NetKind::OUTPUT)) {
                throw std::runtime_error("port '" + port + "' has no input/output declaration");
            }
        }
        for (const auto& name : signal_order) {
            const Signal& sig = signals.at(name);
            for (const auto& bit : sig.bits) {
                if (!bit->inputs.empty() || bit->type == GateType::INPUT) continue;   // driven, or a register output
                if (sig.kind == NetKind::OUTPUT) {
                    throw std::runtime_error("output '" + bit->name + "' is never assigned");
                }
                if ((sig.kind == NetKind::WIRE || sig.kind == NetKind::REG) && !bit->outputs.empty()) {
                    throw std::runtime_error(std::string(sig.kind == NetKind::REG ? "reg '" : "wire '") + bit->name +
                                             "' is used but never assigned");
                }
            }
        }
    }

    PortInfo portInfo(const std::string& name) const {
        const Signal& sig = signals.at(name);
        return {name, sig.kind == NetKind::INPUT, sig.is_vector, static_cast<int>(sig.msb), static_cast<int>(sig.lsb)};
    }

    std::vector<std::string> portNames() const {
        if (!module.header_ports.empty()) return module.header_ports;
        std::vector<std::string> names;
        for (const auto& name : signal_order) {
            NetKind k = signals.at(name).kind;
            if (k == NetKind::INPUT || k == NetKind::OUTPUT) names.push_back(name);
        }
        return names;
    }

    // Flattening: each instance is elaborated into the same netlist under "instance." names;
    // its ports become nets driven by (inputs) or driving (outputs) the parent's expressions
    void elaborateInstances() {
        for (const auto& inst : module.instances) {
            auto it = sh.modules.find(inst.module);
            if (it == sh.modules.end()) {
                auto bb = sh.blackboxes.find(inst.module);
                if (bb == sh.blackboxes.end()) errorAt(inst.line, "unknown module '" + inst.module + "'");
                elaborateBlackBox(inst, *bb->second);
                continue;
            }
            const Module& child_module = *it->second;
            if (std::find(sh.stack.begin(), sh.stack.end(), inst.module) != sh.stack.end()) {
                errorAt(inst.line, "module '" + inst.module + "' instantiates itself (recursive hierarchy)");
            }

            std::vector<std::string> overridable;
            for (const auto& p : child_module.params) {
                if (!p.is_local) overridable.push_back(p.name);
            }
            Overrides values;
            for (size_t k = 0; k < inst.params.size(); ++k) {
                const Connection& c = inst.params[k];
                if (!c.expr) errorAt(c.line, "empty parameter override");
                std::string name = c.name;
                if (name.empty()) {
                    if (k >= overridable.size()) errorAt(c.line, "too many parameter values for module '" + inst.module + "'");
                    name = overridable[k];
                } else if (std::find(overridable.begin(), overridable.end(), name) == overridable.end()) {
                    errorAt(c.line, "module '" + inst.module + "' has no parameter '" + name + "' that can be overridden");
                }
                values[name] = [this, expr = c.expr](long long width) { return constantLiteral(*expr, width); };
            }

            sh.stack.push_back(inst.module);
            Elaborator child(child_module, sh, prefix + inst.name + ".", values, false);
            child.declareParams();
            child.expandGenerates();
            child.declareNets();
            std::vector<std::string> ports = child.portNames();

            std::unordered_map<std::string, const Connection*> conn;
            bool positional = !inst.ports.empty() && inst.ports[0].name.empty();
            for (size_t k = 0; k < inst.ports.size(); ++k) {
                const Connection& c = inst.ports[k];
                std::string name = c.name;
                if (positional) {
                    if (k >= ports.size()) errorAt(c.line, "too many port connections for module '" + inst.module + "'");
                    name = ports[k];
                } else if (std::find(ports.begin(), ports.end(), name) == ports.end()) {
                    errorAt(c.line, "module '" + inst.module + "' has no port '" + name + "'");
                }
                if (conn.count(name)) errorAt(c.line, "port '" + name + "' of '" + inst.name + "' is connected twice");
                conn[name] = &c;
            }
            for (const auto& port : ports) {
                Signal& cs = child.signals.at(port);
                auto cit = conn.find(port);
                const Expr* e = cit == conn.end() ? nullptr : cit->second->expr.get();
                long long width = static_cast<long long>(cs.bits.size());
                if (cs.kind == NetKind::INPUT) {
                    if (!e) errorAt(inst.line, "input '" + port + "' of instance '" + inst.name + "' is not connected");
                    Bits v = evaluate(*e, std::max(width, selfWidth(*e)));   // truncated / zero-extended to the port
                    for (long long i = 0; i < width; ++i) cs.bits[i]->addFanin(v[i]);
                } else {
                    if (!e) continue;   // unconnected output
                    Bits targets = lvalueBits(*e);
                    for (size_t i = 0; i < targets.size() && static_cast<long long>(i) < width; ++i) {
                        const NodePtr& t = targets[i];
                        if (t->type == GateType::INPUT || input_bits.count(t.get())) {
                            errorAt(inst.line, "output '" + port + "' of '" + inst.name + "' drives input '" + t->name + "'");
                        }
                        if (!t->inputs.empty()) errorAt(inst.line, "'" + t->name + "' has more than one driver (output '" + port + "' of '" + inst.name + "')");
                        t->addFanin(cs.bits[i]);
                    }
                }
            }
            child.elaborateBody();
            sh.stack.pop_back();
        }
    }

    // A hard macro: its pins become cut points (see MacroInstance)
    void elaborateBlackBox(const InstanceDecl& inst, const BlackBoxModule& bb) {
        if (!inst.params.empty()) errorAt(inst.line, "macro '" + bb.name + "' has no parameters");
        std::unordered_map<std::string, const Connection*> conn;
        bool positional = !inst.ports.empty() && inst.ports[0].name.empty();
        for (size_t k = 0; k < inst.ports.size(); ++k) {
            const Connection& c = inst.ports[k];
            std::string name = c.name;
            if (positional) {
                if (k >= bb.ports.size()) errorAt(c.line, "too many port connections for macro '" + bb.name + "'");
                name = bb.ports[k].name;
            } else {
                bool found = false;
                for (const auto& p : bb.ports) found |= p.name == name;
                if (!found) errorAt(c.line, "macro '" + bb.name + "' has no pin '" + name + "'");
            }
            if (conn.count(name)) errorAt(c.line, "pin '" + name + "' of '" + inst.name + "' is connected twice");
            conn[name] = &c;
        }
        MacroInstance mi;
        mi.name = prefix + inst.name;
        mi.cell = bb.name;
        for (const auto& port : bb.ports) {
            long long width = std::abs(port.msb - port.lsb) + 1;
            auto pinName = [&](long long i) {
                long long index = port.msb >= port.lsb ? port.lsb + i : port.lsb - i;
                return port.is_vector ? port.name + "[" + std::to_string(index) + "]" : port.name;
            };
            auto cit = conn.find(port.name);
            const Expr* e = cit == conn.end() ? nullptr : cit->second->expr.get();
            if (port.is_input) {
                if (!e) errorAt(inst.line, "input '" + port.name + "' of macro instance '" + inst.name + "' is not connected (tie unused pins to 0)");
                Bits v = evaluate(*e, std::max(width, selfWidth(*e)));
                for (long long i = 0; i < width; ++i) {
                    auto root = std::make_shared<GateNode>(sh.next_id++, GateType::OUTPUT, macroInputName(mi.name, pinName(i)));
                    root->addFanin(v[i]);
                    sh.macro_roots.push_back(root);
                    mi.inputs.emplace_back(pinName(i), root->name);
                }
            } else {
                Bits targets;
                if (e) targets = lvalueBits(*e);
                for (long long i = 0; i < width; ++i) {
                    auto q = std::make_shared<GateNode>(sh.next_id++, GateType::INPUT, macroOutputName(mi.name, pinName(i)));
                    mi.outputs.emplace_back(pinName(i), q->name);
                    if (i < static_cast<long long>(targets.size())) {
                        const NodePtr& t = targets[i];
                        if (t->type == GateType::INPUT || input_bits.count(t.get())) errorAt(inst.line, "macro output drives input '" + t->name + "'");
                        if (!t->inputs.empty()) errorAt(inst.line, "'" + t->name + "' has more than one driver (pin '" + port.name + "' of '" + inst.name + "')");
                        t->addFanin(q);
                    }
                }
            }
        }
        sh.macros.push_back(std::move(mi));
    }

    void collectRegDecls() {
        for (const auto& name : signal_order) {
            const Signal& sig = signals.at(name);
            if (!sig.is_reg) continue;
            RegisterDecl d;
            d.name = prefix + name;
            d.is_vector = sig.is_vector;
            d.msb = static_cast<int>(sig.msb);
            d.lsb = static_cast<int>(sig.lsb);
            d.is_memory = sig.is_memory;
            d.first = static_cast<int>(sig.first);
            d.last = static_cast<int>(sig.last);
            d.is_port = is_top && sig.kind == NetKind::OUTPUT;
            sh.reg_decls.push_back(d);
        }
    }

    // ---------- SRAM inference ----------

    static void countRefs(const Expr& e, const std::string& name, int& count) {
        bool select = e.kind == ExprKind::IDENTIFIER || e.kind == ExprKind::BIT_SELECT || e.kind == ExprKind::PART_SELECT ||
                      e.kind == ExprKind::INDEXED_PART_SELECT;
        if (select && e.name == name) count++;
        for (const auto& op : e.operands) {
            if (op) countRefs(*op, name, count);
        }
        if (e.word) countRefs(*e.word, name, count);
    }

    template <typename F>
    static void walk(const Stmt& s, bool in_for, const F& f) {
        f(s, in_for);
        for (const auto& st : s.stmts) walk(*st, in_for, f);
        if (s.then_stmt) walk(*s.then_stmt, in_for, f);
        if (s.else_stmt) walk(*s.else_stmt, in_for, f);
        for (const auto& item : s.items) walk(*item.body, in_for, f);
        if (s.body) walk(*s.body, true, f);
    }

    // Every place an identifier is used in this module
    int referenceCount(const std::string& name) const {
        int count = 0;
        for (const auto& a : module.assigns) {
            countRefs(*a.lhs, name, count);
            countRefs(*a.rhs, name, count);
        }
        for (const auto& ab : module.always) {
            for (const auto& ev : ab.events) count += ev.signal == name;
            walk(*ab.body, false, [&](const Stmt& st, bool) {
                for (const Expr* e : {st.lhs.get(), st.rhs.get(), st.cond.get()}) {
                    if (e) countRefs(*e, name, count);
                }
                for (const auto& item : st.items) {
                    for (const auto& l : item.labels) countRefs(*l, name, count);
                }
            });
        }
        for (const auto& inst : module.instances) {
            for (const auto& c : inst.ports) {
                if (c.expr) countRefs(*c.expr, name, count);
            }
        }
        return count;
    }

    const SramMacro* bestMacro(int ports, long long words, long long width, double limit) const {
        const SramMacro* best = nullptr;
        for (const auto& m : sh.srams->macros) {
            if (m.ports != ports || m.words < words || m.width < width || m.area >= limit) continue;
            if (!best || m.area < best->area) best = &m;
        }
        return best;
    }

    void inferSrams() {
        if (!sh.srams || !sh.srams->enabled) return;
        for (const auto& name : signal_order) {
            const Signal& mem = signals.at(name);
            if (!mem.is_memory) continue;
            long long words = (mem.first <= mem.last ? mem.last - mem.first : mem.first - mem.last) + 1;
            long long width = mem.wordWidth();
            std::string label = "memory '" + prefix + name + "' (" + std::to_string(words) + " x " + std::to_string(width) + ")";
            double flops = static_cast<double>(words * width) * sh.srams->flop_area;
            const SramMacro* one = bestMacro(1, words, width, flops);
            const SramMacro* two = bestMacro(2, words, width, flops);
            auto keep = [&](const std::string& why) { sh.notes.push_back(label + " kept as flip-flops: " + why); };
            if (!one && !two) {
                keep("no SRAM macro that fits is smaller than its " + std::to_string(static_cast<long long>(flops)) + " um^2 of flip-flops");
                continue;
            }

            // Find the accesses
            struct Access { const Stmt* stmt; const AlwaysBlock* block; bool in_for; };
            std::vector<Access> writes, reads;
            for (const auto& ab : module.always) {
                walk(*ab.body, false, [&](const Stmt& st, bool in_for) {
                    if (st.kind != StmtKind::BLOCKING && st.kind != StmtKind::NONBLOCKING) return;
                    int lhs_refs = 0;
                    countRefs(*st.lhs, name, lhs_refs);
                    if (lhs_refs) writes.push_back({&st, &ab, in_for});
                    if (st.rhs->kind == ExprKind::BIT_SELECT && st.rhs->name == name && !st.rhs->word) reads.push_back({&st, &ab, in_for});
                });
            }
            auto clocked = [](const AlwaysBlock* ab) {
                return ab->events.size() == 1 && ab->events[0].posedge;
            };
            if (writes.size() != 1) { keep(writes.empty() ? "it is never written" : "it is written in more than one place"); continue; }
            if (reads.empty() && referenceCount(name) > static_cast<int>(writes.size())) {
                keep("it is read asynchronously (an SRAM's read data is registered: use q <= " + name + "[addr])");
                continue;
            }
            if (reads.size() != 1) { keep("it needs exactly one registered read (q <= " + name + "[addr]) to use an SRAM"); continue; }
            const Access& w = writes[0];
            const Access& r = reads[0];
            if (w.stmt->kind != StmtKind::NONBLOCKING || w.stmt->lhs->kind != ExprKind::BIT_SELECT || w.stmt->lhs->word || w.in_for) {
                keep("the write is not a single whole-word non-blocking write (a partial-word write or a loop)");
                continue;
            }
            if (r.stmt->kind != StmtKind::NONBLOCKING || r.in_for || r.stmt->lhs->kind != ExprKind::IDENTIFIER) {
                keep("the read is not a whole-word non-blocking read into a register");
                continue;
            }
            if (!clocked(w.block) || !clocked(r.block) || w.block->events[0].signal != r.block->events[0].signal) {
                keep("the write and the read must be in rising-edge blocks on the same clock, without a reset");
                continue;
            }
            if (referenceCount(name) != 2) { keep("it is also used elsewhere (an asynchronous read?)"); continue; }
            const std::string& target = r.stmt->lhs->name;
            const Signal* t = signals.count(target) ? &signals.at(target) : nullptr;
            if (!t || !t->is_reg || t->is_memory || static_cast<long long>(t->bits.size()) != width) {
                keep("the read register '" + target + "' must be a reg exactly one word wide");
                continue;
            }
            int target_writes = 0;
            for (const auto& ab : module.always) {
                walk(*ab.body, false, [&](const Stmt& st, bool) {
                    if (st.lhs) countRefs(*st.lhs, target, target_writes);
                });
            }
            if (target_writes != 1) { keep("the read register '" + target + "' is also assigned elsewhere"); continue; }
            if (std::min(mem.first, mem.last) != 0) { keep("its word indices do not start at 0"); continue; }
            long long aw = std::max(selfWidth(*w.stmt->lhs->operands[0]), selfWidth(*r.stmt->rhs->operands[0]));
            if (aw < 63 && (1LL << aw) > words) { keep("its address can point past the last word"); continue; }

            Shared::PendingSram p;
            p.instance = prefix + name + "_sram";
            p.memory = prefix + name;
            p.where = "line " + std::to_string(w.stmt->line) + context();
            p.words = words;
            p.width = width;
            p.clock = portBit(w.block->events[0].signal, w.block->line, "clock");
            p.one_port = one;
            p.two_port = two;
            for (long long i = 0; i < width; ++i) {
                auto q = std::make_shared<GateNode>(sh.next_id++, GateType::INPUT, p.instance + ".rdata[" + std::to_string(i) + "]");
                const NodePtr& bit = t->bits[i];
                if (!bit->inputs.empty()) errorAt(r.stmt->line, "'" + bit->name + "' has more than one driver");
                bit->addFanin(q);
                p.q.push_back(q);
            }
            sram_access[w.stmt] = {sh.pending_srams.size(), true};
            sram_access[r.stmt] = {sh.pending_srams.size(), false};
            sh.pending_srams.push_back(std::move(p));
        }
    }

    void elaborateBody() {
        inferSrams();
        connectAssigns();
        elaborateAlwaysBlocks();
        elaborateInstances();
        checkDrivers();
        collectRegDecls();
    }

    // Picks the macro for each inferred SRAM and wires it up. A 1-port macro needs the write and
    // the read never to happen in the same cycle (with WEN and REN both high it writes through,
    // returning the new data, where the RTL returns the old one): proven with SAT. Otherwise a
    // 2-port macro writes on port A and reads on port B (old data on a same-address collision,
    // as in the PDK's functional model).
    void resolveSrams() {
        for (auto& p : sh.pending_srams) {
            bool same_address = p.waddr == p.raddr;
            bool exclusive = false;
            if (same_address && p.one_port) {
                auto both = std::make_shared<GateNode>(sh.next_id++, GateType::AND, "and");
                both->addFanin(p.wcond);
                both->addFanin(p.rcond);
                auto lhs = std::make_shared<GateNode>(sh.next_id++, GateType::OUTPUT, "x");
                lhs->addFanin(both);
                auto rhs = std::make_shared<GateNode>(sh.next_id++, GateType::OUTPUT, "x");
                rhs->addFanin(constant(false));
                exclusive = proveEquivalence({lhs}, {rhs}).equivalent;
            }
            const SramMacro* m = same_address && exclusive && p.one_port ? p.one_port : p.two_port;
            if (!m) {
                throw std::runtime_error(p.where + ": memory '" + p.memory + "': its write and read can happen in the same cycle, "
                                         "where a 1-port SRAM would return the new data instead of the old; no 2-port SRAM fits");
            }
            auto bb = sh.blackboxes.find(m->name);
            if (bb == sh.blackboxes.end()) throw std::runtime_error("SRAM " + m->name + " has no port description");
            bool two = m->ports == 2;
            NodePtr one = constant(true), zero = constant(false);
            auto bit = [&](const Bits& v, long long i) { return i < static_cast<long long>(v.size()) ? v[i] : zero; };
            // What drives input pin `port` bit i
            auto drive = [&](const std::string& port, long long i) -> NodePtr {
                if (port == "A_CLK" || port == "B_CLK") return p.clock;
                if (port == "A_DLY" || port == "B_DLY") return one;
                if (!two) {
                    if (port == "A_MEN") return orGate(p.wcond, p.rcond);
                    if (port == "A_WEN") return p.wcond;
                    if (port == "A_REN") return p.rcond;
                    if (port == "A_ADDR") return bit(p.waddr, i);
                    if (port == "A_DIN") return bit(p.wdata, i);
                    if (port == "A_BM") return i < p.width ? one : zero;
                } else {
                    if (port == "A_MEN" || port == "A_WEN") return p.wcond;
                    if (port == "A_ADDR") return bit(p.waddr, i);
                    if (port == "A_DIN") return bit(p.wdata, i);
                    if (port == "A_BM") return i < p.width ? one : zero;
                    if (port == "B_MEN" || port == "B_REN") return p.rcond;
                    if (port == "B_ADDR") return bit(p.raddr, i);
                }
                return zero;   // the other enables, BIST pins
            };
            std::string read_port = two ? "B_DOUT" : "A_DOUT";
            MacroInstance mi;
            mi.name = p.instance;
            mi.cell = m->name;
            for (const auto& port : bb->second->ports) {
                long long width = std::abs(port.msb - port.lsb) + 1;
                for (long long i = 0; i < width; ++i) {
                    long long index = port.msb >= port.lsb ? port.lsb + i : port.lsb - i;
                    std::string pin = port.is_vector ? port.name + "[" + std::to_string(index) + "]" : port.name;
                    if (port.is_input) {
                        auto root = std::make_shared<GateNode>(sh.next_id++, GateType::OUTPUT, macroInputName(mi.name, pin));
                        root->addFanin(drive(port.name, i));
                        sh.macro_roots.push_back(root);
                        mi.inputs.emplace_back(pin, root->name);
                    } else if (port.name == read_port && i < p.width) {
                        mi.outputs.emplace_back(pin, p.q[i]->name);
                    } else {
                        mi.outputs.emplace_back(pin, macroOutputName(mi.name, pin));   // unused output
                    }
                }
            }
            sh.macros.push_back(std::move(mi));
            std::ostringstream note;
            note << "memory '" << p.memory << "' (" << p.words << " x " << p.width << ") -> " << m->name << " (" << m->ports
                 << "-port" << (two && same_address ? ": write and read can coincide" : "") << "), "
                 << static_cast<long long>(m->area) << " um^2 instead of about "
                 << static_cast<long long>(static_cast<double>(p.words * p.width) * sh.srams->flop_area) << " um^2 of flip-flops";
            sh.notes.push_back(note.str());
        }
    }

    // A register's clock or reset, followed through instance ports and wires to its source
    static NodePtr traceNet(NodePtr n) {
        std::unordered_set<const GateNode*> seen;
        while ((n->type == GateType::ASSIGN || n->type == GateType::OUTPUT || n->type == GateType::BUF) &&
               n->inputs.size() == 1 && seen.insert(n.get()).second) {
            n = n->inputs[0];
        }
        return n;
    }

public:
    Elaborator(const Module& m, Shared& shared, std::string pre, Overrides values, bool top)
        : module(m), sh(shared), prefix(std::move(pre)), is_top(top), param_values(std::move(values)) {}

    Design run() {
        sh.stack.push_back(module.name);
        declareParams();
        expandGenerates();
        declareNets();
        elaborateBody();

        Design design;
        design.name = module.name;
        for (const auto& name : portNames()) design.ports.push_back(portInfo(name));
        for (const auto& name : signal_order) {
            const Signal& sig = signals.at(name);
            if (sig.kind != NetKind::OUTPUT) continue;
            for (const auto& bit : sig.bits) design.forest.push_back(bit);
        }

        resolveSrams();

        // Every clock and reset must come straight from a top-level input
        for (auto& pr : sh.registers) {
            NodePtr c = traceNet(pr.clock);
            if (!sh.top_inputs.count(c.get())) {
                throw std::runtime_error(pr.where + ": the clock of register '" + pr.bit.name +
                                         "' must come straight from a top-level input (gated or derived clocks are not supported)");
            }
            pr.bit.clock = c->name;
            if (!pr.reset) continue;
            NodePtr r = traceNet(pr.reset);
            if (r->type == GateType::CONST0 || r->type == GateType::CONST1) {
                bool high = r->type == GateType::CONST1;
                if (high != pr.bit.reset_active_low) {
                    throw std::runtime_error(pr.where + ": the reset of register '" + pr.bit.name + "' is tied to its active level");
                }
                pr.bit.reset.clear();   // tied inactive: no reset
                continue;
            }
            if (!sh.top_inputs.count(r.get())) {
                throw std::runtime_error(pr.where + ": the reset of register '" + pr.bit.name +
                                         "' must come straight from a top-level input");
            }
            pr.bit.reset = r->name;
        }

        // Keep only registers that can affect an output (directly or through other registers),
        // as Yosys's opt_clean does; the others are dead logic
        std::unordered_map<std::string, size_t> reg_index;
        for (size_t i = 0; i < sh.registers.size(); ++i) reg_index[sh.registers[i].bit.name] = i;
        std::vector<bool> live(sh.registers.size(), false);
        std::unordered_set<const GateNode*> seen;
        std::vector<const GateNode*> work;
        for (const auto& root : design.forest) work.push_back(root.get());
        for (const auto& root : sh.macro_roots) work.push_back(root.get());   // macros are always kept
        while (!work.empty()) {
            const GateNode* n = work.back();
            work.pop_back();
            if (!seen.insert(n).second) continue;
            if (n->type == GateType::INPUT) {
                auto it = reg_index.find(n->name);
                if (it != reg_index.end() && !live[it->second]) {
                    live[it->second] = true;
                    work.push_back(sh.registers[it->second].next.get());
                }
            }
            for (const auto& in : n->inputs) work.push_back(in.get());
        }
        std::unordered_set<std::string> live_signals;
        for (size_t i = 0; i < sh.registers.size(); ++i) {
            if (!live[i]) continue;
            design.registers.push_back(sh.registers[i].bit);
            design.forest.push_back(sh.registers[i].next);
            const std::string& n = sh.registers[i].bit.name;
            live_signals.insert(n.substr(0, n.find('[')));
        }
        for (const auto& d : sh.reg_decls) {
            if (live_signals.count(d.name)) design.reg_decls.push_back(d);
        }
        design.macros = sh.macros;
        design.notes = sh.notes;
        for (const auto& root : sh.macro_roots) design.forest.push_back(root);

        topologicalOrder(design.forest);   // throws on combinational loops
        compactFanouts(design.forest);     // folded-away gates leave stale fanout links
        return design;
    }
};

} // namespace

Design elaborate(const std::vector<Module>& modules, const std::string& top, const std::vector<BlackBoxModule>& blackboxes,
                 const SramInference& srams, AdderStyle adder) {
    Shared sh;
    sh.srams = &srams;
    sh.adder = adder;
    for (const auto& bb : blackboxes) sh.blackboxes[bb.name] = &bb;
    for (const auto& m : modules) {
        if (!sh.modules.emplace(m.name, &m).second) {
            throw std::runtime_error("line " + std::to_string(m.line) + ": module '" + m.name + "' is defined twice");
        }
    }
    const Module* root = nullptr;
    if (!top.empty()) {
        auto it = sh.modules.find(top);
        if (it == sh.modules.end()) throw std::runtime_error("no module named '" + top + "' (--top)");
        root = it->second;
    } else {
        // The top is the module nobody instantiates
        std::unordered_set<std::string> used;
        std::function<void(const Module&)> collect = [&](const Module& m) {
            for (const auto& inst : m.instances) used.insert(inst.module);
            for (const auto& g : m.generates) {   // instances inside generate blocks count too
                if (g.body) collect(*g.body);
                if (g.else_body) collect(*g.else_body);
                for (const auto& c : g.cases) collect(*c.second);
            }
        };
        for (const auto& m : modules) collect(m);
        std::vector<const Module*> candidates;
        for (const auto& m : modules) {
            if (!used.count(m.name)) candidates.push_back(&m);
        }
        if (candidates.size() != 1) {
            std::string names;
            for (const Module* m : candidates) names += (names.empty() ? "" : ", ") + m->name;
            throw std::runtime_error(candidates.empty() ? "every module is instantiated by another (recursive hierarchy)"
                                                        : "several top-level modules (" + names + "): choose one with --top");
        }
        root = candidates[0];
    }
    return Elaborator(*root, sh, "", {}, true).run();
}

Design elaborate(const Module& module) {
    return elaborate(std::vector<Module>{module}, "");
}
