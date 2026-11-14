#include "../include/parser.hpp"
#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace {

constexpr int kMaxLiteralWidth = 1 << 16;

bool isSymbol(const Token& token, const std::string& symbol) {
    return token.type == TokenType::SYMBOL && token.value == symbol;
}

std::string describe(const Token& token) {
    return token.type == TokenType::END_OF_FILE ? "end of file" : "'" + token.value + "'";
}

// Binary operator precedence (higher binds tighter), following the Verilog standard
int binaryPrecedence(const Token& token) {
    static const std::unordered_map<std::string, int> table = {
        {"||", 1}, {"&&", 2}, {"|", 3}, {"^", 4}, {"~^", 4}, {"&", 5},
        {"==", 6}, {"!=", 6}, {"<", 7}, {"<=", 7}, {">", 7}, {">=", 7},
        {"<<", 8}, {">>", 8}, {"<<<", 8}, {">>>", 8}, {"+", 9}, {"-", 9}, {"*", 10}, {"/", 10}, {"%", 10},
    };
    if (token.type != TokenType::SYMBOL) return -1;
    auto it = table.find(token.value);
    return it == table.end() ? -1 : it->second;
}

bool isUnaryOperator(const Token& token) {
    static const std::unordered_set<std::string> ops = {"+", "-", "!", "~", "&", "~&", "|", "~|", "^", "~^"};
    return token.type == TokenType::SYMBOL && ops.count(token.value);
}

} // namespace

VerilogParser::VerilogParser(std::vector<Token> t) : tokens(std::move(t)) {}

Token VerilogParser::peek(size_t ahead) const {
    if (index + ahead >= tokens.size()) {
        return {TokenType::END_OF_FILE, "", tokens.empty() ? 0 : tokens.back().line};
    }
    return tokens[index + ahead];
}

Token VerilogParser::advance() {
    Token token = peek();
    if (index < tokens.size()) index++;
    return token;
}

bool VerilogParser::match(TokenType type) {
    if (peek().type == type) {
        advance();
        return true;
    }
    return false;
}

bool VerilogParser::matchSymbol(const std::string& symbol) {
    if (isSymbol(peek(), symbol)) {
        advance();
        return true;
    }
    return false;
}

Token VerilogParser::expect(TokenType type, const std::string& what) {
    if (peek().type != type) errorAt(peek(), "expected " + what + " but found " + describe(peek()));
    return advance();
}

void VerilogParser::expectSymbol(const std::string& symbol) {
    if (!matchSymbol(symbol)) errorAt(peek(), "expected '" + symbol + "' but found " + describe(peek()));
}

void VerilogParser::errorAt(const Token& token, const std::string& message) const {
    throw std::runtime_error("line " + std::to_string(token.line) + ": " + message);
}

// Clear messages for common Verilog we can't handle yet, instead of a generic syntax error
// Verilog-2005 keywords that are not otherwise handled: they cannot be used as names
static const std::unordered_set<std::string> kReserved = {
    "small", "medium", "large", "strong0", "strong1", "weak0", "weak1", "pull0", "pull1", "highz0", "highz1",
    "and", "or", "nand", "nor", "xor", "xnor", "not", "buf", "bufif0", "bufif1", "notif0", "notif1",
    "event", "time", "realtime", "edge", "pulldown", "pullup", "primitive", "table", "config", "library",
    "cell", "use", "design", "instance", "liblist", "localparam", "macromodule", "scalared", "vectored",
    "triand", "trior", "trireg", "tri0", "tri1", "wand", "wor", "rcmos", "cmos", "nmos", "pmos", "rnmos",
    "rpmos", "tran", "tranif0", "tranif1", "rtran", "rtranif0", "rtranif1", "specparam", "disable",
    "deassign", "force", "release",
};

void VerilogParser::rejectUnsupportedKeyword(const Token& token) const {
    if (token.type == TokenType::IDENTIFIER && kReserved.count(token.value)) {
        errorAt(token, "'" + token.value + "' is a reserved word in Verilog and cannot be used here");
    }
    static const std::unordered_set<std::string> unsupported = {
        "always_comb", "always_ff", "always_latch", "logic", "real", "inout", "while", "repeat", "forever",
        "wait", "fork", "supply0", "supply1", "tri", "specify", "defparam",
    };
    if (token.type == TokenType::IDENTIFIER && unsupported.count(token.value)) {
        errorAt(token, "'" + token.value + "' is not supported yet");
    }
    if (token.type == TokenType::IDENTIFIER && token.value == "initial") {
        errorAt(token, "'initial' blocks are not synthesizable (use a reset to give registers a start value)");
    }
}

ExprPtr VerilogParser::makeExpr(ExprKind kind, const Token& at, std::vector<ExprPtr> operands) {
    auto expr = std::make_shared<Expr>();
    expr->kind = kind;
    expr->line = at.line;
    expr->operands = std::move(operands);
    return expr;
}

// NAME = constant_expression
void VerilogParser::parseParameterAssignment() {
    ExprPtr msb, lsb;
    if (matchSymbol("[")) {   // parameter [7:0] NAME = ...
        msb = parseExpression();
        expectSymbol(":");
        lsb = parseExpression();
        expectSymbol("]");
    }
    if (peek().type == TokenType::IDENTIFIER && (peek().value == "integer" || peek().value == "signed")) {
        errorAt(peek(), "typed parameters (" + peek().value + ") are not supported");
    }
    Token name = expect(TokenType::IDENTIFIER, "a parameter name");
    expectSymbol("=");
    ParamDecl decl;
    decl.name = name.value;
    decl.value = parseExpression();
    decl.line = name.line;
    decl.msb = msb;
    decl.lsb = lsb;
    module.params.push_back(decl);
}

// input [7:0] a, b  |  output wire y  |  wire [3:0] n1, n2 = a & b
void VerilogParser::parseDeclaration() {
    Token keyword = advance();
    NetKind kind = keyword.type == TokenType::INPUT  ? NetKind::INPUT
                 : keyword.type == TokenType::OUTPUT ? NetKind::OUTPUT
                                                     : NetKind::WIRE;
    bool is_reg = false;
    if (kind == NetKind::OUTPUT && peek().type == TokenType::IDENTIFIER && peek().value == "reg") {
        advance();   // output reg
        is_reg = true;
    } else if (kind != NetKind::WIRE) {
        match(TokenType::WIRE);
    }
    if (kind == NetKind::INPUT && peek().type == TokenType::IDENTIFIER && peek().value == "reg") {
        errorAt(peek(), "an input cannot be a reg");
    }
    bool is_signed = false;
    if (isKeyword("signed")) {
        advance();
        is_signed = true;
        if (kind == NetKind::OUTPUT && !is_reg && isKeyword("reg")) {   // output signed reg
            advance();
            is_reg = true;
        }
    }
    rejectUnsupportedKeyword(peek());

    // One range applies to every name in the declaration
    ExprPtr msb, lsb;
    if (matchSymbol("[")) {
        msb = parseExpression();
        expectSymbol(":");
        lsb = parseExpression();
        expectSymbol("]");
    }

    while (true) {
        Token name = expect(TokenType::IDENTIFIER, "a net name");
        rejectUnsupportedKeyword(name);
        NetDecl decl;
        decl.kind = kind;
        decl.name = name.value;
        decl.msb = msb;
        decl.lsb = lsb;
        decl.line = name.line;
        decl.is_reg = is_reg;
        decl.is_signed = is_signed;
        module.nets.push_back(decl);
        if (is_reg && matchSymbol("=")) errorAt(name, "initial values on registers are not supported (use a reset)");

        // wire w = expr;  is shorthand for a declaration plus a continuous assign
        if (kind == NetKind::WIRE && matchSymbol("=")) {
            auto target = makeExpr(ExprKind::IDENTIFIER, name);
            target->name = name.value;
            module.assigns.push_back({target, parseExpression(), name.line});
        }

        // A comma followed by a keyword (ANSI header: "input a, output y") ends this declaration
        if (!(isSymbol(peek(), ",") && peek(1).type == TokenType::IDENTIFIER)) break;
        advance();
    }
}

// reg [7:0] a, b;  |  reg [7:0] mem [0:15];  |  integer i;
void VerilogParser::parseRegDeclaration(bool is_integer) {
    advance();
    bool is_signed = is_integer;
    if (!is_integer && isKeyword("signed")) {
        advance();
        is_signed = true;
    }
    ExprPtr msb, lsb;
    if (!is_integer && matchSymbol("[")) {
        msb = parseExpression();
        expectSymbol(":");
        lsb = parseExpression();
        expectSymbol("]");
    }
    do {
        Token name = expect(TokenType::IDENTIFIER, is_integer ? "an integer name" : "a reg name");
        rejectUnsupportedKeyword(name);
        NetDecl decl;
        decl.kind = is_integer ? NetKind::INTEGER : NetKind::REG;
        decl.name = name.value;
        decl.msb = msb;
        decl.lsb = lsb;
        decl.line = name.line;
        decl.is_signed = is_signed;
        if (!is_integer && matchSymbol("[")) {   // memory: reg [w] mem [first:last]
            decl.first = parseExpression();
            expectSymbol(":");
            decl.last = parseExpression();
            expectSymbol("]");
            if (isSymbol(peek(), "[")) errorAt(peek(), "multi-dimensional memories are not supported");
        }
        if (matchSymbol("=")) errorAt(name, "initial values on registers are not supported (use a reset)");
        module.nets.push_back(decl);
    } while (matchSymbol(","));
    expectSymbol(";");
}

bool VerilogParser::isKeyword(const std::string& word) const {
    const Token& t = tokens[std::min(index, tokens.size() - 1)];
    return index < tokens.size() && t.type == TokenType::IDENTIFIER && t.value == word;
}

// always @(posedge clk or negedge rst_n) stmt  |  always @(*) stmt  |  always @(a or b) stmt
void VerilogParser::parseAlways() {
    AlwaysBlock block;
    block.line = advance().line;
    expectSymbol("@");
    if (matchSymbol("*")) {
        block.star = true;
    } else {
        expectSymbol("(");
        if (matchSymbol("*")) {
            block.star = true;
        } else {
            for (;;) {   // events separated by "or" or ","
                EdgeEvent ev;
                ev.line = peek().line;
                if (isKeyword("posedge")) { advance(); ev.posedge = true; }
                else if (isKeyword("negedge")) { advance(); ev.negedge = true; }
                Token sig = expect(TokenType::IDENTIFIER, "a signal in the sensitivity list");
                if (isSymbol(peek(), "[")) errorAt(peek(), "bit selects in sensitivity lists are not supported");
                ev.signal = sig.value;
                block.events.push_back(ev);
                if (isKeyword("or")) advance();
                else if (!matchSymbol(",")) break;
            }
        }
        expectSymbol(")");
    }
    block.body = parseStatement();
    module.always.push_back(block);
}

// lhs = expr  or  lhs <= expr (without the ';')
StmtPtr VerilogParser::parseProceduralAssign(bool allow_nonblocking) {
    Token at = peek();
    auto stmt = std::make_shared<Stmt>();
    stmt->line = at.line;
    stmt->lhs = parsePrimary();
    if (matchSymbol("=")) {
        stmt->kind = StmtKind::BLOCKING;
    } else if (allow_nonblocking && matchSymbol("<=")) {
        stmt->kind = StmtKind::NONBLOCKING;
    } else {
        errorAt(peek(), "expected '=' or '<=' but found " + describe(peek()));
    }
    if (isSymbol(peek(), "#")) errorAt(peek(), "delays (#) are not synthesizable");
    stmt->rhs = parseExpression();
    return stmt;
}

StmtPtr VerilogParser::parseStatement() {
    Token t = peek();
    auto stmt = std::make_shared<Stmt>();
    stmt->line = t.line;
    if (matchSymbol(";")) return stmt;   // empty statement
    if (isSymbol(t, "#") || isSymbol(t, "@")) errorAt(t, "timing controls inside always blocks are not synthesizable");
    if (t.type != TokenType::IDENTIFIER && !isSymbol(t, "{")) errorAt(t, "expected a statement but found " + describe(t));
    rejectUnsupportedKeyword(t);

    if (isKeyword("begin")) {
        advance();
        if (matchSymbol(":")) expect(TokenType::IDENTIFIER, "a block name");
        stmt->kind = StmtKind::BLOCK;
        while (!isKeyword("end")) {
            if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing 'end'");
            stmt->stmts.push_back(parseStatement());
        }
        advance();
        return stmt;
    }
    if (isKeyword("if")) {
        advance();
        stmt->kind = StmtKind::IF;
        expectSymbol("(");
        stmt->cond = parseExpression();
        expectSymbol(")");
        stmt->then_stmt = parseStatement();
        if (isKeyword("else")) {
            advance();
            stmt->else_stmt = parseStatement();
        }
        return stmt;
    }
    if (isKeyword("case") || isKeyword("casez") || isKeyword("casex")) {
        stmt->case_wild = isKeyword("casez") ? 1 : isKeyword("casex") ? 2 : 0;
        advance();
        stmt->kind = StmtKind::CASE;
        expectSymbol("(");
        stmt->cond = parseExpression();
        expectSymbol(")");
        while (!isKeyword("endcase")) {
            if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing 'endcase'");
            CaseItem item;
            item.line = peek().line;
            if (isKeyword("default")) {
                advance();
                matchSymbol(":");
            } else {
                do {
                    item.labels.push_back(parseExpression());
                } while (matchSymbol(","));
                expectSymbol(":");
            }
            item.body = parseStatement();
            stmt->items.push_back(item);
        }
        advance();
        return stmt;
    }
    if (isKeyword("for")) {
        advance();
        stmt->kind = StmtKind::FOR;
        expectSymbol("(");
        stmt->init = parseProceduralAssign(false);
        expectSymbol(";");
        stmt->cond = parseExpression();
        expectSymbol(";");
        stmt->step = parseProceduralAssign(false);
        expectSymbol(")");
        stmt->body = parseStatement();
        return stmt;
    }
    if (isKeyword("end") || isKeyword("endcase") || isKeyword("else")) errorAt(t, "unexpected '" + t.value + "'");
    if (t.type == TokenType::IDENTIFIER && (isSymbol(peek(1), "(") || isSymbol(peek(1), ";"))) {
        advance();   // task call: name(args); or name;
        stmt->kind = StmtKind::TASK_CALL;
        stmt->name = t.value;
        if (matchSymbol("(")) {
            if (!isSymbol(peek(), ")")) {
                do {
                    stmt->args.push_back(parseExpression());
                } while (matchSymbol(","));
            }
            expectSymbol(")");
        }
        expectSymbol(";");
        if (t.value[0] == '$') stmt->kind = StmtKind::EMPTY;   // $display & co.: simulation only
        return stmt;
    }
    stmt = parseProceduralAssign(true);
    expectSymbol(";");
    return stmt;
}

// assign lhs = expr;  (or assign a = x, {c, s} = y;)
void VerilogParser::parseAssign() {
    advance();
    do {
        Token at = peek();
        auto lhs = parsePrimary();   // identifier, select or concatenation; checked by the elaborator
        expectSymbol("=");
        module.assigns.push_back({lhs, parseExpression(), at.line});
    } while (matchSymbol(","));
    expectSymbol(";");
}

// cond ? a : b  (right associative, lowest precedence)
ExprPtr VerilogParser::parseExpression() {
    Token at = peek();
    auto cond = parseBinary(1);
    if (!matchSymbol("?")) return cond;
    auto if_true = parseExpression();
    expectSymbol(":");
    auto if_false = parseExpression();
    return makeExpr(ExprKind::TERNARY, at, {cond, if_true, if_false});
}

// Precedence climbing: keep absorbing operators that bind at least as tightly as min_precedence
ExprPtr VerilogParser::parseBinary(int min_precedence) {
    auto left = parseUnary();
    while (binaryPrecedence(peek()) >= min_precedence) {
        Token op = advance();
        auto right = parseBinary(binaryPrecedence(op) + 1);   // +1 makes operators left associative
        auto node = makeExpr(ExprKind::BINARY, op, {left, right});
        node->op = op.value;
        left = node;
    }
    return left;
}

// Prefix operators bind tightest: ~a & b is (~a) & b, &a is an AND-reduction of a's bits
ExprPtr VerilogParser::parseUnary() {
    if (isUnaryOperator(peek())) {
        Token op = advance();
        auto node = makeExpr(ExprKind::UNARY, op, {parseUnary()});
        node->op = op.value;
        return node;
    }
    return parsePrimary();
}

ExprPtr VerilogParser::parsePrimary() {
    Token t = peek();
    if (isSymbol(t, "(")) {
        advance();
        auto node = parseExpression();
        expectSymbol(")");
        return node;
    }

    if (isSymbol(t, "{")) {
        advance();
        auto first = parseExpression();
        // {n{...}} replication: the inner braces are an ordinary concatenation
        if (isSymbol(peek(), "{")) {
            auto inner = parsePrimary();
            expectSymbol("}");
            return makeExpr(ExprKind::REPLICATE, t, {first, inner});
        }
        std::vector<ExprPtr> parts = {first};
        while (matchSymbol(",")) parts.push_back(parseExpression());
        expectSymbol("}");
        return makeExpr(ExprKind::CONCAT, t, std::move(parts));
    }

    if (t.type == TokenType::IDENTIFIER) {
        rejectUnsupportedKeyword(t);
        advance();
        if (matchSymbol("(")) {   // f(x, y), $signed(x), ...
            auto call = makeExpr(ExprKind::CALL, t);
            call->name = t.value;
            if (!isSymbol(peek(), ")")) {
                do {
                    call->operands.push_back(parseExpression());
                } while (matchSymbol(","));
            }
            expectSymbol(")");
            return call;
        }
        if (!matchSymbol("[")) {
            auto node = makeExpr(ExprKind::IDENTIFIER, t);
            node->name = t.value;
            return node;
        }
        ExprPtr node;
        auto first = parseExpression();
        if (matchSymbol(":")) {
            node = makeExpr(ExprKind::PART_SELECT, t, {first, parseExpression()});
        } else if (isSymbol(peek(), "+:") || isSymbol(peek(), "-:")) {
            std::string op = advance().value;
            node = makeExpr(ExprKind::INDEXED_PART_SELECT, t, {first, parseExpression()});
            node->op = op;
        } else {
            node = makeExpr(ExprKind::BIT_SELECT, t, {first});
        }
        expectSymbol("]");
        node->name = t.value;
        if (node->kind == ExprKind::BIT_SELECT && isSymbol(peek(), "[")) {
            // mem[word][bit] or mem[word][msb:lsb]: a select inside one memory word
            advance();
            ExprPtr word = node->operands[0];
            auto inner_first = parseExpression();
            if (matchSymbol(":")) {
                node = makeExpr(ExprKind::PART_SELECT, t, {inner_first, parseExpression()});
            } else {
                node = makeExpr(ExprKind::BIT_SELECT, t, {inner_first});
            }
            expectSymbol("]");
            node->name = t.value;
            node->word = word;
        }
        return node;
    }

    if (t.type == TokenType::NUMBER) {
        advance();
        return parseNumber(t);
    }

    errorAt(t, "expected an expression but found " + describe(t));
}

// 42 -> 32-bit unsized, 8'hFF -> 8 bits, 'b101 -> 32-bit unsized, ...
ExprPtr VerilogParser::parseNumber(const Token& token) {
    std::string text;
    for (char c : token.value) {
        if (c != '_') text += c;
    }

    int size = -1;   // -1 = unsized
    char base = 'd';
    bool is_signed = false;
    std::string digits = text;
    size_t tick = text.find('\'');
    if (tick != std::string::npos) {
        if (tick > 0) {
            try {
                size = std::stoi(text.substr(0, tick));
            } catch (const std::exception&) {
                size = -2;
            }
            if (size < 1 || size > kMaxLiteralWidth) errorAt(token, "literal width in '" + token.value + "' is out of range");
        }
        size_t p = tick + 1;
        if (text[p] == 's' || text[p] == 'S') {
            is_signed = true;
            p++;
        }
        base = static_cast<char>(std::tolower(static_cast<unsigned char>(text[p])));
        digits = text.substr(p + 1);
    } else {
        is_signed = true;   // a plain decimal number is a (signed) integer
    }

    std::vector<bool> bits, wild;   // least significant bit first
    if (base == 'd') {
        for (char c : digits) {
            char lc = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (lc == 'x' || lc == 'z' || lc == '?') errorAt(token, "x/z digits need a binary, octal or hex literal ('" + token.value + "')");
        }
        unsigned long long value = 0;
        try {
            size_t used = 0;
            value = std::stoull(digits, &used, 10);
            if (used != digits.size()) throw std::invalid_argument(digits);
        } catch (const std::out_of_range&) {
            errorAt(token, "decimal literal '" + token.value + "' does not fit in 64 bits");
        } catch (const std::exception&) {
            errorAt(token, "invalid number literal '" + token.value + "'");
        }
        for (; value; value >>= 1) bits.push_back(value & 1);
    } else {
        int bits_per_digit = base == 'b' ? 1 : base == 'o' ? 3 : 4;
        for (auto it = digits.rbegin(); it != digits.rend(); ++it) {
            char c = static_cast<char>(std::tolower(static_cast<unsigned char>(*it)));
            bool w = c == 'x' || c == 'z' || c == '?';
            int v = w ? 0 : std::isdigit(static_cast<unsigned char>(c)) ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : 99;
            if (v >= (1 << bits_per_digit)) errorAt(token, "invalid digit '" + std::string(1, *it) + "' in '" + token.value + "'");
            for (int k = 0; k < bits_per_digit; ++k) {
                bits.push_back((v >> k) & 1);
                wild.push_back(w);
            }
        }
    }

    // Sized literals are truncated or extended to their size (with x/z if the leftmost digit is
    // one); unsized ones are at least 32 bits
    bool wild_top = !wild.empty() && wild.back();
    size_t width = size > 0 ? static_cast<size_t>(size) : std::max<size_t>(32, bits.size());
    bits.resize(width, false);
    wild.resize(width, wild_top);
    bool any_wild = false;
    for (bool w : wild) any_wild |= w;

    auto node = makeExpr(ExprKind::NUMBER, token);
    node->bits = std::move(bits);
    if (any_wild) node->wild = std::move(wild);
    node->is_signed = is_signed;
    return node;
}

// (.a(x), .b(), ...) or (x, y, ...): the parentheses are already consumed up to here
std::vector<Connection> VerilogParser::parseConnections(bool allow_empty) {
    std::vector<Connection> out;
    if (isSymbol(peek(), ")")) return out;
    bool named = isSymbol(peek(), ".");
    do {
        Connection c;
        c.line = peek().line;
        if (named) {
            expectSymbol(".");
            if (isSymbol(peek(), "*")) errorAt(peek(), ".* connections are not supported");
            c.name = expect(TokenType::IDENTIFIER, "a port or parameter name").value;
            expectSymbol("(");
            if (!isSymbol(peek(), ")")) c.expr = parseExpression();
            expectSymbol(")");
        } else {
            if (isSymbol(peek(), ".")) errorAt(peek(), "mixing named and positional connections");
            if (!(allow_empty && (isSymbol(peek(), ",") || isSymbol(peek(), ")")))) c.expr = parseExpression();
        }
        out.push_back(c);
    } while (matchSymbol(","));
    return out;
}

// child #(.W(8)) u1 (.a(x), .y(z)), u2 (...);
void VerilogParser::parseInstances() {
    Token mod = advance();
    InstanceDecl base;
    base.module = mod.value;
    base.line = mod.line;
    if (matchSymbol("#")) {
        expectSymbol("(");
        base.params = parseConnections(false);
        expectSymbol(")");
    }
    do {
        InstanceDecl inst = base;
        Token name = expect(TokenType::IDENTIFIER, "an instance name");
        inst.name = name.value;
        inst.line = name.line;
        if (isSymbol(peek(), "[")) errorAt(peek(), "instance arrays are not supported");
        expectSymbol("(");
        inst.ports = parseConnections(true);
        expectSymbol(")");
        module.instances.push_back(inst);
    } while (matchSymbol(","));
    expectSymbol(";");
}

void VerilogParser::parseModuleItem() {
    Token t = peek();
    if (t.type == TokenType::INPUT || t.type == TokenType::OUTPUT || t.type == TokenType::WIRE) {
        parseDeclaration();
        expectSymbol(";");
    } else if (t.type == TokenType::PARAMETER || t.type == TokenType::LOCALPARAM) {
        advance();
        rejectUnsupportedKeyword(peek());
        do {
            parseParameterAssignment();
            module.params.back().is_local = t.type == TokenType::LOCALPARAM;
        } while (matchSymbol(","));
        expectSymbol(";");
    } else if (t.type == TokenType::ASSIGN) {
        parseAssign();
    } else if (t.type == TokenType::END_OF_FILE || t.type == TokenType::MODULE) {
        errorAt(t, "missing 'endmodule'");
    } else if (isKeyword("reg") || isKeyword("integer")) {
        parseRegDeclaration(t.value == "integer");
    } else if (isKeyword("always")) {
        parseAlways();
    } else if (isKeyword("function") || isKeyword("task")) {
        parseFunction(t.value == "task");
    } else if (isKeyword("genvar")) {
        advance();
        do {
            module.genvars.push_back(expect(TokenType::IDENTIFIER, "a genvar name").value);
        } while (matchSymbol(","));
        expectSymbol(";");
    } else if (isKeyword("generate")) {
        advance();
        while (!isKeyword("endgenerate")) {
            if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing 'endgenerate'");
            parseModuleItem();
        }
        advance();
    } else if (isKeyword("for")) {
        parseGenerateFor();
    } else if (isKeyword("if")) {
        parseGenerateIf();
    } else if (isKeyword("case")) {
        parseGenerateCase();
    } else if (isKeyword("begin")) {
        GenerateBlock g;
        g.kind = GenerateBlock::BLOCK;
        g.line = t.line;
        g.body = parseGenerateBody(g.label);
        module.generates.push_back(g);
    } else if (t.type == TokenType::IDENTIFIER && (peek(1).type == TokenType::IDENTIFIER || isSymbol(peek(1), "#"))) {
        rejectUnsupportedKeyword(t);
        parseInstances();
    } else if (t.type == TokenType::IDENTIFIER) {
        rejectUnsupportedKeyword(t);
        errorAt(t, "unexpected " + describe(t));
    } else {
        errorAt(t, "unexpected " + describe(t));
    }
}

// begin [: label] items end, or a single item
std::shared_ptr<Module> VerilogParser::parseGenerateBody(std::string& label) {
    Module saved = std::move(module);
    module = Module{};
    if (isKeyword("begin")) {
        advance();
        if (matchSymbol(":")) label = expect(TokenType::IDENTIFIER, "a block name").value;
        while (!isKeyword("end")) {
            if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing 'end'");
            parseModuleItem();
        }
        advance();
    } else {
        parseModuleItem();
    }
    auto body = std::make_shared<Module>(std::move(module));
    module = std::move(saved);
    return body;
}

// for (i = 0; i < N; i = i + 1) begin : label ... end
void VerilogParser::parseGenerateFor() {
    GenerateBlock g;
    g.kind = GenerateBlock::FOR;
    g.line = advance().line;
    expectSymbol("(");
    if (isKeyword("genvar")) advance();
    g.genvar = expect(TokenType::IDENTIFIER, "a genvar").value;
    expectSymbol("=");
    g.init = parseExpression();
    expectSymbol(";");
    g.cond = parseExpression();
    expectSymbol(";");
    if (expect(TokenType::IDENTIFIER, "the genvar").value != g.genvar) errorAt(peek(), "the generate loop must step its own genvar");
    expectSymbol("=");
    g.step = parseExpression();
    expectSymbol(")");
    g.body = parseGenerateBody(g.label);
    module.generates.push_back(g);
}

void VerilogParser::parseGenerateIf() {
    GenerateBlock g;
    g.kind = GenerateBlock::IF;
    g.line = advance().line;
    expectSymbol("(");
    g.cond = parseExpression();
    expectSymbol(")");
    g.body = parseGenerateBody(g.label);
    if (isKeyword("else")) {
        advance();
        std::string else_label;
        g.else_body = parseGenerateBody(else_label);
        if (!else_label.empty()) g.else_body->name = else_label;   // the else branch's own block name
    }
    module.generates.push_back(g);
}

void VerilogParser::parseGenerateCase() {
    GenerateBlock g;
    g.kind = GenerateBlock::CASE;
    g.line = advance().line;
    expectSymbol("(");
    g.cond = parseExpression();
    expectSymbol(")");
    while (!isKeyword("endcase")) {
        if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing 'endcase'");
        std::vector<ExprPtr> labels;
        if (isKeyword("default")) {
            advance();
            matchSymbol(":");
        } else {
            do {
                labels.push_back(parseExpression());
            } while (matchSymbol(","));
            expectSymbol(":");
        }
        std::string label;
        auto body = parseGenerateBody(label);
        body->name = label;
        g.cases.emplace_back(labels, body);
    }
    advance();
    module.generates.push_back(g);
}

// function [signed] [7:0] f(input [7:0] a, ...); ... endfunction   (or with port declarations)
void VerilogParser::parseFunction(bool is_task) {
    FunctionDecl f;
    f.is_task = is_task;
    f.line = advance().line;
    if (isKeyword("automatic")) advance();
    if (!is_task) {
        if (isKeyword("signed")) {
            advance();
            f.is_signed = true;
        }
        if (isKeyword("integer")) {
            advance();
            f.is_signed = true;
            Token at = peek();
            auto hi = makeExpr(ExprKind::NUMBER, at), lo = makeExpr(ExprKind::NUMBER, at);
            hi->bits = {true, true, true, true, true};   // 31
            hi->bits.resize(32, false);
            lo->bits.assign(32, false);
            f.msb = hi;
            f.lsb = lo;
        } else if (matchSymbol("[")) {
            f.msb = parseExpression();
            expectSymbol(":");
            f.lsb = parseExpression();
            expectSymbol("]");
        }
    }
    f.name = expect(TokenType::IDENTIFIER, is_task ? "a task name" : "a function name").value;
    Module saved = std::move(module);
    module = Module{};
    if (matchSymbol("(")) {   // ANSI ports
        if (!isSymbol(peek(), ")")) {
            do {
                if (peek().type != TokenType::INPUT && peek().type != TokenType::OUTPUT) errorAt(peek(), "expected input or output");
                parseDeclaration();
            } while (matchSymbol(","));
        }
        expectSymbol(")");
    }
    expectSymbol(";");
    std::vector<StmtPtr> body;
    std::string end = is_task ? "endtask" : "endfunction";
    while (!isKeyword(end)) {
        if (peek().type == TokenType::END_OF_FILE) errorAt(peek(), "missing '" + end + "'");
        if (peek().type == TokenType::INPUT || peek().type == TokenType::OUTPUT) {
            parseDeclaration();
            expectSymbol(";");
        } else if (isKeyword("reg") || isKeyword("integer")) {
            parseRegDeclaration(peek().value == "integer");
        } else {
            body.push_back(parseStatement());
        }
    }
    advance();
    for (const auto& n : module.nets) {
        if (n.kind == NetKind::INPUT || n.kind == NetKind::OUTPUT) {
            if (!is_task && n.kind == NetKind::OUTPUT) errorAt(peek(), "functions cannot have outputs (use a task)");
            f.ports.push_back(n);
        } else {
            f.locals.push_back(n);
        }
    }
    module = std::move(saved);
    auto block = std::make_shared<Stmt>();
    block->kind = StmtKind::BLOCK;
    block->line = f.line;
    block->stmts = std::move(body);
    f.body = block;
    module.functions.push_back(std::move(f));
}

std::vector<Module> VerilogParser::parseModules() {
    std::vector<Module> modules;
    do {
        modules.push_back(parseModule());
    } while (peek().type == TokenType::MODULE);
    if (peek().type != TokenType::END_OF_FILE) errorAt(peek(), "unexpected " + describe(peek()) + " after 'endmodule'");
    return modules;
}

Module VerilogParser::parse() {
    std::vector<Module> modules = parseModules();
    if (modules.size() != 1) throw std::runtime_error("expected one module, found " + std::to_string(modules.size()));
    return modules[0];
}

Module VerilogParser::parseModule() {
    module = Module{};
    module.line = peek().line;
    expect(TokenType::MODULE, "'module'");
    module.name = expect(TokenType::IDENTIFIER, "a module name").value;

    // #(parameter W = 8, D = 2)
    if (matchSymbol("#")) {
        expectSymbol("(");
        do {
            match(TokenType::PARAMETER);
            parseParameterAssignment();
        } while (matchSymbol(","));
        expectSymbol(")");
    }

    if (matchSymbol("(")) {
        if (!matchSymbol(")")) {
            do {
                if (peek().type == TokenType::INPUT || peek().type == TokenType::OUTPUT) {
                    parseDeclaration();   // ANSI style: module m(input a, output y);
                } else {
                    rejectUnsupportedKeyword(peek());
                    module.header_ports.push_back(expect(TokenType::IDENTIFIER, "a port name").value);
                }
            } while (matchSymbol(","));
            expectSymbol(")");
        }
    }
    expectSymbol(";");

    while (!match(TokenType::ENDMODULE)) parseModuleItem();

    return std::move(module);
}
