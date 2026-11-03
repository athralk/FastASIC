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
        {"<<", 8}, {">>", 8}, {"+", 9}, {"-", 9}, {"*", 10}, {"/", 10}, {"%", 10},
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
void VerilogParser::rejectUnsupportedKeyword(const Token& token) const {
    static const std::unordered_set<std::string> unsupported = {
        "always", "always_comb", "always_ff", "initial", "reg", "logic", "integer", "real", "inout",
        "signed", "function", "task", "generate", "genvar", "if", "case", "for", "begin",
        "supply0", "supply1", "tri", "specify", "defparam",
    };
    if (token.type == TokenType::IDENTIFIER && unsupported.count(token.value)) {
        errorAt(token, "'" + token.value + "' is not supported yet (only wires and continuous assigns)");
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
    Token name = expect(TokenType::IDENTIFIER, "a parameter name");
    expectSymbol("=");
    module.params.push_back({name.value, parseExpression(), name.line});
}

// input [7:0] a, b  |  output wire y  |  wire [3:0] n1, n2 = a & b
void VerilogParser::parseDeclaration() {
    Token keyword = advance();
    NetKind kind = keyword.type == TokenType::INPUT  ? NetKind::INPUT
                 : keyword.type == TokenType::OUTPUT ? NetKind::OUTPUT
                                                     : NetKind::WIRE;
    if (kind != NetKind::WIRE) match(TokenType::WIRE);
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
        module.nets.push_back({kind, name.value, msb, lsb, name.line});

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
        if (text[p] == 's' || text[p] == 'S') errorAt(token, "signed literals are not supported yet");
        base = static_cast<char>(std::tolower(static_cast<unsigned char>(text[p])));
        digits = text.substr(p + 1);
    }

    std::vector<bool> bits;   // least significant bit first
    for (char c : digits) {
        char lc = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (lc == 'x' || lc == 'z' || lc == '?') errorAt(token, "x/z values are not supported ('" + token.value + "')");
    }
    if (base == 'd') {
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
            int v = std::isdigit(static_cast<unsigned char>(c)) ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : 99;
            if (v >= (1 << bits_per_digit)) errorAt(token, "invalid digit '" + std::string(1, *it) + "' in '" + token.value + "'");
            for (int k = 0; k < bits_per_digit; ++k) bits.push_back((v >> k) & 1);
        }
    }

    // Sized literals are truncated or zero-extended to their size; unsized ones are at least 32 bits
    while (!bits.empty() && !bits.back()) bits.pop_back();
    size_t width = size > 0 ? static_cast<size_t>(size) : std::max<size_t>(32, bits.size());
    bits.resize(width, false);

    auto node = makeExpr(ExprKind::NUMBER, token);
    node->bits = std::move(bits);
    return node;
}

Module VerilogParser::parse() {
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

    while (!match(TokenType::ENDMODULE)) {
        Token t = peek();
        if (t.type == TokenType::INPUT || t.type == TokenType::OUTPUT || t.type == TokenType::WIRE) {
            parseDeclaration();
            expectSymbol(";");
        } else if (t.type == TokenType::PARAMETER || t.type == TokenType::LOCALPARAM) {
            advance();
            rejectUnsupportedKeyword(peek());
            if (isSymbol(peek(), "[")) errorAt(peek(), "parameter ranges are not supported yet");
            do {
                parseParameterAssignment();
            } while (matchSymbol(","));
            expectSymbol(";");
        } else if (t.type == TokenType::ASSIGN) {
            parseAssign();
        } else if (t.type == TokenType::END_OF_FILE) {
            errorAt(t, "missing 'endmodule'");
        } else if (t.type == TokenType::IDENTIFIER) {
            rejectUnsupportedKeyword(t);
            errorAt(t, "unexpected " + describe(t) + " (module instances are not supported yet)");
        } else {
            errorAt(t, "unexpected " + describe(t));
        }
    }

    if (peek().type == TokenType::MODULE) errorAt(peek(), "only one module per file is supported for now");
    if (peek().type != TokenType::END_OF_FILE) errorAt(peek(), "unexpected " + describe(peek()) + " after 'endmodule'");
    return std::move(module);
}
