#pragma once
#include "lexer.hpp"
#include "ast.hpp"
#include <string>
#include <vector>

// Turns tokens into a Module syntax tree. It only checks syntax: names, widths
// and drivers are checked when the elaborator turns the tree into gates.
class VerilogParser {
    std::vector<Token> tokens;
    size_t index = 0;
    Module module;

    Token peek(size_t ahead = 0) const;
    Token advance();
    bool match(TokenType type);
    bool matchSymbol(const std::string& symbol);
    Token expect(TokenType type, const std::string& what);
    void expectSymbol(const std::string& symbol);
    [[noreturn]] void errorAt(const Token& token, const std::string& message) const;

    void parseParameterAssignment();
    void parseDeclaration();
    void parseAssign();
    void rejectUnsupportedKeyword(const Token& token) const;

    // Expressions: precedence climbing over the Verilog operator table, ?: lowest
    ExprPtr parseExpression();
    ExprPtr parseBinary(int min_precedence);
    ExprPtr parseUnary();
    ExprPtr parsePrimary();
    ExprPtr parseNumber(const Token& token);
    ExprPtr makeExpr(ExprKind kind, const Token& at, std::vector<ExprPtr> operands = {});

public:
    explicit VerilogParser(std::vector<Token> t);
    Module parse();
};
