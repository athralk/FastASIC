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
    void parseRegDeclaration(bool is_integer);
    void parseAssign();
    void parseAlways();
    void parseInstances();
    void parseModuleItem();
    void parseFunction(bool is_task);
    std::shared_ptr<Module> parseGenerateBody(std::string& label);
    void parseGenerateFor();
    void parseGenerateIf();
    void parseGenerateCase();
    std::vector<Connection> parseConnections(bool allow_empty);
    Module parseModule();
    StmtPtr parseStatement();
    StmtPtr parseProceduralAssign(bool allow_nonblocking);
    bool isKeyword(const std::string& word) const;
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
    // Every module in the file, in source order
    std::vector<Module> parseModules();
    // The file's only module (throws if there are several)
    Module parse();
};
