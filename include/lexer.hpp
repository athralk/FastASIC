#pragma once
#include <string>
#include <vector>

enum class TokenType {
    MODULE, ENDMODULE, INPUT, OUTPUT, WIRE, ASSIGN, PARAMETER, LOCALPARAM,
    IDENTIFIER, NUMBER, SYMBOL, END_OF_FILE
};

struct Token {
    TokenType type;
    std::string value;
    size_t line;
};

class VerilogLexer {
    std::string src;
    size_t pos = 0;
    size_t line = 1;

    char peek(size_t ahead = 0) const;
    char advance();
    void skipWhitespaceAndComments();
    void skipDirective();
    Token lexNumber();

public:
    explicit VerilogLexer(std::string source);
    std::vector<Token> tokenize();
};
