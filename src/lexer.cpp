#include "../include/lexer.hpp"
#include <cctype>
#include <stdexcept>

namespace {

bool isAlpha(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_'; }
bool isDigit(char c) { return std::isdigit(static_cast<unsigned char>(c)); }
bool isAlnum(char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; }
bool isBaseChar(char c) { return std::string("bBoOdDhH").find(c) != std::string::npos; }

// Multi-character operators, matched before single characters
const char* const kLongSymbols[] = {
    ">>>", "<<<", "~^", "^~", "~&", "~|", "&&", "||", "==", "!=", "<=", ">=", "<<", ">>", "+:", "-:",
};
// '@' and '.' aren't used by any supported construct; lexing them lets the parser explain
// what's unsupported (an always block, a module instance) instead of failing on the character
const std::string kShortSymbols = "=;,()[]{}&|^~!?:+-*/%<>#@.";

} // namespace

char VerilogLexer::peek(size_t ahead) const { return pos + ahead < src.size() ? src[pos + ahead] : '\0'; }
char VerilogLexer::advance() {
    char c = peek();
    if (c == '\n') line++;
    pos++;
    return c;
}

void VerilogLexer::skipWhitespaceAndComments() {
    while (pos < src.size()) {
        char c = peek();
        if (std::isspace(static_cast<unsigned char>(c))) {
            advance();
        } else if (c == '/' && peek(1) == '/') {
            while (peek() != '\n' && peek() != '\0') advance();
        } else if (c == '/' && peek(1) == '*') {
            size_t start_line = line;
            advance(); advance();
            while (!(peek() == '*' && peek(1) == '/')) {
                if (pos >= src.size()) {
                    throw std::runtime_error("line " + std::to_string(start_line) + ": unterminated block comment");
                }
                advance();
            }
            advance(); advance();
        } else {
            break;
        }
    }
}

// `timescale and friends only matter to simulators, so they are skipped. `define, `ifdef and
// `include are handled by the preprocessor before lexing.
void VerilogLexer::skipDirective() {
    advance();
    size_t start = pos;
    while (isAlnum(peek())) advance();
    std::string name = src.substr(start, pos - start);
    if (name == "timescale" || name == "default_nettype" || name == "resetall" ||
        name == "celldefine" || name == "endcelldefine") {
        while (peek() != '\n' && peek() != '\0') advance();
        return;
    }
    throw std::runtime_error("line " + std::to_string(line) + ": compiler directive `" + name + " is not supported");
}

// Numeric literals: 42, 8'hFF, 4'b10_10, 'd7, ... (the parser works out their value and width)
Token VerilogLexer::lexNumber() {
    size_t start = pos;
    while (isDigit(peek()) || peek() == '_') advance();
    if (peek() == '\'') {
        advance();
        if (peek() == 's' || peek() == 'S') advance();
        if (!isBaseChar(peek())) {
            throw std::runtime_error("line " + std::to_string(line) + ": malformed number literal");
        }
        advance();
        if (!isAlnum(peek()) && peek() != '?') {
            throw std::runtime_error("line " + std::to_string(line) + ": number literal has no digits");
        }
        while (isAlnum(peek()) || peek() == '?') advance();
    }
    return {TokenType::NUMBER, src.substr(start, pos - start), line};
}

VerilogLexer::VerilogLexer(std::string source) : src(std::move(source)) {}

std::vector<Token> VerilogLexer::tokenize() {
    std::vector<Token> tokens;
    while (pos < src.size()) {
        skipWhitespaceAndComments();
        if (pos >= src.size()) break;
        char c = peek();

        if (c == '`') {
            skipDirective();
            continue;
        }

        if (isAlpha(c) || (c == '$' && isAlpha(peek(1)))) {   // identifiers and $system names
            size_t start = pos;
            advance();
            while (isAlnum(peek()) || peek() == '$') advance();
            std::string word = src.substr(start, pos - start);
            TokenType type = TokenType::IDENTIFIER;
            if (word == "module") type = TokenType::MODULE;
            else if (word == "endmodule") type = TokenType::ENDMODULE;
            else if (word == "input") type = TokenType::INPUT;
            else if (word == "output") type = TokenType::OUTPUT;
            else if (word == "wire") type = TokenType::WIRE;
            else if (word == "assign") type = TokenType::ASSIGN;
            else if (word == "parameter") type = TokenType::PARAMETER;
            else if (word == "localparam") type = TokenType::LOCALPARAM;
            tokens.push_back({type, word, line});
            continue;
        }

        if (isDigit(c) || (c == '\'' && (isBaseChar(peek(1)) || peek(1) == 's' || peek(1) == 'S'))) {
            tokens.push_back(lexNumber());
            continue;
        }

        bool matched = false;
        for (const char* symbol : kLongSymbols) {
            std::string sym(symbol);
            bool hit = true;
            for (size_t k = 0; k < sym.size(); ++k) hit &= peek(k) == sym[k];
            if (hit) {
                // ^~ and ~^ are the same operator; keep one spelling
                tokens.push_back({TokenType::SYMBOL, sym == "^~" ? "~^" : sym, line});
                for (size_t k = 0; k < sym.size(); ++k) advance();
                matched = true;
                break;
            }
        }
        if (matched) continue;

        if (kShortSymbols.find(c) != std::string::npos) {
            tokens.push_back({TokenType::SYMBOL, std::string(1, c), line});
            advance();
            continue;
        }

        // Fail loudly instead of silently dropping syntax we don't support yet
        throw std::runtime_error("line " + std::to_string(line) + ": unsupported character '" + std::string(1, c) + "'");
    }
    tokens.push_back({TokenType::END_OF_FILE, "", line});
    return tokens;
}
