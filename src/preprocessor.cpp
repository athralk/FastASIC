#include "../include/preprocessor.hpp"
#include "../include/file_reader.hpp"
#include <cctype>
#include <fstream>
#include <stdexcept>

namespace {

bool identChar(char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '$'; }

struct Macro {
    std::vector<std::string> params;
    bool function_like = false;
    std::string body;
};

class Preprocessor {
    const PreprocessorOptions& options;
    std::map<std::string, Macro> macros;
    int depth = 0;

    [[noreturn]] static void fail(const std::string& path, int line, const std::string& m) {
        throw std::runtime_error(path + ":" + std::to_string(line) + ": " + m);
    }

    // Directives that only matter to simulators: left in place for the lexer to skip
    static bool passthrough(const std::string& d) {
        return d == "timescale" || d == "default_nettype" || d == "resetall" || d == "celldefine" || d == "endcelldefine";
    }

    std::string findInclude(const std::string& name, const std::string& from) const {
        std::vector<std::string> dirs;
        size_t slash = from.rfind('/');
        dirs.push_back(slash == std::string::npos ? "." : from.substr(0, slash));
        for (const auto& d : options.include_dirs) dirs.push_back(d);
        for (const auto& d : dirs) {
            std::string p = d + "/" + name;
            if (std::ifstream(p).good()) return p;
        }
        return "";
    }

    // Expands macro uses in a piece of text (a macro body or arguments)
    std::string expand(const std::string& text, const std::string& path, int line) {
        std::string out;
        process(text, path, out, line, true);
        return out;
    }

public:
    explicit Preprocessor(const PreprocessorOptions& o) : options(o) {
        for (const auto& [name, value] : o.defines) macros[name] = {{}, false, value};
    }

    // inline_only: expanding a body, where conditionals and includes cannot appear
    void process(const std::string& src, const std::string& path, std::string& out, int line, bool inline_only = false) {
        if (++depth > 64) fail(path, line, "macros or includes nested too deeply (recursive?)");
        std::vector<std::pair<bool, bool>> conds;   // (this branch active, some branch taken)
        auto active = [&] {
            for (const auto& c : conds) {
                if (!c.first) return false;
            }
            return true;
        };
        size_t i = 0;
        while (i < src.size()) {
            char c = src[i];
            if (c == '\n') {
                out += '\n';
                line++;
                i++;
                continue;
            }
            if (c == '/' && i + 1 < src.size() && src[i + 1] == '/') {   // line comment
                size_t end = src.find('\n', i);
                if (end == std::string::npos) end = src.size();
                if (active()) out += src.substr(i, end - i);
                i = end;
                continue;
            }
            if (c == '/' && i + 1 < src.size() && src[i + 1] == '*') {   // block comment (keeps its newlines)
                size_t end = src.find("*/", i + 2);
                end = end == std::string::npos ? src.size() : end + 2;
                for (size_t k = i; k < end; ++k) {
                    if (src[k] == '\n') { out += '\n'; line++; }
                    else if (active()) out += src[k];
                }
                i = end;
                continue;
            }
            if (c == '"') {   // string
                size_t end = i + 1;
                while (end < src.size() && src[end] != '"' && src[end] != '\n') end += src[end] == '\\' ? 2 : 1;
                end = std::min(end + 1, src.size());
                if (active()) out += src.substr(i, end - i);
                i = end;
                continue;
            }
            if (c != '`') {
                if (active()) out += c;
                i++;
                continue;
            }

            size_t start = ++i;
            while (i < src.size() && identChar(src[i])) i++;
            std::string name = src.substr(start, i - start);
            auto restOfLine = [&]() {   // directive argument, with \ continuations
                std::string text;
                while (i < src.size() && src[i] != '\n') {
                    if (src[i] == '\\' && i + 1 < src.size() && src[i + 1] == '\n') {
                        text += ' ';
                        out += '\n';
                        line++;
                        i += 2;
                        continue;
                    }
                    if (src[i] == '/' && i + 1 < src.size() && src[i + 1] == '/') {   // trailing comment
                        while (i < src.size() && src[i] != '\n') i++;
                        break;
                    }
                    text += src[i++];
                }
                return text;
            };
            auto word = [&](const std::string& text) {
                size_t a = text.find_first_not_of(" \t"), b = a;
                while (b < text.size() && identChar(text[b])) b++;
                return a == std::string::npos ? std::string() : text.substr(a, b - a);
            };

            if (name == "ifdef" || name == "ifndef" || name == "elsif" || name == "else" || name == "endif") {
                if (inline_only) fail(path, line, "`" + name + " inside a macro");
                std::string arg = name == "else" || name == "endif" ? "" : word(restOfLine());
                bool defined = macros.count(arg) > 0;
                if (name == "ifdef" || name == "ifndef") {
                    bool take = name == "ifdef" ? defined : !defined;
                    conds.push_back({take, take});
                } else {
                    if (conds.empty()) fail(path, line, "`" + name + " without `ifdef");
                    auto& top = conds.back();
                    if (name == "endif") conds.pop_back();
                    else if (name == "else") { top.first = !top.second; top.second = true; }
                    else { bool take = !top.second && defined; top.first = take; top.second |= take; }
                }
                continue;
            }
            if (!active()) continue;
            if (passthrough(name)) {
                out += "`" + name;
                continue;
            }
            if (name == "define") {
                size_t p = i;
                while (p < src.size() && (src[p] == ' ' || src[p] == '\t')) p++;
                size_t ns = p;
                while (p < src.size() && identChar(src[p])) p++;
                std::string mname = src.substr(ns, p - ns);
                if (mname.empty()) fail(path, line, "`define needs a name");
                Macro m;
                i = p;
                if (i < src.size() && src[i] == '(') {   // function-like: no space before '('
                    m.function_like = true;
                    size_t close = src.find(')', i);
                    if (close == std::string::npos) fail(path, line, "unterminated macro parameter list");
                    std::string params = src.substr(i + 1, close - i - 1);
                    std::string cur;
                    for (char ch : params + ",") {
                        if (ch == ',') {
                            size_t a = cur.find_first_not_of(" \t"), b = cur.find_last_not_of(" \t");
                            if (a != std::string::npos) m.params.push_back(cur.substr(a, b - a + 1));
                            cur.clear();
                        } else {
                            cur += ch;
                        }
                    }
                    i = close + 1;
                }
                m.body = restOfLine();
                macros[mname] = m;
                continue;
            }
            if (name == "undef") {
                macros.erase(word(restOfLine()));
                continue;
            }
            if (name == "include") {
                if (inline_only) fail(path, line, "`include inside a macro");
                std::string arg = restOfLine();
                size_t a = arg.find('"'), b = arg.rfind('"');
                if (a == std::string::npos || a == b) fail(path, line, "`include needs a \"file name\"");
                std::string file = findInclude(arg.substr(a + 1, b - a - 1), path);
                if (file.empty()) fail(path, line, "cannot find included file " + arg.substr(a, b - a + 1));
                std::string included;
                process(readVerilogFile(file), file, included, 1);
                out += included;
                continue;
            }
            auto it = macros.find(name);
            if (it == macros.end()) fail(path, line, "undefined macro `" + name);
            const Macro& m = it->second;
            std::string body = m.body;
            if (m.function_like) {
                size_t p = i;
                while (p < src.size() && std::isspace(static_cast<unsigned char>(src[p]))) p++;
                if (p >= src.size() || src[p] != '(') fail(path, line, "macro `" + name + " needs arguments");
                std::vector<std::string> args;
                std::string cur;
                int nest = 0;
                for (p = p + 1; p < src.size(); ++p) {
                    char ch = src[p];
                    if (ch == '\n') line++;
                    if ((ch == ',' && nest == 0) || (ch == ')' && nest == 0)) {
                        args.push_back(cur);
                        cur.clear();
                        if (ch == ')') break;
                        continue;
                    }
                    if (ch == '(' || ch == '[' || ch == '{') nest++;
                    if (ch == ')' || ch == ']' || ch == '}') nest--;
                    cur += ch;
                }
                if (p >= src.size()) fail(path, line, "unterminated arguments of `" + name);
                i = p + 1;
                if (args.size() != m.params.size()) fail(path, line, "macro `" + name + " takes " + std::to_string(m.params.size()) + " arguments");
                // substitute parameters (whole identifiers only)
                std::string result;
                for (size_t k = 0; k < body.size();) {
                    if (identChar(body[k]) && (k == 0 || !identChar(body[k - 1]))) {
                        size_t e = k;
                        while (e < body.size() && identChar(body[e])) e++;
                        std::string w = body.substr(k, e - k);
                        size_t idx = 0;
                        while (idx < m.params.size() && m.params[idx] != w) idx++;
                        result += idx < m.params.size() ? args[idx] : w;
                        k = e;
                    } else {
                        result += body[k++];
                    }
                }
                body = result;
            }
            out += expand(body, path, line);
        }
        if (!conds.empty() && !inline_only) fail(path, line, "missing `endif");
        depth--;
    }
};

} // namespace

std::string preprocessVerilog(const std::string& text, const std::string& path, const PreprocessorOptions& options) {
    Preprocessor p(options);
    std::string out;
    p.process(text, path, out, 1);
    return out;
}
