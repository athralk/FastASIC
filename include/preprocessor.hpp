#pragma once
#include <map>
#include <string>
#include <vector>

// Verilog preprocessor, run on the source text before lexing:
//   `define NAME text / `define NAME(a, b) text (continued with \), `undef NAME, `NAME / `NAME(x, y)
//   `ifdef / `ifndef / `elsif / `else / `endif
//   `include "file" (relative to the including file, then the include directories)
// Comments and strings are copied untouched; lines removed by directives stay as empty lines,
// so line numbers outside included files are unchanged. `timescale & co. are left to the lexer.
struct PreprocessorOptions {
    std::map<std::string, std::string> defines;   // -D NAME=value
    std::vector<std::string> include_dirs;        // -I dir
};

std::string preprocessVerilog(const std::string& text, const std::string& path, const PreprocessorOptions& options = {});
