#pragma once
#include <memory>
#include <string>
#include <vector>

// Syntax tree for the supported Verilog subset. The parser only builds this tree;
// parameter values, bit widths and gates are worked out later by the elaborator.

enum class ExprKind {
    IDENTIFIER,           // a, W
    NUMBER,               // 42, 8'hFF
    UNARY,                // op operand: ~a, -a, !a, and reductions like &a
    BINARY,               // lhs op rhs
    TERNARY,              // operands: cond, if_true, if_false
    CONCAT,               // {a, b, c}       operands: a, b, c (a is the most significant)
    REPLICATE,            // {n{a, b}}       operands: n, CONCAT{a, b}
    BIT_SELECT,           // a[i]            operands: i
    PART_SELECT,          // a[msb:lsb]      operands: msb, lsb
    INDEXED_PART_SELECT,  // a[base +: w]    op "+:" or "-:", operands: base, w
};

struct Expr;
using ExprPtr = std::shared_ptr<Expr>;

struct Expr {
    ExprKind kind;
    size_t line = 0;
    std::string op;              // UNARY, BINARY, INDEXED_PART_SELECT
    std::string name;            // IDENTIFIER and the selects: the signal being read
    std::vector<bool> bits;      // NUMBER: value, least significant bit first; bits.size() is the width
    std::vector<ExprPtr> operands;
};

enum class NetKind { INPUT, OUTPUT, WIRE };

struct NetDecl {
    NetKind kind;
    std::string name;
    ExprPtr msb, lsb;   // both null for a scalar
    size_t line = 0;
};

struct ParamDecl {
    std::string name;
    ExprPtr value;
    size_t line = 0;
};

struct ContinuousAssign {
    ExprPtr lhs;
    ExprPtr rhs;
    size_t line = 0;
};

struct Module {
    std::string name;
    std::vector<std::string> header_ports;   // non-ANSI port list: module m(a, b, y);
    std::vector<ParamDecl> params;           // parameter and localparam, in source order
    std::vector<NetDecl> nets;               // in source order
    std::vector<ContinuousAssign> assigns;
};
