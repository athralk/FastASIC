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
    CALL,                 // f(x, y), $signed(x), $unsigned(x), $clog2(x)   operands: the arguments
};

struct Expr;
using ExprPtr = std::shared_ptr<Expr>;

struct Expr {
    ExprKind kind;
    size_t line = 0;
    std::string op;              // UNARY, BINARY, INDEXED_PART_SELECT
    std::string name;            // IDENTIFIER and the selects: the signal being read
    std::vector<bool> bits;      // NUMBER: value, least significant bit first; bits.size() is the width
    std::vector<bool> wild;      // NUMBER: x / z / ? digits (only meaningful in casez / casex labels)
    bool is_signed = false;      // NUMBER: 8'sd5, or an unsized decimal (Verilog integers are signed)
    std::vector<ExprPtr> operands;
    ExprPtr word;                // mem[word][bit] / mem[word][msb:lsb]: the memory word index
};

enum class NetKind { INPUT, OUTPUT, WIRE, REG, INTEGER };

struct NetDecl {
    NetKind kind;
    std::string name;
    ExprPtr msb, lsb;          // both null for a scalar
    size_t line = 0;
    bool is_reg = false;       // output reg
    bool is_signed = false;    // input signed [7:0] a
    ExprPtr first, last;       // reg [7:0] mem [first:last]: a memory
};

struct ParamDecl {
    std::string name;
    ExprPtr value;
    size_t line = 0;
    bool is_local = false;   // localparam: cannot be overridden by an instance
    ExprPtr msb, lsb;        // parameter [msb:lsb] NAME: the value is truncated to that width
};

// One port connection or parameter override of an instance: .name(expr), or positional
struct Connection {
    std::string name;        // empty when positional
    ExprPtr expr;            // null: left unconnected, .port()
    size_t line = 0;
};

// child_module #(.W(8)) u_child (.a(x), .y(z));
struct InstanceDecl {
    std::string module;
    std::string name;
    std::vector<Connection> params;
    std::vector<Connection> ports;
    size_t line = 0;
};

struct ContinuousAssign {
    ExprPtr lhs;
    ExprPtr rhs;
    size_t line = 0;
};

// ---------- Procedural code (always blocks) ----------

enum class StmtKind { BLOCK, BLOCKING, NONBLOCKING, IF, CASE, FOR, TASK_CALL, EMPTY };

struct Stmt;
using StmtPtr = std::shared_ptr<Stmt>;

struct CaseItem {
    std::vector<ExprPtr> labels;   // empty: default
    StmtPtr body;
    size_t line = 0;
};

struct Stmt {
    StmtKind kind = StmtKind::EMPTY;
    size_t line = 0;
    ExprPtr lhs, rhs;              // BLOCKING (=), NONBLOCKING (<=)
    ExprPtr cond;                  // IF condition, CASE subject, FOR condition
    std::vector<StmtPtr> stmts;    // BLOCK
    StmtPtr then_stmt, else_stmt;  // IF
    std::vector<CaseItem> items;   // CASE
    StmtPtr init, step, body;      // FOR (init and step are BLOCKING)
    int case_wild = 0;             // CASE: 0 case, 1 casez (z ? are wildcards), 2 casex (x too)
    std::string name;              // TASK_CALL
    std::vector<ExprPtr> args;     // TASK_CALL
};

struct EdgeEvent {
    bool posedge = false, negedge = false;   // neither: level-sensitive
    std::string signal;
    size_t line = 0;
};

struct AlwaysBlock {
    size_t line = 0;
    bool star = false;                       // @* / @(*)
    std::vector<EdgeEvent> events;
    StmtPtr body;
};

// function [7:0] f(input [7:0] a); ... endfunction   /   task t; input a; output b; ... endtask
struct FunctionDecl {
    std::string name;
    bool is_task = false;
    bool is_signed = false;
    ExprPtr msb, lsb;              // return range (functions); null: 1 bit
    std::vector<NetDecl> ports;    // inputs / outputs, in argument order
    std::vector<NetDecl> locals;   // reg / integer declarations inside
    StmtPtr body;
    size_t line = 0;
};

struct Module;

// generate for / if / case and named blocks: expanded by the elaborator into ordinary items
struct GenerateBlock {
    enum Kind { FOR, IF, CASE, BLOCK } kind = BLOCK;
    std::string label;                         // begin : label (named blocks scope their contents)
    std::string genvar;                        // FOR
    ExprPtr init, cond, step;                  // FOR: genvar = init; cond; genvar = step.   IF / CASE: cond
    std::shared_ptr<Module> body, else_body;   // FOR, IF, BLOCK body; IF else
    std::vector<std::pair<std::vector<ExprPtr>, std::shared_ptr<Module>>> cases;   // CASE (no labels: default)
    size_t line = 0;
};

struct Module {
    std::string name;
    std::vector<std::string> header_ports;   // non-ANSI port list: module m(a, b, y);
    std::vector<ParamDecl> params;           // parameter and localparam, in source order
    std::vector<NetDecl> nets;               // in source order
    std::vector<ContinuousAssign> assigns;
    std::vector<AlwaysBlock> always;
    std::vector<InstanceDecl> instances;
    std::vector<FunctionDecl> functions;
    std::vector<GenerateBlock> generates;
    std::vector<std::string> genvars;
    size_t line = 0;
};
