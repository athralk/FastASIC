module alu4 (
    input  [3:0] a, b,
    input  [2:0] op,
    output [3:0] y,
    output       zero, lt, eq
);
    localparam OP_ADD = 3'd0, OP_SUB = 3'd1, OP_AND = 3'd2, OP_OR = 3'd3,
               OP_XOR = 3'd4, OP_SHL = 3'd5, OP_SHR = 3'd6;

    assign y = op == OP_ADD ? a + b  :
               op == OP_SUB ? a - b  :
               op == OP_AND ? a & b  :
               op == OP_OR  ? a | b  :
               op == OP_XOR ? a ^ b  :
               op == OP_SHL ? a << 1 :
               op == OP_SHR ? a >> 1 :
                              ~a;
    assign zero = ~|y;
    assign lt   = a < b;
    assign eq   = a == b;
endmodule
