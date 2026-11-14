// `include, object and function-like `define, `ifdef / `else, a macro inside a macro
`include "defs.vh"
`define DOUBLE(x) {x, x}
`ifdef WIDE
`define W2 16
`else
`define W2 `WIDTH
`endif
module preprocess (input [`WIDTH-1:0] a, b, output [`W2-1:0] m, output [2*`WIDTH-1:0] d);
    assign m = `MAX(a, b);   /* `MAX in a comment stays a comment */
    assign d = `DOUBLE(a ^ b);
endmodule
