module params #(parameter WIDTH = 6, parameter SHIFT = 2) (
    input  [WIDTH-1:0] x,
    input  [WIDTH-1:0] y,
    output [WIDTH:0]   sum,
    output [WIDTH-1:0] shifted,
    output             all_ones, any_set, parity
);
    localparam MASK = (1 << (WIDTH - 1)) - 1;   // clears the top bit
    wire [WIDTH-1:0] masked = x & MASK;

    assign sum      = x + y;
    assign shifted  = masked << SHIFT;
    assign all_ones = &x;
    assign any_set  = |y;
    assign parity   = ^{x, y};
endmodule
