// expect-error: '*' is only supported between constants
module m (input [3:0] a, b, output [7:0] y);
    assign y = a * b;
endmodule
