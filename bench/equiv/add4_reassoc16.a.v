module m (input [15:0] a, b, c, d, output [15:0] y);
    assign y = ((a + b) + c) + d;
endmodule
