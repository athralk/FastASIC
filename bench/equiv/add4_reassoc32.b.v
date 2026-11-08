module m (input [31:0] a, b, c, d, output [31:0] y);
    assign y = (a + c) + (b + d);
endmodule
