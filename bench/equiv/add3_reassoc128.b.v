module m (input [127:0] a, b, c, output [127:0] y);
    assign y = a + (b + c);
endmodule
