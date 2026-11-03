module m (input [7:0] a, output [7:0] y, output [7:0] z);
    assign y = a << 2;
    assign z = a >> 3;
endmodule
