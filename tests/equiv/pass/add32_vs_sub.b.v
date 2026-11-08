// two's complement: -(~b) = b + 1, so a - ~b - 1 = a + b
module m (input [31:0] a, b, output [31:0] y);
    assign y = a - ~b - 1;
endmodule
