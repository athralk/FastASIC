// expect-error: x/z values are not supported
module m (input [3:0] a, output [3:0] y);
    assign y = a & 4'b10x1;
endmodule
