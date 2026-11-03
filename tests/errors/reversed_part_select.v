// expect-error: is reversed relative to its declaration
module m (input [7:0] a, output [3:0] y);
    assign y = a[0:3];
endmodule
