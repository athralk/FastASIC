// expect-error: bit select index must be a constant
module m (input [3:0] a, input [1:0] s, output y);
    assign y = a[s];
endmodule
