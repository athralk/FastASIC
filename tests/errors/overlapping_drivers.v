// expect-error: 'y[1]' has more than one driver
module m (input [1:0] a, input b, output [1:0] y);
    assign y[1:0] = a;
    assign y[1] = b;
endmodule
