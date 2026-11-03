// expect-error: output 'y[2]' is never assigned
module m (input [1:0] a, output [3:0] y);
    assign y[1:0] = a;
endmodule
