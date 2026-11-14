// Variable bit select: a 4:1 decoder-and-OR
module m (input [3:0] a, input [1:0] s, output y);
    assign y = a[s];
endmodule
