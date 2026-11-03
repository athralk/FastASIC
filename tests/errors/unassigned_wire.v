// expect-error: wire 'w' is used but never assigned
module m (input a, output y);
    wire w;
    assign y = a & w;
endmodule
