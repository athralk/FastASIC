// expect-error: compiler directive `define is not supported
`define WIDTH 4
module m (input a, output y);
    assign y = a;
endmodule
