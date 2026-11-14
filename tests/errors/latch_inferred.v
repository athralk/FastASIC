// expect-error: latch inferred: 'y' is not assigned on every path
module m (input a, input en, output reg y);
    always @(*) if (en) y = a;
endmodule
