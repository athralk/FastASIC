// expect-error: more than one driver
module m (input a, input b, output y);
    assign y = a;
    assign y = b;
endmodule
