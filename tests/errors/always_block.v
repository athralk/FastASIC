// expect-error: 'always' is not supported yet
module m (input a, output y);
    always @(*) y = a;
endmodule
