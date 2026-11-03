// expect-error: undeclared net 'c'
module m (input a, input b, output y);
    assign y = a & c;
endmodule
