// expect-error: undeclared identifier 'B'
module m (input a, output y);
    parameter A = B + 1;
    assign y = a;
endmodule
