// expect-error: too many port connections for module 'inv'
module inv (input a, output y); assign y = ~a; endmodule
module m (input x, output z); inv u (x, z, x); endmodule
