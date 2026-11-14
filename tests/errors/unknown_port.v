// expect-error: module 'inv' has no port 'b'
module inv (input a, output y); assign y = ~a; endmodule
module m (input x, output z); inv u (.a(x), .b(z)); endmodule
