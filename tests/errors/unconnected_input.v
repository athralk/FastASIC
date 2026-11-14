// expect-error: input 'b' of instance 'u' is not connected
module and2 (input a, b, output y); assign y = a & b; endmodule
module m (input x, output z); and2 u (.a(x), .y(z)); endmodule
