// expect-error: several top-level modules (a, b): choose one with --top
module a (input x, output y); assign y = x; endmodule
module b (input x, output y); assign y = ~x; endmodule
