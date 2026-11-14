// expect-error: module 'a' is defined twice
module a (input x, output y); assign y = x; endmodule
module a (input x, output y); assign y = ~x; endmodule
