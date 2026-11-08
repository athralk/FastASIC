// Purely combinational 64-bit adder (timed against a virtual clock)
module add64 (input [63:0] a, b, input cin, output [63:0] s, output cout);
    assign {cout, s} = a + b + cin;
endmodule
