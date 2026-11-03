// The carry survives because operands are extended to the 9-bit target before adding
module adder8 (input [7:0] a, b, input cin, output [7:0] sum, output cout);
    assign {cout, sum} = a + b + cin;
endmodule
