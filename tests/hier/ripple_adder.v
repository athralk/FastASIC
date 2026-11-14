// 4-bit ripple-carry adder from full-adder instances: named and positional connections
module full_adder (input a, input b, input cin, output s, output cout);
    assign s = a ^ b ^ cin;
    assign cout = (a & b) | (cin & (a ^ b));
endmodule

module ripple_adder (input [3:0] a, b, input cin, output [3:0] sum, output cout);
    wire c1, c2, c3;
    full_adder fa0 (.a(a[0]), .b(b[0]), .cin(cin), .s(sum[0]), .cout(c1));
    full_adder fa1 (.a(a[1]), .b(b[1]), .cin(c1), .s(sum[1]), .cout(c2));
    full_adder fa2 (a[2], b[2], c2, sum[2], c3);
    full_adder fa3 (a[3], b[3], c3, sum[3], cout);
endmodule
