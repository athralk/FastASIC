// 4-bit ripple-carry adder, one net per bit (buses are not supported yet)
module ripple_adder4 (
    input a0, a1, a2, a3,
    input b0, b1, b2, b3,
    input cin,
    output s0, s1, s2, s3,
    output cout
);
    wire c1, c2, c3;
    assign s0 = a0 ^ b0 ^ cin;
    assign c1 = (a0 & b0) | (cin & (a0 ^ b0));
    assign s1 = a1 ^ b1 ^ c1;
    assign c2 = (a1 & b1) | (c1 & (a1 ^ b1));
    assign s2 = a2 ^ b2 ^ c2;
    assign c3 = (a2 & b2) | (c2 & (a2 ^ b2));
    assign s3 = a3 ^ b3 ^ c3;
    assign cout = (a3 & b3) | (c3 & (a3 ^ b3));
endmodule
