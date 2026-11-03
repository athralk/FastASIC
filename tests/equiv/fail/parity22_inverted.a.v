// 22 inputs: too many for exhaustive checking, exercises random simulation
module parity22 (
    input x0,
    input x1,
    input x2,
    input x3,
    input x4,
    input x5,
    input x6,
    input x7,
    input x8,
    input x9,
    input x10,
    input x11,
    input x12,
    input x13,
    input x14,
    input x15,
    input x16,
    input x17,
    input x18,
    input x19,
    input x20,
    input x21,
    output y
);
    assign y = x0 ^ x1 ^ x2 ^ x3 ^ x4 ^ x5 ^ x6 ^ x7 ^ x8 ^ x9 ^ x10 ^ x11 ^ x12 ^ x13 ^ x14 ^ x15 ^ x16 ^ x17 ^ x18 ^ x19 ^ x20 ^ x21;
endmodule
