// Signed arithmetic: sign extension in context, signed vs mixed comparisons, $signed /
// $unsigned, arithmetic shift, negative literals, signed constants
module signed_math (input signed [7:0] a, b, input [7:0] u, input [2:0] k,
                    output signed [11:0] sum, output signed [11:0] diff, output [11:0] mixed,
                    output lt_signed, output lt_mixed, output lt_cast,
                    output signed [7:0] sra, output [7:0] srl, output signed [9:0] neg, output [7:0] ext);
    assign sum = a + b;                   // both signed: sign-extended to 12 bits
    assign diff = a - b - 8'sd3;
    assign mixed = a + u;                 // u unsigned: the whole sum is unsigned (zero-extended)
    assign lt_signed = a < b;
    assign lt_mixed = a < u;              // unsigned comparison
    assign lt_cast = a < $signed(u);
    assign sra = a >>> k;                 // arithmetic
    assign srl = $unsigned(a) >> k;       // logical
    assign neg = -a + -4;
    assign ext = {4'b0, a[3:0]} + $signed(a[7:4]);
endmodule
