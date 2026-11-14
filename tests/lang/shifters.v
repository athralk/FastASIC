// Variable shifts: logarithmic barrel shifters, amounts wider than log2(width), signed >>>
module shifters (input [15:0] x, input signed [15:0] s, input [4:0] amt, input [1:0] small_amt,
                 output [15:0] shl, output [15:0] shr, output signed [15:0] sar, output [15:0] sal,
                 output [7:0] narrow, output [31:0] wide);
    assign shl = x << amt;      // amt >= 16 shifts everything out
    assign shr = x >> amt;
    assign sar = s >>> amt;
    assign sal = s <<< small_amt;
    assign narrow = x[7:0] >> small_amt;
    assign wide = x << amt;     // context width 32 keeps the shifted-out bits
endmodule
