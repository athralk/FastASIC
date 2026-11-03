`timescale 1ns / 1ps
`default_nettype none
module literals (
    input  [7:0]  a,
    output [7:0]  y_hex, y_oct, y_dec, y_bin,
    output [15:0] y_wide,
    output [3:0]  y_trunc
);
    assign y_hex   = a ^ 8'hA5;
    assign y_oct   = a & 8'o17;
    assign y_dec   = a | 8'd200;
    assign y_bin   = a + 8'b0000_0011;
    assign y_wide  = {a, 8'hFF} - 16'd1;
    assign y_trunc = 'hABCD;   // unsized literal truncated to the 4-bit target
endmodule
