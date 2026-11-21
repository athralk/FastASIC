// Folding a ? a : 0 must not turn input a into a recursive mux descriptor.
module mux_fold_identity(input a, b, output reg y, output reg z);
always @* begin
    y = 1'b0;
    if (a) y = a;
    if (b) y = 1'b0;

    z = 1'b1;
    if (a) z = a;
    if (b) z = 1'b1;
end
endmodule
