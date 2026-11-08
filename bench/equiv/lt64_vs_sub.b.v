module m (input [63:0] a, b, output y);
    wire [64:0] d = {1'b0, a} - {1'b0, b};
    assign y = d[64];
endmodule
