module m (input [31:0] a, b, output y);
    wire [32:0] d = {1'b0, a} - {1'b0, b};
    assign y = d[32];
endmodule
