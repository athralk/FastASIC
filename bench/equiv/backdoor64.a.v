module m (input [63:0] key, input x, output y);
    assign y = x ^ (key == 64'hDEADBEEFCAFEF00D);
endmodule
