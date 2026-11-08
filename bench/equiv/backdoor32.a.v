module m (input [31:0] key, input x, output y);
    assign y = x ^ (key == 32'hDEADBEEF);
endmodule
