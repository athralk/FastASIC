// sat-only: differs on 1 of 2^33 patterns, which random simulation will not hit
module m (input [31:0] key, input x, output y);
    assign y = x ^ (key == 32'hDEADBEEF);
endmodule
