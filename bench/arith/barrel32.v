module barrel32 (input [31:0] x, input [4:0] n, output [31:0] y); assign y = x << n; endmodule
