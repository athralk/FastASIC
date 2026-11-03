module m (input [3:0] a, b, output [3:0] sum, output cout);
    wire [4:0] c;
    assign c[0] = 1'b0;
    assign sum = a ^ b ^ c[3:0];
    assign c[4:1] = (a & b) | (c[3:0] & (a ^ b));
    assign cout = c[4];
endmodule
