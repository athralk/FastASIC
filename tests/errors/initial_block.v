// expect-error: 'initial' blocks are not synthesizable
module m (input clk, input d, output reg q);
    initial q = 0;
    always @(posedge clk) q <= d;
endmodule
