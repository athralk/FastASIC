// expect-error: 'q' is driven by more than one always block
module m (input clk, input a, input b, output reg q);
    always @(posedge clk) q <= a;
    always @(posedge clk) q <= b;
endmodule
