// expect-error: must come straight from a top-level input (gated or derived clocks are not supported)
module flop (input clk, input d, output reg q); always @(posedge clk) q <= d; endmodule
module m (input clk, input en, input d, output q); flop u (.clk(clk & en), .d(d), .q(q)); endmodule
