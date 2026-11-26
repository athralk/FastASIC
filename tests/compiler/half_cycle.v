module half_cycle(input clk, input d, output reg q);
  reg r1, r2;
  always @(posedge clk) r1 <= d;
  always @(negedge clk) r2 <= r1;
  always @(posedge clk) q <= r2;
endmodule
