module clock_domains(input clk_a, clk_b, cts_L1_0_n, hold_buf_0_n, output reg [39:0] qa, qb);
always @(posedge clk_a) qa <= {qa[38:0], cts_L1_0_n};
always @(posedge clk_b) qb <= {qb[38:0], hold_buf_0_n};
endmodule
