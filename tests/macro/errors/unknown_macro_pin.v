// expect-error: macro 'RM_IHPSG13_1P_1024x8_c2_bm_bist' has no pin 'A_WE'
module m (input clk, output [7:0] q);
    RM_IHPSG13_1P_1024x8_c2_bm_bist u_mem (.A_CLK(clk), .A_WE(1'b0), .A_DOUT(q));
endmodule
