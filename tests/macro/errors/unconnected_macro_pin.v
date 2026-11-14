// expect-error: input 'A_BIST_BM' of macro instance 'u_mem' is not connected (tie unused pins to 0)
module m (input clk, input [9:0] a, output [7:0] q);
    RM_IHPSG13_1P_1024x8_c2_bm_bist u_mem (.A_CLK(clk), .A_MEN(1'b1), .A_WEN(1'b0), .A_REN(1'b1), .A_ADDR(a),
        .A_DIN(8'd0), .A_DLY(1'b1), .A_DOUT(q), .A_BM(8'hFF), .A_BIST_CLK(1'b0), .A_BIST_EN(1'b0),
        .A_BIST_MEN(1'b0), .A_BIST_WEN(1'b0), .A_BIST_REN(1'b0), .A_BIST_ADDR(10'd0), .A_BIST_DIN(8'd0));
endmodule
