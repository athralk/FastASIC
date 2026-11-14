// A 1024 x 8 frame buffer in an IHP SRAM macro: write port and registered read, with the
// built-in self-test (BIST) port tied off. The macro is found in the PDK by name.
module sram_buffer (input clk, input rst_n, input we, input re, input [9:0] addr, input [7:0] wdata,
                    output [7:0] rdata, output reg [7:0] checksum);
    wire [7:0] dout;
    RM_IHPSG13_1P_1024x8_c2_bm_bist u_mem (
        .A_CLK(clk), .A_MEN(we | re), .A_WEN(we), .A_REN(re), .A_ADDR(addr), .A_DIN(wdata),
        .A_DLY(1'b1), .A_DOUT(dout), .A_BM(8'hFF),
        .A_BIST_CLK(1'b0), .A_BIST_EN(1'b0), .A_BIST_MEN(1'b0), .A_BIST_WEN(1'b0), .A_BIST_REN(1'b0),
        .A_BIST_ADDR(10'd0), .A_BIST_DIN(8'd0), .A_BIST_BM(8'd0));
    assign rdata = dout;
    // a running checksum of everything read
    always @(posedge clk or negedge rst_n)
        if (!rst_n) checksum <= 8'd0;
        else if (re) checksum <= checksum ^ dout;
endmodule
