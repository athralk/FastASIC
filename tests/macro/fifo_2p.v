// 256 x 8 FIFO on IHP's dual-port SRAM: port A writes, port B reads. The SRAM sits inside a
// wrapper module, so it gets a hierarchical instance name (u_fifo.u_ram); pointers and the
// level counter are ordinary registers around it.
module ram_2p (input clk, input we, input [7:0] waddr, input [7:0] wdata, input re, input [7:0] raddr,
               output [7:0] rdata);
    RM_IHPSG13_2P_256x8_c2_bm_bist u_ram (
        .A_CLK(clk), .A_MEN(we), .A_WEN(we), .A_REN(1'b0), .A_ADDR(waddr), .A_DIN(wdata), .A_DLY(1'b1),
        .A_DOUT(), .A_BM(8'hFF),
        .A_BIST_CLK(1'b0), .A_BIST_EN(1'b0), .A_BIST_MEN(1'b0), .A_BIST_WEN(1'b0), .A_BIST_REN(1'b0),
        .A_BIST_ADDR(8'd0), .A_BIST_DIN(8'd0), .A_BIST_BM(8'd0),
        .B_CLK(clk), .B_MEN(re), .B_WEN(1'b0), .B_REN(re), .B_ADDR(raddr), .B_DIN(8'd0), .B_DLY(1'b1),
        .B_DOUT(rdata), .B_BM(8'hFF),
        .B_BIST_CLK(1'b0), .B_BIST_EN(1'b0), .B_BIST_MEN(1'b0), .B_BIST_WEN(1'b0), .B_BIST_REN(1'b0),
        .B_BIST_ADDR(8'd0), .B_BIST_DIN(8'd0), .B_BIST_BM(8'd0));
endmodule

module fifo (input clk, input rst_n, input push, input pop, input [7:0] din, output [7:0] dout,
             output empty, output full);
    reg [7:0] wptr, rptr;
    reg [8:0] level;
    wire do_push = push & ~full;
    wire do_pop = pop & ~empty;
    ram_2p u_ram2 (.clk(clk), .we(do_push), .waddr(wptr), .wdata(din), .re(do_pop), .raddr(rptr), .rdata(dout));
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            wptr <= 0; rptr <= 0; level <= 0;
        end else begin
            if (do_push) wptr <= wptr + 1;
            if (do_pop) rptr <= rptr + 1;
            level <= level + do_push - do_pop;
        end
    assign empty = level == 0;
    assign full = level == 9'd256;
endmodule

module fifo_2p (input clk, input rst_n, input push, input pop, input [7:0] din, output [7:0] dout,
                output empty, output full);
    fifo u_fifo (.clk(clk), .rst_n(rst_n), .push(push), .pop(pop), .din(din), .dout(dout), .empty(empty), .full(full));
endmodule
