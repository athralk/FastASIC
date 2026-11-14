// expect-note: -> RM_IHPSG13_2P_256x8_c2_bm_bist (2-port)
// The FIFO from tests/macro, written with a plain reg memory: separate read and write
// addresses -> a 2-port SRAM is inferred. The rest stays ordinary logic.
module fifo_infer (input clk, input rst_n, input push, input pop, input [7:0] din,
                   output reg [7:0] dout, output empty, output full);
    reg [7:0] mem [0:255];
    reg [7:0] wptr, rptr;
    reg [8:0] level;
    wire do_push = push & ~full;
    wire do_pop = pop & ~empty;
    always @(posedge clk) begin
        if (do_push) mem[wptr] <= din;
        if (do_pop) dout <= mem[rptr];
    end
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
