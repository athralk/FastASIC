// expect-note: -> RM_IHPSG13_2P_256x16_c2_bm_bist (2-port: write and read can coincide)
// Read and write in the same cycle (read-first: rdata gets the old word); a 1-port SRAM would
// write through, so this needs the 2-port macro
module ram_read_first (input clk, input we, input [7:0] addr, input [15:0] wdata, output reg [15:0] rdata);
    reg [15:0] mem [0:255];
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        rdata <= mem[addr];
    end
endmodule
