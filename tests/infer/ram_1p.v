// expect-note: -> RM_IHPSG13_1P_1024x8_c2_bm_bist (1-port)
// 1024 x 8 memory written and read through one address: write and read are exclusive
// (if/else), so it can be a 1-port SRAM
module ram_1p (input clk, input we, input en, input [9:0] addr, input [7:0] wdata, output reg [7:0] rdata);
    reg [7:0] mem [0:1023];
    always @(posedge clk)
        if (we) mem[addr] <= wdata;
        else if (en) rdata <= mem[addr];
endmodule
