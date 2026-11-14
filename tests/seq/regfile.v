// 8 x 8 register file: one synchronous write port, two asynchronous read ports
module regfile (input clk, input we, input [2:0] waddr, raddr_a, raddr_b, input [7:0] wdata,
                output [7:0] rdata_a, rdata_b);
    reg [7:0] mem [0:7];
    always @(posedge clk)
        if (we) mem[waddr] <= wdata;
    assign rdata_a = mem[raddr_a];
    assign rdata_b = mem[raddr_b];
endmodule
