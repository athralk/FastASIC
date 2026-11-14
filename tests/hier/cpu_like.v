// A small datapath: register file submodule (memory), ALU submodule, accumulator, all
// connected through ports; the regfile is instantiated twice
module regfile (input clk, input we, input [1:0] waddr, raddr, input [7:0] wdata, output [7:0] rdata);
    reg [7:0] mem [0:3];
    always @(posedge clk) if (we) mem[waddr] <= wdata;
    assign rdata = mem[raddr];
endmodule

module alu (input [1:0] op, input [7:0] a, b, output reg [7:0] y);
    always @(*)
        case (op)
            2'd0: y = a + b;
            2'd1: y = a - b;
            2'd2: y = a & b;
            2'd3: y = a ^ b;
        endcase
endmodule

module cpu_like (input clk, input rst_n, input we, input [1:0] waddr, raddr_a, raddr_b, op,
                 input [7:0] wdata, output reg [7:0] acc);
    wire [7:0] a, b, y;
    regfile rf_a (.clk(clk), .we(we), .waddr(waddr), .raddr(raddr_a), .wdata(wdata), .rdata(a));
    regfile rf_b (.clk(clk), .we(we), .waddr(waddr), .raddr(raddr_b), .wdata(wdata), .rdata(b));
    alu u_alu (.op(op), .a(a), .b(b), .y(y));
    always @(posedge clk or negedge rst_n)
        if (!rst_n) acc <= 8'd0;
        else acc <= y;
endmodule
