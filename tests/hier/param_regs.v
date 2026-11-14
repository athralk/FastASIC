// Parameterized register with enable, overridden by name and by position, three levels deep;
// the reset reaches the flops through two levels of ports, one instance has it tied off
module en_reg #(parameter W = 4, parameter [W-1:0] INIT = 0) (input clk, input rst_n, input en,
                                                              input [W-1:0] d, output reg [W-1:0] q);
    localparam unused = W * 2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) q <= INIT;
        else if (en) q <= d;
endmodule

module stage #(parameter W = 4) (input clk, input rst_n, input [W-1:0] d, output [W-1:0] q);
    wire [W-1:0] mid;
    en_reg #(.W(W), .INIT(5)) r0 (.clk(clk), .rst_n(rst_n), .en(1'b1), .d(d), .q(mid));
    en_reg #(W) r1 (clk, rst_n, d[0], mid ^ d, q);
endmodule

module param_regs (input clk, input rst_n, input [7:0] x, output [7:0] y, output [2:0] z);
    stage #(.W(8)) s8 (.clk(clk), .rst_n(rst_n), .d(x), .q(y));
    en_reg #(.W(3)) free (.clk(clk), .rst_n(1'b1), .en(x[7]), .d(x[2:0]), .q(z));
endmodule
