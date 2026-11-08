// 16-bit accumulator with asynchronous active-low reset
module acc16 (input clk, input rst_n, input [15:0] in, output reg [15:0] acc);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) acc <= 16'd0;
        else acc <= acc + in;
endmodule
