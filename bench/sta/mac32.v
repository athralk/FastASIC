// 32-bit multiply-accumulate with synchronous clear
module mac32 (input clk, input clr, input [31:0] a, b, output reg [63:0] acc);
    always @(posedge clk)
        if (clr) acc <= 0;
        else acc <= acc + a * b;
endmodule
