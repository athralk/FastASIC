// 4 x 4 memory cleared by a synchronous reset in a for loop, with a registered read
module memory_clear (input clk, input clr, input we, input [1:0] addr, input [3:0] din, output reg [3:0] dout);
    reg [3:0] mem [0:3];
    integer i;
    always @(posedge clk) begin
        if (clr) begin
            for (i = 0; i < 4; i = i + 1) mem[i] <= 4'd0;
        end else if (we) begin
            mem[addr] <= din;
        end
        dout <= mem[addr];
    end
endmodule
