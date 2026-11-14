// Blocking temporaries inside a clocked block (combinational logic before the register),
// and a register file write with a variable bit index
module blocking_in_clocked (input clk, input [7:0] a, b, input [2:0] idx, input bit_in,
                            output reg [7:0] q, output reg [7:0] flags);
    reg [7:0] sum;
    always @(posedge clk) begin
        sum = a + b;
        q <= sum ^ {sum[3:0], sum[7:4]};
        flags[idx] <= bit_in;
    end
endmodule
