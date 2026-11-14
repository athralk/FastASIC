// 8-bit serial-in / parallel-out shift register, no reset
module shift_register (input clk, input d, output reg [7:0] q);
    always @(posedge clk) q <= {q[6:0], d};
endmodule
