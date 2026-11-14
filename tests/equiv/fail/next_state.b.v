// rotates the other way
module m (input clk, input [3:0] d, input load, output reg [3:0] q);
    always @(posedge clk) if (load) q <= d; else q <= {q[0], q[3:1]};
endmodule
