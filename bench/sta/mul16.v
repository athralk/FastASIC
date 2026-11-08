// Registered 16x16 multiplier: input registers -> combinational multiply -> output register
module mul16 (input clk, input rst_n, input [15:0] a, b, output reg [31:0] p);
    reg [15:0] ra, rb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ra <= 0; rb <= 0; p <= 0; end
        else begin ra <= a; rb <= b; p <= ra * rb; end
endmodule
