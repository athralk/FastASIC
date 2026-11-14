// Every register style: async active-high reset to 1s, sync reset with enable, falling edge
module reset_styles (input clk, input rst, input srst, input en, input [3:0] d,
                     output reg [3:0] q_set, output reg [3:0] q_sync, output reg q_neg);
    always @(posedge clk or posedge rst)
        if (rst) q_set <= 4'b1010;
        else q_set <= d;
    always @(posedge clk)
        if (srst) q_sync <= 4'd0;
        else if (en) q_sync <= q_sync + d;
    always @(negedge clk) q_neg <= ^d;
endmodule
