// 8-bit counter with enable, asynchronous active-low reset and a terminal-count flag
module counter (input clk, input rst_n, input en, output reg [7:0] count, output tc);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) count <= 8'd0;
        else if (en) count <= count + 1;
    assign tc = &count;
endmodule
