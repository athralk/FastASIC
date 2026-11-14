// Same counter: next state in a combinational block, a mux instead of if/else
module m (input clk, input rst_n, input en, output reg [3:0] q);
    reg [3:0] next;
    always @(*) next = en ? q - 4'b1111 : q;
    always @(posedge clk or negedge rst_n)
        if (rst_n == 1'b0) q <= 0;
        else q <= next;
endmodule
