// expect-error: the asynchronous reset value of 'q' must be a constant
module m (input clk, input rst_n, input d, input v, output reg q);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) q <= v;
        else q <= d;
endmodule
