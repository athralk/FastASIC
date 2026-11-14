// expect-error: negedge rst_n is an active-low reset: test it as !rst_n
module m (input clk, input rst_n, input d, output reg q);
    always @(posedge clk or negedge rst_n)
        if (rst_n) q <= 1'b0;
        else q <= d;
endmodule
