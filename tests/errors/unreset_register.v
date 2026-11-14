// expect-error: register 'q2' is not given a value in the reset branch
module m (input clk, input rst_n, input d, output reg q1, output reg q2);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) q1 <= 1'b0;
        else begin q1 <= d; q2 <= q1; end
endmodule
