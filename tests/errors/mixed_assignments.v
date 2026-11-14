// expect-error: 'q' is assigned with both = and <= in one block
module m (input clk, input a, input b, output reg q);
    always @(posedge clk) begin
        q = a;
        q <= b;
    end
endmodule
