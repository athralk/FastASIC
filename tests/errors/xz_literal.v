// expect-error: x/z/? digits are only allowed in casez / casex labels
module m (input [3:0] a, output [3:0] y);
    assign y = a & 4'b10x1;
endmodule
