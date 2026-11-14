// formerly an unsupported-feature error test
module m (input [7:0] a, input [2:0] s, output [7:0] y);
    assign y = a << s;
endmodule
