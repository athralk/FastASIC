module m (input s, input [3:0] a, b, output [3:0] y);
    assign y = ({4{s}} & a) | ({4{~s}} & b);
endmodule
