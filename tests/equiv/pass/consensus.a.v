module m (input s, input a, input b, output y);
    assign y = (s & a) | (~s & b) | (a & b);
endmodule
