module m (input s, input a, input b, output y);
    assign y = (s & b) | (~s & a);
endmodule
