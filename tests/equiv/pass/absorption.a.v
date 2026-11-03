module m (input a, input b, output y);
    assign y = a | (a & b);
endmodule
