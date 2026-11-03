// b is declared but unused: the checker still treats it as a free input
module m (input a, input b, output y);
    assign y = a;
endmodule
