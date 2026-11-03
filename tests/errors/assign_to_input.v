// expect-error: cannot assign to input
module m (input a, output y);
    assign a = 1'b1;
    assign y = a;
endmodule
