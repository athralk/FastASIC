module m (input a, input b, output y);
    wire t1, t2;
    assign t1 = a & ~b;
    assign t2 = ~a & b;
    assign y = t1 | t2;
endmodule
