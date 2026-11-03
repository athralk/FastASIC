// Wires may be read before the assign that drives them
module use_before_define (input a, input b, output y);
    wire t1, t2;
    assign y = t1 ^ t2;
    assign t2 = ~a & b;
    assign t1 = a & ~b;
endmodule
