// Outputs the optimizer should reduce to constants or plain wires
module constant_folding (input a, input b, output y0, output y1, output y_b, output y_a);
    assign y0  = a & ~a;
    assign y1  = a | 1'b1;
    assign y_b = (a ^ a) | b;
    assign y_a = ~~a;
endmodule
