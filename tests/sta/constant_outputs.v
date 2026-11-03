// Constant outputs never switch; outputs wired straight to inputs arrive at the input delay
// expect: (2 constant outputs never switch and are not timed)
// expect: Cells: 0
module constant_outputs (input a, input b, output y0, output y1, output y_b, output y_a);
    assign y0  = a & ~a;
    assign y1  = a | 1'b1;
    assign y_b = (a ^ a) | b;
    assign y_a = ~~a;
endmodule
