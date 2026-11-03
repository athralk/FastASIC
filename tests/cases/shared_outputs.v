// Identical logic on two outputs, and an output read by another output
module shared_outputs (input a, input b, input c, output y1, output y2, output y3);
    assign y1 = a & b;
    assign y2 = b & a;
    assign y3 = y1 | c;
endmodule
