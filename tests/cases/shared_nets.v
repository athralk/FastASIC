// A net used in several places must become one node with several fanouts
module shared_nets (input a, input b, input c, output y);
    wire n;
    assign n = a & b;
    assign y = n | (a & c);
endmodule
