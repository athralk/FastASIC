module mux2 (input wire s, input wire a, input wire b, output wire y);
    assign y = (s & b) | (~s & a);
endmodule
