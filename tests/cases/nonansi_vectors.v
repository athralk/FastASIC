// Non-ANSI ports with vectors, a wire redeclaration of an output, and a wire driven in pieces
module nonansi_vectors (a, b, y);
    input  [3:0] a;
    input  [3:0] b;
    output [4:0] y;
    wire   [4:0] y;
    wire   [3:0] t;
    assign t[1:0] = a[1:0] & b[1:0];
    assign t[3:2] = a[3:2] | b[3:2];
    assign y = {1'b0, t} + 5'd1;
endmodule
