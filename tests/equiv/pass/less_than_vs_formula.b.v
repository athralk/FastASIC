module m (input [1:0] a, b, output lt);
    assign lt = (~a[1] & b[1]) | (~(a[1] ^ b[1]) & ~a[0] & b[0]);
endmodule
