module m (input [255:0] a, b, output [255:0] y);
    assign y = a - ~b - 1;
endmodule
