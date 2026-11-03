// expect-error: index 8 is out of range for 'a[7:0]'
module m (input [7:0] a, output y);
    assign y = a[8];
endmodule
