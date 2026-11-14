// expect-error: integer 'i' can only be used as a for-loop variable
module m (input [3:0] a, output y);
    integer i;
    assign y = a[i];
endmodule
