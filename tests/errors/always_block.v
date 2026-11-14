// expect-error: 'y' is a wire: only regs can be assigned in always blocks
module m (input a, output y);
    always @(*) y = a;
endmodule
