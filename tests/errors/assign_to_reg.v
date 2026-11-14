// expect-error: 'r' is a reg: assign it in an always block, not with assign
module m (input a, output y);
    reg r;
    assign r = a;
    assign y = r;
endmodule
