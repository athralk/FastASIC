// expect-error: combinational loop
module m (input a, output y);
    wire p, q;
    assign p = q & a;
    assign q = ~p;
    assign y = p;
endmodule
