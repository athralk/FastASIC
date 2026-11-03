// expect-error: output 'z' is never assigned
module m (input a, output y, output z);
    assign y = a;
endmodule
