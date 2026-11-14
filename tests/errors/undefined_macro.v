// expect-error: undefined macro `WIDTH
module m (input [`WIDTH-1:0] a, output y);
    assign y = ^a;
endmodule
