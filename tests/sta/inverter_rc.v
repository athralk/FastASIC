// One inverter: rise 19.589 + 2.7682 * 6.0 = 36.20 ps beats fall 19.701 + 2.2105 * 6.0 = 32.96 ps
// expect: Critical path: a -> y (1 cell)
// expect: data arrival time 36.20
// expect: y ^ 36.20
module inverter_rc (input a, output y);
    assign y = ~a;
endmodule
