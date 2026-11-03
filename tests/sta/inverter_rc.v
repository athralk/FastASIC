// One inverter: rise 18.779 + 5.6270 * 33.442 = 206.96 ps beats fall 13.315 + 3.3299 * 33.442 = 124.67 ps
// expect: Critical path: a -> y (1 cell)
// expect: data arrival time 206.96
// expect: y ^ 206.96
module inverter_rc (input a, output y);
    assign y = ~a;
endmodule
