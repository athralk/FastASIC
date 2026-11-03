// One NAND driving the default 33.442 fF output load. The slower pin B sets the critical rise:
//   29.268 + 6.1195 * 33.442 = 233.92 ps
// expect: Critical path: b -> y (1 cell)
// expect: data arrival time 233.92
// expect: slack 9766.08 (MET)
module nand2_rc (input a, b, output y);
    assign y = ~(a & b);
endmodule
