// One NAND driving the default 6.0 fF output load. In sg13cmos5l the pull-down (two NMOS in
// series) is the weak side, so the slower pin B sets a critical *fall*:
//   32.346 + 4.0796 * 6.0 = 56.82 ps     (B rise would be 25.111 + 2.7683 * 6.0 = 41.72 ps)
// expect: Critical path: b -> y (1 cell)
// expect: sg13cmos5l_stdcell, typ 1.20V 25C
// expect: data arrival time 56.82
// expect: slack 9943.18 (MET)
module nand2_rc (input a, b, output y);
    assign y = ~(a & b);
endmodule
