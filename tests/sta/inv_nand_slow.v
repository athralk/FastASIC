// Slow corner (1.08 V, 125 C). The optimizer builds NAND(A = ~a, B = b), so the inverter
// drives the slow-corner pin A cap of 2.722 fF:
//   inverter rise: 26.035 + 4.3306 * 2.722 = 37.82 ps
//   NAND fall via A: 40.169 + 6.7692 * 6.0 = 80.78 ps   (arrival 118.61; via B only 87.54)
// sta-args: --corner slow
// expect: sg13cmos5l_stdcell, slow 1.08V 125C
// expect: 37.82 37.82 ^ 2.72
// expect: 80.78 118.61 v 6.00
// expect: data arrival time 118.61
module inv_nand_slow (input a, b, output y);
    assign y = ~(~a & b);
endmodule
