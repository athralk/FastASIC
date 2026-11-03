// Fast corner (1.32 V, -40 C): pin B fall 24.280 + 2.8014 * 6.0 = 41.09 ps
// sta-args: --corner fast
// expect: sg13cmos5l_stdcell, fast 1.32V -40C
// expect: data arrival time 41.09
// expect: slack 9958.91 (MET)
module nand2_fast (input a, b, output y);
    assign y = ~(a & b);
endmodule
