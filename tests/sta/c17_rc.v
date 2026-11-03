// ISCAS-85 c17. The optimizer puts N3/N6 on N11's A/B pins, N2/N11 on N16's, N10/N16 on N22's,
// so every internal net drives one A and one B pin: 2.886 + 2.978 = 5.864 fF.
// Critical path N6 ^ -> N11 v -> N16 ^ -> N22 v:
//   N11 fall via B: 32.346 + 4.0796 * 5.864 = 56.27 ps
//   N16 rise via B: 25.111 + 2.7683 * 5.864 = 41.34 ps   (arrival 97.61)
//   N22 fall via B: 32.346 + 4.0796 * 6.0   = 56.82 ps   (arrival 154.44)
// N23's worst is a fall from N16 rising on pin A: 97.61 + 28.063 + 4.0814 * 6.0 = 150.16 ps
// expect: Critical path: N6 -> N22 (3 cells)
// expect: 56.27 56.27 v 5.86
// expect: 41.34 97.61 ^ 5.86
// expect: data arrival time 154.44
// expect: N23 v 150.16 10000.00 9849.84
module c17 (N1, N2, N3, N6, N7, N22, N23);
    input N1, N2, N3, N6, N7;
    output N22, N23;
    wire N10, N11, N16, N19;
    assign N10 = ~(N1 & N3);
    assign N11 = ~(N3 & N6);
    assign N16 = ~(N2 & N11);
    assign N19 = ~(N11 & N7);
    assign N22 = ~(N10 & N16);
    assign N23 = ~(N16 & N19);
endmodule
