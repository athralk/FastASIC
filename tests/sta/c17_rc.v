// ISCAS-85 c17. Critical path N6 -> N11 -> N16 -> N22, each internal net loading two NAND pins
// (2.315 + 2.324 = 4.639 fF):
//   (29.268 + 6.1195 * 4.639) + (23.156 + 5.0397 * 4.639) + (29.268 + 6.1195 * 33.442) = 338.11 ps
// expect: Critical path: N6 -> N22 (3 cells)
// expect: 57.66 57.66 ^ 4.64
// expect: 46.54 104.19 v 4.64
// expect: data arrival time 338.11
// expect: N23 ^ 332.84 10000.00 9667.16
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
