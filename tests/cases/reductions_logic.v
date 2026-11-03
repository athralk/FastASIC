module reductions_logic (
    input  [4:0] v, w,
    output r_and, r_nand, r_or, r_nor, r_xor, r_xnor,
    output l_not, l_and, l_or,
    output ne, ge, le, gt
);
    assign r_and  = &v;
    assign r_nand = ~&v;
    assign r_or   = |v;
    assign r_nor  = ~|v;
    assign r_xor  = ^v;
    assign r_xnor = ~^v;
    assign l_not  = !v;
    assign l_and  = v && w;
    assign l_or   = v || w;
    assign ne     = v != w;
    assign ge     = v >= w;
    assign le     = v <= w;
    assign gt     = v > w;
endmodule
