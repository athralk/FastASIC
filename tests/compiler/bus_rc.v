module bus_rc(a, y);
    input [1:0] a;
    output [1:0] y;
    wire alias_a;
    assign alias_a = a[0];
    wire [1:0] stage;
    wire \literal[3] ;
    sg13cmos5l_inv_1 \i[0]_buf (.A(alias_a), .Y(stage[0]));
    sg13cmos5l_inv_1 i1 (.A(stage[0]), .Y(\literal[3] ));
    sg13cmos5l_inv_1 i2 (.A(\literal[3] ), .Y(y[0]));
    sg13cmos5l_inv_1 i3 (.A(a[1]), .Y(stage[1]));
    sg13cmos5l_inv_1 i4 (.A(stage[1]), .Y(y[1]));
endmodule
