module rc_test(input clk, input d, output q);
  wire n1, n2;
  sg13cmos5l_dfrbpq_1 r1 (.CLK(clk), .RESET_B(1'b1), .D(d), .Q(n1));
  sg13cmos5l_buf_1 b1 (.A(n1), .X(n2));
  sg13cmos5l_dfrbpq_1 r2 (.CLK(clk), .RESET_B(1'b1), .D(n2), .Q(q));
endmodule
