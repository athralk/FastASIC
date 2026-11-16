// A cell netlist that uses a cell the LEF does not have: fastpnr must refuse it
module unknown_cell(input a, output y);
  not_a_real_cell u0 (.A(a), .Y(y));
endmodule
