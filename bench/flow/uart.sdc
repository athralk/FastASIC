# 50 MHz clock on clk; inputs arrive and outputs are needed at the clock edge
create_clock -name clk -period 20 [get_ports clk]
set_input_delay 0 -clock clk [all_inputs -no_clocks]
set_output_delay 0 -clock clk [all_outputs]
set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs -no_clocks]
set_load 0.006 [all_outputs]
