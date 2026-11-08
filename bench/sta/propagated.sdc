# Same as default.sdc, but the clock is propagated through the clock tree
set period 5.0
create_clock -name clk -period $period [get_ports clk]
set_propagated_clock [get_clocks clk]
set_clock_uncertainty 0.1 [get_clocks clk]
set_input_transition 0.15 [get_ports clk]
set_input_delay [expr $period * 0.25] -clock clk [all_inputs -no_clocks]
set_output_delay [expr $period * 0.25] -clock clk [all_outputs]
set_load 0.006 [all_outputs]
set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs -no_clocks]
