# Combinational block between registers elsewhere: a virtual clock
create_clock -name vclk -period 4.0
set_input_delay 0.5 -clock vclk [all_inputs]
set_output_delay 0.5 -clock vclk [all_outputs]
set_load 0.006 [all_outputs]
set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin X [all_inputs]
