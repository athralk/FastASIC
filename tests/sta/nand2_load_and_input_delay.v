// --load 10 --input-delay 100:   100 + 32.346 + 4.0796 * 10 = 173.14 ps
// sta-args: --load 10 --input-delay 100
// expect: data arrival time 173.14
// expect: output load 10 fF
module nand2_load_and_input_delay (input a, b, output y);
    assign y = ~(a & b);
endmodule
