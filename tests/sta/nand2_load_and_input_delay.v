// --load 10 --input-delay 100:   100 + 29.268 + 6.1195 * 10 = 190.46 ps
// sta-args: --load 10 --input-delay 100
// expect: data arrival time 190.46
// expect: output load 10 fF
module nand2_load_and_input_delay (input a, b, output y);
    assign y = ~(a & b);
endmodule
