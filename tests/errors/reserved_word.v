// expect-error: 'small' is a reserved word in Verilog
module m (input [1:0] small, output y);
    assign y = ^small;
endmodule
