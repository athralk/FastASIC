// expect-error: module instances are not supported yet
module m (input a, output y);
    inverter u1 (.a(a), .y(y));
endmodule
