// expect-error: unknown module 'inverter'
module m (input a, output y);
    inverter u1 (.a(a), .y(y));
endmodule
