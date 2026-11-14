// expect-error: instantiates itself
module top (input a, output y); loop u (.a(a), .y(y)); endmodule
module loop (input a, output y); loop again (.a(a), .y(y)); endmodule
