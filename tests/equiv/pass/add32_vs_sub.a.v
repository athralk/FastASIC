// 64 inputs: far beyond exhaustive simulation, so only the SAT proof covers it
module m (input [31:0] a, b, output [31:0] y);
    assign y = a + b;
endmodule
