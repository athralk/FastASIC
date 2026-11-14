// yosys-equiv: skip (multiplier: checked by co-simulation)
// formerly an unsupported-feature error test
module m (input [3:0] a, b, output [7:0] y);
    assign y = a * b;
endmodule
