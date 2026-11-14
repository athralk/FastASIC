// yosys-equiv: skip (two multiplier structures are too hard to prove equal with SAT: checked by co-simulation)
// Multipliers: unsigned and signed full products, a truncated product, odd sizes,
// constant multiplication, and a multiply-accumulate expression
module multipliers (input [7:0] a, b, input signed [7:0] sa, sb, input [5:0] c, input [4:0] d,
                    output [15:0] p_u, output signed [15:0] p_s, output [7:0] p_trunc,
                    output [10:0] p_odd, output [11:0] p_const, output [16:0] mac);
    assign p_u = a * b;
    assign p_s = sa * sb;
    assign p_trunc = a * b;          // only the low 8 bits
    assign p_odd = c * d;
    assign p_const = a * 4'd13;
    assign mac = a * b + {c, d};
endmodule
