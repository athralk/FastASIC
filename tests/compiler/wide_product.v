module wide_product(input [32:0] a, b, output [65:0] product,
                    output signed [65:0] signed_product);
    assign product = a * b;
    assign signed_product = $signed(a) * $signed(b);
endmodule
