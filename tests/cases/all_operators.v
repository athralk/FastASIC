/* Every supported operator, precedence level and literal form */
module all_operators (input a, b, c, output y_and, y_or, y_xor, y_xnor1, y_xnor2, y_not, y_prec, y_const);
    assign y_and = a & b,
           y_or  = a | b;
    assign y_xor   = a ^ b;
    assign y_xnor1 = a ~^ b;
    assign y_xnor2 = a ^~ b;
    assign y_not   = ~(a | ~b);
    assign y_prec  = a | b & c ^ ~a;   // a | ((b & c) ^ ~a)
    assign y_const = (a & 1'b1) | (b & 1'b0) | (c & 1) | 0;
endmodule
