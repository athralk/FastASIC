// formerly an unsupported-feature error test
module m (input [1:0] s, output reg y);
    always @(*) casez (s) 2'b1?: y = 1; default: y = 0; endcase
endmodule
