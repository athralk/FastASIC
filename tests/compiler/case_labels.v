module case_labels(input signed [2:0] s, output reg [2:0] result,
                   output reg [1:0] mixed, z_match, x_match);
    always @* begin
        result = 0;
        case (s[1:0])
            3'd0: result = 1;
            3'd1: result = 2;
            3'd2: result = 3;
            3'd7: result = 4;
        endcase
        case (s)
            -4'sd1: mixed = 1;
            3'd2: mixed = 2;
            default: mixed = 0;
        endcase
        casez (s)
            3'b1x?: z_match = 1;
            3'b1z?: z_match = 2;
            default: z_match = 0;
        endcase
        casex (s)
            3'b1x?: x_match = 1;
            default: x_match = 0;
        endcase
    end
endmodule
