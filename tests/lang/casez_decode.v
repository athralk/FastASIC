// casez priority encoder, casex with wildcards, a casez that covers every value without default
module casez_decode (input [7:0] req, input [3:0] op, output reg [2:0] grant, output reg valid,
                     output reg [1:0] cls, output reg [1:0] quad);
    always @(*) begin
        valid = 1'b1;
        casez (req)
            8'b???????1: grant = 3'd0;
            8'b??????10: grant = 3'd1;
            8'b?????100: grant = 3'd2;
            8'b????1000: grant = 3'd3;
            8'b???10000: grant = 3'd4;
            8'b??100000: grant = 3'd5;
            8'b?1000000: grant = 3'd6;
            8'b10000000: grant = 3'd7;
            default: begin grant = 3'd0; valid = 1'b0; end
        endcase
    end
    always @(*)
        casex (op)
            4'b1xxx: cls = 2'd3;
            4'b01xx: cls = 2'd2;
            4'b001x: cls = 2'd1;
            default: cls = 2'd0;
        endcase
    always @(*)
        casez (op[1:0])
            2'b0?: quad = op[3:2];
            2'b1?: quad = ~op[3:2];
        endcase
endmodule
