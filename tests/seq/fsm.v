// Moore FSM (sequence detector for 1011) with a reset to a non-zero state:
// next-state logic in a combinational always with case, state register with async reset
module fsm (input clk, input rst_n, input x, output reg found);
    localparam IDLE = 3'd5, S1 = 3'd1, S10 = 3'd2, S101 = 3'd3, S1011 = 3'd4;
    reg [2:0] state, next;
    always @(*) begin
        next = IDLE;
        case (state)
            IDLE:  next = x ? S1 : IDLE;
            S1:    next = x ? S1 : S10;
            S10:   next = x ? S101 : IDLE;
            S101:  next = x ? S1011 : S10;
            S1011: next = x ? S1 : S10;
            default: next = IDLE;
        endcase
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= next;
    always @(*) found = (state == S1011);
endmodule
