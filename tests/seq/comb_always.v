// Combinational always blocks: a case covering every value (no default needed), a default
// assignment followed by an if, and blocking temporaries
module comb_always (input [1:0] sel, input [3:0] a, b, c, d, input en, output reg [3:0] y, output reg [3:0] z,
                    output reg flag);
    reg [3:0] t;
    always @(*) begin
        case (sel)
            2'd0: y = a;
            2'd1: y = b;
            2'd2: y = c;
            2'd3: y = d;
        endcase
    end
    always @(*) begin
        z = 4'd0;
        if (en) z = a ^ b;
    end
    always @(a or b) begin
        t = a + b;
        flag = t[3] | (t == 4'd0);
    end
endmodule
