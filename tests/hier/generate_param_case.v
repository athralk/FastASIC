module param_child #(parameter [7:0] A=0, parameter B=0)(output [7:0] a, b);
    assign a=A, b=B;
endmodule
module parameters(output [7:0] a,b, output c,d);
    localparam P=-4'd1;
    param_child #(.A(P), .B(P)) child(a,b);
    generate
        case (3'sb111)
           -4'sd1: assign c=1;
           3'd2: assign c=1;
           default: assign c=0;
        endcase
        case (70'h200000000000000001)
           70'h200000000000000001: assign d=1;
           default: assign d=0;
        endcase
    endgenerate
endmodule
