// Vector declarations, bit/part/indexed selects, concatenation and replication
module vectors_bitwise (
    input  [7:0]  a, b,
    input  [0:3]  asc,              // ascending range: asc[0] is the most significant bit
    output [7:0]  y_and, y_or, y_xor, y_not,
    output [3:0]  y_part,
    output        y_bit,
    output [11:0] y_cat,
    output [7:0]  y_rep,
    output [3:0]  y_asc,
    output [3:0]  y_up, y_down
);
    assign y_and  = a & b;
    assign y_or   = a | b;
    assign y_xor  = a ^ b;
    assign y_not  = ~a;
    assign y_part = a[6:3];
    assign y_bit  = b[7];
    assign y_cat  = {a[3:0], b};
    assign y_rep  = {4{a[1:0]}};
    assign y_asc  = {asc[3], asc[1:2], asc[0]};
    assign y_up   = a[2 +: 4];
    assign y_down = b[7 -: 4];
endmodule
