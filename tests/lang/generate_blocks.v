// generate: a for loop of named blocks with local wires and instances, if / case on
// parameters, a nested loop, a localparam inside a generate block
module full_add (input a, b, c, output s, co);
    assign s = a ^ b ^ c;
    assign co = (a & b) | (c & (a ^ b));
endmodule

module generate_blocks #(parameter N = 6, parameter MODE = 1) (input [N-1:0] a, b, input cin,
                        output [N-1:0] sum, output cout, output [N-1:0] mode_out, output [3:0] tri_out);
    wire [N:0] carry;
    assign carry[0] = cin;
    genvar i, j;
    generate
        for (i = 0; i < N; i = i + 1) begin : stage
            wire local_x = a[i] ^ b[i];
            full_add fa (.a(a[i]), .b(b[i]), .c(carry[i]), .s(sum[i]), .co(carry[i + 1]));
            if (MODE == 1) begin : m1
                assign mode_out[i] = local_x & carry[i];
            end else begin : m0
                assign mode_out[i] = ~local_x;
            end
        end
        case (MODE)
            0: assign cout = 1'b0;
            default: assign cout = carry[N];
        endcase
        for (i = 0; i < 2; i = i + 1) begin : outer
            for (j = 0; j < 2; j = j + 1) begin : inner
                localparam K = 2 * i + j;
                assign tri_out[K] = a[K] & b[(K + 1) % N];
            end
        end
    endgenerate
endmodule
