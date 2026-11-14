// expect-note: memory 'big' (512 x 8) kept as flip-flops: it is read asynchronously
// Stays as flip-flops: a 16 x 8 memory (smaller than any SRAM macro) and a 512 x 8 memory
// with an asynchronous read (an SRAM's read data is registered)
module small_and_async (input clk, input we, input [3:0] a, input [8:0] b, input [7:0] d,
                        output reg [7:0] q, output [7:0] y);
    reg [7:0] tiny [0:15];
    reg [7:0] big [0:511];
    always @(posedge clk) begin
        if (we) tiny[a] <= d;
        q <= tiny[a];
        if (we) big[b] <= d;
    end
    assign y = big[b];
endmodule
