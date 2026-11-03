module m (input [7:0] a, output [7:0] y, output [7:0] z);
    assign y = {a[5:0], 2'b00};
    assign z = {3'b000, a[7:3]};
endmodule
