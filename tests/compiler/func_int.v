module func_int(input [3:0] a, output reg [3:0] y);
    integer i;
    function [3:0] rev(input [3:0] x);
        integer i;
        begin
            for (i = 0; i < 4; i = i + 1) rev[i] = x[3 - i];
        end
    endfunction
    wire [3:0] t = rev(a);
    always @(*) begin
        for (i = 0; i < 4; i = i + 1) y[i] = t[i];
    end
endmodule
