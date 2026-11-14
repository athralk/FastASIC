// Functions (with a loop, a call inside a call, a signed result) and a task with outputs
module functions (input [7:0] x, y, input signed [5:0] s, output [3:0] ones, output [7:0] m,
                  output signed [7:0] sq, output reg [7:0] lo, output reg [7:0] hi);
    function [3:0] popcount(input [7:0] v);
        integer i;
        begin
            popcount = 0;
            for (i = 0; i < 8; i = i + 1) popcount = popcount + v[i];
        end
    endfunction
    function [7:0] max2;
        input [7:0] p, q;
        max2 = p > q ? p : q;
    endfunction
    function signed [7:0] scaled;
        input signed [5:0] v;
        scaled = v * 3 - 1;
    endfunction
    task order(input [7:0] p, input [7:0] q, output [7:0] lo_v, output [7:0] hi_v);
        begin
            lo_v = p < q ? p : q;
            hi_v = max2(p, q);
        end
    endtask
    assign ones = popcount(x ^ y);
    assign m = max2(popcount(x), y);
    assign sq = scaled(s);
    always @(*) order(x, y, lo, hi);
endmodule
