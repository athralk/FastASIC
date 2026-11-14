// a shift-and-add multiplier that drops partial product row 3
// SAT cannot prove equal in reasonable time (Yosys's sat times out at 8 x 8); SAT sweeping can
module m (input [5:0] a, b, output [11:0] p);
    function [11:0] shift_add(input [5:0] x, input [5:0] y);
        integer i;
        begin
            shift_add = 0;
            for (i = 0; i < 6; i = i + 1)
                if (y[i] && i != 3) shift_add = shift_add + ({6'd0, x} << i);
        end
    endfunction
    assign p = shift_add(a, b);
endmodule
