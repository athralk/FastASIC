module loop_clobber(input [3:0] a, output reg [3:0] y);
  integer i;
  function [0:0] id1(input [0:0] x);
    integer i;
    begin
      id1 = 1'b0;
      for (i = 0; i < 2; i = i + 1) id1 = x;
    end
  endfunction
  always @(*) begin
    y = 4'd0;
    for (i = 0; i < 4; i = i + 1) begin
      y[i] = id1(a[i]);
    end
  end
endmodule
