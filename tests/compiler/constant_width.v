module constant_width(input [7:0] a, output [7:0] div, cmp, shifted, param_value,
                      output [7:0] override_value, log_neg, log_wide, padded, output [69:0] wide_div, wide_rem);
    // A ranged parameter supplies an assignment width to the override expression.
    ranged_constant #(.VALUE(-4'd1)) child(override_value);
    localparam Q = -4'd1;
    localparam N = 8;
    assign div = (~4'b0000) / 8'd2;
    assign cmp = 8'd1 * ((4'd1 - 4'd2) < 4'd0);
    assign shifted = a >> (4'd15 + 4'd1);
    assign param_value = Q;
    assign log_neg = $clog2(-1);
    assign log_wide = $clog2(70'h200000000000000001);
    assign padded = {{(8-N){1'b0}}, a, {0{1'b1}}};
    assign wide_div = -70'sh15000000000000abce / 70'sd19;
    assign wide_rem = -70'sh15000000000000abce % 70'sd19;
endmodule

module ranged_constant #(parameter [7:0] VALUE = 0)(output [7:0] y);
    assign y = VALUE;
endmodule
