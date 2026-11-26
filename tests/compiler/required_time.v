module required_time(input a, b, c, d, output y);
    assign y = (a ^ b) & (c ^ d);
endmodule
