// expect-error: expression width 0 is out of range
module empty_replication(output y);
    assign y = {0{1'b1}};
endmodule
