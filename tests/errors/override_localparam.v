// expect-error: module 'child' has no parameter 'L' that can be overridden
module child (input a, output y); localparam L = 1; assign y = a; endmodule
module m (input x, output z); child #(.L(2)) u (.a(x), .y(z)); endmodule
