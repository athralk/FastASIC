// The A pin is smaller than a via landing pad; the a -> u/A net cannot be routed.
module inaccessible_pin(input a, output y);
    inaccessible u (.A(a), .X(y));
endmodule
