// Non-ANSI port style
module full_adder (a, b, cin, sum, cout);
    input a, b, cin;
    output sum, cout;
    wire p;
    assign p = a ^ b;
    assign sum = p ^ cin;
    assign cout = (a & b) | (p & cin);
endmodule
