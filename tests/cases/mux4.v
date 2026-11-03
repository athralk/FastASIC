module mux4 #(parameter W = 4) (
    input  [W-1:0] d0, d1, d2, d3,
    input  [1:0]   sel,
    output [W-1:0] y
);
    assign y = sel[1] ? (sel[0] ? d3 : d2) : (sel[0] ? d1 : d0);
endmodule
