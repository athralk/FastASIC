// Loopback test of the three UART tops: txd drives rxd, every byte sent must come back with no
// framing error. A bad frame (stop bit held low) must be flagged.
// usage: iverilog -g2005 -o uart_tb bench/flow/uart.v bench/flow/uart_tb.v && vvp uart_tb
`timescale 1ns/1ps
module uart_tb;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;   // 50 MHz

    reg [7:0] tx_data;
    reg tx_valid = 0;
    reg force_low = 0;       // pulls the line low to break a frame
    wire [2:0] ready, txd, valid, ferr;
    wire [7:0] rx0, rx1, rx2;
    uart_115200 a (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(ready[0]), .txd(txd[0]),
                   .rxd(txd[0] & !force_low), .rx_data(rx0), .rx_valid(valid[0]), .rx_frame_err(ferr[0]));
    uart_57600  b (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(ready[1]), .txd(txd[1]),
                   .rxd(txd[1] & !force_low), .rx_data(rx1), .rx_valid(valid[1]), .rx_frame_err(ferr[1]));
    uart_921600 c (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(ready[2]), .txd(txd[2]),
                   .rxd(txd[2] & !force_low), .rx_data(rx2), .rx_valid(valid[2]), .rx_frame_err(ferr[2]));

    integer errors = 0, received = 0;
    reg [7:0] expect_byte;
    reg expect_err = 0;
    always @(posedge clk) begin
        if (valid[0]) begin received = received + 1; if (rx0 !== expect_byte || ferr[0] !== expect_err) begin errors = errors + 1; $display("115200: got %h err %b, want %h err %b", rx0, ferr[0], expect_byte, expect_err); end end
        if (valid[1]) begin received = received + 1; if (rx1 !== expect_byte || ferr[1] !== expect_err) begin errors = errors + 1; $display("57600: got %h err %b, want %h err %b", rx1, ferr[1], expect_byte, expect_err); end end
        if (valid[2]) begin received = received + 1; if (rx2 !== expect_byte || ferr[2] !== expect_err) begin errors = errors + 1; $display("921600: got %h err %b, want %h err %b", rx2, ferr[2], expect_byte, expect_err); end end
    end

    // Send one byte on all three at once and wait until all are idle again
    task send(input [7:0] d);
        begin
            @(negedge clk); tx_data = d; tx_valid = 1; expect_byte = d;
            @(negedge clk); tx_valid = 0;
            wait (ready == 3'b111);
            repeat (2000) @(negedge clk);   // let the receivers finish their stop bit
        end
    endtask

    integer k, before;
    reg [7:0] r;
    initial begin
        repeat (5) @(negedge clk); rst = 0;
        repeat (5) @(negedge clk);
        send(8'h55); send(8'hA3); send(8'h00); send(8'hFF); send(8'h01); send(8'h80);
        for (k = 0; k < 10; k = k + 1) begin r = $random; send(r); end
        if (received != 16 * 3) begin errors = errors + 1; $display("received %0d bytes, want %0d", received, 16 * 3); end
        // framing error: hold the line low from the middle of the 921600 stop bit on
        before = received;
        expect_err = 1; expect_byte = 8'hC3;
        @(negedge clk); tx_data = 8'hC3; tx_valid = 1; @(negedge clk); tx_valid = 0;
        repeat (54 * 9 + 27) @(negedge clk); force_low = 1;
        repeat (60) @(negedge clk);
        if (received != before + 1 || !ferr[2]) begin errors = errors + 1; $display("921600 framing error not flagged"); end
        if (errors == 0) $display("PASS: 48 bytes looped back at 115200, 57600 and 921600 baud; framing error flagged");
        else $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
