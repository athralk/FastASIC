// UART, 8 data bits, no parity, 1 stop bit (8N1), for the full-flow benchmark.
// One top per baud rate at a 50 MHz clock: uart_115200, uart_57600, uart_921600.
//   transmit: present a byte on tx_data with tx_valid while tx_ready is high; txd idles high
//   receive:  rxd is synchronized by two flip-flops; each bit is sampled at its middle;
//             rx_valid pulses for one cycle with the byte, rx_frame_err if the stop bit was low
// Synchronous active-high reset.

module uart_tx #(parameter DIV = 434, parameter CW = 9) (
    input clk, rst,
    input [7:0] tx_data,
    input tx_valid,
    output tx_ready,
    output reg txd
);
    reg [CW-1:0] cnt;
    reg [3:0] bitn;      // 0: start bit, 1-8: data bits, 9: stop bit
    reg [7:0] shift;
    reg busy;
    assign tx_ready = !busy;
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; txd <= 1'b1; cnt <= 0; bitn <= 4'd0; shift <= 8'd0;
        end else if (!busy) begin
            txd <= 1'b1;
            if (tx_valid) begin
                busy <= 1'b1; shift <= tx_data; bitn <= 4'd0; cnt <= 0;
                txd <= 1'b0;                       // start bit
            end
        end else if (cnt == DIV - 1) begin
            cnt <= 0;
            if (bitn == 4'd9) begin
                busy <= 1'b0; txd <= 1'b1;
            end else begin
                bitn <= bitn + 4'd1;
                if (bitn == 4'd8) txd <= 1'b1;     // stop bit
                else begin
                    txd <= shift[0];
                    shift <= {1'b0, shift[7:1]};
                end
            end
        end else cnt <= cnt + 1'b1;
    end
endmodule

module uart_rx #(parameter DIV = 434, parameter CW = 9) (
    input clk, rst,
    input rxd,
    output reg [7:0] rx_data,
    output reg rx_valid,
    output reg rx_frame_err
);
    reg s0, s1;          // synchronizer
    reg [CW-1:0] cnt;
    reg [3:0] bitn;      // 0: start bit, 1-8: data bits, 9: stop bit
    reg [7:0] shift;
    reg busy;
    always @(posedge clk) begin
        if (rst) begin
            s0 <= 1'b1; s1 <= 1'b1; busy <= 1'b0; cnt <= 0; bitn <= 4'd0; shift <= 8'd0;
            rx_data <= 8'd0; rx_valid <= 1'b0; rx_frame_err <= 1'b0;
        end else begin
            s0 <= rxd; s1 <= s0;
            rx_valid <= 1'b0;
            if (!busy) begin
                if (!s1) begin                     // falling edge: start bit begins
                    busy <= 1'b1; bitn <= 4'd0; cnt <= DIV / 2;
                end
            end else if (cnt == DIV - 1) begin     // middle of a bit
                cnt <= 0;
                if (bitn == 4'd0) begin
                    if (s1) busy <= 1'b0;          // glitch, not a start bit
                    else bitn <= 4'd1;
                end else if (bitn == 4'd9) begin
                    busy <= 1'b0;
                    rx_data <= shift; rx_valid <= 1'b1; rx_frame_err <= !s1;
                end else begin
                    shift <= {s1, shift[7:1]};
                    bitn <= bitn + 4'd1;
                end
            end else cnt <= cnt + 1'b1;
        end
    end
endmodule

module uart #(parameter DIV = 434, parameter CW = 9) (
    input clk, rst,
    input [7:0] tx_data,
    input tx_valid,
    output tx_ready,
    output txd,
    input rxd,
    output [7:0] rx_data,
    output rx_valid,
    output rx_frame_err
);
    uart_tx #(.DIV(DIV), .CW(CW)) tx (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready), .txd(txd));
    uart_rx #(.DIV(DIV), .CW(CW)) rx (.clk(clk), .rst(rst), .rxd(rxd), .rx_data(rx_data), .rx_valid(rx_valid), .rx_frame_err(rx_frame_err));
endmodule

// 50 MHz / 115200 = 434 cycles per bit
module uart_115200 (input clk, rst, input [7:0] tx_data, input tx_valid, output tx_ready, output txd,
                    input rxd, output [7:0] rx_data, output rx_valid, output rx_frame_err);
    uart #(.DIV(434), .CW(9)) u (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready), .txd(txd),
                                 .rxd(rxd), .rx_data(rx_data), .rx_valid(rx_valid), .rx_frame_err(rx_frame_err));
endmodule

// 50 MHz / 57600 = 868 cycles per bit
module uart_57600 (input clk, rst, input [7:0] tx_data, input tx_valid, output tx_ready, output txd,
                   input rxd, output [7:0] rx_data, output rx_valid, output rx_frame_err);
    uart #(.DIV(868), .CW(10)) u (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready), .txd(txd),
                                  .rxd(rxd), .rx_data(rx_data), .rx_valid(rx_valid), .rx_frame_err(rx_frame_err));
endmodule

// 50 MHz / 921600 = 54 cycles per bit (0.47% fast, within the 8N1 tolerance)
module uart_921600 (input clk, rst, input [7:0] tx_data, input tx_valid, output tx_ready, output txd,
                    input rxd, output [7:0] rx_data, output rx_valid, output rx_frame_err);
    uart #(.DIV(54), .CW(6)) u (.clk(clk), .rst(rst), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready), .txd(txd),
                                .rxd(rxd), .rx_data(rx_data), .rx_valid(rx_valid), .rx_frame_err(rx_frame_err));
endmodule
