// UART transmitter, 8-N-1, LSB first, idle high. One bit period is
// CLOCKS_PER_PULSE clocks. Pulse data_en for one clock with data_in valid;
// tx_busy stays high until the stop bit has been held a full bit period.

`timescale 1ns / 1ps

module uart_tx #(
    parameter CLOCKS_PER_PULSE = 87,        // clk cycles per bit period
    parameter DATA_WIDTH       = 8
)(
    input  wire                  clk,
    input  wire                  rstn,      // async reset, active low
    input  wire [DATA_WIDTH-1:0] data_in,   // byte to send
    input  wire                  data_en,   // pulse to start a transfer
    output reg                   tx,        // serial output line
    output wire                  tx_busy    // high while a byte is in flight
);

    localparam TX_IDLE  = 2'b00,
               TX_START = 2'b01,
               TX_DATA  = 2'b11,
               TX_END   = 2'b10;

    reg [1:0]                          state;
    reg [DATA_WIDTH-1:0]               data;
    // Holds DATA_WIDTH, not DATA_WIDTH-1, so the terminal count fits.
    reg [$clog2(DATA_WIDTH+1)-1:0]     c_bits;
    reg [$clog2(CLOCKS_PER_PULSE)-1:0] c_clocks;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state    <= TX_IDLE;
            data     <= {DATA_WIDTH{1'b0}};
            c_bits   <= 0;
            c_clocks <= 0;
            tx       <= 1'b1;               // line idles high
        end else begin
            case (state)

                TX_IDLE: begin
                    tx <= 1'b1;
                    if (data_en) begin
                        data     <= data_in;
                        c_bits   <= 0;
                        c_clocks <= 0;
                        state    <= TX_START;
                    end
                end

                // Start bit: hold the line low for one full bit period.
                TX_START: begin
                    tx <= 1'b0;
                    if (c_clocks == CLOCKS_PER_PULSE-1) begin
                        c_clocks <= 0;
                        state    <= TX_DATA;
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                // Data bits, LSB first.
                TX_DATA: begin
                    tx <= data[c_bits[$clog2(DATA_WIDTH)-1:0]];
                    if (c_clocks == CLOCKS_PER_PULSE-1) begin
                        c_clocks <= 0;
                        if (c_bits == DATA_WIDTH-1) begin
                            c_bits <= 0;
                            state  <= TX_END;
                        end else
                            c_bits <= c_bits + 1'b1;
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                // Stop bit: hold the line high for one full bit period.
                TX_END: begin
                    tx <= 1'b1;
                    if (c_clocks == CLOCKS_PER_PULSE-1) begin
                        c_clocks <= 0;
                        state    <= TX_IDLE;
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                default: state <= TX_IDLE;
            endcase
        end
    end

    assign tx_busy = (state != TX_IDLE);

endmodule
