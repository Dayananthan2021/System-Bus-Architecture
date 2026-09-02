// UART receiver, 8-N-1, LSB first, idle high. On the start-bit falling edge
// it waits half a bit period so every later sample lands mid-bit; that is
// what tolerates baud mismatch, since error only accumulates over 9.5 bit
// periods to the stop bit (about 2.1% at 10 MHz / 115200).
//
// data_out is latched at end of frame rather than exposing the shift
// register, so a consumer cannot sample a half-assembled byte.

`timescale 1ns / 1ps

module uart_rx #(
    parameter CLOCKS_PER_PULSE = 87,        // clk cycles per bit period
    parameter DATA_WIDTH       = 8
)(
    input  wire                  clk,
    input  wire                  rstn,      // async reset, active low
    input  wire                  rx,        // serial input line (async)
    output reg                   ready,     // one-clock pulse: data_out valid
    output reg [DATA_WIDTH-1:0]  data_out   // received byte
);

    localparam RX_IDLE  = 2'b00,
               RX_START = 2'b01,
               RX_DATA  = 2'b11,
               RX_END   = 2'b10;

    reg [1:0]                          state;
    // Holds DATA_WIDTH, not DATA_WIDTH-1, so the terminal count fits.
    reg [$clog2(DATA_WIDTH+1)-1:0]     c_bits;
    reg [$clog2(CLOCKS_PER_PULSE)-1:0] c_clocks;
    reg [DATA_WIDTH-1:0]               temp_data;

    // rx crosses from the remote board's clock domain, so two flops.
    reg rx_meta, rx_sync;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state     <= RX_IDLE;
            c_bits    <= 0;
            c_clocks  <= 0;
            temp_data <= {DATA_WIDTH{1'b0}};
            data_out  <= {DATA_WIDTH{1'b0}};
            ready     <= 1'b0;
        end else begin
            // ready is a single-cycle pulse.
            ready <= 1'b0;

            case (state)

                RX_IDLE: begin
                    // Falling edge on the line = start bit.
                    if (!rx_sync) begin
                        c_clocks <= 0;
                        state    <= RX_START;
                    end
                end

                // Wait half a bit period to reach the centre of the start bit.
                RX_START: begin
                    if (c_clocks == (CLOCKS_PER_PULSE/2)-1) begin
                        c_clocks <= 0;
                        // Still low => a real start bit, not a glitch.
                        if (!rx_sync) begin
                            c_bits <= 0;
                            state  <= RX_DATA;
                        end else
                            state  <= RX_IDLE;   // spurious edge, resynchronise
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                // Sample each data bit at its centre, LSB first.
                RX_DATA: begin
                    if (c_clocks == CLOCKS_PER_PULSE-1) begin
                        c_clocks           <= 0;
                        temp_data[c_bits[$clog2(DATA_WIDTH)-1:0]] <= rx_sync;
                        if (c_bits == DATA_WIDTH-1) begin
                            c_bits <= 0;
                            state  <= RX_END;
                        end else
                            c_bits <= c_bits + 1'b1;
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                // Stop bit. Publish the byte only if the line really is high;
                // a low stop bit is a framing error, so the byte is dropped.
                RX_END: begin
                    if (c_clocks == CLOCKS_PER_PULSE-1) begin
                        c_clocks <= 0;
                        if (rx_sync) begin
                            data_out <= temp_data;
                            ready    <= 1'b1;
                        end
                        state <= RX_IDLE;
                    end else
                        c_clocks <= c_clocks + 1'b1;
                end

                default: state <= RX_IDLE;
            endcase
        end
    end

endmodule
