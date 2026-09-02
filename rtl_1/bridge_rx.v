// ============================================================================
// File: bridge_rx.v
// Description: Serial-to-AHB transaction receiver
//              Shared-clock FPGA-to-FPGA bridge with hardware flow control
// ============================================================================

`include "bus_params.vh"

module bridge_rx #(
    parameter MASTER_ID = `MASTER_3_ID
)(
    input wire                    HCLK,
    input wire                    HRESETn,

    // Serial connection from remote FPGA
    input wire                    SERIAL_RX,

    // Hardware flow control to remote FPGA
    output reg                    BUSY_OUT,

    // Local AHB master interface
    input wire                    HGRANT,
    input wire                    HREADY_IN,
    input wire [1:0]              HRESP,
    input wire [`DATA_WIDTH-1:0]  HRDATA,

    output reg                    HBUSREQ,
    output reg [`ADDR_WIDTH-1:0]  HADDR,
    output reg [1:0]              HTRANS,
    output reg                    HWRITE,
    output reg [`DATA_WIDTH-1:0]  HWDATA,

    // Returned read data
    output reg [`DATA_WIDTH-1:0]  HRDATA_OUT
);

    // ------------------------------------------------------------------------
    // Packet configuration
    //
    // Packet format:
    //
    // [PACKET_WIDTH-1]                     : START
    // [PACKET_WIDTH-2]                     : HWRITE
    // [DATA_WIDTH+ADDR_WIDTH-1:DATA_WIDTH] : HADDR
    // [DATA_WIDTH-1:0]                     : HWDATA
    //
    // Total = 1 + 1 + ADDR_WIDTH + DATA_WIDTH
    // ------------------------------------------------------------------------

    localparam PACKET_WIDTH = 2 + `ADDR_WIDTH + `DATA_WIDTH;

    localparam COUNTER_WIDTH = $clog2(PACKET_WIDTH + 1);

    // ------------------------------------------------------------------------
    // AHB transfer types
    // ------------------------------------------------------------------------

    localparam HTRANS_IDLE   = 2'b00;
    localparam HTRANS_NONSEQ = 2'b10;

    // ------------------------------------------------------------------------
    // State definitions
    // ------------------------------------------------------------------------

    localparam S_IDLE  = 3'd0;
    localparam S_SHIFT = 3'd1;
    localparam S_REQ   = 3'd2;
    localparam S_ADDR  = 3'd3;
    localparam S_DATA  = 3'd4;
    localparam S_DONE  = 3'd5;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Received packet
    // ------------------------------------------------------------------------

    reg [PACKET_WIDTH-2:0] rx_shift_reg;

    reg [COUNTER_WIDTH-1:0] bit_counter;

    // ------------------------------------------------------------------------
    // Sequential logic
    //
    // Shared clock architecture:
    //
    // TX changes SERIAL_TX on negedge HCLK.
    // RX samples SERIAL_RX on posedge HCLK.
    // ------------------------------------------------------------------------

    always @(posedge HCLK or negedge HRESETn) begin

        if (!HRESETn) begin

            state        <= S_IDLE;

            HBUSREQ      <= 1'b0;
            HADDR        <= {`ADDR_WIDTH{1'b0}};
            HTRANS       <= HTRANS_IDLE;
            HWRITE       <= 1'b0;
            HWDATA       <= {`DATA_WIDTH{1'b0}};
            HRDATA_OUT   <= {`DATA_WIDTH{1'b0}};

            BUSY_OUT     <= 1'b0;

            rx_shift_reg <= {(PACKET_WIDTH-1){1'b0}};
            bit_counter  <= {COUNTER_WIDTH{1'b0}};

        end

        else begin

            case (state)

                // ------------------------------------------------------------
                // Wait for packet start bit
                // ------------------------------------------------------------

                S_IDLE: begin

                    HBUSREQ  <= 1'b0;
                    HTRANS   <= HTRANS_IDLE;
                    BUSY_OUT <= 1'b0;

                    if (SERIAL_RX == 1'b1) begin

                        // Start bit detected.
                        // The start bit itself is not stored.
                        // Receive the remaining payload bits.
                        rx_shift_reg <= {(PACKET_WIDTH-1){1'b0}};
                        bit_counter  <= PACKET_WIDTH - 1;

                        BUSY_OUT <= 1'b1;

                        state <= S_SHIFT;

                    end
                end

                // ------------------------------------------------------------
                // Receive payload
                // ------------------------------------------------------------

                S_SHIFT: begin

                    BUSY_OUT <= 1'b1;

                    if (bit_counter != 0) begin

                        rx_shift_reg <= {
                            rx_shift_reg[PACKET_WIDTH-3:0],
                            SERIAL_RX
                        };

                        bit_counter <= bit_counter - 1'b1;

                    end

                    else begin

                        // Complete packet received.
                        HBUSREQ <= 1'b1;

                        state <= S_REQ;

                    end
                end

                // ------------------------------------------------------------
                // Request local AHB bus
                // ------------------------------------------------------------

                S_REQ: begin

                    HBUSREQ  <= 1'b1;
                    BUSY_OUT <= 1'b1;
                    HTRANS   <= HTRANS_IDLE;

                    if (HGRANT && HREADY_IN) begin

                        // The received address is preserved exactly.
                        //
                        // Any decision about whether this address belongs
                        // to a local slave or another address region belongs
                        // to the local bus integration/decoder.
                        HADDR <= rx_shift_reg[
                            `DATA_WIDTH + `ADDR_WIDTH - 1 :
                            `DATA_WIDTH
                        ];

                        HWRITE <= rx_shift_reg[`DATA_WIDTH + `ADDR_WIDTH];

                        HWDATA <= rx_shift_reg[`DATA_WIDTH-1:0];

                        HTRANS <= HTRANS_NONSEQ;

                        state <= S_ADDR;

                    end
                end

                // ------------------------------------------------------------
                // Address phase
                // ------------------------------------------------------------

                S_ADDR: begin

                    BUSY_OUT <= 1'b1;

                    if (HREADY_IN) begin

                        HTRANS  <= HTRANS_IDLE;
                        HBUSREQ <= 1'b0;

                        state <= S_DATA;

                    end
                end

                // ------------------------------------------------------------
                // Data phase
                // ------------------------------------------------------------

                S_DATA: begin

                    BUSY_OUT <= 1'b1;

                    if (HREADY_IN) begin

                        if (HRESP == `HRESP_OKAY) begin

                            if (!HWRITE)
                                HRDATA_OUT <= HRDATA;

                            BUSY_OUT <= 1'b0;
                            state    <= S_DONE;

                        end

                        else if ((HRESP == `HRESP_RETRY) ||
                                 (HRESP == `HRESP_SPLIT)) begin

                            // Keep the received transaction locally until
                            // the slave allows it to proceed.
                            HBUSREQ <= 1'b1;
                            HTRANS  <= HTRANS_IDLE;

                            BUSY_OUT <= 1'b1;

                            state <= S_REQ;

                        end

                        else begin

                            // ERROR response.
                            HBUSREQ <= 1'b0;
                            HTRANS  <= HTRANS_IDLE;

                            BUSY_OUT <= 1'b0;

                            state <= S_DONE;

                        end
                    end
                end

                // ------------------------------------------------------------
                // Transaction complete
                // ------------------------------------------------------------

                S_DONE: begin

                    HBUSREQ  <= 1'b0;
                    HTRANS   <= HTRANS_IDLE;
                    BUSY_OUT <= 1'b0;

                    // Wait until the serial line returns to idle.
                    if (SERIAL_RX == 1'b0)
                        state <= S_IDLE;

                end

                // ------------------------------------------------------------
                // Recovery
                // ------------------------------------------------------------

                default: begin

                    state        <= S_IDLE;

                    HBUSREQ      <= 1'b0;
                    HADDR        <= {`ADDR_WIDTH{1'b0}};
                    HTRANS       <= HTRANS_IDLE;
                    HWRITE       <= 1'b0;
                    HWDATA       <= {`DATA_WIDTH{1'b0}};
                    BUSY_OUT     <= 1'b0;

                    rx_shift_reg <= {(PACKET_WIDTH-1){1'b0}};
                    bit_counter  <= {COUNTER_WIDTH{1'b0}};

                end

            endcase

        end
    end

endmodule