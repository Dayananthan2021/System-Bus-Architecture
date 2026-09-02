// ============================================================================
// File: bridge_tx.v
// Description: AHB-to-serial transaction transmitter with hardware
//              flow control. Packet width is parameterized by bus widths.
// ============================================================================

`include "bus_params.vh"

module bridge_tx (
    input wire                    HCLK,
    input wire                    HRESETn,

    // Local AHB slave interface
    input wire                    HSEL_BRIDGE,
    input wire [`ADDR_WIDTH-1:0]  HADDR,
    input wire [1:0]              HTRANS,
    input wire                    HWRITE,
    input wire [`DATA_WIDTH-1:0]  HWDATA,
    input wire                    HREADY_IN,

    output reg                    HREADY_OUT,
    output reg [1:0]              HRESP,

    // Serial output
    output reg                    SERIAL_TX,

    // Hardware flow control from remote FPGA
    input wire                    BUSY_IN
);

    // ------------------------------------------------------------------------
    // Packet configuration
    //
    // Packet format:
    //
    // [PACKET_WIDTH-1]              : START
    // [PACKET_WIDTH-2]              : HWRITE
    // [DATA_WIDTH+ADDR_WIDTH-1:DATA_WIDTH]
    //                                : HADDR
    // [DATA_WIDTH-1:0]              : HWDATA
    //
    // Total = 1 + 1 + ADDR_WIDTH + DATA_WIDTH
    // ------------------------------------------------------------------------

    localparam PACKET_WIDTH = 2 + `ADDR_WIDTH + `DATA_WIDTH;

    // ------------------------------------------------------------------------
    // State definitions
    // ------------------------------------------------------------------------

    localparam S_IDLE  = 2'd0;
    localparam S_WAIT  = 2'd1;
    localparam S_SHIFT = 2'd2;
    localparam S_DONE  = 2'd3;

    reg [1:0] state;

    // ------------------------------------------------------------------------
    // Address phase information
    // ------------------------------------------------------------------------

    reg [`ADDR_WIDTH-1:0] latched_addr;
    reg                   latched_write;

    // ------------------------------------------------------------------------
    // Serial packet
    // ------------------------------------------------------------------------

    reg [PACKET_WIDTH-1:0] shift_reg;

    // Number of bits remaining.
    // $clog2(PACKET_WIDTH + 1) gives enough bits to represent PACKET_WIDTH.
    localparam COUNTER_WIDTH = $clog2(PACKET_WIDTH + 1);

    reg [COUNTER_WIDTH-1:0] bit_counter;

    // ------------------------------------------------------------------------
    // Sequential logic
    //
    // TX changes on the falling edge.
    // RX samples on the rising edge.
    // This assumes HCLK is shared between the two FPGAs.
    // ------------------------------------------------------------------------

    always @(negedge HCLK or negedge HRESETn) begin

        if (!HRESETn) begin

            state         <= S_IDLE;

            HREADY_OUT    <= 1'b1;
            HRESP         <= `HRESP_OKAY;
            SERIAL_TX     <= 1'b0;

            latched_addr  <= {`ADDR_WIDTH{1'b0}};
            latched_write <= 1'b0;

            shift_reg     <= {PACKET_WIDTH{1'b0}};
            bit_counter   <= {COUNTER_WIDTH{1'b0}};

        end

        else begin

            case (state)

                // ------------------------------------------------------------
                // Address phase
                // ------------------------------------------------------------

                S_IDLE: begin

                    HREADY_OUT <= 1'b1;
                    HRESP      <= `HRESP_OKAY;
                    SERIAL_TX  <= 1'b0;

                    if (HSEL_BRIDGE &&
                        HREADY_IN &&
                        (HTRANS == 2'b10)) begin

                        // Capture address/control information.
                        latched_addr  <= HADDR;
                        latched_write <= HWRITE;

                        // Hold the AHB transfer while we prepare the packet.
                        HREADY_OUT <= 1'b0;

                        state <= S_WAIT;
                    end
                end

                // ------------------------------------------------------------
                // Data phase / remote flow control
                // ------------------------------------------------------------

                S_WAIT: begin

                    HREADY_OUT <= 1'b0;

                    if (!BUSY_IN) begin

                        // Packet:
                        // START + HWRITE + HADDR + HWDATA
                        shift_reg <= {
                            1'b1,
                            latched_write,
                            latched_addr,
                            HWDATA
                        };

                        bit_counter <= PACKET_WIDTH;

                        state <= S_SHIFT;
                    end
                end

                // ------------------------------------------------------------
                // Serial transmission
                // ------------------------------------------------------------

                S_SHIFT: begin

                    HREADY_OUT <= 1'b0;

                    if (bit_counter != 0) begin

                        // Send MSB first.
                        SERIAL_TX <= shift_reg[PACKET_WIDTH-1];

                        shift_reg <= {
                            shift_reg[PACKET_WIDTH-2:0],
                            1'b0
                        };

                        bit_counter <= bit_counter - 1'b1;

                    end

                    else begin

                        SERIAL_TX <= 1'b0;
                        state     <= S_DONE;

                    end
                end

                // ------------------------------------------------------------
                // Release local AHB transaction
                // ------------------------------------------------------------

                S_DONE: begin

                    HREADY_OUT <= 1'b1;
                    HRESP      <= `HRESP_OKAY;
                    SERIAL_TX  <= 1'b0;

                    state <= S_IDLE;

                end

                // ------------------------------------------------------------
                // Recovery
                // ------------------------------------------------------------

                default: begin

                    state         <= S_IDLE;

                    HREADY_OUT    <= 1'b1;
                    HRESP         <= `HRESP_OKAY;
                    SERIAL_TX     <= 1'b0;

                    latched_addr  <= {`ADDR_WIDTH{1'b0}};
                    latched_write <= 1'b0;

                    shift_reg     <= {PACKET_WIDTH{1'b0}};
                    bit_counter   <= {COUNTER_WIDTH{1'b0}};

                end

            endcase

        end
    end

endmodule