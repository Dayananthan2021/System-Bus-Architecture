// ============================================================================
// File: master_device.v
// Description: Synthesizable AHB Master for DE0-Nano demonstration
// ============================================================================

`include "bus_params.vh"

module master_device (
    input wire                    HCLK,
    input wire                    HRESETn,

    // Demonstration inputs
    input wire                    trigger_transfer,
    input wire                    demo_write,
    input wire [`ADDR_WIDTH-1:0]  demo_addr,
    input wire [`DATA_WIDTH-1:0]  demo_data,

    // AHB handshake
    input wire                    HGRANT,
    input wire                    HREADY,
    input wire [1:0]              HRESP,
    input wire [`DATA_WIDTH-1:0]  HRDATA,

    // AHB master outputs
    output reg                    HBUSREQ,
    output reg [`ADDR_WIDTH-1:0]  HADDR,
    output reg [`DATA_WIDTH-1:0]  HWDATA,
    output reg                    HWRITE,
    output reg [1:0]              HTRANS,

    // Read result
    output reg [`DATA_WIDTH-1:0]  HRDATA_OUT
);

    // Master states
    localparam S_IDLE = 3'd0;
    localparam S_WAIT = 3'd1;
    localparam S_ADDR = 3'd2;
    localparam S_DATA = 3'd3;
    localparam S_DONE = 3'd4;

    // AHB HTRANS types
    localparam HTRANS_IDLE   = 2'b00;
    localparam HTRANS_NONSEQ = 2'b10;

    reg [2:0] state;

    always @(posedge HCLK or negedge HRESETn) begin

        if (!HRESETn) begin
            state      <= S_IDLE;
            HBUSREQ    <= 1'b0;
            HADDR      <= {`ADDR_WIDTH{1'b0}};
            HWDATA     <= {`DATA_WIDTH{1'b0}};
            HWRITE     <= 1'b0;
            HTRANS     <= HTRANS_IDLE;
            HRDATA_OUT <= {`DATA_WIDTH{1'b0}};
        end

        else begin

            case (state)

                // Wait for a new transfer request
                S_IDLE: begin
                    HBUSREQ <= 1'b0;
                    HTRANS  <= HTRANS_IDLE;

                    if (trigger_transfer) begin
                        HBUSREQ <= 1'b1;
                        state   <= S_WAIT;
                    end
                end

                // Request the bus and wait for grant
                S_WAIT: begin
                    HBUSREQ <= 1'b1;

                    if (HGRANT && HREADY) begin
                        HADDR  <= demo_addr;
                        HWRITE <= demo_write;
                        HTRANS <= HTRANS_NONSEQ;

                        state <= S_ADDR;
                    end
                end

                // Address phase
                S_ADDR: begin
                    if (HREADY) begin
                        HTRANS  <= HTRANS_IDLE;
                        HBUSREQ <= 1'b0;
                        HWDATA  <= demo_data;

                        state <= S_DATA;
                    end
                end

                // Data phase
                S_DATA: begin
                    if (HREADY) begin

                        if (HRESP == `HRESP_OKAY) begin

                            if (!HWRITE)
                                HRDATA_OUT <= HRDATA;

                            state <= S_DONE;
                        end

                        else if ((HRESP == `HRESP_RETRY) ||
                                 (HRESP == `HRESP_SPLIT)) begin

                            // Back off the bus and request again
                            HBUSREQ <= 1'b1;
                            HTRANS  <= HTRANS_IDLE;
                            state   <= S_WAIT;
                        end

                        else begin
                            // ERROR response
                            HBUSREQ <= 1'b0;
                            HTRANS  <= HTRANS_IDLE;
                            state   <= S_DONE;
                        end
                    end
                end

                // Transfer completed
                S_DONE: begin
                    HBUSREQ <= 1'b0;
                    HTRANS  <= HTRANS_IDLE;

                    // Wait for trigger release
                    if (!trigger_transfer)
                        state <= S_IDLE;
                end

                default: begin
                    state   <= S_IDLE;
                    HBUSREQ <= 1'b0;
                    HTRANS  <= HTRANS_IDLE;
                end

            endcase
        end
    end

endmodule