// Parameterisable AHB slave memory with optional SPLIT support.
//
// Reads have a 2-cycle latency: the address is registered, and the array is
// read on the next edge. That is what lets it infer a real M9K block, so
// anything sampling HRDATA must allow for it.

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module slave_memory #(
    parameter MEM_DEPTH        = 4096,          // locations (bytes, 8-bit data)
    parameter ADDR_INDEX_WIDTH = 12,            // log2(MEM_DEPTH)
    parameter SPLIT_ENABLED    = 0,
    parameter BASE_ADDR        = {`ADDR_WIDTH{1'b0}}
)(
    input wire                    HCLK,
    input wire                    HRESETn,

    // Demonstration control
    input wire                    simulate_split,

    // AHB inputs
    input wire                    HSEL,
    input wire [`ADDR_WIDTH-1:0]  HADDR,
    input wire [1:0]              HTRANS,
    input wire                    HWRITE,
    input wire [`DATA_WIDTH-1:0]  HWDATA,
    input wire                    HREADY_IN,
    input wire [3:0]              HMASTER,

    // AHB outputs
    output reg                    HREADY_OUT,
    output reg [1:0]              HRESP,
    output reg [`DATA_WIDTH-1:0]  HRDATA,

    // SPLIT release signals
    output reg                    HSPLIT1,
    output reg                    HSPLIT2
);

    // Memory
    reg [`DATA_WIDTH-1:0] memory_array [0:MEM_DEPTH-1];

    // Latched address/control
    reg [`ADDR_WIDTH-1:0] latched_addr;
    reg                   latched_write;
    reg                   data_phase_active;
    reg [3:0]             latched_master;

    // The bus is byte-addressed, so the local offset IS the memory index --
    // no shift. Indexing [IDXW+1:2] would be right for a 32-bit word bus but
    // here aliases addresses 0..3 onto index 0.
    wire [`ADDR_WIDTH-1:0]      local_addr = latched_addr - BASE_ADDR;
    wire [ADDR_INDEX_WIDTH-1:0] word_index  = local_addr[ADDR_INDEX_WIDTH-1:0];

    // Offset bits above the index width must be zero for the access to lie
    // inside this slave. Slave 3 is 2K in a 4K-aligned region, so its upper
    // half arrives here with a high bit set and would wrap onto the low half.
    wire addr_out_of_range = |local_addr[`ADDR_WIDTH-1:ADDR_INDEX_WIDTH];

    // SPLIT state
    localparam SP_IDLE  = 2'd0;
    localparam SP_RESP1 = 2'd1;
    localparam SP_RESP2 = 2'd2;
    localparam SP_WAIT  = 2'd3;

    reg [1:0] split_state;
    reg [3:0] split_timer;

    // Set once a master has been split, so its retry is served rather than
    // split again -- otherwise holding simulate_split high defers the same
    // access forever and the master never progresses.
    reg       split_served;

    always @(posedge HCLK or negedge HRESETn) begin

        if (!HRESETn) begin
            HREADY_OUT        <= 1'b1;
            HRESP             <= `HRESP_OKAY;
            HRDATA            <= {`DATA_WIDTH{1'b0}};

            HSPLIT1           <= 1'b0;
            HSPLIT2           <= 1'b0;

            latched_addr      <= {`ADDR_WIDTH{1'b0}};
            latched_write     <= 1'b0;
            latched_master    <= 4'd0;
            data_phase_active <= 1'b0;

            split_state       <= SP_IDLE;
            split_timer       <= 4'd0;
            split_served      <= 1'b0;
        end

        else begin

            // Default: no SPLIT release
            HSPLIT1 <= 1'b0;
            HSPLIT2 <= 1'b0;

            // Address phase. Only the SPLIT-response cycles refuse a new
            // transfer; SP_WAIT must accept the released master's retry or
            // the read never completes.
            if (HSEL && HREADY_IN && (HTRANS == 2'b10) &&
                (!SPLIT_ENABLED || (split_state == SP_IDLE)
                                || (split_state == SP_WAIT))) begin

                latched_addr      <= HADDR;
                latched_write     <= HWRITE;
                latched_master    <= HMASTER;
                data_phase_active <= 1'b1;

            end

            // SPLIT response. Only these two cycles override the bus
            // response; SP_WAIT deliberately does not, so a retry accepted
            // above can run through the normal data phase below.
            if (SPLIT_ENABLED && ((split_state == SP_RESP1) ||
                                  (split_state == SP_RESP2))) begin

                case (split_state)

                    SP_RESP1: begin
                        HREADY_OUT  <= 1'b0;
                        HRESP       <= `HRESP_SPLIT;
                        split_state <= SP_RESP2;
                    end

                    // Second response cycle, then start the fetch delay.
                    SP_RESP2: begin
                        HREADY_OUT  <= 1'b1;
                        HRESP       <= `HRESP_SPLIT;
                        split_timer <= 4'd10;
                        split_state <= SP_WAIT;
                    end

                    default: split_state <= SP_IDLE;

                endcase

            end

            // Fetch delay: models the slave going away for the data. Runs
            // concurrently with the data phase below, not chained to it, so
            // the master's retry can be served while it counts down.
            if (SPLIT_ENABLED && (split_state == SP_WAIT)) begin
                if (split_timer != 0)
                    split_timer <= split_timer - 1'b1;
                else begin
                    if      (latched_master == `MASTER_1_ID) HSPLIT1 <= 1'b1;
                    else if (latched_master == `MASTER_2_ID) HSPLIT2 <= 1'b1;

                    split_state <= SP_IDLE;
                end
            end

            // Normal data phase, guarded against the SPLIT-response cycles.
            if (data_phase_active &&
                !(SPLIT_ENABLED && ((split_state == SP_RESP1) ||
                                    (split_state == SP_RESP2)))) begin

                HREADY_OUT <= 1'b1;
                HRESP      <= `HRESP_OKAY;

                // Only split for a local master; the bridge (Master 3) is
                // replaying remote traffic and must not be parked.
                if (SPLIT_ENABLED && simulate_split && !split_served &&
                    (latched_master != `MASTER_3_ID)) begin

                    // Split in THIS cycle. Letting the defaults above stand
                    // and splitting from SP_RESP1 next cycle would answer
                    // "complete, OK" first; the master takes that and the
                    // split is lost.
                    HREADY_OUT        <= 1'b0;
                    HRESP             <= `HRESP_SPLIT;

                    split_state       <= SP_RESP1;
                    data_phase_active <= 1'b0;

                    // Mark here, not at release: the retry can arrive during
                    // SP_WAIT, before the release pulse.
                    split_served      <= 1'b1;

                end

                // Beyond the populated depth: error rather than alias.
                else if (addr_out_of_range) begin
                    HRESP             <= `HRESP_ERROR;
                    HRDATA            <= {`DATA_WIDTH{1'b0}};
                    data_phase_active <= 1'b0;
                end

                else begin

                    if (latched_write) begin
                        memory_array[word_index] <= HWDATA;
                    end

                    else begin
                        HRDATA <= memory_array[word_index];
                    end

                    data_phase_active <= 1'b0;
                    split_served      <= 1'b0;   // re-arm for the next transfer
                end

            end

            else if (!(SPLIT_ENABLED && ((split_state == SP_RESP1) ||
                                        (split_state == SP_RESP2)))) begin
                // Idle: nothing selected, no split response in flight.
                HREADY_OUT <= 1'b1;
                HRESP      <= `HRESP_OKAY;
            end

        end
    end

endmodule