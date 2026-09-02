// ============================================================================
// File: slave_memory.v
// Description: Parameterizable AHB slave memory with SPLIT support
//              Updated for Bridge Compatibility (Master 3)
// ============================================================================

`include "bus_params.vh"

module slave_memory #(
    parameter MEM_DEPTH        = 1024,          // 1024 words = 4 KB
    parameter ADDR_INDEX_WIDTH = 10,            // log2(MEM_DEPTH)
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

    // Local word address
    wire [`ADDR_WIDTH-1:0] local_addr;
    wire [ADDR_INDEX_WIDTH-1:0] word_index;

    assign local_addr = latched_addr - BASE_ADDR;
    assign word_index = local_addr[ADDR_INDEX_WIDTH+1:2];

    // SPLIT state
    localparam SP_IDLE  = 2'd0;
    localparam SP_RESP1 = 2'd1;
    localparam SP_RESP2 = 2'd2;
    localparam SP_WAIT  = 2'd3;

    reg [1:0] split_state;
    reg [3:0] split_timer;

    // Sequential logic
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
        end

        else begin

            // Default: no SPLIT release
            HSPLIT1 <= 1'b0;
            HSPLIT2 <= 1'b0;

            // Address phase
            if (HSEL && HREADY_IN && (HTRANS == 2'b10)) begin

                latched_addr      <= HADDR;
                latched_write     <= HWRITE;
                latched_master    <= HMASTER;
                data_phase_active <= 1'b1;

            end

            // SPLIT response sequence
            if (SPLIT_ENABLED && (split_state != SP_IDLE)) begin

                case (split_state)

                    // First SPLIT response cycle
                    SP_RESP1: begin
                        HREADY_OUT  <= 1'b0;
                        HRESP       <= `HRESP_SPLIT;
                        split_state <= SP_RESP2;
                    end

                    // Second SPLIT response cycle
                    SP_RESP2: begin
                        HREADY_OUT  <= 1'b1;
                        HRESP       <= `HRESP_SPLIT;
                        split_timer <= 4'd10;
                        split_state <= SP_WAIT;
                    end

                    // Wait before releasing the master
                    SP_WAIT: begin
                        HREADY_OUT <= 1'b1;
                        HRESP      <= `HRESP_OKAY;

                        if (split_timer != 0) begin
                            split_timer <= split_timer - 1'b1;
                        end

                        else begin

                            if (latched_master == `MASTER_1_ID)
                                HSPLIT1 <= 1'b1;

                            else if (latched_master == `MASTER_2_ID)
                                HSPLIT2 <= 1'b1;

                            split_state <= SP_IDLE;
                        end
                    end

                    default: begin
                        split_state <= SP_IDLE;
                    end

                endcase

            end

            // Normal data phase
            else if (data_phase_active) begin

                HREADY_OUT <= 1'b1;
                HRESP      <= `HRESP_OKAY;

                // FIX: Only allow SPLIT if the master is NOT the Bridge (Master 3)
                if (SPLIT_ENABLED && simulate_split && (latched_master != `MASTER_3_ID)) begin

                    split_state       <= SP_RESP1;
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
                end

            end

            else begin
                HREADY_OUT <= 1'b1;
                HRESP      <= `HRESP_OKAY;
            end

        end
    end

endmodule