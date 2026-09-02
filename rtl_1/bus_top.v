// ============================================================================
// File: bus_top.v
// Description: Top-level AHB Bus interconnect. 
//              Wires 3 Masters, 4 Slaves, Arbiter, Decoder, and Muxes.
// ============================================================================

`include "bus_params.vh"

module bus_top (
    input wire                    HCLK,
    input wire                    HRESETn,

    // Physical Inter-Board Connections
    input wire                    SERIAL_RX,
    output wire                   SERIAL_TX,
    input wire                    BUSY_IN,
    output wire                   BUSY_OUT,

    // External Controls (Wired to switches/buttons in de0_top.v)
    // Master 1
    input wire                    m1_trigger,
    input wire                    m1_write,
    input wire [`ADDR_WIDTH-1:0]  m1_addr,
    input wire [`DATA_WIDTH-1:0]  m1_wdata,
    output wire [`DATA_WIDTH-1:0] m1_rdata_out,

    // Master 2
    input wire                    m2_trigger,
    input wire                    m2_write,
    input wire [`ADDR_WIDTH-1:0]  m2_addr,
    input wire [`DATA_WIDTH-1:0]  m2_wdata,
    output wire [`DATA_WIDTH-1:0] m2_rdata_out,

    // Slave Split Simulators
    input wire                    s1_sim_split,
    input wire                    s2_sim_split,
    input wire                    s3_sim_split
);

    // INTERNAL AHB BUS WIRES
    
    // Global Bus state
    wire                   HREADY;     // The master HREADY seen by everyone
    wire [3:0]             HMASTER;    // Current bus owner ID
    
    // Master Request/Grant lines
    wire HBUSREQ1, HBUSREQ2, HBUSREQ3;
    wire HGRANT1,  HGRANT2,  HGRANT3;
    
    // Multiplexed Master Signals (Broadcast to all slaves)
    reg [`ADDR_WIDTH-1:0]  HADDR;
    reg [`DATA_WIDTH-1:0]  HWDATA;
    reg                    HWRITE;
    reg [1:0]              HTRANS;
    
    // Slave Select Lines
    wire HSEL1, HSEL2, HSEL3, HSEL_BRIDGE;
    
    // Slave Outputs
    wire                   hready_out_s1, hready_out_s2, hready_out_s3, hready_out_bridge;
    wire [1:0]             hresp_s1,      hresp_s2,      hresp_s3,      hresp_bridge;
    wire [`DATA_WIDTH-1:0] hrdata_s1,     hrdata_s2,     hrdata_s3;
    
    wire hsplit1_s1, hsplit2_s1;
    wire hsplit1_s2, hsplit2_s2;
    wire hsplit1_s3, hsplit2_s3;
    
    // Multiplexed Slave Signals (Broadcast back to masters)
    reg                    HREADY_OUT_MUX;
    reg [1:0]              HRESP_MUX;
    reg [`DATA_WIDTH-1:0]  HRDATA_MUX;
    
    // Combined SPLIT signals to Arbiter
    wire HSPLIT1 = hsplit1_s1 | hsplit1_s2 | hsplit1_s3;
    wire HSPLIT2 = hsplit2_s1 | hsplit2_s2 | hsplit2_s3;
    
    // Global HREADY logic
    assign HREADY = HREADY_OUT_MUX;

    // 1. BUS ARBITER
    bus_arbiter u_arbiter (
        .HCLK(HCLK),
        .HRESETn(HRESETn),
        .HBUSREQ1(HBUSREQ1),
        .HBUSREQ2(HBUSREQ2),
        .HBUSREQ3(HBUSREQ3),
        .HREADY(HREADY),
        .HRESP(HRESP_MUX),
        .HSPLIT1(HSPLIT1),
        .HSPLIT2(HSPLIT2),
        .HGRANT1(HGRANT1),
        .HGRANT2(HGRANT2),
        .HGRANT3(HGRANT3),
        .HMASTER(HMASTER)
    );

    // 2. MASTER MULTIPLEXER (Combinational)
    wire [`ADDR_WIDTH-1:0] haddr_m1,  haddr_m2,  haddr_m3;
    wire [`DATA_WIDTH-1:0] hwdata_m1, hwdata_m2, hwdata_m3;
    wire                   hwrite_m1, hwrite_m2, hwrite_m3;
    wire [1:0]             htrans_m1, htrans_m2, htrans_m3;

    always @(*) begin
        case (HMASTER)
            `MASTER_1_ID: begin
                HADDR  = haddr_m1;
                HWDATA = hwdata_m1;
                HWRITE = hwrite_m1;
                HTRANS = htrans_m1;
            end
            `MASTER_2_ID: begin
                HADDR  = haddr_m2;
                HWDATA = hwdata_m2;
                HWRITE = hwrite_m2;
                HTRANS = htrans_m2;
            end
            `MASTER_3_ID: begin
                HADDR  = haddr_m3;
                HWDATA = hwdata_m3;
                HWRITE = hwrite_m3;
                HTRANS = htrans_m3;
            end
            default: begin
                HADDR  = {`ADDR_WIDTH{1'b0}};
                HWDATA = {`DATA_WIDTH{1'b0}};
                HWRITE = 1'b0;
                HTRANS = 2'b00;
            end
        endcase
    end

    // 3. ADDRESS DECODER
    wire any_req = (HBUSREQ1 | HBUSREQ2 | HBUSREQ3);
    
    address_decoder u_decoder (
        .HADDR(HADDR),
        .HBUSREQ(any_req),
        .HSEL1(HSEL1),
        .HSEL2(HSEL2),
        .HSEL3(HSEL3),
        .HSEL_BRIDGE(HSEL_BRIDGE)
    );

    // 4. SLAVE MULTIPLEXER (AHB Pipeline Latch)

    reg sel1_latched, sel2_latched, sel3_latched, sel_bridge_latched;
    // Because AHB is pipelined, the data phase happens one cycle AFTER the 
    // address phase. We must latch which slave was selected.
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            sel1_latched <= 1'b0;
            sel2_latched <= 1'b0;
            sel3_latched <= 1'b0;
            sel_bridge_latched <= 1'b0;
        end else if (HREADY) begin
            sel1_latched <= HSEL1;
            sel2_latched <= HSEL2;
            sel3_latched <= HSEL3;
            sel_bridge_latched <= HSEL_BRIDGE;
        end
    end

    always @(*) begin
        if (sel1_latched) begin
            HREADY_OUT_MUX = hready_out_s1;
            HRESP_MUX      = hresp_s1;
            HRDATA_MUX     = hrdata_s1;
        end else if (sel2_latched) begin
            HREADY_OUT_MUX = hready_out_s2;
            HRESP_MUX      = hresp_s2;
            HRDATA_MUX     = hrdata_s2;
        end else if (sel3_latched) begin
            HREADY_OUT_MUX = hready_out_s3;
            HRESP_MUX      = hresp_s3;
            HRDATA_MUX     = hrdata_s3;
        end else if (sel_bridge_latched) begin
            HREADY_OUT_MUX = hready_out_bridge;
            HRESP_MUX      = hresp_bridge;
            HRDATA_MUX     = {`DATA_WIDTH{1'b0}}; // Bridge TX doesn't return data
        end else begin
            HREADY_OUT_MUX = 1'b1;
            HRESP_MUX      = `HRESP_OKAY;
            HRDATA_MUX     = {`DATA_WIDTH{1'b0}};
        end
    end

    // 5. MASTERS
    master_device u_master1 (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .trigger_transfer(m1_trigger), .demo_write(m1_write), .demo_addr(m1_addr), .demo_data(m1_wdata),
        .HGRANT(HGRANT1), .HREADY(HREADY), .HRESP(HRESP_MUX), .HRDATA(HRDATA_MUX),
        .HBUSREQ(HBUSREQ1), .HADDR(haddr_m1), .HWDATA(hwdata_m1), .HWRITE(hwrite_m1), .HTRANS(htrans_m1),
        .HRDATA_OUT(m1_rdata_out)
    );

    master_device u_master2 (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .trigger_transfer(m2_trigger), .demo_write(m2_write), .demo_addr(m2_addr), .demo_data(m2_wdata),
        .HGRANT(HGRANT2), .HREADY(HREADY), .HRESP(HRESP_MUX), .HRDATA(HRDATA_MUX),
        .HBUSREQ(HBUSREQ2), .HADDR(haddr_m2), .HWDATA(hwdata_m2), .HWRITE(hwrite_m2), .HTRANS(htrans_m2),
        .HRDATA_OUT(m2_rdata_out)
    );

    bridge_rx u_bridge_rx (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .SERIAL_RX(SERIAL_RX), .BUSY_OUT(BUSY_OUT),
        .HGRANT(HGRANT3), .HREADY_IN(HREADY), .HRESP(HRESP_MUX), .HRDATA(HRDATA_MUX),
        .HBUSREQ(HBUSREQ3), .HADDR(haddr_m3), .HTRANS(htrans_m3), .HWRITE(hwrite_m3), .HWDATA(hwdata_m3),
        .HRDATA_OUT() // Bridge RX doesn't care about returning read data visually
    );

    // 6. SLAVES
    slave_memory #(
        .BASE_ADDR(`S1_BASE), .SPLIT_ENABLED(0)
    ) u_slave1 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s1_sim_split),
        .HSEL(HSEL1), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE), .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(hready_out_s1), .HRESP(hresp_s1), .HRDATA(hrdata_s1), .HSPLIT1(hsplit1_s1), .HSPLIT2(hsplit2_s1)
    );

    slave_memory #(
        .BASE_ADDR(`S2_BASE), .SPLIT_ENABLED(1)
    ) u_slave2 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s2_sim_split),
        .HSEL(HSEL2), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE), .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(hready_out_s2), .HRESP(hresp_s2), .HRDATA(hrdata_s2), .HSPLIT1(hsplit1_s2), .HSPLIT2(hsplit2_s2)
    );

    slave_memory #(
        .BASE_ADDR(`S3_BASE), .SPLIT_ENABLED(1)
    ) u_slave3 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s3_sim_split),
        .HSEL(HSEL3), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE), .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(hready_out_s3), .HRESP(hresp_s3), .HRDATA(hrdata_s3), .HSPLIT1(hsplit1_s3), .HSPLIT2(hsplit2_s3)
    );

    bridge_tx u_bridge_tx (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HSEL_BRIDGE(HSEL_BRIDGE), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE), .HWDATA(HWDATA), .HREADY_IN(HREADY),
        .HREADY_OUT(hready_out_bridge), .HRESP(hresp_bridge),
        .SERIAL_TX(SERIAL_TX), .BUSY_IN(BUSY_IN)
    );

endmodule