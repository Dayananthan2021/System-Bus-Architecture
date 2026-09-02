// The system bus interconnect: arbiter, decoder, three slave memories and
// the inter-board bridge, plus the master and slave multiplexers.
//
//      M1        M2        bridge (remote traffic, M3)
//       \        |        /
//        +-------+-------+          <- master mux, selected by HMASTER
//                |
//          arbiter + decoder
//                |
//        +-------+-------+-------+  <- slave mux, selected by latched HSELx
//       /        |        \       \
//      S1       S2        S3      bridge
//   4K@0000  4K@1000   2K@2000   remote window @3000
//
// Board-independent by design -- no pins, no clock division, no debouncing.
// That lives in fpga/de0_top.v, which keeps this simulatable without a board.

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_top (
    input  wire                    HCLK,
    input  wire                    HRESETn,

    // ---- Inter-board link (to the other DE0-Nano) ----
    input  wire                    LINK_RX,
    output wire                    LINK_TX,

    // ---- Master 1 control (driven by board switches/keys) ----
    input  wire                    m1_trigger,
    input  wire                    m1_write,
    input  wire [`ADDR_WIDTH-1:0]  m1_addr,
    input  wire [`DATA_WIDTH-1:0]  m1_wdata,
    output wire [`DATA_WIDTH-1:0]  m1_rdata,

    // ---- Master 2 control ----
    input  wire                    m2_trigger,
    input  wire                    m2_write,
    input  wire [`ADDR_WIDTH-1:0]  m2_addr,
    input  wire [`DATA_WIDTH-1:0]  m2_wdata,
    output wire [`DATA_WIDTH-1:0]  m2_rdata,

    // ---- Slave split simulation (demonstrates the split handshake) ----
    input  wire                    s1_sim_split,
    input  wire                    s2_sim_split,
    input  wire                    s3_sim_split,

    // ---- Observability, for LEDs and for testbenches ----
    output wire [3:0]              HMASTER_OUT,
    output wire [1:0]              HRESP_OUT,
    output wire                    HADDR_INVALID_OUT,
    output wire                    bus_busy
);

    // ---- Bus wires ---------------------------------------------------------
    wire                   HREADY;
    wire [3:0]             HMASTER;

    wire HBUSREQ1, HBUSREQ2, HBUSREQ3;
    wire HGRANT1,  HGRANT2,  HGRANT3;

    // Multiplexed master signals, broadcast to every slave.
    reg  [`ADDR_WIDTH-1:0] HADDR;
    reg  [`DATA_WIDTH-1:0] HWDATA;
    reg                    HWRITE;
    reg  [1:0]             HTRANS;

    // Decoder outputs.
    wire HSEL1, HSEL2, HSEL3, HSEL_REMOTE, HADDR_INVALID;

    // Per-slave responses.
    wire                   ready_s1, ready_s2, ready_s3, ready_brg;
    wire [1:0]             resp_s1,  resp_s2,  resp_s3,  resp_brg;
    wire [`DATA_WIDTH-1:0] rdata_s1, rdata_s2, rdata_s3, rdata_brg;

    wire hsplit1_s1, hsplit2_s1;
    wire hsplit1_s2, hsplit2_s2;
    wire hsplit1_s3, hsplit2_s3;
    wire hsplit1_brg, hsplit2_brg;

    // Multiplexed slave response, broadcast back to every master.
    reg                    HREADY_MUX;
    reg  [1:0]             HRESP_MUX;
    reg  [`DATA_WIDTH-1:0] HRDATA_MUX;

    // A split release from any slave, including the bridge.
    wire HSPLIT1 = hsplit1_s1 | hsplit1_s2 | hsplit1_s3 | hsplit1_brg;
    wire HSPLIT2 = hsplit2_s1 | hsplit2_s2 | hsplit2_s3 | hsplit2_brg;

    assign HREADY = HREADY_MUX;

    // ---- 1. Arbiter --------------------------------------------------------
    bus_arbiter u_arbiter (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HBUSREQ1(HBUSREQ1), .HBUSREQ2(HBUSREQ2), .HBUSREQ3(HBUSREQ3),
        .HREADY(HREADY), .HRESP(HRESP_MUX),
        .HSPLIT1(HSPLIT1), .HSPLIT2(HSPLIT2),
        .HGRANT1(HGRANT1), .HGRANT2(HGRANT2), .HGRANT3(HGRANT3),
        .HMASTER(HMASTER)
    );

    // ---- 2. Master multiplexer ---------------------------------------------
    //
    // An explicit mux, not a tri-state net: FPGA fabric has no internal
    // tri-states, and this keeps exactly one driver per signal.
    wire [`ADDR_WIDTH-1:0] addr_m1,  addr_m2,  addr_m3;
    wire [`DATA_WIDTH-1:0] wdata_m1, wdata_m2, wdata_m3;
    wire                   write_m1, write_m2, write_m3;
    wire [1:0]             trans_m1, trans_m2, trans_m3;

    always @(*) begin
        case (HMASTER)
            `MASTER_1_ID: begin
                HADDR  = addr_m1;  HWDATA = wdata_m1;
                HWRITE = write_m1; HTRANS = trans_m1;
            end
            `MASTER_2_ID: begin
                HADDR  = addr_m2;  HWDATA = wdata_m2;
                HWRITE = write_m2; HTRANS = trans_m2;
            end
            `MASTER_3_ID: begin
                HADDR  = addr_m3;  HWDATA = wdata_m3;
                HWRITE = write_m3; HTRANS = trans_m3;
            end
            default: begin
                HADDR  = {`ADDR_WIDTH{1'b0}};
                HWDATA = {`DATA_WIDTH{1'b0}};
                HWRITE = 1'b0;
                HTRANS = `HTRANS_IDLE;
            end
        endcase
    end

    // ---- 3. Address decoder ------------------------------------------------
    //
    // Enabled only for an active transfer, so an idle bus selects nothing and
    // a stale address cannot spuriously flag an invalid-address error.
    address_decoder u_decoder (
        .HADDR(HADDR),
        .HSEL_EN(HTRANS == `HTRANS_NONSEQ),
        .HSEL1(HSEL1), .HSEL2(HSEL2), .HSEL3(HSEL3),
        .HSEL_REMOTE(HSEL_REMOTE),
        .HADDR_INVALID(HADDR_INVALID)
    );

    // ---- 4. Slave multiplexer ----------------------------------------------
    //
    // The bus is pipelined, so the selects are latched to route each response
    // back to the master that asked for it, and held for the whole data
    // phase. One cycle is not enough:
    //
    //   * slave_memory's read data only appears two cycles after the address
    //     phase, so a one-cycle select has gone by and every read returns 0.
    //   * a stretched transfer drives HREADY low and HRESP=SPLIT for several
    //     cycles; if the select has cleared the muxed HRESP reads OKAY and
    //     the master completes a transfer the slave actually deferred.
    //
    // Holding until the slave is ready AND has had its extra read-data cycle
    // covers both with the same logic, at no cost on a fast access.
    reg sel1_q, sel2_q, sel3_q, sel_remote_q, invalid_q;
    reg data_phase_q;      // second cycle of a normal (unstretched) access

    wire any_sel_q = sel1_q | sel2_q | sel3_q | sel_remote_q | invalid_q;

    wire sel_ready = sel1_q       ? ready_s1
                   : sel2_q       ? ready_s2
                   : sel3_q       ? ready_s3
                   : sel_remote_q ? ready_brg
                   :                1'b1;

    // Hold while the slave stalls, or until the read-data cycle is given.
    wire hold_sel = any_sel_q && (!sel_ready || !data_phase_q);

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            sel1_q       <= 1'b0;
            sel2_q       <= 1'b0;
            sel3_q       <= 1'b0;
            sel_remote_q <= 1'b0;
            invalid_q    <= 1'b0;
            data_phase_q <= 1'b0;
        end else if (hold_sel) begin
            // Keep the selection; once the slave stops stalling, mark that
            // the extra data cycle has been granted.
            if (sel_ready) data_phase_q <= 1'b1;
        end else begin
            sel1_q       <= HSEL1;
            sel2_q       <= HSEL2;
            sel3_q       <= HSEL3;
            sel_remote_q <= HSEL_REMOTE;
            invalid_q    <= HADDR_INVALID;
            data_phase_q <= 1'b0;
        end
    end

    wire sel1_active   = sel1_q;
    wire sel2_active   = sel2_q;
    wire sel3_active   = sel3_q;
    wire selrmt_active = sel_remote_q;
    wire invalid_activ = invalid_q;

    always @(*) begin
        if (sel1_active) begin
            HREADY_MUX = ready_s1; HRESP_MUX = resp_s1; HRDATA_MUX = rdata_s1;
        end else if (sel2_active) begin
            HREADY_MUX = ready_s2; HRESP_MUX = resp_s2; HRDATA_MUX = rdata_s2;
        end else if (sel3_active) begin
            HREADY_MUX = ready_s3; HRESP_MUX = resp_s3; HRDATA_MUX = rdata_s3;
        end else if (selrmt_active) begin
            HREADY_MUX = ready_brg; HRESP_MUX = resp_brg; HRDATA_MUX = rdata_brg;
        end else if (invalid_activ) begin
            // No slave here: error rather than stall forever on a HREADY
            // nobody will drive.
            HREADY_MUX = 1'b1;
            HRESP_MUX  = `HRESP_ERROR;
            HRDATA_MUX = {`DATA_WIDTH{1'b0}};
        end else begin
            HREADY_MUX = 1'b1;
            HRESP_MUX  = `HRESP_OKAY;
            HRDATA_MUX = {`DATA_WIDTH{1'b0}};
        end
    end

    // ---- 5. Masters --------------------------------------------------------
    master_device u_master1 (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .trigger_transfer(m1_trigger), .demo_write(m1_write),
        .demo_addr(m1_addr), .demo_data(m1_wdata),
        .HGRANT(HGRANT1), .HREADY(HREADY),
        .HRESP(HRESP_MUX), .HRDATA(HRDATA_MUX),
        .HBUSREQ(HBUSREQ1), .HADDR(addr_m1), .HWDATA(wdata_m1),
        .HWRITE(write_m1), .HTRANS(trans_m1),
        .HRDATA_OUT(m1_rdata)
    );

    master_device u_master2 (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .trigger_transfer(m2_trigger), .demo_write(m2_write),
        .demo_addr(m2_addr), .demo_data(m2_wdata),
        .HGRANT(HGRANT2), .HREADY(HREADY),
        .HRESP(HRESP_MUX), .HRDATA(HRDATA_MUX),
        .HBUSREQ(HBUSREQ2), .HADDR(addr_m2), .HWDATA(wdata_m2),
        .HWRITE(write_m2), .HTRANS(trans_m2),
        .HRDATA_OUT(m2_rdata)
    );

    // ---- 6. Slaves ---------------------------------------------------------
    //
    // Slave 1 cannot split, so there is always a plain always-ready target.
    // Slaves 2 and 3 can, which the assignment requires.
    slave_memory #(
        .MEM_DEPTH(`S1_DEPTH), .ADDR_INDEX_WIDTH(`S1_IDXW),
        .SPLIT_ENABLED(0), .BASE_ADDR(`S1_BASE)
    ) u_slave1 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s1_sim_split),
        .HSEL(HSEL1), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(ready_s1), .HRESP(resp_s1), .HRDATA(rdata_s1),
        .HSPLIT1(hsplit1_s1), .HSPLIT2(hsplit2_s1)
    );

    slave_memory #(
        .MEM_DEPTH(`S2_DEPTH), .ADDR_INDEX_WIDTH(`S2_IDXW),
        .SPLIT_ENABLED(1), .BASE_ADDR(`S2_BASE)
    ) u_slave2 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s2_sim_split),
        .HSEL(HSEL2), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(ready_s2), .HRESP(resp_s2), .HRDATA(rdata_s2),
        .HSPLIT1(hsplit1_s2), .HSPLIT2(hsplit2_s2)
    );

    slave_memory #(
        .MEM_DEPTH(`S3_DEPTH), .ADDR_INDEX_WIDTH(`S3_IDXW),
        .SPLIT_ENABLED(1), .BASE_ADDR(`S3_BASE)
    ) u_slave3 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(s3_sim_split),
        .HSEL(HSEL3), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HWDATA(HWDATA), .HREADY_IN(HREADY), .HMASTER(HMASTER),
        .HREADY_OUT(ready_s3), .HRESP(resp_s3), .HRDATA(rdata_s3),
        .HSPLIT1(hsplit1_s3), .HSPLIT2(hsplit2_s3)
    );

    // ---- 7. Inter-board bridge ---------------------------------------------
    //
    // A slave for the remote window, and Master 3 for inbound traffic.
    bus_bridge u_bridge (
        .HCLK(HCLK), .HRESETn(HRESETn),

        .HSEL_REMOTE(HSEL_REMOTE), .HADDR(HADDR), .HTRANS(HTRANS),
        .HWRITE(HWRITE), .HWDATA(HWDATA), .HREADY_IN(HREADY),
        .HMASTER(HMASTER),
        .HREADY_OUT(ready_brg), .HRESP(resp_brg), .HRDATA(rdata_brg),
        .HSPLIT1(hsplit1_brg), .HSPLIT2(hsplit2_brg),

        .HBUSREQ(HBUSREQ3), .HGRANT(HGRANT3),
        .M_HADDR(addr_m3), .M_HTRANS(trans_m3), .M_HWRITE(write_m3),
        .M_HWDATA(wdata_m3),
        .M_HRDATA(HRDATA_MUX), .M_HRESP(HRESP_MUX),

        .LINK_RX(LINK_RX), .LINK_TX(LINK_TX)
    );

    // ---- 8. Observability --------------------------------------------------
    assign HMASTER_OUT       = HMASTER;
    assign HRESP_OUT         = HRESP_MUX;
    assign HADDR_INVALID_OUT = invalid_activ;
    assign bus_busy          = (HMASTER != `MASTER_NONE) || !HREADY;

endmodule
