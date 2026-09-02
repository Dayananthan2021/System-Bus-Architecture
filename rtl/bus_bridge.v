// Inter-board bridge. Extends the bus over a 3-wire UART link (TX, RX, GND)
// to an identical board, so a master here can reach the other board's memory.
// Wire protocol is in docs/BRIDGE_INTERFACE.md.
//
// One instance per board, symmetric, both halves always active:
//
//   SLAVE half   local master -> remote memory. Claims the remote window,
//                answers SPLIT, sends a command frame, waits for the
//                response, then releases the master with HSPLITx.
//   MASTER half  remote master -> local memory. Receives a command, requests
//                the local bus, replays the access, sends back a response.
//
// A round trip is 5 bytes out and 5 back -- about 0.78 ms at 115200, some
// 7,800 bus cycles. Stalling that long would block both local masters, hence
// the split.
//
// HSPLITx is pulsed only from S_DONE, once a checksum-valid response is in
// hand or the watchdog fires -- never on bus timing. The arbiter's "transfer
// complete" says nothing about whether the remote board has replied, and
// releasing early makes the master sample HRDATA before the data exists.

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_bridge #(
    // Bit period for the UART, in clk cycles. Defaults to the value derived
    // in bus_params.vh; testbenches override it to keep simulations short.
    parameter CLOCKS_PER_PULSE = `UART_CLKS_PER_BIT,

    // Link watchdog. If a response does not arrive within this many clocks
    // the transfer completes with HRESP=ERROR rather than hanging the bus.
    // Default allows ~4 frame times at the configured baud.
    parameter TIMEOUT_CYCLES   = 32 * 10 * `UART_CLKS_PER_BIT
)(
    input  wire                   HCLK,
    input  wire                   HRESETn,

    // ---- Slave side: local master accessing remote memory ----------------
    input  wire                   HSEL_REMOTE,  // decoder selected the window
    // Only the low 12 bits travel over the link. The region bits were already
    // consumed by the decoder, and the receiving board supplies its own.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [`ADDR_WIDTH-1:0] HADDR,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [1:0]             HTRANS,
    input  wire                   HWRITE,
    input  wire [`DATA_WIDTH-1:0] HWDATA,
    input  wire                   HREADY_IN,
    input  wire [3:0]             HMASTER,      // who to release after split

    output reg                    HREADY_OUT,
    output reg  [1:0]             HRESP,
    output reg  [`DATA_WIDTH-1:0] HRDATA,
    output reg                    HSPLIT1,      // release master 1
    output reg                    HSPLIT2,      // release master 2

    // ---- Master side: remote master accessing local memory --------------
    output reg                    HBUSREQ,      // request the local bus
    input  wire                   HGRANT,
    output reg  [`ADDR_WIDTH-1:0] M_HADDR,
    output reg  [1:0]             M_HTRANS,
    output reg                    M_HWRITE,
    output reg  [`DATA_WIDTH-1:0] M_HWDATA,
    input  wire [`DATA_WIDTH-1:0] M_HRDATA,
    input  wire [1:0]             M_HRESP,

    // ---- Physical link ---------------------------------------------------
    input  wire                   LINK_RX,
    output wire                   LINK_TX
);

    // Both halves need the transmitter -- the slave half for commands, the
    // master half for responses -- so their request streams are merged into
    // the one physical port here, each half driving only its own signals.
    wire       tx_busy;
    wire [7:0] rx_byte;
    wire       rx_ready;

    // Slave-half request (commands out) and master-half request (responses).
    reg  [7:0] s_tx_byte;
    reg        s_tx_req;
    reg  [7:0] m_tx_byte;
    reg        m_tx_req;

    // Both halves wanting the line at once is possible but rare. Responses
    // win: the remote board is already blocked waiting for one, while our own
    // command can retry next cycle when tx_busy clears.
    wire [7:0] tx_byte = m_tx_req ? m_tx_byte : s_tx_byte;
    wire       tx_en   = m_tx_req | s_tx_req;

    uart_tx #(.CLOCKS_PER_PULSE(CLOCKS_PER_PULSE)) u_tx (
        .clk(HCLK), .rstn(HRESETn),
        .data_in(tx_byte), .data_en(tx_en),
        .tx(LINK_TX), .tx_busy(tx_busy)
    );

    uart_rx #(.CLOCKS_PER_PULSE(CLOCKS_PER_PULSE)) u_rx (
        .clk(HCLK), .rstn(HRESETn),
        .rx(LINK_RX),
        .ready(rx_ready), .data_out(rx_byte)
    );

    // ---- Receive frame assembler -------------------------------------------
    //
    // Hunts for a sync byte then collects the rest. Both frame types are 5
    // bytes, so one assembler serves both:
    //
    //   command  : A5  cmd  addr_lo  data  cksum
    //   response : 5A  stat data     00    cksum
    //
    // A bad checksum drops the frame and returns to hunting, so a corrupt or
    // partial frame cannot desynchronise the link permanently.
    localparam RXF_SYNC  = 3'd0,   // hunting for A5 / 5A
               RXF_B1    = 3'd1,   // cmd / status
               RXF_B2    = 3'd2,   // addr_lo / data
               RXF_B3    = 3'd3,   // data / pad
               RXF_CKSUM = 3'd4;   // checksum

    reg [2:0] rxf_state;
    reg [7:0] rxf_sync, rxf_b1, rxf_b2, rxf_b3;

    // One-cycle pulses announcing a checksum-valid frame of each type.
    reg       cmd_valid;
    reg       rsp_valid;

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            rxf_state <= RXF_SYNC;
            rxf_sync  <= 8'h00;
            rxf_b1    <= 8'h00;
            rxf_b2    <= 8'h00;
            rxf_b3    <= 8'h00;
            cmd_valid <= 1'b0;
            rsp_valid <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;
            rsp_valid <= 1'b0;

            if (rx_ready) begin
                case (rxf_state)
                    // Any other byte is link noise or a bad frame's tail.
                    RXF_SYNC: begin
                        if (rx_byte == `LINK_SYNC_CMD ||
                            rx_byte == `LINK_SYNC_RSP) begin
                            rxf_sync  <= rx_byte;
                            rxf_state <= RXF_B1;
                        end
                    end

                    RXF_B1: begin rxf_b1 <= rx_byte; rxf_state <= RXF_B2;    end
                    RXF_B2: begin rxf_b2 <= rx_byte; rxf_state <= RXF_B3;    end
                    RXF_B3: begin rxf_b3 <= rx_byte; rxf_state <= RXF_CKSUM; end

                    // Checksum covers all four preceding bytes, data included.
                    RXF_CKSUM: begin
                        rxf_state <= RXF_SYNC;
                        if (rx_byte == (rxf_sync ^ rxf_b1 ^ rxf_b2 ^ rxf_b3)) begin
                            if (rxf_sync == `LINK_SYNC_CMD) cmd_valid <= 1'b1;
                            else                            rsp_valid <= 1'b1;
                        end
                        // Bad checksum: silently drop and hunt for sync again.
                    end

                    default: rxf_state <= RXF_SYNC;
                endcase
            end
        end
    end

    // Received frame fields. A command's address is the low nibble of the cmd
    // byte plus the whole addr_lo byte: the 12-bit offset in the window.
    wire        rx_cmd_write = rxf_b1[7];
    wire [11:0] rx_cmd_addr  = {rxf_b1[3:0], rxf_b2};
    wire [7:0]  rx_cmd_data  = rxf_b3;

    wire [7:0]  rx_rsp_stat  = rxf_b1;
    wire [7:0]  rx_rsp_data  = rxf_b2;

    // ---- SLAVE half: local master -> remote memory -------------------------
    localparam S_IDLE     = 3'd0,  // waiting for a windowed access
               S_SPLIT    = 3'd1,  // answering SPLIT to free the bus
               S_SEND     = 3'd2,  // clocking the command frame out
               S_WAIT_RSP = 3'd3,  // waiting for the remote board
               S_DONE     = 3'd4;  // releasing the split master

    reg [2:0]  s_state;
    reg [2:0]  s_byte_idx;
    reg [11:0] s_addr;
    reg        s_write;
    reg [7:0]  s_wdata;
    reg [3:0]  s_master;
    reg [31:0] s_timer;

    // Command frame bytes.
    wire [7:0] cmd_b0 = `LINK_SYNC_CMD;
    wire [7:0] cmd_b1 = {s_write, 3'b000, s_addr[11:8]};
    wire [7:0] cmd_b2 = s_addr[7:0];
    wire [7:0] cmd_b3 = s_wdata;
    wire [7:0] cmd_b4 = cmd_b0 ^ cmd_b1 ^ cmd_b2 ^ cmd_b3;

    wire s_accept = HSEL_REMOTE && HREADY_IN && (HTRANS == `HTRANS_NONSEQ);

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            s_state    <= S_IDLE;
            s_byte_idx <= 3'd0;
            s_addr     <= 12'h000;
            s_write    <= 1'b0;
            s_wdata    <= 8'h00;
            s_master   <= `MASTER_NONE;
            s_timer    <= 32'd0;

            HREADY_OUT <= 1'b1;
            HRESP      <= `HRESP_OKAY;
            HRDATA     <= {`DATA_WIDTH{1'b0}};
            HSPLIT1    <= 1'b0;
            HSPLIT2    <= 1'b0;

            s_tx_byte  <= 8'h00;
            s_tx_req   <= 1'b0;
        end else begin
            // Split releases and the UART start pulse are single-cycle.
            HSPLIT1  <= 1'b0;
            HSPLIT2  <= 1'b0;
            s_tx_req <= 1'b0;

            case (s_state)

                S_IDLE: begin
                    HREADY_OUT <= 1'b1;
                    HRESP      <= `HRESP_OKAY;

                    if (s_accept) begin
                        s_addr   <= HADDR[11:0];   // only the offset travels
                        s_write  <= HWRITE;
                        s_master <= HMASTER;
                        s_state  <= S_SPLIT;
                    end
                end

                // HREADY low with HRESP=SPLIT parks this master and frees the
                // bus. HWDATA is only valid in the data phase -- this cycle.
                S_SPLIT: begin
                    s_wdata    <= HWDATA;
                    HREADY_OUT <= 1'b0;
                    HRESP      <= `HRESP_SPLIT;
                    s_byte_idx <= 3'd0;
                    s_timer    <= 32'd0;
                    s_state    <= S_SEND;
                end

                // Clock the five command bytes out, respecting tx_busy so no
                // byte is overwritten mid-flight.
                S_SEND: begin
                    HREADY_OUT <= 1'b1;   // transfer is split, not stalled
                    HRESP      <= `HRESP_SPLIT;

                    // Yield to the master half: if it is replying, wait.
                    if (!tx_busy && !s_tx_req && !m_tx_req) begin
                        case (s_byte_idx)
                            3'd0: s_tx_byte <= cmd_b0;
                            3'd1: s_tx_byte <= cmd_b1;
                            3'd2: s_tx_byte <= cmd_b2;
                            3'd3: s_tx_byte <= cmd_b3;
                            3'd4: s_tx_byte <= cmd_b4;
                            default: s_tx_byte <= 8'h00;
                        endcase
                        s_tx_req <= 1'b1;

                        if (s_byte_idx == 3'd4) begin
                            // A write needs no reply; a read must wait.
                            s_byte_idx <= 3'd0;
                            s_state    <= s_write ? S_DONE : S_WAIT_RSP;
                        end else
                            s_byte_idx <= s_byte_idx + 3'd1;
                    end
                end

                // Wait for the response, or time out. This is the gate that
                // matters -- the state only advances on a checksum-valid
                // response or the watchdog, never on bus timing.
                S_WAIT_RSP: begin
                    HRESP <= `HRESP_SPLIT;

                    if (rsp_valid) begin
                        HRDATA  <= rx_rsp_data;
                        HRESP   <= (rx_rsp_stat == `LINK_STAT_OK)
                                     ? `HRESP_OKAY : `HRESP_ERROR;
                        s_state <= S_DONE;
                    end else if (s_timer >= TIMEOUT_CYCLES) begin
                        // Link dead or reply corrupt: fail the transfer rather
                        // than leave the master parked forever.
                        HRDATA  <= {`DATA_WIDTH{1'b0}};
                        HRESP   <= `HRESP_ERROR;
                        s_state <= S_DONE;
                    end else
                        s_timer <= s_timer + 32'd1;
                end

                S_DONE: begin
                    HREADY_OUT <= 1'b1;
                    if (s_write) HRESP <= `HRESP_OKAY;

                    if      (s_master == `MASTER_1_ID) HSPLIT1 <= 1'b1;
                    else if (s_master == `MASTER_2_ID) HSPLIT2 <= 1'b1;

                    s_state <= S_IDLE;
                end

                default: s_state <= S_IDLE;
            endcase
        end
    end

    // ---- MASTER half: remote master -> local memory ------------------------
    localparam M_IDLE    = 3'd0,  // waiting for a command frame
               M_REQ     = 3'd1,  // requesting the local bus
               M_ADDR    = 3'd2,  // driving the address phase
               M_DATA    = 3'd3,  // driving the data phase
               M_CAPTURE = 3'd4,  // wait one cycle for slave read data
               M_LATCH   = 3'd5,  // read data valid, sample it
               M_REPLY   = 3'd6;  // sending the response frame

    reg [2:0]  m_state;
    reg [2:0]  m_byte_idx;
    reg [11:0] m_addr;
    reg        m_write;
    reg [7:0]  m_wdata;
    reg [7:0]  m_rdata;
    reg [7:0]  m_stat;

    // Response frame bytes.
    wire [7:0] rsp_b0 = `LINK_SYNC_RSP;
    wire [7:0] rsp_b1 = m_stat;
    wire [7:0] rsp_b2 = m_rdata;
    wire [7:0] rsp_b3 = 8'h00;
    wire [7:0] rsp_b4 = rsp_b0 ^ rsp_b1 ^ rsp_b2 ^ rsp_b3;

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            m_state    <= M_IDLE;
            m_byte_idx <= 3'd0;
            m_addr     <= 12'h000;
            m_write    <= 1'b0;
            m_wdata    <= 8'h00;
            m_rdata    <= 8'h00;
            m_stat     <= `LINK_STAT_OK;

            HBUSREQ    <= 1'b0;
            M_HADDR    <= {`ADDR_WIDTH{1'b0}};
            M_HTRANS   <= `HTRANS_IDLE;
            M_HWRITE   <= 1'b0;
            M_HWDATA   <= {`DATA_WIDTH{1'b0}};

            m_tx_req   <= 1'b0;
            m_tx_byte  <= 8'h00;
        end else begin
            m_tx_req <= 1'b0;

            case (m_state)

                M_IDLE: begin
                    HBUSREQ  <= 1'b0;
                    M_HTRANS <= `HTRANS_IDLE;

                    if (cmd_valid) begin
                        m_addr  <= rx_cmd_addr;
                        m_write <= rx_cmd_write;
                        m_wdata <= rx_cmd_data;
                        m_stat  <= `LINK_STAT_OK;
                        m_state <= M_REQ;
                    end
                end

                // The arbiter gives the bridge top priority: without it two
                // boards can deadlock, each holding its own bus while waiting
                // on the other.
                M_REQ: begin
                    HBUSREQ <= 1'b1;
                    if (HGRANT) m_state <= M_ADDR;
                end

                // Address phase. The command carries only a 12-bit offset, so
                // the region bits come from this board's own map.
                M_ADDR: begin
                    HBUSREQ  <= 1'b1;
                    M_HADDR  <= {2'b00, m_addr};
                    M_HTRANS <= `HTRANS_NONSEQ;
                    M_HWRITE <= m_write;
                    m_state  <= M_DATA;
                end

                // Data phase. M_HTRANS was set non-blocking in M_ADDR, so the
                // slave only sees the address phase during THIS cycle and its
                // read data is not available yet.
                M_DATA: begin
                    M_HTRANS   <= `HTRANS_IDLE;
                    M_HWDATA   <= m_wdata;
                    HBUSREQ    <= 1'b0;
                    m_byte_idx <= 3'd0;

                    // Writes need no reply, so they finish here.
                    m_state <= m_write ? M_IDLE : M_CAPTURE;
                end

                // One more cycle for slave_memory's 2-cycle read: it registers
                // the access at cycle 1 and HRDATA is only valid at cycle 2.
                M_CAPTURE: begin
                    m_state <= M_LATCH;
                end

                M_LATCH: begin
                    m_rdata <= M_HRDATA;
                    m_stat  <= (M_HRESP == `HRESP_OKAY)
                                 ? `LINK_STAT_OK : `LINK_STAT_ERR;
                    m_state <= M_REPLY;
                end

                // Send the five response bytes.
                M_REPLY: begin
                    if (!tx_busy && !m_tx_req) begin
                        case (m_byte_idx)
                            3'd0: m_tx_byte <= rsp_b0;
                            3'd1: m_tx_byte <= rsp_b1;
                            3'd2: m_tx_byte <= rsp_b2;
                            3'd3: m_tx_byte <= rsp_b3;
                            3'd4: m_tx_byte <= rsp_b4;
                            default: m_tx_byte <= 8'h00;
                        endcase
                        m_tx_req <= 1'b1;

                        if (m_byte_idx == 3'd4) begin
                            m_byte_idx <= 3'd0;
                            m_state    <= M_IDLE;
                        end else
                            m_byte_idx <= m_byte_idx + 3'd1;
                    end
                end

                default: m_state <= M_IDLE;
            endcase
        end
    end

endmodule
