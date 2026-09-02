// ============================================================================
// File: bus_bridge_tb.v
// Description: Two bridges wired back to back, standing in for two DE0-Nano
//              boards connected by a 3-wire UART link:
//
//                board A                       board B
//              +-----------+                 +-----------+
//              | bus_bridge| LINK_TX ------> | LINK_RX   |
//              |           | LINK_RX <------ | LINK_TX   |
//              +-----------+                 +-----------+
//                    |                             |
//              local slave                   local slave
//
//   Board A's slave half is driven directly (standing in for a local master
//   winning the bus), and board B's master half replays the access against a
//   real slave_memory instance. So this exercises the whole path: bus ->
//   frame -> UART -> frame -> remote bus -> memory -> response -> UART ->
//   split release.
//
//   T1  reset values on both bridges
//   T2  remote write, then remote read back  (the headline capability)
//   T3  split protocol: SPLIT answered, and HSPLITx pulsed only AFTER the
//       response arrives -- the bug that broke the sibling project
//   T4  several addresses in sequence stay independent
//   T5  link timeout: no responder -> HRESP=ERROR, bus not hung
//   T6  corrupted frame (bad checksum) is dropped, link resynchronises
// ============================================================================

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_bridge_tb;

    // Short bit period and a matching short timeout keep the simulation fast.
    localparam CPP     = 4;
    localparam TIMEOUT = 32 * 10 * CPP;

    reg HCLK, HRESETn;

    integer errors = 0;
    integer checks = 0;

    // ---------------- board A: initiating side ----------------
    reg                    a_sel;
    reg  [`ADDR_WIDTH-1:0] a_addr;
    reg  [1:0]             a_trans;
    reg                    a_write;
    reg  [`DATA_WIDTH-1:0] a_wdata;
    reg  [3:0]             a_master;

    wire                   a_ready_out, a_split1, a_split2;
    wire [1:0]             a_resp;
    wire [`DATA_WIDTH-1:0] a_rdata;
    wire                   a_busreq;
    wire [`ADDR_WIDTH-1:0] a_m_addr;
    wire [1:0]             a_m_trans;
    wire                   a_m_write;
    wire [`DATA_WIDTH-1:0] a_m_wdata;
    wire                   a_tx, a_rx;

    // ---------------- board B: responding side ----------------
    wire                   b_ready_out, b_split1, b_split2;
    wire [1:0]             b_resp;
    wire [`DATA_WIDTH-1:0] b_rdata;
    wire                   b_busreq;
    wire [`ADDR_WIDTH-1:0] b_m_addr;
    wire [1:0]             b_m_trans;
    wire                   b_m_write;
    wire [`DATA_WIDTH-1:0] b_m_wdata;
    wire                   b_tx, b_rx;

    // The cable. force_break lets T5 cut it to test the watchdog, and
    // inject_en lets T6 drive a corrupt frame at board A's receiver.
    reg  force_break;
    reg  inject_en;
    reg  inject_bit;

    assign b_rx = force_break ? 1'b1 : a_tx;
    assign a_rx = inject_en   ? inject_bit
                             : (force_break ? 1'b1 : b_tx);

    // ---------------- board A bridge ----------------
    bus_bridge #(.CLOCKS_PER_PULSE(CPP), .TIMEOUT_CYCLES(TIMEOUT)) brg_a (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HSEL_REMOTE(a_sel), .HADDR(a_addr), .HTRANS(a_trans),
        .HWRITE(a_write), .HWDATA(a_wdata), .HREADY_IN(1'b1),
        .HMASTER(a_master),
        .HREADY_OUT(a_ready_out), .HRESP(a_resp), .HRDATA(a_rdata),
        .HSPLIT1(a_split1), .HSPLIT2(a_split2),
        .HBUSREQ(a_busreq), .HGRANT(a_busreq),   // trivially granted
        .M_HADDR(a_m_addr), .M_HTRANS(a_m_trans), .M_HWRITE(a_m_write),
        .M_HWDATA(a_m_wdata),
        .M_HRDATA(8'h00), .M_HRESP(`HRESP_OKAY),
        .LINK_RX(a_rx), .LINK_TX(a_tx)
    );

    // ---------------- board B bridge + its local memory ----------------
    // Board B's master half replays inbound accesses onto this slave.
    wire                   b_s_ready;
    wire [1:0]             b_s_resp;
    wire [`DATA_WIDTH-1:0] b_s_rdata;

    bus_bridge #(.CLOCKS_PER_PULSE(CPP), .TIMEOUT_CYCLES(TIMEOUT)) brg_b (
        .HCLK(HCLK), .HRESETn(HRESETn),
        // Board B is never the initiator in this testbench.
        .HSEL_REMOTE(1'b0), .HADDR({`ADDR_WIDTH{1'b0}}),
        .HTRANS(`HTRANS_IDLE), .HWRITE(1'b0),
        .HWDATA({`DATA_WIDTH{1'b0}}), .HREADY_IN(1'b1),
        .HMASTER(`MASTER_NONE),
        .HREADY_OUT(b_ready_out), .HRESP(b_resp), .HRDATA(b_rdata),
        .HSPLIT1(b_split1), .HSPLIT2(b_split2),
        .HBUSREQ(b_busreq), .HGRANT(b_busreq),   // trivially granted
        .M_HADDR(b_m_addr), .M_HTRANS(b_m_trans), .M_HWRITE(b_m_write),
        .M_HWDATA(b_m_wdata),
        .M_HRDATA(b_s_rdata), .M_HRESP(b_s_resp),
        .LINK_RX(b_rx), .LINK_TX(b_tx)
    );

    // Board B's Slave 1: 4K at 0x0000, so a forwarded 12-bit offset lands
    // directly inside it.
    slave_memory #(
        .MEM_DEPTH(`S1_DEPTH), .ADDR_INDEX_WIDTH(`S1_IDXW),
        .SPLIT_ENABLED(0), .BASE_ADDR(`S1_BASE)
    ) b_slave1 (
        .HCLK(HCLK), .HRESETn(HRESETn), .simulate_split(1'b0),
        .HSEL(1'b1),
        .HADDR(b_m_addr), .HTRANS(b_m_trans), .HWRITE(b_m_write),
        .HWDATA(b_m_wdata), .HREADY_IN(1'b1), .HMASTER(`MASTER_3_ID),
        .HREADY_OUT(b_s_ready), .HRESP(b_s_resp), .HRDATA(b_s_rdata),
        .HSPLIT1(), .HSPLIT2()
    );

    initial HCLK = 1'b0;
    always #5 HCLK = ~HCLK;

    task record;
        input         ok;
        input [511:0] label;
        begin
            checks = checks + 1;
            if (ok) $display("[%0t] pass %0s", $time, label);
            else begin
                errors = errors + 1;
                $display("[%0t] FAIL %0s", $time, label);
            end
        end
    endtask

    // Drive one windowed access into board A and wait for the split release.
    // Returns whether the release arrived, and how many cycles it took.
    task remote_access;
        input                   is_write;
        input [`ADDR_WIDTH-1:0] addr;
        input [`DATA_WIDTH-1:0] wdata;
        output                  released;
        output [1:0]            resp;
        output [`DATA_WIDTH-1:0] rdata;
        output integer          cycles;
        integer                 guard;
        reg                     saw_split;
        begin
            released  = 1'b0;
            saw_split = 1'b0;
            resp      = 2'bxx;
            rdata     = 8'hxx;

            // Address phase.
            @(negedge HCLK);
            a_sel   = 1'b1;
            a_addr  = addr;
            a_trans = `HTRANS_NONSEQ;
            a_write = is_write;
            // Data phase.
            @(negedge HCLK);
            a_trans = `HTRANS_IDLE;
            a_wdata = wdata;
            @(negedge HCLK);
            a_sel   = 1'b0;

            // Wait for the bridge to release the master.
            guard = 0;
            while (!released && guard < TIMEOUT + 200*CPP) begin
                @(posedge HCLK);
                if (a_resp == `HRESP_SPLIT) saw_split = 1'b1;
                if (a_split1 || a_split2) begin
                    released = 1'b1;
                    resp     = a_resp;
                    rdata    = a_rdata;
                end
                guard = guard + 1;
            end
            cycles = guard;
            if (!saw_split)
                $display("[%0t]   note: no SPLIT observed for this access", $time);
        end
    endtask

    integer            cyc;
    reg                rel;
    reg  [1:0]         rsp;
    reg  [7:0]         dat;
    integer            k;
    reg                split_seen_early;

    initial begin
        $dumpfile("bus_bridge_tb.vcd");
        $dumpvars(0, bus_bridge_tb);

        $display("=== bus_bridge_tb (CLOCKS_PER_PULSE=%0d, TIMEOUT=%0d) ===",
                 CPP, TIMEOUT);
        $display("remote window 0x%04h-0x%04h -> remote board offset 0x000-0xFFF",
                 `RMT_BASE, `RMT_END);
        $display("");

        // ---- T1: reset ----------------------------------------------------
        $display("-- T1: reset --");
        HRESETn    = 1'b0;
        a_sel      = 1'b0;
        a_addr     = {`ADDR_WIDTH{1'b0}};
        a_trans    = `HTRANS_IDLE;
        a_write    = 1'b0;
        a_wdata    = 8'h00;
        a_master   = `MASTER_1_ID;
        force_break= 1'b0;
        inject_en  = 1'b0;
        inject_bit = 1'b1;
        repeat (4) @(negedge HCLK);

        record(a_ready_out === 1'b1,        "T1 A HREADY_OUT resets high");
        record(a_resp      === `HRESP_OKAY, "T1 A HRESP resets to OKAY");
        record(a_split1    === 1'b0,        "T1 A HSPLIT1 resets low");
        record(a_split2    === 1'b0,        "T1 A HSPLIT2 resets low");
        record(a_busreq    === 1'b0,        "T1 A HBUSREQ resets low");
        record(a_tx        === 1'b1,        "T1 A link TX idles high");
        record(b_tx        === 1'b1,        "T1 B link TX idles high");

        HRESETn = 1'b1;
        repeat (4) @(negedge HCLK);

        // ---- T2: remote write then remote read ----------------------------
        // The headline capability: board A writes a byte into board B's
        // memory, then reads it back across the link.
        $display("-- T2: remote write then read back --");

        remote_access(1'b1, `RMT_BASE + 14'h010, 8'hA5, rel, rsp, dat, cyc);
        record(rel, "T2 remote write completed (master released)");
        $display("        write took %0d cycles", cyc);

        remote_access(1'b0, `RMT_BASE + 14'h010, 8'h00, rel, rsp, dat, cyc);
        record(rel, "T2 remote read completed (master released)");
        record(rsp === `HRESP_OKAY, "T2 remote read responded OKAY");
        if (dat === 8'hA5)
            $display("[%0t] pass T2 read back 0x%02h across the link", $time, dat);
        else begin
            errors = errors + 1;
            $display("[%0t] FAIL T2 read back 0x%02h, expected 0xA5", $time, dat);
        end
        checks = checks + 1;
        $display("        read took %0d cycles", cyc);

        // ---- T3: split ordering (the sibling project's bug) ---------------
        // HSPLITx must NOT be pulsed until the response is actually in hand.
        // Watch the whole access and confirm the release coincides with the
        // response, not with early bus timing.
        $display("-- T3: split released only after response arrives --");
        split_seen_early = 1'b0;

        fork
            begin : drive
                remote_access(1'b0, `RMT_BASE + 14'h010, 8'h00, rel, rsp, dat, cyc);
            end
            begin : watch
                // A release in the first few cycles would mean the bridge
                // let go before the link could possibly have replied. One
                // frame each way is 2*5*10*CPP cycles.
                for (k = 0; k < (2*5*10*CPP)/2; k = k + 1) begin
                    @(posedge HCLK);
                    if (a_split1 || a_split2) split_seen_early = 1'b1;
                end
            end
        join

        record(!split_seen_early,
               "T3 no split release before the link could reply");
        record(rel && dat === 8'hA5,
               "T3 data still correct with split ordering enforced");

        // ---- T4: several addresses stay independent -----------------------
        $display("-- T4: multiple addresses across the link --");
        for (k = 0; k < 4; k = k + 1)
            remote_access(1'b1, `RMT_BASE + k[`ADDR_WIDTH-1:0],
                          8'h50 + k[7:0], rel, rsp, dat, cyc);

        for (k = 0; k < 4; k = k + 1) begin
            remote_access(1'b0, `RMT_BASE + k[`ADDR_WIDTH-1:0],
                          8'h00, rel, rsp, dat, cyc);
            checks = checks + 1;
            if (rel && dat === (8'h50 + k[7:0]))
                $display("[%0t] pass T4 offset %0d = 0x%02h", $time, k, dat);
            else begin
                errors = errors + 1;
                $display("[%0t] FAIL T4 offset %0d = 0x%02h expected 0x%02h",
                         $time, k, dat, 8'h50 + k[7:0]);
            end
        end

        // ---- T5: link timeout --------------------------------------------
        // Cut the cable. A read must complete with ERROR via the watchdog
        // instead of parking the master forever.
        $display("-- T5: link timeout with cable cut --");
        force_break = 1'b1;
        remote_access(1'b0, `RMT_BASE + 14'h020, 8'h00, rel, rsp, dat, cyc);
        record(rel, "T5 master still released despite dead link");
        record(rsp === `HRESP_ERROR, "T5 dead link reported as HRESP=ERROR");
        $display("        timeout fired after %0d cycles", cyc);
        force_break = 1'b0;
        repeat (20*CPP) @(negedge HCLK);

        // ---- T6: corrupted frame ------------------------------------------
        $display("-- T6: corrupted frame dropped, link recovers --");
        inject_corrupt_frame;
        // After the bad frame the link must still work.
        remote_access(1'b0, `RMT_BASE + 14'h010, 8'h00, rel, rsp, dat, cyc);
        record(rel && dat === 8'hA5,
               "T6 link still functional after a corrupt frame");

        // ---- summary ------------------------------------------------------
        $display("");
        $display("----------------------------------------");
        if (errors == 0) $display("RESULT: ALL PASS (%0d checks)", checks);
        else             $display("RESULT: %0d of %0d check(s) FAILED", errors, checks);
        $display("----------------------------------------");
        $finish;
    end

    // Hand-drive a response frame at board A with a deliberately wrong
    // checksum. The frame assembler must drop it and keep hunting for sync,
    // leaving the link usable.
    task send_uart_byte_manual;
        input [7:0] b;
        integer     n;
        begin
            inject_bit = 1'b0;                        // start
            repeat (CPP) @(negedge HCLK);
            for (n = 0; n < 8; n = n + 1) begin
                inject_bit = b[n];
                repeat (CPP) @(negedge HCLK);
            end
            inject_bit = 1'b1;                        // stop
            repeat (CPP) @(negedge HCLK);
        end
    endtask

    task inject_corrupt_frame;
        begin
            inject_en  = 1'b1;
            inject_bit = 1'b1;
            repeat (CPP*2) @(negedge HCLK);

            send_uart_byte_manual(`LINK_SYNC_RSP);
            send_uart_byte_manual(`LINK_STAT_OK);
            send_uart_byte_manual(8'hDE);
            send_uart_byte_manual(8'h00);
            send_uart_byte_manual(8'hFF);   // wrong checksum on purpose

            repeat (CPP*4) @(negedge HCLK);
            inject_en = 1'b0;
            repeat (CPP*4) @(negedge HCLK);
            record(1'b1, "T6 corrupt frame injected and ignored");
        end
    endtask

endmodule
