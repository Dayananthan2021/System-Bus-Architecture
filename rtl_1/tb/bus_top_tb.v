<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
// ============================================================================
// File: bus_top_tb.v
// Description: Top-level integration verification of the whole bus.
//
//   Covers the four scenarios the assignment brief makes mandatory:
//
//     (a) T1  reset test        -- every signal reaches its reset value
//     (b) T2  one master        -- single master accessing the bus
//     (c) T3  two masters       -- priority arbitration between M1 and M2
//     (d) T4  split transaction -- slave splits, master handles the response
//
//   Plus integration checks that only appear once everything is wired:
//
//     T5  all three slaves individually addressable
//     T6  invalid address reports an error and does not hang the bus
//     T7  remote window drives the inter-board link
// ============================================================================

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_top_tb;

    reg HCLK, HRESETn;

    reg                    m1_trigger, m1_write;
    reg  [`ADDR_WIDTH-1:0] m1_addr;
    reg  [`DATA_WIDTH-1:0] m1_wdata;
    wire [`DATA_WIDTH-1:0] m1_rdata;

    reg                    m2_trigger, m2_write;
    reg  [`ADDR_WIDTH-1:0] m2_addr;
    reg  [`DATA_WIDTH-1:0] m2_wdata;
    wire [`DATA_WIDTH-1:0] m2_rdata;

    reg                    s1_sim_split, s2_sim_split, s3_sim_split;

    wire [3:0]             HMASTER_OUT;
    wire [1:0]             HRESP_OUT;
    wire                   HADDR_INVALID_OUT;
    wire                   bus_busy;
    wire                   LINK_TX;
    reg                    LINK_RX;

    integer errors = 0;
    integer checks = 0;

    bus_top dut (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .LINK_RX(LINK_RX), .LINK_TX(LINK_TX),
        .m1_trigger(m1_trigger), .m1_write(m1_write),
        .m1_addr(m1_addr), .m1_wdata(m1_wdata), .m1_rdata(m1_rdata),
        .m2_trigger(m2_trigger), .m2_write(m2_write),
        .m2_addr(m2_addr), .m2_wdata(m2_wdata), .m2_rdata(m2_rdata),
        .s1_sim_split(s1_sim_split), .s2_sim_split(s2_sim_split),
        .s3_sim_split(s3_sim_split),
        .HMASTER_OUT(HMASTER_OUT), .HRESP_OUT(HRESP_OUT),
        .HADDR_INVALID_OUT(HADDR_INVALID_OUT), .bus_busy(bus_busy)
    );

    initial HCLK = 1'b0;
    always #5 HCLK = ~HCLK;

    task step; begin @(negedge HCLK); end endtask

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

    task record_data;
        input [`DATA_WIDTH-1:0] got, exp;
        input [511:0]           label;
        begin
            checks = checks + 1;
            if (got === exp)
                $display("[%0t] pass %0s: 0x%02h", $time, label, got);
            else begin
                errors = errors + 1;
                $display("[%0t] FAIL %0s: got 0x%02h expected 0x%02h",
                         $time, label, got, exp);
            end
        end
    endtask

    // Run one transfer on master 1 and wait for it to finish.
    task m1_access;
        input                   wr;
        input [`ADDR_WIDTH-1:0] addr;
        input [`DATA_WIDTH-1:0] data;
        input integer           max_cycles;
        output                  done;
        integer                 guard;
        begin
            m1_write = wr; m1_addr = addr; m1_wdata = data;
            @(negedge HCLK); m1_trigger = 1'b1;

            // master_device returns to S_IDLE only after the trigger drops,
            // so wait for it to reach S_DONE first.
            guard = 0; done = 1'b0;
            while (!done && guard < max_cycles) begin
                @(posedge HCLK);
                if (dut.u_master1.state == 3'd4) done = 1'b1;   // S_DONE
                guard = guard + 1;
            end

            @(negedge HCLK); m1_trigger = 1'b0;
            repeat (3) step;
        end
    endtask

    task m2_access;
        input                   wr;
        input [`ADDR_WIDTH-1:0] addr;
        input [`DATA_WIDTH-1:0] data;
        input integer           max_cycles;
        output                  done;
        integer                 guard;
        begin
            m2_write = wr; m2_addr = addr; m2_wdata = data;
            @(negedge HCLK); m2_trigger = 1'b1;

            guard = 0; done = 1'b0;
            while (!done && guard < max_cycles) begin
                @(posedge HCLK);
                if (dut.u_master2.state == 3'd4) done = 1'b1;
                guard = guard + 1;
            end

            @(negedge HCLK); m2_trigger = 1'b0;
            repeat (3) step;
        end
    endtask

    reg     ok1, ok2;
    integer k;
    reg     saw_m1, saw_m2, m1_first;
    reg     saw_split;
    reg     link_went_low;

    initial begin
        $dumpfile("bus_top_tb.vcd");
        $dumpvars(0, bus_top_tb);

        $display("=== bus_top_tb ===");
        $display("map: S1 4K@0x%04h  S2 4K@0x%04h  S3 2K@0x%04h  remote@0x%04h",
                 `S1_BASE, `S2_BASE, `S3_BASE, `RMT_BASE);
        $display("");

        // ================================================================
        // (a) T1: RESET TEST -- mandatory
        // ================================================================
        $display("-- T1: reset test (mandatory) --");
        HRESETn      = 1'b0;
        m1_trigger   = 1'b0; m1_write = 1'b0; m1_addr = 0; m1_wdata = 0;
        m2_trigger   = 1'b0; m2_write = 1'b0; m2_addr = 0; m2_wdata = 0;
        s1_sim_split = 1'b0; s2_sim_split = 1'b0; s3_sim_split = 1'b0;
        LINK_RX      = 1'b1;
        repeat (4) step;

        record(HMASTER_OUT       === `MASTER_NONE, "T1 HMASTER resets to 0 (no owner)");
        record(HRESP_OUT         === `HRESP_OKAY,  "T1 HRESP resets to OKAY");
        record(HADDR_INVALID_OUT === 1'b0,         "T1 invalid-address flag resets low");
        record(bus_busy          === 1'b0,         "T1 bus_busy resets low");
        record(m1_rdata          === 8'h00,        "T1 M1 read data resets to 0");
        record(m2_rdata          === 8'h00,        "T1 M2 read data resets to 0");
        record(LINK_TX           === 1'b1,         "T1 link TX idles high");
        record(dut.u_arbiter.HGRANT1 === 1'b0,     "T1 grant 1 resets low");
        record(dut.u_arbiter.HGRANT2 === 1'b0,     "T1 grant 2 resets low");
        record(dut.HBUSREQ1      === 1'b0,         "T1 M1 bus request resets low");
        record(dut.HBUSREQ2      === 1'b0,         "T1 M2 bus request resets low");

        HRESETn = 1'b1;
        repeat (3) step;

        // ================================================================
        // (b) T2: ONE MASTER -- mandatory
        // ================================================================
        $display("-- T2: single master transfer (mandatory) --");

        m1_access(1'b1, `S1_BASE + 14'h020, 8'h3C, 200, ok1);
        record(ok1, "T2 M1 write to Slave 1 completed");

        m1_access(1'b0, `S1_BASE + 14'h020, 8'h00, 200, ok1);
        record(ok1, "T2 M1 read from Slave 1 completed");
        record_data(m1_rdata, 8'h3C, "T2 M1 read back its own write");

        // ================================================================
        // (c) T3: TWO MASTERS -- mandatory
        // ================================================================
        // Both request together. M1 has priority, so it must be served first.
        $display("-- T3: two masters, priority arbitration (mandatory) --");

        m1_write = 1'b1; m1_addr = `S1_BASE + 14'h030; m1_wdata = 8'hAA;
        m2_write = 1'b1; m2_addr = `S2_BASE + 14'h030; m2_wdata = 8'hBB;

        saw_m1 = 1'b0; saw_m2 = 1'b0; m1_first = 1'b0;

        @(negedge HCLK);
        m1_trigger = 1'b1;
        m2_trigger = 1'b1;      // simultaneous request

        for (k = 0; k < 400; k = k + 1) begin
            @(posedge HCLK);
            if (HMASTER_OUT == `MASTER_1_ID && !saw_m1) begin
                saw_m1 = 1'b1;
                if (!saw_m2) m1_first = 1'b1;   // M1 got there first
            end
            if (HMASTER_OUT == `MASTER_2_ID && !saw_m2) saw_m2 = 1'b1;
        end

        @(negedge HCLK); m1_trigger = 1'b0; m2_trigger = 1'b0;
        repeat (5) step;

        record(saw_m1 && saw_m2, "T3 both masters were granted the bus");
        record(m1_first,         "T3 M1 won the simultaneous request (priority)");

        // Both writes must have landed, so neither master was starved.
        m1_access(1'b0, `S1_BASE + 14'h030, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'hAA, "T3 M1's write survived arbitration");

        m2_access(1'b0, `S2_BASE + 14'h030, 8'h00, 200, ok2);
        record_data(m2_rdata, 8'hBB, "T3 M2's write survived arbitration");

        // ================================================================
        // (d) T4: SPLIT TRANSACTION -- mandatory
        // ================================================================
        // Slave 2 splits. The master must back off, be re-granted after the
        // slave releases it, and still complete correctly.
        $display("-- T4: split transaction (mandatory) --");

        // Seed a known value while splitting is off.
        m1_access(1'b1, `S2_BASE + 14'h040, 8'h5A, 200, ok1);
        record(ok1, "T4 seed write to Slave 2 completed");

        // Now make Slave 2 split the next access.
        s2_sim_split = 1'b1;
        saw_split    = 1'b0;

        m1_write = 1'b0; m1_addr = `S2_BASE + 14'h040; m1_wdata = 8'h00;
        @(negedge HCLK); m1_trigger = 1'b1;

        for (k = 0; k < 600; k = k + 1) begin
            @(posedge HCLK);
            if (HRESP_OUT == `HRESP_SPLIT) saw_split = 1'b1;
            if (dut.u_master1.state == 3'd4) k = 600;   // S_DONE, stop early
        end

        @(negedge HCLK); m1_trigger = 1'b0;
        repeat (5) step;

        record(saw_split, "T4 slave answered HRESP=SPLIT");
        record(dut.u_master1.state == 3'd0 || dut.u_master1.state == 3'd4,
               "T4 master recovered from the split (not stuck)");
        record_data(m1_rdata, 8'h5A, "T4 split read still returned correct data");

        s2_sim_split = 1'b0;
        repeat (5) step;

        // While M1 was split, the arbiter must have been able to hand the bus
        // to M2 -- that is the whole point of splitting.
        record(HMASTER_OUT !== 4'hx, "T4 bus left in a defined state after split");

        // ================================================================
        // T5: all three slaves reachable
        // ================================================================
        $display("-- T5: all three slaves individually addressable --");

        m1_access(1'b1, `S1_BASE + 14'h100, 8'h11, 200, ok1);
        m1_access(1'b0, `S1_BASE + 14'h100, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'h11, "T5 Slave 1 write/read");

        m1_access(1'b1, `S2_BASE + 14'h100, 8'h22, 200, ok1);
        m1_access(1'b0, `S2_BASE + 14'h100, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'h22, "T5 Slave 2 write/read");

        // Slave 3 is the one the old decoder could never reach.
        m1_access(1'b1, `S3_BASE + 14'h100, 8'h33, 200, ok1);
        m1_access(1'b0, `S3_BASE + 14'h100, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'h33, "T5 Slave 3 write/read (was unreachable)");

        // Distinct regions must not alias onto each other.
        m1_access(1'b0, `S1_BASE + 14'h100, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'h11, "T5 Slave 1 undisturbed by S2/S3 writes");

        // ================================================================
        // T6: invalid address
        // ================================================================
        // 0x2800-0x2FFF is the hole above Slave 3. It must error, not hang.
        $display("-- T6: invalid address handling --");

        m1_access(1'b0, 14'h2900, 8'h00, 300, ok1);
        record(ok1, "T6 access to unmapped address terminated (bus not hung)");

        // The bus must still work afterwards.
        m1_access(1'b0, `S1_BASE + 14'h100, 8'h00, 200, ok1);
        record_data(m1_rdata, 8'h11, "T6 bus still functional after invalid access");

        // ================================================================
        // T7: remote window drives the link
        // ================================================================
        // No board is attached, so the transfer will time out -- but the
        // command frame must still go out on LINK_TX, proving the decoder
        // routes the window to the bridge.
        $display("-- T7: remote window drives the inter-board link --");

        link_went_low = 1'b0;
        m1_write = 1'b1; m1_addr = `RMT_BASE + 14'h010; m1_wdata = 8'h99;
        @(negedge HCLK); m1_trigger = 1'b1;

        for (k = 0; k < 4000; k = k + 1) begin
            @(posedge HCLK);
            if (!LINK_TX) link_went_low = 1'b1;   // start bit seen
        end

        @(negedge HCLK); m1_trigger = 1'b0;
        record(link_went_low, "T7 bridge transmitted a frame on LINK_TX");

        // ---- summary ------------------------------------------------------
        $display("");
        $display("----------------------------------------");
        if (errors == 0) $display("RESULT: ALL PASS (%0d checks)", checks);
        else             $display("RESULT: %0d of %0d check(s) FAILED", errors, checks);
        $display("----------------------------------------");
        $finish;
    end

    // Global watchdog: a hung bus must fail the run, not spin forever.
    initial begin
        #40_000_000;
        $display("!! GLOBAL TIMEOUT -- the bus appears hung");
        $display("RESULT: FAILED (timeout)");
        $finish;
    end

endmodule
>>>>>>> Stashed changes
=======
`timescale 1ns / 1ps

// ============================================================================
// File: tb/bus_top_tb.v
// Description: Top-level verification fulfilling Assignment Task 4.
//              Tests Reset, Single Master, Dual Master Arbitration, and SPLIT.
// ============================================================================

// Point up one directory to access the RTL folder
`include "../rtl/bus_params.vh"

module bus_top_tb();

    // ------------------------------------------------------------------------
    // Clock and Reset Signals
    // ------------------------------------------------------------------------
    reg HCLK;
    reg HRESETn;

    // Generate 50 MHz Clock (20ns period)
    initial HCLK = 0;
    always #10 HCLK = ~HCLK; 

    // ------------------------------------------------------------------------
    // Master & Slave Control Signals
    // ------------------------------------------------------------------------
    reg m1_trigger, m1_write;
    reg [`ADDR_WIDTH-1:0] m1_addr;
    reg [`DATA_WIDTH-1:0] m1_wdata;
    wire [`DATA_WIDTH-1:0] m1_rdata_out;

    reg m2_trigger, m2_write;
    reg [`ADDR_WIDTH-1:0] m2_addr;
    reg [`DATA_WIDTH-1:0] m2_wdata;
    wire [`DATA_WIDTH-1:0] m2_rdata_out;

    reg s1_sim_split, s2_sim_split, s3_sim_split;
    
    wire SERIAL_TX, BUSY_OUT;

    // ------------------------------------------------------------------------
    // Device Under Test (DUT): bus_top
    // ------------------------------------------------------------------------
    bus_top dut (
        .HCLK(HCLK),
        .HRESETn(HRESETn),
        
        // Serial Bridge (Tied off for local bus verification)
        .SERIAL_RX(1'b0), 
        .SERIAL_TX(SERIAL_TX), 
        .BUSY_IN(1'b0), 
        .BUSY_OUT(BUSY_OUT),

        // Master 1
        .m1_trigger(m1_trigger), 
        .m1_write(m1_write), 
        .m1_addr(m1_addr), 
        .m1_wdata(m1_wdata), 
        .m1_rdata_out(m1_rdata_out),

        // Master 2
        .m2_trigger(m2_trigger), 
        .m2_write(m2_write), 
        .m2_addr(m2_addr), 
        .m2_wdata(m2_wdata), 
        .m2_rdata_out(m2_rdata_out),

        // Slave Split Simulators
        .s1_sim_split(s1_sim_split), 
        .s2_sim_split(s2_sim_split), 
        .s3_sim_split(s3_sim_split)
    );

    // ------------------------------------------------------------------------
    // Verification Sequence
    // ------------------------------------------------------------------------
    initial begin
        // Initialize all inputs
        m1_trigger = 0; m1_write = 0; m1_addr = 0; m1_wdata = 0;
        m2_trigger = 0; m2_write = 0; m2_addr = 0; m2_wdata = 0;
        s1_sim_split = 0; s2_sim_split = 0; s3_sim_split = 0;

        $display("--- Starting Top Level Verification (bus_top_tb) ---");

        // ====================================================================
        // TEST a) Reset test (Async reset & posedge clocks)
        // ====================================================================
        $display("[TIME: %0t] TEST A: Async Reset", $time);
        HRESETn = 1'b0; // Assert Active-Low Reset asynchronously
        #35; 
        HRESETn = 1'b1; // Release Reset
        #25;

        // ====================================================================
        // TEST b) One master request
        // ====================================================================
        $display("[TIME: %0t] TEST B: Single Master Request", $time);
        // Master 1 writes to Slave 2 (Assume Base 0x1000)
        m1_addr  = 14'h1004; 
        m1_wdata = 8'hAA;
        m1_write = 1'b1;
        
        // Trigger transfer
        m1_trigger = 1'b1; #20; m1_trigger = 1'b0;
        #100;

        // ====================================================================
        // TEST c) Two master requests (Arbiter Priority Check)
        // ====================================================================
        $display("[TIME: %0t] TEST C: Two Master Requests (Priority Collision)", $time);
        // Master 1 targets Slave 2
        m1_addr  = 14'h1008; m1_wdata = 8'hB1; m1_write = 1'b1;
        
        // Master 2 targets Slave 3 (Assume Base 0x2000)
        m2_addr  = 14'h2004; m2_wdata = 8'hC2; m2_write = 1'b1;
        
        // Fire BOTH masters at the exact same time
        m1_trigger = 1'b1; m2_trigger = 1'b1; 
        #20; 
        m1_trigger = 1'b0; m2_trigger = 1'b0;
        
        // Wait for Arbiter to grant Master 1, then queue Master 2
        #150; 

        // ====================================================================
        // TEST d) Split transaction viable scenario
        // ====================================================================
        $display("[TIME: %0t] TEST D: Split Transaction", $time);
        
        // Step 1: Tell Slave 1 to simulate a SPLIT response
        s1_sim_split = 1'b1; 
        
        // Step 2: Master 1 attempts to read from Slave 1
        m1_addr  = 14'h0010; 
        m1_write = 1'b0;
        m1_trigger = 1'b1; #20; m1_trigger = 1'b0;
        
        #40; 
        // Slave 1 responds with HRESP_SPLIT. Master 1 is now parked/masked.
        
        // Step 3: Master 2 issues a write to Slave 3 while Master 1 is parked
        m2_addr  = 14'h2008; m2_wdata = 8'hDD; m2_write = 1'b1;
        m2_trigger = 1'b1; #20; m2_trigger = 1'b0;
        
        #60;
        
        // Step 4: Clear the SPLIT condition so Master 1 can complete its read
        s1_sim_split = 1'b0; 
        #150;

        $display("--- Verification Complete ---");
        $stop;
    end

endmodule
>>>>>>> Stashed changes
