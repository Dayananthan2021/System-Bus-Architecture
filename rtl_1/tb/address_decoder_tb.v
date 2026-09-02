// ============================================================================
// File: address_decoder_tb.v
// Description: Verification of the address decoder (3 slaves + remote window).
//
//   Covers the assignment's required decoder scenarios:
//     T1  decode enable low  -> nothing selected  (reset-equivalent state)
//     T2  Slave 1 region, including both boundaries
//     T3  Slave 2 region, including both boundaries
//     T4  Slave 3 region, including both boundaries  <-- regression: this
//         previously routed to the bridge and Slave 3 was unreachable
//     T5  unmapped hole above Slave 3 flagged invalid
//     T6  remote (bridge) window 0x3000-0x3FFF selects the bridge
//     T7  mutual exclusion: exactly one output asserted at every address
// ============================================================================

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module address_decoder_tb;

    reg  [`ADDR_WIDTH-1:0] HADDR;
    reg                    HSEL_EN;
    wire                   HSEL1, HSEL2, HSEL3, HSEL_REMOTE, HADDR_INVALID;

    integer errors = 0;
    integer checks = 0;

    address_decoder dut (
        .HADDR(HADDR),
        .HSEL_EN(HSEL_EN),
        .HSEL1(HSEL1),
        .HSEL2(HSEL2),
        .HSEL3(HSEL3),
        .HSEL_REMOTE(HSEL_REMOTE),
        .HADDR_INVALID(HADDR_INVALID)
    );

    // Check the full output vector {S1,S2,S3,INVALID} for one address.
    task expect_sel;
        input [`ADDR_WIDTH-1:0] addr;
        input                   en;
        input [4:0]             exp;    // {HSEL1,HSEL2,HSEL3,HSEL_REMOTE,HADDR_INVALID}
        input [511:0]           label;
        reg   [4:0]             got;
        begin
            HADDR   = addr;
            HSEL_EN = en;
            #1;
            got    = {HSEL1, HSEL2, HSEL3, HSEL_REMOTE, HADDR_INVALID};
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL: %0s (addr=0x%04h en=%b) got {S1,S2,S3,RMT,INV}=%b expected %b",
                         label, addr, en, got, exp);
            end else begin
                $display("pass: %0s (addr=0x%04h) {S1,S2,S3,RMT,INV}=%b", label, addr, got);
            end
            // Mutual exclusion must hold for every address we visit.
            if ($countones(got) > 1) begin
                errors = errors + 1;
                $display("FAIL: %0s asserted %0d outputs at once (addr=0x%04h)",
                         label, $countones(got), addr);
            end
        end
    endtask

    integer a;

    initial begin
        $dumpfile("address_decoder_tb.vcd");
        $dumpvars(0, address_decoder_tb);

        $display("=== address_decoder_tb ===");
        $display("map: S1 0x%04h-0x%04h  S2 0x%04h-0x%04h  S3 0x%04h-0x%04h",
                 `S1_BASE, `S1_END, `S2_BASE, `S2_END, `S3_BASE, `S3_END);
        $display("");

        // ---- T1: decode disabled -> all outputs low ---------------------
        $display("-- T1: decode disabled --");
        expect_sel(14'h0000, 1'b0, 5'b00000, "T1 disabled, S1 addr");
        expect_sel(14'h2000, 1'b0, 5'b00000, "T1 disabled, S3 addr");
        expect_sel(14'h3FFF, 1'b0, 5'b00000, "T1 disabled, remote addr");

        // ---- T2: Slave 1 -----------------------------------------------
        $display("-- T2: Slave 1 (4K) --");
        expect_sel(`S1_BASE, 1'b1, 5'b10000, "T2 S1 base");
        expect_sel(14'h0800, 1'b1, 5'b10000, "T2 S1 middle");
        expect_sel(`S1_END,  1'b1, 5'b10000, "T2 S1 end");

        // ---- T3: Slave 2 -----------------------------------------------
        $display("-- T3: Slave 2 (4K) --");
        expect_sel(`S2_BASE, 1'b1, 5'b01000, "T3 S2 base");
        expect_sel(14'h1800, 1'b1, 5'b01000, "T3 S2 middle");
        expect_sel(`S2_END,  1'b1, 5'b01000, "T3 S2 end");

        // ---- T4: Slave 3 (regression) ----------------------------------
        // Previously HADDR[13] was stolen as a "route to bridge" flag, so
        // every address >= 0x2000 selected the bridge and Slave 3 could
        // never be reached. These three checks fail on the old decoder.
        $display("-- T4: Slave 3 (2K) -- regression --");
        expect_sel(`S3_BASE, 1'b1, 5'b00100, "T4 S3 base");
        expect_sel(14'h2400, 1'b1, 5'b00100, "T4 S3 middle");
        expect_sel(`S3_END,  1'b1, 5'b00100, "T4 S3 end");

        // ---- T5: hole above Slave 3 ------------------------------------
        $display("-- T5: unmapped hole 0x2800-0x2FFF --");
        expect_sel(14'h2800, 1'b1, 5'b00001, "T5 hole start");
        expect_sel(14'h2FFF, 1'b1, 5'b00001, "T5 hole end");

        // ---- T6: remote window 0x3000-0x3FFF ---------------------------
        // This region is the window onto the other board's memory, served by
        // the bridge -- not an invalid address.
        $display("-- T6: remote (bridge) window 0x3000-0x3FFF --");
        expect_sel(`RMT_BASE, 1'b1, 5'b00010, "T6 remote window base");
        expect_sel(14'h3800,  1'b1, 5'b00010, "T6 remote window middle");
        expect_sel(`RMT_END,  1'b1, 5'b00010, "T6 remote window end");

        // ---- T7: exhaustive sweep of mutual exclusion -------------------
        // Walk the whole 16K space in 64-byte steps and confirm exactly one
        // output is asserted everywhere.
        $display("-- T7: exhaustive mutual-exclusion sweep --");
        HSEL_EN = 1'b1;
        for (a = 0; a < (1 << `ADDR_WIDTH); a = a + 64) begin
            HADDR = a[`ADDR_WIDTH-1:0];
            #1;
            checks = checks + 1;
            if ($countones({HSEL1, HSEL2, HSEL3, HSEL_REMOTE, HADDR_INVALID}) !== 1) begin
                errors = errors + 1;
                $display("FAIL: addr=0x%04h asserted %0d outputs (expected exactly 1)",
                         a, $countones({HSEL1, HSEL2, HSEL3, HSEL_REMOTE, HADDR_INVALID}));
            end
        end
        $display("pass: swept %0d addresses, exactly one output each", (1<<`ADDR_WIDTH)/64);

        // ---- summary ----------------------------------------------------
        $display("");
        $display("----------------------------------------");
        if (errors == 0)
            $display("RESULT: ALL PASS (%0d checks)", checks);
        else
            $display("RESULT: %0d of %0d check(s) FAILED", errors, checks);
        $display("----------------------------------------");
        $finish;
    end

endmodule
