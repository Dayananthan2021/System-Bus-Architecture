// ============================================================================
// File: slave_memory_tb.v
// Description: Verification of the AHB-style slave memory.
//
//   T1  reset drives every output to its defined reset value
//   T2  single write then read back
//   T3  no byte-address aliasing (regression): consecutive addresses must be
//       independent storage locations, not aliases of each other
//   T4  full-depth boundary access (first and last location)
//   T5  BASE_ADDR offset is applied (slave sees its own local address space)
//   T6  SPLIT / busy handshake: slave stretches the transfer, answers SPLIT,
//       then releases the owning master with HSPLITx
// ============================================================================

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module slave_memory_tb;

    reg                    HCLK, HRESETn;
    reg                    simulate_split;
    reg                    HSEL, HWRITE, HREADY_IN;
    reg  [`ADDR_WIDTH-1:0] HADDR;
    reg  [1:0]             HTRANS;
    reg  [`DATA_WIDTH-1:0] HWDATA;
    reg  [3:0]             HMASTER;

    wire                    HREADY_OUT, HSPLIT1, HSPLIT2;
    wire [1:0]              HRESP;
    wire [`DATA_WIDTH-1:0]  HRDATA;

    integer errors = 0;
    integer checks = 0;

    // Slave 2 geometry: 4K @ 0x1000, split-capable so T6 can exercise it.
    slave_memory #(
        .MEM_DEPTH        (`S2_DEPTH),
        .ADDR_INDEX_WIDTH (`S2_IDXW),
        .SPLIT_ENABLED    (1),
        .BASE_ADDR        (`S2_BASE)
    ) dut (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .simulate_split(simulate_split),
        .HSEL(HSEL), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HWDATA(HWDATA), .HREADY_IN(HREADY_IN), .HMASTER(HMASTER),
        .HREADY_OUT(HREADY_OUT), .HRESP(HRESP), .HRDATA(HRDATA),
        .HSPLIT1(HSPLIT1), .HSPLIT2(HSPLIT2)
    );

    initial HCLK = 1'b0;
    always #5 HCLK = ~HCLK;

    task step; begin @(negedge HCLK); end endtask

    task record;
        input        ok;
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

    // Compare a read result against an expectation, reporting both values.
    task record_data;
        input [`DATA_WIDTH-1:0] got;
        input [`DATA_WIDTH-1:0] exp;
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

    // Address phase then data phase for a write.
    task bus_write;
        input [`ADDR_WIDTH-1:0] addr;
        input [`DATA_WIDTH-1:0] data;
        begin
            HSEL = 1'b1; HADDR = addr; HTRANS = `HTRANS_NONSEQ;
            HWRITE = 1'b1; HREADY_IN = 1'b1;
            step;
            HTRANS = `HTRANS_IDLE; HWDATA = data;
            step;
            HSEL = 1'b0; HWDATA = {`DATA_WIDTH{1'b0}};
        end
    endtask

    // Address phase then data phase for a read; result lands in HRDATA.
    task bus_read;
        input [`ADDR_WIDTH-1:0] addr;
        begin
            HSEL = 1'b1; HADDR = addr; HTRANS = `HTRANS_NONSEQ;
            HWRITE = 1'b0; HREADY_IN = 1'b1;
            step;
            HTRANS = `HTRANS_IDLE;
            step;
            HSEL = 1'b0;
        end
    endtask

    // Second instance with Slave 3 geometry: 2K of storage sitting inside a
    // 4K-aligned region, so its upper half is out of range. This is the only
    // slave where the out-of-range path is reachable.
    reg                    s3_sel;
    reg  [`ADDR_WIDTH-1:0] s3_addr;
    reg  [1:0]             s3_trans;
    wire                   s3_ready;
    wire [1:0]             s3_resp;
    wire [`DATA_WIDTH-1:0] s3_rdata;

    slave_memory #(
        .MEM_DEPTH        (`S3_DEPTH),
        .ADDR_INDEX_WIDTH (`S3_IDXW),
        .SPLIT_ENABLED    (0),
        .BASE_ADDR        (`S3_BASE)
    ) dut_s3 (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .simulate_split(1'b0),
        .HSEL(s3_sel), .HADDR(s3_addr), .HTRANS(s3_trans), .HWRITE(1'b0),
        .HWDATA(8'h00), .HREADY_IN(1'b1), .HMASTER(`MASTER_1_ID),
        .HREADY_OUT(s3_ready), .HRESP(s3_resp), .HRDATA(s3_rdata),
        .HSPLIT1(), .HSPLIT2()
    );

    // Drive a read into the Slave 3 instance and settle its data phase.
    task s3_read;
        input [`ADDR_WIDTH-1:0] addr;
        begin
            s3_sel = 1'b1; s3_addr = addr; s3_trans = `HTRANS_NONSEQ;
            step;
            s3_trans = `HTRANS_IDLE;
            step;
            s3_sel = 1'b0;
        end
    endtask

    integer i;
    reg [`DATA_WIDTH-1:0] expect_val;

    initial begin
        $dumpfile("slave_memory_tb.vcd");
        $dumpvars(0, slave_memory_tb);

        $display("=== slave_memory_tb ===");
        $display("Slave 2: depth=%0d idx_width=%0d base=0x%04h",
                 `S2_DEPTH, `S2_IDXW, `S2_BASE);
        $display("");

        // ---- T1: reset ---------------------------------------------------
        $display("-- T1: reset --");
        HRESETn = 1'b0;
        simulate_split = 1'b0;
        HSEL = 1'b0; HADDR = 0; HTRANS = `HTRANS_IDLE; HWRITE = 1'b0;
        HWDATA = 0; HREADY_IN = 1'b1; HMASTER = `MASTER_1_ID;
        s3_sel = 1'b0; s3_addr = 0; s3_trans = `HTRANS_IDLE;
        step; step;

        record(HREADY_OUT === 1'b1,          "T1 HREADY_OUT resets high");
        record(HRESP      === `HRESP_OKAY,   "T1 HRESP resets to OKAY");
        record(HRDATA     === {`DATA_WIDTH{1'b0}}, "T1 HRDATA resets to 0");
        record(HSPLIT1    === 1'b0,          "T1 HSPLIT1 resets low");
        record(HSPLIT2    === 1'b0,          "T1 HSPLIT2 resets low");

        HRESETn = 1'b1;
        step;

        // ---- T2: write / read round trip ---------------------------------
        $display("-- T2: single write then read --");
        bus_write(`S2_BASE + 14'd5, 8'hA5);
        bus_read (`S2_BASE + 14'd5);
        record_data(HRDATA, 8'hA5, "T2 read back");

        // ---- T3: no aliasing (regression) --------------------------------
        // With the old >>2 indexing, addresses 0..3 all mapped to index 0,
        // so this loop would read back only the last value written.
        $display("-- T3: byte-address aliasing regression --");
        for (i = 0; i < 8; i = i + 1)
            bus_write(`S2_BASE + i[`ADDR_WIDTH-1:0], 8'h10 + i[7:0]);

        for (i = 0; i < 8; i = i + 1) begin
            bus_read(`S2_BASE + i[`ADDR_WIDTH-1:0]);
            expect_val = 8'h10 + i[7:0];
            $write("       (addr+%0d) ", i);
            record_data(HRDATA, expect_val, "T3 distinct byte");
        end

        // ---- T4: depth boundaries ----------------------------------------
        $display("-- T4: first and last location --");
        bus_write(`S2_BASE, 8'h11);
        bus_write(`S2_BASE + (`S2_DEPTH - 1), 8'hEE);

        bus_read(`S2_BASE);
        record_data(HRDATA, 8'h11, "T4 first location");
        bus_read(`S2_BASE + (`S2_DEPTH - 1));
        record_data(HRDATA, 8'hEE, "T4 last location");
        // First location must not have been disturbed by the last write.
        bus_read(`S2_BASE);
        record(HRDATA === 8'h11, "T4 first location undisturbed by last write");

        // ---- T5: BASE_ADDR offset ----------------------------------------
        // 0x1000 is local index 0. Writing local 0 then reading local 1 must
        // not return the same byte.
        $display("-- T5: BASE_ADDR offset applied --");
        bus_write(`S2_BASE + 14'd0, 8'h77);
        bus_write(`S2_BASE + 14'd1, 8'h88);
        bus_read (`S2_BASE + 14'd0);
        record_data(HRDATA, 8'h77, "T5 local 0");
        bus_read (`S2_BASE + 14'd1);
        record_data(HRDATA, 8'h88, "T5 local 1");

        // ---- T6: SPLIT / busy handshake ----------------------------------
        $display("-- T6: SPLIT handshake --");
        simulate_split = 1'b1;
        HMASTER        = `MASTER_1_ID;

        HSEL = 1'b1; HADDR = `S2_BASE + 14'd9; HTRANS = `HTRANS_NONSEQ;
        HWRITE = 1'b0; HREADY_IN = 1'b1;
        step;
        HTRANS = `HTRANS_IDLE;
        step;                       // enters split sequence

        // Slave must signal busy (HREADY_OUT low) and answer SPLIT.
        step;
        record(HREADY_OUT === 1'b0, "T6 slave asserts busy (HREADY_OUT low)");
        record(HRESP === `HRESP_SPLIT, "T6 slave answers HRESP=SPLIT");

        // Then wait for the release pulse to the owning master.
        HSEL = 1'b0;
        simulate_split = 1'b0;
        fork : wait_split
            begin
                repeat (40) begin
                    step;
                    if (HSPLIT1) begin
                        record(1'b1, "T6 HSPLIT1 released owning master 1");
                        disable wait_split;
                    end
                end
                record(1'b0, "T6 HSPLIT1 never asserted (timeout)");
            end
        join

        // Master 2 must not be released by a split belonging to master 1.
        record(HSPLIT2 === 1'b0, "T6 HSPLIT2 stayed low (wrong master)");

        // ---- T7: out-of-range access inside a slave's region --------------
        // Slave 3 holds 2K but sits in a 4K-aligned region, so 0x2800.. is
        // selected by the decoder yet lies past the end of its storage. It
        // must answer ERROR rather than wrapping onto the populated low half.
        $display("-- T7: Slave 3 out-of-range access --");
        $display("   Slave 3: depth=%0d idx_width=%0d base=0x%04h",
                 `S3_DEPTH, `S3_IDXW, `S3_BASE);

        s3_read(`S3_BASE);                  // in range, first location
        record(s3_resp === `HRESP_OKAY, "T7 in-range access -> OKAY");

        s3_read(`S3_BASE + (`S3_DEPTH - 1)); // in range, last location
        record(s3_resp === `HRESP_OKAY, "T7 last in-range access -> OKAY");

        s3_read(`S3_BASE + `S3_DEPTH);       // first byte past the end
        record(s3_resp === `HRESP_ERROR, "T7 first out-of-range -> ERROR");

        s3_read(14'h2FFF);                   // top of the 4K region
        record(s3_resp === `HRESP_ERROR, "T7 top of region -> ERROR");

        // ---- summary ------------------------------------------------------
        $display("");
        $display("----------------------------------------");
        if (errors == 0) $display("RESULT: ALL PASS (%0d checks)", checks);
        else             $display("RESULT: %0d of %0d check(s) FAILED", errors, checks);
        $display("----------------------------------------");
        $finish;
    end

endmodule
