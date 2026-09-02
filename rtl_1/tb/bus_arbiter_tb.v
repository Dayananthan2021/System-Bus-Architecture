// ============================================================================
// File: bus_arbiter_tb.v
// Description: Self-checking testbench for bus_arbiter.
//              Covers reset, priority, HREADY stalling and SPLIT mask/release.
// ============================================================================

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_arbiter_tb;

	reg        HCLK;
	reg        HRESETn;
	reg        HBUSREQ1, HBUSREQ2, HBUSREQ3;
	reg        HREADY;
	reg  [1:0] HRESP;
	reg        HSPLIT1, HSPLIT2;

	wire       HGRANT1, HGRANT2, HGRANT3;
	wire [3:0] HMASTER;

	integer errors = 0;

	bus_arbiter dut (
		.HCLK     (HCLK),
		.HRESETn  (HRESETn),
		.HBUSREQ1 (HBUSREQ1),
		.HBUSREQ2 (HBUSREQ2),
		.HBUSREQ3 (HBUSREQ3),
		.HREADY   (HREADY),
		.HRESP    (HRESP),
		.HSPLIT1  (HSPLIT1),
		.HSPLIT2  (HSPLIT2),
		.HGRANT1  (HGRANT1),
		.HGRANT2  (HGRANT2),
		.HGRANT3  (HGRANT3),
		.HMASTER  (HMASTER)
	);

	// 100 MHz bus clock
	initial HCLK = 1'b0;
	always #5 HCLK = ~HCLK;

	// Stimulus is applied on the negedge so it is stable well before the
	// posedge the DUT samples on.
	task step;
		begin
			@(negedge HCLK);
		end
	endtask

	task check_master;
		input [3:0] expected;
		input [511:0] label;
		begin
			if (HMASTER !== expected) begin
				$display("[%0t] FAIL %0s: HMASTER=%0d expected %0d",
				         $time, label, HMASTER, expected);
				errors = errors + 1;
			end else begin
				$display("[%0t] pass %0s: HMASTER=%0d", $time, label, HMASTER);
			end
		end
	endtask

	task check_grant;
		input g1_exp, g2_exp;
		input [511:0] label;
		begin
			if (HGRANT1 !== g1_exp || HGRANT2 !== g2_exp) begin
				$display("[%0t] FAIL %0s: HGRANT1=%b HGRANT2=%b expected %b/%b",
				         $time, label, HGRANT1, HGRANT2, g1_exp, g2_exp);
				errors = errors + 1;
			end else begin
				$display("[%0t] pass %0s: HGRANT1=%b HGRANT2=%b",
				         $time, label, HGRANT1, HGRANT2);
			end
		end
	endtask

	initial begin
		$dumpfile("bus_arbiter_tb.vcd");
		$dumpvars(0, bus_arbiter_tb);

		// ---- reset -----------------------------------------------------
		HRESETn  = 1'b0;
		HBUSREQ1 = 1'b0;
		HBUSREQ2 = 1'b0;
		HBUSREQ3 = 1'b0;
		HREADY   = 1'b1;
		HRESP    = `HRESP_OKAY;
		HSPLIT1  = 1'b0;
		HSPLIT2  = 1'b0;

		step; step;
		check_master(4'd0, "reset holds HMASTER at 0");
		// Reset must clear the grants as well, not just the owner.
		check_grant(1'b0, 1'b0, "reset clears both grants");

		HRESETn = 1'b1;
		step;

		// ---- T0: no master requests (4th request state) -----------------
		check_grant(1'b0, 1'b0, "T0 no requests -> no grant");
		check_master(4'd0, "T0 no requests -> HMASTER 0");

		// ---- T1: master 1 alone ----------------------------------------
		HBUSREQ1 = 1'b1;
		#1 check_grant(1'b1, 1'b0, "T1 req1 -> grant1");
		step;
		check_master(`MASTER_1_ID, "T1 master 1 owns bus");

		// ---- T2: master 2 alone ----------------------------------------
		HBUSREQ1 = 1'b0;
		HBUSREQ2 = 1'b1;
		#1 check_grant(1'b0, 1'b1, "T2 req2 -> grant2");
		step;
		check_master(`MASTER_2_ID, "T2 master 2 owns bus");

		// ---- T3: both request, master 1 has priority -------------------
		HBUSREQ1 = 1'b1;
		HBUSREQ2 = 1'b1;
		#1 check_grant(1'b1, 1'b0, "T3 both req -> master 1 priority");
		step;
		check_master(`MASTER_1_ID, "T3 master 1 wins arbitration");

		// ---- T4: HREADY low freezes ownership --------------------------
		HREADY   = 1'b0;
		HBUSREQ1 = 1'b0;          // master 1 drops out mid-transfer
		step; step;
		check_master(`MASTER_1_ID, "T4 HMASTER frozen while HREADY low");
		HREADY   = 1'b1;
		HBUSREQ1 = 1'b1;
		step;

		// ---- T5: SPLIT on master 1 -------------------------------------
		// Slave splits master 1. Master 1 must be masked out and master 2
		// must take the bus; master 1 stays out until its HSPLIT is pulsed.
		check_master(`MASTER_1_ID, "T5 master 1 owns bus before SPLIT");
		HRESP = `HRESP_SPLIT;
		step;                      // SPLIT registered on this edge
		HRESP = `HRESP_OKAY;

		check_master(`MASTER_2_ID, "T5 master 2 takes over after SPLIT");
		// #1 lets the combinational grants settle after HRESP is deasserted
		// above; sampling in the same delta reads pre-settle values.
		#1 check_grant(1'b0, 1'b1, "T5 master 1 masked, grant2 asserted");

		step;
		check_master(`MASTER_2_ID, "T5 master 1 stays split-masked");

		// ---- T6: SPLIT release restores master 1 -----------------------
		HSPLIT1 = 1'b1;
		step;
		HSPLIT1 = 1'b0;
		#1 check_grant(1'b1, 1'b0, "T6 HSPLIT1 unmasks master 1");
		step;
		check_master(`MASTER_1_ID, "T6 master 1 regains bus");

		// ---- T7: SPLIT must mask in the SAME cycle (regression) ---------
		// The original arbiter updated the mask registers and HMASTER on the
		// same edge, so next_master was computed from the pre-mask request
		// vector and the just-split master won one extra undeserved cycle.
		// With both masters requesting, a SPLIT on M1 must hand the bus to
		// M2 immediately -- grant must flip in the very cycle the SPLIT is
		// visible, before any clock edge.
		$display("-- T7: same-cycle split masking (regression) --");
		HBUSREQ1 = 1'b1;
		HBUSREQ2 = 1'b1;
		step;
		check_master(`MASTER_1_ID, "T7 master 1 owns bus");
		check_grant(1'b1, 1'b0, "T7 grant1 before SPLIT");

		HRESP = `HRESP_SPLIT;
		#1;   // combinational settle only -- no clock edge yet
		check_grant(1'b0, 1'b1, "T7 grant flips to M2 in same cycle as SPLIT");
		step;
		HRESP = `HRESP_OKAY;
		check_master(`MASTER_2_ID, "T7 master 2 owns bus after SPLIT edge");

		// Release M1 again so it does not stay masked into T8.
		HSPLIT1 = 1'b1;
		step;
		HSPLIT1 = 1'b0;
		step;

		// ---- T8: bridge (master 3) has top priority ---------------------
		$display("-- T8: bridge priority --");
		HBUSREQ1 = 1'b1;
		HBUSREQ2 = 1'b1;
		HBUSREQ3 = 1'b1;
		#1;
		if (HGRANT3 !== 1'b1 || HGRANT1 !== 1'b0 || HGRANT2 !== 1'b0) begin
			$display("[%0t] FAIL T8 bridge priority: G1=%b G2=%b G3=%b expected 0/0/1",
			         $time, HGRANT1, HGRANT2, HGRANT3);
			errors = errors + 1;
		end else begin
			$display("[%0t] pass T8 bridge wins over both local masters: G3=%b",
			         $time, HGRANT3);
		end
		step;
		check_master(`MASTER_3_ID, "T8 bridge owns bus");

		// ---- T9: bridge is never split-masked ---------------------------
		// A SPLIT while the bridge owns the bus must not lock it out, or the
		// two boards can deadlock waiting on each other.
		$display("-- T9: bridge immune to split masking --");
		HRESP = `HRESP_SPLIT;
		step;
		HRESP = `HRESP_OKAY;
		#1;
		if (HGRANT3 !== 1'b1) begin
			$display("[%0t] FAIL T9 bridge was split-masked: G3=%b expected 1",
			         $time, HGRANT3);
			errors = errors + 1;
		end else begin
			$display("[%0t] pass T9 bridge still granted after SPLIT: G3=%b",
			         $time, HGRANT3);
		end

		// ---- summary ----------------------------------------------------
		step;
		$display("----------------------------------------");
		if (errors == 0)
			$display("RESULT: all checks passed");
		else
			$display("RESULT: %0d check(s) FAILED", errors);
		$display("----------------------------------------");
		$finish;
	end

endmodule
