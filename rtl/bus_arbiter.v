// Priority bus arbiter with SPLIT support. Priority is M3 > M1 > M2; M3 is
// the bridge replaying remote traffic, so it is never masked -- starving it
// can deadlock the two boards against each other.

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module bus_arbiter (
    input  wire       HCLK,      // Bus clock
    input  wire       HRESETn,   // Asynchronous reset, active low

    // Master bus requests
    input  wire       HBUSREQ1,  // Local Master 1 requests the bus
    input  wire       HBUSREQ2,  // Local Master 2 requests the bus
    input  wire       HBUSREQ3,  // Bridge (remote traffic), highest priority

    input  wire       HREADY,    // Current transfer complete
    input  wire [1:0] HRESP,     // Slave response for the current transfer

    input  wire       HSPLIT1,   // Slave releases Master 1 from split
    input  wire       HSPLIT2,   // Slave releases Master 2 from split

    // Grants
    output wire       HGRANT1,   // Grant to Local Master 1
    output wire       HGRANT2,   // Grant to Local Master 2
    output wire       HGRANT3,   // Grant to Bridge

    output reg  [3:0] HMASTER    // Current bus owner (0 = none)
);

    // A split master is held out of arbitration until its slave releases it.
    reg m1_split_masked;
    reg m2_split_masked;

    // Fold the in-flight SPLIT into the mask combinationally. The registered
    // bits only update at the clock edge, so arbitrating on those alone
    // re-grants the just-split master for one extra cycle.
    wire split_now = HREADY && (HRESP == `HRESP_SPLIT);

    wire m1_split_this_cycle = split_now && (HMASTER == `MASTER_1_ID);
    wire m2_split_this_cycle = split_now && (HMASTER == `MASTER_2_ID);

    // Masked, or being masked now, minus a release arriving the same cycle.
    wire m1_masked = (m1_split_masked || m1_split_this_cycle) && !HSPLIT1;
    wire m2_masked = (m2_split_masked || m2_split_this_cycle) && !HSPLIT2;

    wire req1_valid = HBUSREQ1 && !m1_masked;
    wire req2_valid = HBUSREQ2 && !m2_masked;
    wire req3_valid = HBUSREQ3;                 // bridge is never masked

    reg [3:0] next_master;

    always @(*) begin
        if      (req3_valid) next_master = `MASTER_3_ID;
        else if (req1_valid) next_master = `MASTER_1_ID;
        else if (req2_valid) next_master = `MASTER_2_ID;
        else                 next_master = `MASTER_NONE;
    end

    assign HGRANT1 = (next_master == `MASTER_1_ID);
    assign HGRANT2 = (next_master == `MASTER_2_ID);
    assign HGRANT3 = (next_master == `MASTER_3_ID);

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            HMASTER         <= `MASTER_NONE;
            m1_split_masked <= 1'b0;
            m2_split_masked <= 1'b0;
        end else begin
            // Ownership changes only at a transfer boundary, so a master
            // keeps the bus while a slave stretches the transfer.
            if (HREADY)
                HMASTER <= next_master;

            // Release wins over a same-cycle split, so a slave that splits
            // and releases together cannot leave the master stuck masked.
            if (HSPLIT1)                     m1_split_masked <= 1'b0;
            else if (m1_split_this_cycle)    m1_split_masked <= 1'b1;

            if (HSPLIT2)                     m2_split_masked <= 1'b0;
            else if (m2_split_this_cycle)    m2_split_masked <= 1'b1;
        end
    end

endmodule
