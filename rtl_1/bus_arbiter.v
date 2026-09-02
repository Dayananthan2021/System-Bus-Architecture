// ============================================================================
// File: bus_arbiter.v
// Description: Priority based BUS_arbiter with SPLIT support and Bridge Priority
// ============================================================================

`include "bus_params.vh"

module bus_arbiter (
    input wire           HCLK,      // Bus Clock
    input wire           HRESETn,   // Reset (Active low)
    
    // Master Requests
    input wire           HBUSREQ1,  // Local Master 1
    input wire           HBUSREQ2,  // Local Master 2
    input wire           HBUSREQ3,  // Remote Bridge RX (Highest Priority)
    
    input wire           HREADY,    // Current AHB transfer ready/complete
    
    input wire [1:0]     HRESP,     // Transfer Response
    input wire           HSPLIT1,   // SPLIT Release signals
    input wire           HSPLIT2,
    
    // Grant signals
    output wire          HGRANT1,   // Grant to Local Master 1
    output wire          HGRANT2,   // Grant to Local Master 2  
    output wire          HGRANT3,   // Grant to Remote Bridge RX
    
    output reg [3:0]     HMASTER    // Current Bus Owner
);

    // ------------------------------------------------------------------------
    // SPLIT Masking Logic
    // ------------------------------------------------------------------------
    reg m1_split_masked; // Remembers if M1 is blocked because of a SPLIT
    reg m2_split_masked; // Remembers if M2 is blocked because of a SPLIT
    // Note: Master 3 (Bridge) cannot be split-masked. It must flow freely.

    wire req1_valid;     // Master Request Eligible or not (only unmasked)
    wire req2_valid;
    wire req3_valid;
    
    assign req1_valid = HBUSREQ1 && !m1_split_masked;
    assign req2_valid = HBUSREQ2 && !m2_split_masked;
    assign req3_valid = HBUSREQ3; // Never masked
    
    // ------------------------------------------------------------------------
    // Priority Arbitration Logic (M3 > M1 > M2)
    // ------------------------------------------------------------------------
    reg [3:0] next_master;
    
    always @(*) begin   
        // Highest Priority: Remote traffic entering the board
        if (req3_valid)
            next_master = `MASTER_3_ID; 
            
        // Priority 2: Local Master 1
        else if (req1_valid)
            next_master = `MASTER_1_ID;
            
        // Priority 3: Local Master 2
        else if (req2_valid)
            next_master = `MASTER_2_ID;
            
        // Default: No requests
        else
            next_master = 4'd0;
    end
    
    // Grant signals are combinational based on next_master
    assign HGRANT1 = (next_master == `MASTER_1_ID);
    assign HGRANT2 = (next_master == `MASTER_2_ID);
    assign HGRANT3 = (next_master == `MASTER_3_ID);
    
    // ------------------------------------------------------------------------
    // Sequential State Update
    // ------------------------------------------------------------------------
    always @(posedge HCLK or negedge HRESETn) begin
        
        if (!HRESETn) begin                 // Reset condition
            HMASTER         <= 4'd0;
            m1_split_masked <= 1'b0;
            m2_split_masked <= 1'b0;
        end
        else begin
            // 1. Change bus ownership ONLY when current transaction is READY
            if (HREADY) begin               
                HMASTER <= next_master; 
                
                // 2. Handle new SPLIT responses from Slaves
                if (HRESP == `HRESP_SPLIT) begin    
                    if (HMASTER == `MASTER_1_ID)
                        m1_split_masked <= 1'b1;
                    else if (HMASTER == `MASTER_2_ID)
                        m2_split_masked <= 1'b1;
                    // Note: If HMASTER 3 gets a split, we ignore the mask. 
                    // Remote masters cannot be safely put into a local split state.
                end         
            end 
            
            // 3. Unmask masters when the Slave finishes fetching data
            if (HSPLIT1)                                
                m1_split_masked <= 1'b0;
            if (HSPLIT2)
                m2_split_masked <= 1'b0;
            
        end
    end

endmodule