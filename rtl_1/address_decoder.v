// ============================================================================
// File: address_decoder.v
// Description: Fully parameterizable address decoder for 3 slaves + Remote Bridge
// ============================================================================

`include "bus_params.vh"

module address_decoder (
    input wire [`ADDR_WIDTH-1:0] HADDR,
    input wire                   HBUSREQ,

    // Local Slaves
    output reg                   HSEL1,
    output reg                   HSEL2,
    output reg                   HSEL3,
    
    // Remote Bridge (The "Ghost" Portal)
    output reg                   HSEL_BRIDGE
);

    always @(*) begin
        // Default: Deselect everything
        HSEL1       = 1'b0;
        HSEL2       = 1'b0;
        HSEL3       = 1'b0;
        HSEL_BRIDGE = 1'b0;

        if (HBUSREQ) begin
            
            // Check the "Mirror Universe" Bit dynamically using the MSB of ADDR_WIDTH
            // If the highest bit is 1, route to the Bridge TX
            if (HADDR[`ADDR_WIDTH-1] == 1'b1) begin
                HSEL_BRIDGE = 1'b1;
            end 
            
            // Otherwise, decode normally based on local memory map
            else begin
                if ((HADDR >= `S1_BASE) && (HADDR <= `S1_END)) begin
                    HSEL1 = 1'b1;
                end
                else if ((HADDR >= `S2_BASE) && (HADDR <= `S2_END)) begin
                    HSEL2 = 1'b1;
                end
                else if ((HADDR >= `S3_BASE) && (HADDR <= `S3_END)) begin
                    HSEL3 = 1'b1;   
                end
            end
            
        end
    end

endmodule