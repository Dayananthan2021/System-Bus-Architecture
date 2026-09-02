// Combinational address decoder. Selects on HADDR[13:12]:
//
//   00  0x0000-0x0FFF  Slave 1, 4K
//   01  0x1000-0x1FFF  Slave 2, 4K
//   10  0x2000-0x27FF  Slave 3, 2K   (0x2800-0x2FFF is an unmapped hole)
//   11  0x3000-0x3FFF  remote board, via the bridge
//
// Exactly one output is asserted while HSEL_EN is high.

`timescale 1ns / 1ps
`include "../rtl/bus_params.vh"

module address_decoder (
    input  wire [`ADDR_WIDTH-1:0] HADDR,        // address from the granted master
    input  wire                   HSEL_EN,      // decode enable (active transfer)

    output reg                    HSEL1,        // select Slave 1
    output reg                    HSEL2,        // select Slave 2
    output reg                    HSEL3,        // select Slave 3
    output reg                    HSEL_REMOTE,  // select the inter-board bridge
    output reg                    HADDR_INVALID // address maps to nothing
);

    always @(*) begin
        // Default: nothing selected
        HSEL1         = 1'b0;
        HSEL2         = 1'b0;
        HSEL3         = 1'b0;
        HSEL_REMOTE   = 1'b0;
        HADDR_INVALID = 1'b0;

        if (HSEL_EN) begin
            case (HADDR[`ADDR_WIDTH-1:`ADDR_WIDTH-2])
                `SEL_S1: HSEL1 = 1'b1;
                `SEL_S2: HSEL2 = 1'b1;

                // Slave 3 region: only the lower 2 KB is populated.
                `SEL_S3: if (HADDR <= `S3_END) HSEL3         = 1'b1;
                         else                  HADDR_INVALID = 1'b1;

                // Remote window: forwarded to the other board by the bridge.
                `SEL_REMOTE: HSEL_REMOTE = 1'b1;

                default: HADDR_INVALID = 1'b1;
            endcase
        end
    end

endmodule
