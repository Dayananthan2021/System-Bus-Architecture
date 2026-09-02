// Single point of configuration for the bus: widths, address map, response
// codes, master IDs and the inter-board link timing. Change values here only.

`ifndef BUS_PARAMS_VH
`define BUS_PARAMS_VH

// AHB response codes
`define HRESP_OKAY   2'b00
`define HRESP_ERROR  2'b01
`define HRESP_RETRY  2'b10
`define HRESP_SPLIT  2'b11

// Transfer types
`define HTRANS_IDLE  2'b00
`define HTRANS_NONSEQ 2'b10

// 8-bit data over a 14-bit byte address => 16K space, 10K populated.
`define DATA_WIDTH  8
`define ADDR_WIDTH 14

// Address map, selected by HADDR[13:12]. The remote window forwards to the
// other board with the low 12 bits unchanged, so 0x3000 here reaches 0x0000
// there.
`define SEL_S1      2'b00
`define SEL_S2      2'b01
`define SEL_S3      2'b10
`define SEL_REMOTE  2'b11

// Remote (bridge) window: 4K @ 0x3000 - 0x3FFF
`define RMT_BASE 14'h3000
`define RMT_END  14'h3FFF

// Slave 1: 4 KB @ 0x0000 - 0x0FFF
`define S1_BASE  14'h0000
`define S1_END   14'h0FFF
`define S1_DEPTH 4096
`define S1_IDXW  12               // log2(S1_DEPTH)

// Slave 2: 4 KB @ 0x1000 - 0x1FFF
`define S2_BASE  14'h1000
`define S2_END   14'h1FFF
`define S2_DEPTH 4096
`define S2_IDXW  12

// Slave 3: 2 KB @ 0x2000 - 0x27FF  (0x2800 - 0x2FFF is an unmapped hole)
`define S3_BASE  14'h2000
`define S3_END   14'h27FF
`define S3_DEPTH 2048
`define S3_IDXW  11

// Master IDs, carried on HMASTER
`define MASTER_NONE 4'd0
`define MASTER_1_ID 4'd1
`define MASTER_2_ID 4'd2
`define MASTER_3_ID 4'd3          // remote traffic replayed by the bridge

// System and link configuration.
`define CLK_FREQ_HZ  10_000_000   // bus clock (50 MHz / 5, exact -- no PLL)
`define CLK_DIV               5   // CLOCK_50 -> bus clock divider
`define UART_BAUD       115_200   // inter-board link, 8-N-1

// Derived, not hand-edited. The +BAUD/2 rounds to nearest: at 10 MHz/115200
// the exact value is 86.81, so 87 gives 2.1% error at the stop bit where
// truncating to 86 would give 8.8%.
`define UART_CLKS_PER_BIT (((`CLK_FREQ_HZ) + ((`UART_BAUD)/2)) / (`UART_BAUD))

// Inter-board frame markers, see docs/BRIDGE_INTERFACE.md
`define LINK_SYNC_CMD  8'hA5      // start of a command frame
`define LINK_SYNC_RSP  8'h5A      // start of a response frame
`define LINK_STAT_OK   8'h00      // response status: success
`define LINK_STAT_ERR  8'h01      // response status: error / bad address

`endif
