// ============================================================================
// File: bus_params.vh
// Description: Common shared parameters
// ============================================================================

`ifndef BUS_PARAMS_VH
`define BUS_PARAMS_VH

// AHB response codes
`define HRESP_OKAY   2'b00
`define HRESP_ERROR  2'b01
`define HRESP_RETRY  2'b10
`define HRESP_SPLIT  2'b11

// Bus widths
`define DATA_WIDTH 8
`define ADDR_WIDTH 14

// Address map
// Total address space = 16 KB

// Slave 1: 4 KB
`define S1_BASE 14'h0000
`define S1_END  14'h0FFF

// Slave 2: 4 KB
`define S2_BASE 14'h1000
`define S2_END  14'h1FFF

// Slave 3: 2 KB
`define S3_BASE 14'h2000
`define S3_END  14'h27FF

// Master IDs
`define MASTER_1_ID 4'd1
`define MASTER_2_ID 4'd2
`define MASTER_3_ID 4'd3

`endif