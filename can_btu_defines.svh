//-----------------------------------------------------------------------------
// Module: can_btu_defines.svh
// Description: Common definitions for CAN Bit Timing Unit (BTU)
// Author: CAN Controller Project
// Version: 1.0
//-----------------------------------------------------------------------------

`ifndef CAN_BTU_DEFINES_SVH
`define CAN_BTU_DEFINES_SVH

//-----------------------------------------------------------------------------
// Bit Timing Parameters
//-----------------------------------------------------------------------------

// Timing segment widths
localparam int CAN_PRESC_WIDTH   = 8;   // Prescaler width
localparam int CAN_PROP_WIDTH    = 3;   // Propagation segment width
localparam int CAN_PHASE_WIDTH   = 3;   // Phase segment width
localparam int CAN_SJW_WIDTH     = 2;   // Sync Jump Width width

// Default timing values (for 500 kbps @ 50 MHz clock)
localparam bit [7:0]  DEFAULT_PRESCALER = 8'd4;
localparam bit [2:0]  DEFAULT_PROP_SEG  = 3'd2;
localparam bit [2:0]  DEFAULT_PHASE_SEG1 = 3'd4;
localparam bit [2:0]  DEFAULT_PHASE_SEG2 = 3'd4;
localparam bit [1:0]  DEFAULT_SJW       = 2'd2;

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

// Dominant and recessive bit values
`define CAN_DOMINANT 1'b0
`define CAN_RECESSIVE 1'b1

`endif // CAN_BTU_DEFINES_SVH