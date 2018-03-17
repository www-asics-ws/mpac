/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Dual clock FIFO controller                                 ////
////                                                             ////
////                                                             ////
////  Author: Rudolf Usselmann                                   ////
////           (rudi@asics.ws)                                   ////
////                                                             ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2000-2018 ASICS World Services, LTD.          ////
////                         www.asics.ws                        ////
////                         info@asics.ws                       ////
////                                                             ////
////                                                             ////
//// Redistribution and use in source and binary forms, with or  ////
//// without modification, are permitted provided that the       ////
//// following conditions are met:                               ////
////                                                             ////
////  1. Redistributions of source code must retain the above    ////
////     copyright notice, this list of conditions and the       ////
////     following disclaimer.                                   ////
////                                                             ////
////  2. Redistributions in binary form must reproduce the       ////
////     above copyright notice, this list of conditions and     ////
////     the following disclaimer in the documentation and/or    ////
////     other materials provided with the distribution.         ////
////                                                             ////
////  3. Neither the name of the copyright holder nor the names  ////
////     of its contributors may be used to endorse or promote   ////
////     products derived from this software without specific    ////
////     prior written permission.                               ////
////                                                             ////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND  ////
//// CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, ////
//// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF    ////
//// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE    ////
//// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR       ////
//// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,            ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY     ////
//// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR     ////
//// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY     ////
//// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     ////
//// THE POSSIBILITY OF SUCH DAMAGE.                             ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

`include "mpac_config.v"

/*
Description
===========

I/Os
----
rd_clk_i	Read Port Clock
wr_clk_i	Write Port Clock
reset_i	low active, either sync. or async. master reset (see below how to select)
clear_i	synchronous clear (just like reset but always synchronous), high active
re_i	read enable, synchronous, high active
we_i	read enable, synchronous, high active

full_o	Indicates the FIFO is full (driven at the rising edge of wr_clk_i)
empty_o	Indicates the FIFO is empty (driven at the rising edge of rd_clk_i)

wr_level_o	indicates the FIFO level:
		2'b00	0-25%	 full
		2'b01	25-50%	 full
		2'b10	50-75%	 full
		2'b11	%75-100% full

rd_level_o	indicates the FIFO level:
		2'b00	0-25%	 empty
		2'b01	25-50%	 empty
		2'b10	50-75%	 empty
		2'b11	%75-100% empty

Status Timing
-------------
All status outputs are registered. They are asserted immediately
as the full/empty condition occurs, however, there is a 2 cycle
delay before they are de-asserted once the condition is not true
anymore.

Parameters
----------
The FIFO takes 2 parameters:
dw	Data bus width
aw	Address bus width (Determines the FIFO size by evaluating 2^aw)

Synthesis Results
-----------------
In a Spartan 2e a 8 bit wide, 8 entries deep FIFO, takes 97 LUTs and runs
at about 113 MHz (IO insertion disabled). 

Misc
----
This design assumes you will do appropriate status checking externally.

IMPORTANT ! writing while the FIFO is full or reading while the FIFO is
empty will place the FIFO in an undefined state.
*/

module mpac_fifo_dc_ext_mem #(
parameter aw	=	8
) (
input	wire		rd_clk_i,
input	wire		wr_clk_i,
input	wire		reset_i,
input	wire		clear_i,
input	wire		we_i,		// wr_clk
input	wire		re_i,		// rd_clk
output	wire	[aw-1:0]wr_addr_o,	// wr_clk
output	wire	[aw-1:0]rd_addr_o,	// rd_clk
output	reg		full_o,		// wr_clk
output	reg		empty_o,	// rd_clk
output	reg	[1:0]	wr_level_o,	// wr_clk
output	reg	[1:0]	rd_level_o	// rd_clk
);

////////////////////////////////////////////////////////////////////
//
// Local Wires
//

reg	[aw:0]		wp_bin, wp_gray;
reg	[aw:0]		rp_bin, rp_gray;
reg	[aw:0]		wp_s, rp_s;

wire	[aw:0]		wp_bin_next, wp_gray_next;
wire	[aw:0]		rp_bin_next, rp_gray_next;

wire	[aw:0]		wp_bin_x, rp_bin_x;
reg	[aw-1:0]	d1, d2;

reg			rd_rst, wr_rst;
reg			rd_rst_r, wr_rst_r;
reg			rd_clr, wr_clr;
reg			rd_clr_r, wr_clr_r;

// aliases
assign wr_addr_o = wp_bin[aw-1:0];
assign rd_addr_o = rp_bin[aw-1:0];

////////////////////////////////////////////////////////////////////
//
// Reset Logic
//

always @(posedge rd_clk_i or negedge reset_i)
	if(!reset_i)	rd_rst <= #1 1'b0;
	else
	if(rd_rst_r)	rd_rst <= #1 1'b1;		// Release Reset

always @(posedge rd_clk_i or negedge reset_i)
	if(!reset_i)	rd_rst_r <= #1 1'b0;
	else		rd_rst_r <= #1 1'b1;

always @(posedge wr_clk_i or negedge reset_i)
	if(!reset_i)	wr_rst <= #1 1'b0;
	else
	if(wr_rst_r)	wr_rst <= #1 1'b1;		// Release Reset

always @(posedge wr_clk_i or negedge reset_i)
	if(!reset_i)	wr_rst_r <= #1 1'b0;
	else		wr_rst_r <= #1 1'b1;

always @(posedge rd_clk_i or posedge clear_i)
	if(clear_i)	rd_clr <= #1 1'b1;
	else
	if(!rd_clr_r)	rd_clr <= #1 1'b0;		// Release Clear

always @(posedge rd_clk_i or posedge clear_i)
	if(clear_i)	rd_clr_r <= #1 1'b1;
	else		rd_clr_r <= #1 1'b0;

always @(posedge wr_clk_i or posedge clear_i)
	if(clear_i)	wr_clr <= #1 1'b1;
	else
	if(!wr_clr_r)	wr_clr <= #1 1'b0;		// Release Clear

always @(posedge wr_clk_i or posedge clear_i)
	if(clear_i)	wr_clr_r <= #1 1'b1;
	else		wr_clr_r <= #1 1'b0;

////////////////////////////////////////////////////////////////////
//
// Read/Write Pointers Logic
//

always @(posedge wr_clk_i)
	if(!wr_rst)	wp_bin <= #1 {aw+1{1'b0}};
	else
	if(wr_clr)	wp_bin <= #1 {aw+1{1'b0}};
	else
	if(we_i)	wp_bin <= #1 wp_bin_next;

always @(posedge wr_clk_i)
	if(!wr_rst)	wp_gray <= #1 {aw+1{1'b0}};
	else
	if(wr_clr)	wp_gray <= #1 {aw+1{1'b0}};
	else
	if(we_i)	wp_gray <= #1 wp_gray_next;

assign wp_bin_next  = wp_bin + {{aw{1'b0}},1'b1};
assign wp_gray_next = wp_bin_next ^ {1'b0, wp_bin_next[aw:1]};

always @(posedge rd_clk_i)
	if(!rd_rst)	rp_bin <= #1 {aw+1{1'b0}};
	else
	if(rd_clr)	rp_bin <= #1 {aw+1{1'b0}};
	else
	if(re_i)	rp_bin <= #1 rp_bin_next;

always @(posedge rd_clk_i)
	if(!rd_rst)	rp_gray <= #1 {aw+1{1'b0}};
	else
	if(rd_clr)	rp_gray <= #1 {aw+1{1'b0}};
	else
	if(re_i)	rp_gray <= #1 rp_gray_next;

assign rp_bin_next  = rp_bin + {{aw{1'b0}},1'b1};
assign rp_gray_next = rp_bin_next ^ {1'b0, rp_bin_next[aw:1]};

////////////////////////////////////////////////////////////////////
//
// Synchronization Logic
//

// write pointer
always @(posedge rd_clk_i)	wp_s <= #1 wp_gray;

// read pointer
always @(posedge wr_clk_i)	rp_s <= #1 rp_gray;

////////////////////////////////////////////////////////////////////
//
// Registered Full & Empty Flags
//

assign wp_bin_x = wp_s ^ {1'b0, wp_bin_x[aw:1]};	// convert gray to binary
assign rp_bin_x = rp_s ^ {1'b0, rp_bin_x[aw:1]};	// convert gray to binary

always @(posedge rd_clk_i)
        empty_o <= #1 (wp_s == rp_gray) | (re_i & (wp_s == rp_gray_next));

always @(posedge wr_clk_i)
        full_o <= #1 ((wp_bin[aw-1:0] == rp_bin_x[aw-1:0]) & (wp_bin[aw] != rp_bin_x[aw])) |
        (we_i & (wp_bin_next[aw-1:0] == rp_bin_x[aw-1:0]) & (wp_bin_next[aw] != rp_bin_x[aw]));

////////////////////////////////////////////////////////////////////
//
// Registered Level Indicators
//
reg	[aw-1:0]	wp_bin_xr, rp_bin_xr;
reg			full_rc;
reg			full_wc;

always @(posedge wr_clk_i)	full_wc <= #1 full_o;
always @(posedge wr_clk_i)	rp_bin_xr <= #1  ~rp_bin_x[aw-1:0] + {{aw-1{1'b0}}, 1'b1};
always @(posedge wr_clk_i)	d1 <= #1 wp_bin[aw-1:0] + rp_bin_xr[aw-1:0];

always @(posedge wr_clk_i)	wr_level_o <= #1 {d1[aw-1] | full_o | full_wc, d1[aw-2] | full_o | full_wc};

always @(posedge rd_clk_i)	wp_bin_xr <= #1  ~wp_bin_x[aw-1:0];
always @(posedge rd_clk_i)	d2 <= #1 rp_bin[aw-1:0] + wp_bin_xr[aw-1:0];

always @(posedge rd_clk_i)	full_rc <= #1 full_o;
always @(posedge rd_clk_i)	rd_level_o <= #1 full_rc ? 2'h0 : {d2[aw-1] | empty_o, d2[aw-2] | empty_o};

////////////////////////////////////////////////////////////////////
//
// Sanity Check
//

// synopsys translate_off
always @(posedge wr_clk_i)
	if(we_i && full_o)
		$display("%m WARNING: Writing while fifo is FULL (%t)",$time);

always @(posedge rd_clk_i)
	if(re_i && empty_o)
		$display("%m WARNING: Reading while fifo is EMPTY (%t)",$time);
// synopsys translate_on

endmodule

