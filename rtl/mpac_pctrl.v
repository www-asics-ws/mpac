/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Protocol Controller and Sync generator                     ////
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

//  $Id: $

`include "mpac_config.v"

module mpac_pctrl #(
parameter integer	NO_IN_CH	= 8,	// Max number of input channels supported by hardware (1-16)
parameter integer	NO_OUT_CH	= 8,	// Max number of output channels supported by hardware (1-16)
parameter integer	MAX_FIFO_DEPTH	= 10	// FIFO (buffer) depth for each channel (as Log2) in DW
) (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// i2s Clock
input	wire		rst_i,		// i2s Reset
input	wire		soft_rst_i,	// i2s Software Reset

	// ===============================================================================
	// Internal Interface

output	reg		ser_out_ld_o,	// Output Serializer load
output	reg	[3:0]	out_ch_sel_o,	// Output channel select

output	reg		ser_in_ld_o,	// Input Serializer load
output	reg	[3:0]	in_ch_sel_o,	// Input channel select

	// ===============================================================================
	// Register File Interface

input	wire		enable_i,	// IP Core enabled

input	wire	[3:0]	no_ich_i,	// Number of input channels
					// Must be less than or equal to NO_IN_CH
input	wire	[3:0]	no_och_i,	// Number of output channels
					// Must be less than or equal to NO_OUT_CH

input	wire	[2:0]	prot_sel_i,	// Protocol Select
					// 0 - I2S (2 channel only)
					// 1 - TDM
					// 2 - AC97
					// 3-7 RESERVED

input	wire	[2:0]	sample_size_i,	// Sample size from SoC
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED

input	wire	[2:0]	ser_slot_width_i, // Serial Slot Width
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED

input	wire	[1:0]	sync_start_i,	// Sync start
					// 0 - sync with serial data
					// 1 - one clock before serial data
					// 2-3 RESERVED

input	wire	[1:0]	sync_dur_i,	// Sync duration
					// 0 - One cycle active at the beginning
					// 1 - 50% duty cycle
					// 2 - One cycle in-active at the end
					// 3 RESERVED

input wire		sync_inv_i,	// Invert Sync
					// 0 - not inverted (first high than low)
					// 1 - inverted (first low than high)

	// ===============================================================================
	// i2s Interface

output	reg		i2s_wc_sync_o		// I2S Word Clock/Sync
);


// ---------------------------------------------------------------------------------------
// Various functions


// ---------------------------------------------------------------------------------------
// Local Parameters


// ---------------------------------------------------------------------------------------
// Wires and Registers

genvar		n;
integer		i;

reg		enable_r;
reg		in_enable_r;
reg	[3:0]	ch_cnt, ch_cnt_r;

// ---------------------------------------------------------------------------------------
// 

// -----------------------------------------------
// Slot Timer
reg	[4:0]	bit_cnt;
reg		b_ld;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)		bit_cnt <= #1 5'h0;
	else
	if(soft_rst_i || !enable_i)	bit_cnt <= #1 5'h0;
	else
	if(b_ld)			bit_cnt <= #1 5'h0;
	else				bit_cnt <= #1 bit_cnt + 5'h1;

// Slot synch
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	b_ld <= #1 1'b1;
	else
	   begin
		case(sample_size_i)	// actual count less 2
		   3'h0:	b_ld <= #1 bit_cnt == 5'd14;
		   3'h1:	b_ld <= #1 bit_cnt == 5'd16;
		   3'h2:	b_ld <= #1 bit_cnt == 5'd18;
		   3'h3:	b_ld <= #1 bit_cnt == 5'd22;
		   3'h4:	b_ld <= #1 bit_cnt == 5'd30;
		   default:	b_ld <= #1 1'b1;
		endcase
	   end

// -----------------------------------------------
// Frame Timer

reg	[3:0]	frm_cnt;
reg		f_ld;
reg		h_ld;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)		frm_cnt <= #1 4'h0;
	else
	if(soft_rst_i)			frm_cnt <= #1 4'h0;
	else
	if(f_ld)			frm_cnt <= #1 4'h0;
	else
	if(b_ld)			frm_cnt <= #1 frm_cnt + 4'h1;

// frame sync
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	f_ld <= #1 1'b0;
	else
	if(b_ld)		f_ld <= #1 frm_cnt == no_och_i;
	else			f_ld <= #1 1'b0;

always @(posedge clk_i `MPAC_ACRT)	// Half Frame Timer
	if(rst_i == `MPAC_ACR)	h_ld <= #1 1'b0;
	else
	if(b_ld)		h_ld <= #1 frm_cnt == {1'b0, no_och_i[3:1]};
	else			h_ld <= #1 1'b0;

// Synchronize enable sugnals for internal usage
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	enable_r <= #1 1'b0;
	else
	if(!enable_i)		enable_r <= #1 1'b0;
	else
	if(f_ld)		enable_r <= #1 enable_i;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	in_enable_r <= #1 1'b0;
	else
	if(!enable_i)		in_enable_r <= #1 1'b0;
	else
	if(ser_out_ld_o)	in_enable_r <= #1 enable_r;

// -----------------------------------------------
// Channel Counter

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	ch_cnt <= #1 4'h0;
	else
	if(f_ld)		ch_cnt <= #1 4'h0;
	else
	if(b_ld)		ch_cnt <= #1 ch_cnt + 4'h1;

always @(posedge clk_i) out_ch_sel_o <= #1 ch_cnt;

always @(posedge clk_i) if(b_ld)	ch_cnt_r <= #1 ch_cnt;		// FIX_ME
always @(posedge clk_i) if(ser_in_ld_o)	in_ch_sel_o <= #1 ch_cnt_r;	// FIX_ME	<<<<<<<<<

// -----------------------------------------------
// Channel Latch Enable

reg	[15:0]	ser_out_ld_r;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	ser_out_ld_r <= #1 16'h0;
	else
	if(!enable_i)		ser_out_ld_r <= #1 16'h0;
	else			ser_out_ld_r <= #1 {ser_out_ld_r[14:0], b_ld};

always @(posedge clk_i) ser_out_ld_o <= #1 ser_out_ld_r[6] & enable_r;		// FIX_ME
always @(posedge clk_i) ser_in_ld_o  <= #1 ser_out_ld_r[7] & in_enable_r;	// FIX_ME

// -----------------------------------------------
// Sync Generation

reg	[7:0]	i2s_wc_sync_sr;
reg		i2s_wc_sync_r, i2s_wc_sync_r1;
wire		i2s_wc_sync_d;

always @(posedge clk_i)	i2s_wc_sync_o <= #1 (((sync_start_i==2'h1) ? i2s_wc_sync_d : i2s_wc_sync_r1) ^ sync_inv_i) & enable_r;
always @(posedge clk_i)	i2s_wc_sync_r1 <= #1 i2s_wc_sync_d;

assign i2s_wc_sync_d = i2s_wc_sync_sr[4];

always @(posedge clk_i)	i2s_wc_sync_sr <= #1 {i2s_wc_sync_sr[6:0], i2s_wc_sync_r};

always @(posedge clk_i `MPAC_ACRT)	// Half Frame Timer
	if(rst_i == `MPAC_ACR)	i2s_wc_sync_r <= #1 1'b0;
	else
	if(!enable_i)		i2s_wc_sync_r <= #1 1'b0;
	else
	if(f_ld)		i2s_wc_sync_r <= #1 1'b1;
	else
	   begin
		case(sync_dur_i)
		   2'h0:	i2s_wc_sync_r <= #1 1'b0;
		   2'h1:	if(h_ld)				i2s_wc_sync_r <= #1 1'b0;
		   2'h2:	if(b_ld && (frm_cnt == no_och_i))	i2s_wc_sync_r <= #1 1'b0;
		endcase
	   end

endmodule
