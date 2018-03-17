/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Data Mux                                                   ////
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

module mpac_data_mux #(
parameter integer	NO_IN_CH	= 8,	// Max number of input channels supported by hardware (1-16)
parameter integer	NO_OUT_CH	= 8	// Max number of output channels supported by hardware (1-16)
) (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// i2s Clock
input	wire		rst_i,		// i2s Reset
input	wire		soft_rst_i,	// i2s Software Reset

	// ===============================================================================
	// Register File Interface

input	wire		enable_i,	// IP Core enabled
input	wire	[3:0]	no_ich_i,	// Number of input channels
					// Must be less than or equal to NO_IN_CH
input	wire	[3:0]	no_och_i,	// Number of output channels
					// Must be less than or equal to NO_OUT_CH

input	wire	[31:0]	mute_val_i,	// Mute Value
input	wire	[15:0]	mute_och_i,	// Mute output channel (each bit corresponds to one channel)
input	wire	[15:0]	mute_ich_i,	// Mute input channel (each bit corresponds to one channel)

input	wire	[2:0]	sample_size_i,	// Sample size from SoC
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED

input	wire		sample_align_i,	// Sample alignment
					// 0 - right (lsb)
					// 1 - left (msb)

input	wire		sample_pac_i,	// Samples are packed
					// 0 - no
					// 1 - yes

	// ===============================================================================
	// Protocol Controller Interface

input	wire		ser_in_le_i,	// Input Serializer load
input	wire	[3:0]	in_ch_sel_i,	// Input channel select

input	wire		ser_out_ld_i,	// Output Serializer load
input	wire	[3:0]	out_ch_sel_i,	// Output channel select

	// ===============================================================================
	// Buffer Interface

output	reg		buf_in_we_o,	// Input buffer write enable
input	wire	[31:0]	buf_din_i,	// IN FIFO Data input  (SoC to Codec)

output	reg		buf_out_rd_o,	// Output buffer read
output	reg	[31:0]	buf_dout_o,	// OUT FIFO Data output  (Codec to SoC)

	// ===============================================================================
	// Serializer Interface

input	wire	[31:0]	ser_din_i,	// IN FIFO Data input  (Codec to SoC)
output	reg	[31:0]	ser_dout_o	// OUT FIFO Data output  (SoC to Codec)
);

// ---------------------------------------------------------------------------------------
// Various functions

function integer ilog2;
input [31:0] value;
   begin
	case(value)
	   2:  ilog2 = 1;
	   3:  ilog2 = 2;
	   4:  ilog2 = 2;
	   5:  ilog2 = 3;
	   6:  ilog2 = 3;
	   7:  ilog2 = 3;
	   8:  ilog2 = 3;
	   9:  ilog2 = 4;
	   10: ilog2 = 4;
	   11: ilog2 = 4;
	   12: ilog2 = 4;
	   13: ilog2 = 4;
	   14: ilog2 = 4;
	   15: ilog2 = 4;
	   16: ilog2 = 4;
	default: ilog2 = 10000;
	endcase
   end
endfunction

// ---------------------------------------------------------------------------------------
// Local Parameters

localparam integer out_ch_l2 = ilog2(NO_OUT_CH);
localparam integer in_ch_l2  = ilog2(NO_IN_CH);

// ---------------------------------------------------------------------------------------
// Wires and Registers

reg	[31:0]	bout_r1, bout_r2;
reg		ser_in_le_r, ser_in_le_r1, ser_in_le_r2;
reg	[4:0]	in_cnt;
wire	[31:0]	tmp_ibuf0, tmp_ibuf1;
reg	[31:0]	ibuf0[NO_IN_CH-1:0];
reg	[31:0]	ibuf1[NO_IN_CH-1:0];
reg		in_mute;

// ---------------------------------------------------------------------------------------
// Misc Logic

// ---------------------------------------------------------------------------------------
// Data mux Logic

// ====================================================================================
// Codec to SoC

// -------------------------------
// Buffer write enable delay line
always @(posedge clk_i) ser_in_le_r1 <= #1 ser_in_le_i;
always @(posedge clk_i) ser_in_le_r  <= #1 ser_in_le_r1;
always @(posedge clk_i) ser_in_le_r2 <= #1 ser_in_le_r;

// -------------------------------
// Buffer write enable control 
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	buf_in_we_o <= #1 1'b0;
	else
	if(soft_rst_i)		buf_in_we_o <= #1 1'b0;
	else
	if(!sample_pac_i)	buf_in_we_o <= #1 ser_in_le_r2;
	else
		case(sample_size_i)
		   3'h0:if(in_cnt==5'h1)	buf_in_we_o <= #1 ser_in_le_r2;	// 16 bit packing write enable
			else			buf_in_we_o <= #1 1'b0;

		   3'h1:if(in_cnt==5'h1)	buf_in_we_o <= #1 ser_in_le_r2;	// 18 bit packing write enable
			else
		   	if(in_cnt==5'h3)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h5)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h7)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h8)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'ha)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'hc)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'he)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'hf)	buf_in_we_o <= #1 ser_in_le_r2;
			else			buf_in_we_o <= #1 1'b0;

		   3'h2:if(in_cnt==5'h1)	buf_in_we_o <= #1 ser_in_le_r2;	// 20 bit packing write enable
			else
		   	if(in_cnt==5'h3)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h4)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h6)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h7)	buf_in_we_o <= #1 ser_in_le_r2;
			else			buf_in_we_o <= #1 1'b0;

		   3'h3:if(in_cnt==5'h1)	buf_in_we_o <= #1 ser_in_le_r2;	// 24 bit packing write enable
			else
		   	if(in_cnt==5'h2)	buf_in_we_o <= #1 ser_in_le_r2;
			else
		   	if(in_cnt==5'h3)	buf_in_we_o <= #1 ser_in_le_r2;
			else			buf_in_we_o <= #1 1'b0;

		    default:	buf_in_we_o <= #1 ser_in_le_r2;			// 32 bit no packing
		endcase


// -------------------------------
// In Packing Track Counter
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)				in_cnt <= #1 5'h0;
	else
	if(soft_rst_i || !enable_i)			in_cnt <= #1 5'h0;
	else
	if((in_ch_sel_i == no_ich_i) && ser_in_le_i)	in_cnt <= #1 in_cnt + 5'h1;
	else
		case(sample_size_i)	// Reset packing counter
		   3'h0:	if(in_cnt==5'h02)	in_cnt <= #1 5'h0;	// 16 bit packing
		   3'h1:	if(in_cnt==5'h10)	in_cnt <= #1 5'h0;	// 18 bit packing
		   3'h2:	if(in_cnt==5'h08)	in_cnt <= #1 5'h0;	// 20 bit packing
		   3'h3:	if(in_cnt==5'h04)	in_cnt <= #1 5'h0;	// 24 bit packing
		default:				in_cnt <= #1 5'h0;	// 32 bit no packing
		endcase


// -------------------------------
// Temp Buffer for in packer

// Write
always @(posedge clk_i) if(ser_in_le_r)	ibuf0[in_ch_sel_i[in_ch_l2-1:0]] <= #1 bout_r1;
always @(posedge clk_i) if(ser_in_le_r)	ibuf1[in_ch_sel_i[in_ch_l2-1:0]] <= #1 tmp_ibuf0;
	
// Read
assign tmp_ibuf0 = ibuf0[in_ch_sel_i[in_ch_l2-1:0]];
assign tmp_ibuf1 = ibuf1[in_ch_sel_i[in_ch_l2-1:0]];

// -------------------------------
// Input register
always @(posedge clk_i)	bout_r1 <= #1 ser_din_i;

// -------------------------------
// Data packer
always @(posedge clk_i)
		if(sample_pac_i)
		   begin
			case(sample_size_i)
			   3'h0:	bout_r2 <= #1 {bout_r1[15:00], tmp_ibuf0[15:00]};			// 16 bit packing

			   3'h1:
				case(in_cnt[3:0])						// 18 bit packing
				   4'h1:	bout_r2 <= #1 {bout_r1[13:0], tmp_ibuf0[17:0]};
				   4'h3:	bout_r2 <= #1 {bout_r1[09:0], tmp_ibuf0[17:0], tmp_ibuf1[17:14]};
				   4'h5:	bout_r2 <= #1 {bout_r1[05:0], tmp_ibuf0[17:0], tmp_ibuf1[17:10]};
				   4'h7:	bout_r2 <= #1 {bout_r1[01:0], tmp_ibuf0[17:0], tmp_ibuf1[17:06]};
				   4'h8:	bout_r2 <= #1 {bout_r1[15:0], tmp_ibuf0[17:02]};
				   4'ha:	bout_r2 <= #1 {bout_r1[11:0], tmp_ibuf0[17:0], tmp_ibuf1[17:16]};
				   4'hc:	bout_r2 <= #1 {bout_r1[07:0], tmp_ibuf0[17:0], tmp_ibuf1[17:12]};
				   4'he:	bout_r2 <= #1 {bout_r1[03:0], tmp_ibuf0[17:0], tmp_ibuf1[17:08]};
				   default:	bout_r2 <= #1 {bout_r1[17:0], tmp_ibuf0[17:04]};
				endcase

			   3'h2:
				case(in_cnt[2:0])						// 20 bit packing
				   3'h1:	bout_r2 <= #1 {bout_r1[11:0], tmp_ibuf0[19:0]};
				   3'h3:	bout_r2 <= #1 {bout_r1[03:0], tmp_ibuf0[19:0], tmp_ibuf1[19:12]};
				   3'h4:	bout_r2 <= #1 {bout_r1[15:0], tmp_ibuf0[19:4]};
				   3'h6:	bout_r2 <= #1 {bout_r1[07:0], tmp_ibuf0[19:0], tmp_ibuf1[19:16]};
				   default:	bout_r2 <= #1 {bout_r1[19:0], tmp_ibuf0[19:08]};
				endcase

			   3'h3:
				case(in_cnt[1:0])						// 24 bit packing
				   2'h1:	bout_r2 <= #1 {bout_r1[07:0], tmp_ibuf0[23:00]};
				   2'h2:	bout_r2 <= #1 {bout_r1[15:0], tmp_ibuf0[23:08]};
				   default:	bout_r2 <= #1 {bout_r1[23:0], tmp_ibuf0[23:16]};
				endcase

			default:	bout_r2 <= #1 bout_r1;						// 32 bit no-packing
			endcase
		   end
		else	bout_r2 <= #1 bout_r1;


// -------------------------------
// Data Alignment & Channel muting

always @(posedge clk_i)
	in_mute <= #1 mute_ich_i[in_ch_sel_i];

always @(posedge clk_i)
		if(in_mute)	buf_dout_o <= #1 mute_val_i;	// Mute Channel
		else
		if(sample_align_i && !sample_pac_i)
		   begin
			case(sample_size_i)	// Align data left
			   3'h0:	buf_dout_o <= #1 {bout_r2[15:0], 16'h0};	// 16 bit
			   3'h1:	buf_dout_o <= #1 {bout_r2[17:0], 14'h0};	// 18 bit
			   3'h2:	buf_dout_o <= #1 {bout_r2[19:0], 12'h0};	// 20 bit
			   3'h3:	buf_dout_o <= #1 {bout_r2[23:0],  8'h0};	// 24 bit
			default:	buf_dout_o <= #1 bout_r2;			// 32 bit
			endcase
		   end
		else		buf_dout_o <= #1 bout_r2;	// No realignment

// ====================================================================================
// SoC to Codec

reg	[31:0]	sout_r1, sout_r2;
reg	[4:0]	out_cnt;
wire	[31:0]	tmp_obuf0, tmp_obuf1;
reg	[31:0]	obuf0[NO_OUT_CH-1:0];
reg	[31:0]	obuf1[NO_OUT_CH-1:0];
reg		out_mute;

// -------------------------------
// Buffer Read Enable
always @(posedge clk_i)
	if(sample_pac_i)
		case(sample_size_i)	// Reset packing counter
		   3'h0:
			if(out_cnt==5'h01)	buf_out_rd_o <= #1 ser_out_ld_i;	// 16 bit un-packing
			else			buf_out_rd_o <= #1 1'b0;

		   3'h1:
			if(out_cnt==5'h00)	buf_out_rd_o <= #1 ser_out_ld_i;	// 18 bit un-packing
			else
			if(out_cnt==5'h01)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h03)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h05)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h07)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h08)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h0a)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h0c)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h0e)	buf_out_rd_o <= #1 ser_out_ld_i;
			else			buf_out_rd_o <= #1 1'b0;

		   3'h2:
			if(out_cnt==5'h00)	buf_out_rd_o <= #1 ser_out_ld_i;	// 20 bit un-packing
			else
			if(out_cnt==5'h01)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h03)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h04)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h06)	buf_out_rd_o <= #1 ser_out_ld_i;
			else			buf_out_rd_o <= #1 1'b0;

		   3'h3:
			if(out_cnt==5'h00)	buf_out_rd_o <= #1 ser_out_ld_i;	// 24 bit un-packing
			else
			if(out_cnt==5'h01)	buf_out_rd_o <= #1 ser_out_ld_i;
			else
			if(out_cnt==5'h02)	buf_out_rd_o <= #1 ser_out_ld_i;
			else			buf_out_rd_o <= #1 1'b0;

		  default:	buf_out_rd_o <= #1 ser_out_ld_i;			// 32 bit no packing
		endcase

	else	buf_out_rd_o <= #1 ser_out_ld_i;

// -------------------------------
// Out Packing Track Counter
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)				out_cnt <= #1 5'h0;
	else
	if(soft_rst_i || !enable_i)			out_cnt <= #1 5'h0;
	else
	if((out_ch_sel_i == no_och_i) && ser_out_ld_i)	out_cnt <= #1 out_cnt + 5'h1;
	else
		case(sample_size_i)	// Reset packing counter
		   3'h0:	if(out_cnt==5'h02)	out_cnt <= #1 5'h0;	// 16 bit packing
		   3'h1:	if(out_cnt==5'h10)	out_cnt <= #1 5'h0;	// 18 bit packing
		   3'h2:	if(out_cnt==5'h08)	out_cnt <= #1 5'h0;	// 20 bit packing
		   3'h3:	if(out_cnt==5'h04)	out_cnt <= #1 5'h0;	// 24 bit packing
		  default:				out_cnt <= #1 5'h0;	// 32 bit no packing
		endcase

// -------------------------------
// Temp Buffer for out packer

// Write
always @(posedge clk_i) if(ser_out_ld_i)	obuf0[out_ch_sel_i[out_ch_l2-1:0]] <= #1 sout_r1;
always @(posedge clk_i) if(ser_out_ld_i)	obuf1[out_ch_sel_i[out_ch_l2-1:0]] <= #1 tmp_obuf0;
	
// Read
assign tmp_obuf0 = obuf0[out_ch_sel_i[out_ch_l2-1:0]];
assign tmp_obuf1 = obuf1[out_ch_sel_i[out_ch_l2-1:0]];

// -------------------------------
// Data Alignment
always @(posedge clk_i)		// Align data right
		if(sample_align_i && !sample_pac_i)
		   begin
			case(sample_size_i)
			   3'h0:	sout_r1 <= #1 {16'h0, buf_din_i[31:16]};	// 16 bit
			   3'h1:	sout_r1 <= #1 {14'h0, buf_din_i[31:14]};	// 18 bit
			   3'h2:	sout_r1 <= #1 {12'h0, buf_din_i[31:12]};	// 20 bit
			   3'h3:	sout_r1 <= #1 { 8'h0, buf_din_i[31:8]};		// 24 bit
			default:	sout_r1 <= #1 buf_din_i;			// 32 bit
			endcase
		   end
		else		sout_r1 <= #1 buf_din_i;	// No realignment

// -------------------------------
// Data Unpacking
always @(posedge clk_i)
	if(sample_pac_i)
		case(sample_size_i)
		   3'h0:
			if(out_cnt[0])	sout_r2 <= #1 {16'h0, tmp_obuf0[31:16]};	// 16 bit un-packing
		   	else		sout_r2 <= #1 {16'h0, sout_r1[15:0]};

		   3'h1:
			case(out_cnt[3:0])						// 18 bit un-packing
			   4'h0:	sout_r2 <= #1 {14'h0, sout_r1[17:0]};
			   4'h1:	sout_r2 <= #1 {14'h0, sout_r1[03:0], tmp_obuf0[31:18]};
			   4'h2:	sout_r2 <= #1 {14'h0, tmp_obuf0[21:04]};
			   4'h3:	sout_r2 <= #1 {14'h0, sout_r1[07:0], tmp_obuf1[31:22]};
			   4'h4:	sout_r2 <= #1 {14'h0, tmp_obuf0[25:08]};
			   4'h5:	sout_r2 <= #1 {14'h0, sout_r1[11:0], tmp_obuf1[31:26]};
			   4'h6:	sout_r2 <= #1 {14'h0, tmp_obuf0[29:12]};
			   4'h7:	sout_r2 <= #1 {14'h0, sout_r1[15:0], tmp_obuf1[31:30]};
			   4'h8:	sout_r2 <= #1 {14'h0, sout_r1[01:0], tmp_obuf0[31:16]};
			   4'h9:	sout_r2 <= #1 {14'h0, tmp_obuf0[19:02]};
			   4'ha:	sout_r2 <= #1 {14'h0, sout_r1[05:0], tmp_obuf1[31:20]};
			   4'hb:	sout_r2 <= #1 {14'h0, tmp_obuf0[23:06]};
			   4'hc:	sout_r2 <= #1 {14'h0, sout_r1[09:0], tmp_obuf1[31:24]};
			   4'hd:	sout_r2 <= #1 {14'h0, tmp_obuf0[27:10]};
			   4'he:	sout_r2 <= #1 {14'h0, sout_r1[13:0], tmp_obuf1[31:28]};
			   default:	sout_r2 <= #1 {14'h0, tmp_obuf0[31:14]};
			endcase

		   3'h2:
			case(out_cnt[2:0])						// 20 bit un-packing
			   3'h0:	sout_r2 <= #1 {12'h0, sout_r1[19:0]};
			   3'h1:	sout_r2 <= #1 {12'h0, sout_r1[07:0], tmp_obuf0[31:20]};
			   3'h2:	sout_r2 <= #1 {12'h0, tmp_obuf0[27:08]};
			   3'h3:	sout_r2 <= #1 {12'h0, sout_r1[15:0], tmp_obuf1[31:28]};
			   3'h4:	sout_r2 <= #1 {12'h0, sout_r1[03:0], tmp_obuf0[31:16]};
			   3'h5:	sout_r2 <= #1 {12'h0, tmp_obuf0[23:04]};
			   3'h6:	sout_r2 <= #1 {12'h0, sout_r1[11:0], tmp_obuf1[31:24]};
			   default:	sout_r2 <= #1 {12'h0, tmp_obuf0[31:12]};
			endcase

		   3'h3:
			case(out_cnt[1:0])						// 24 bit un-packing
			   2'h0:	sout_r2 <= #1 {8'h0, sout_r1[23:0]};
			   2'h1:	sout_r2 <= #1 {8'h0, sout_r1[15:0], tmp_obuf0[31:24]};
			   2'h2:	sout_r2 <= #1 {8'h0, sout_r1[07:0], tmp_obuf0[31:16]};
			   default:	sout_r2 <= #1 {8'h0, tmp_obuf0[31:08]};
			endcase

		  default:	sout_r2 <= #1 sout_r1;					// 32 bit no packing
		endcase
	else	sout_r2 <= #1 sout_r1;	// No packing

// -------------------------------
// Channel muting

always @(posedge clk_i)
	out_mute <= #1 mute_och_i[out_ch_sel_i];

always @(posedge clk_i)
	if(out_mute)	ser_dout_o <= #1 mute_val_i;	// Mute Channel
	else		ser_dout_o <= #1 sout_r2;

endmodule
