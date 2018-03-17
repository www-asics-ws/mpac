/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  FIFO Buffers                                               ////
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

/*
Two dual port (and dual clock) Memories are used: One for OUT FIFOs and one for IN FIFOs

ON = last output buffer number
IN = last input buffer number
max_buf = max fifo depth

Addr		Description
------------	--------------------
{tid,fifo_ctrl}
0,0		Start output buffer 0
0,max_buf	End output Buffer 0
1,0		Start output buffer 1
1,max_buf	End output Buffer 1
...
ON,0		Start output buffer ON
ON,max_buf	End output Buffer ON

0,0		Start input buffer 0
0,max_buf	End input Buffer 0
1,0		Start input buffer 1
1,max_buf	End input Buffer 1
...
IN,0		Start input buffer IN
IN,max_buf	End input Buffer IN
*/

module mpac_buffer #(
parameter integer	NO_IN_CH	= 8,	// Max number of input channels supported by hardware (1-16)
parameter integer	NO_OUT_CH	= 8,	// Max number of output channels supported by hardware (1-16)
parameter integer	MAX_FIFO_DEPTH	= 10	// FIFO (buffer) depth for each channel (as Log2) in DW
) (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// SoC Clock
input	wire		rst_i,		// SoC Reset
input	wire		soft_rst_i,	// SoC Software Reset

	// ===============================================================================
	// AXI Stream Signals.

		// AXI-Stream IN (Codec to SoC) Data Channel Signals
input	wire	[3:0]	axs_in_tid_i,		// Receive Data channel ID
output	reg		axs_in_tvalid_o,	// Transmit data channel valid
input	wire		axs_in_tready_i,	// Transmit data channel ready
output	wire		axs_in_tlast_o,		// Transmit data channel last word
output	wire	[31:0]	axs_in_tdata_o,		// Transmit data channel data

		// AXI-Stream OUT (SoC to Codec) Data Channel Signals
input	wire	[3:0]	axs_out_tid_i,		// Receive Data channel ID
input	wire		axs_out_tvalid_i,	// Receive Data channel valid
output	reg		axs_out_tready_o,	// Receive Data channel ready
input	wire		axs_out_tlast_i,	// Receive Data channel last word
input	wire	[31:0]	axs_out_tdata_i,	// Receive Data channel data

	// ===============================================================================
	// Register File Interface

input	wire	[31:0]			mute_val_i,	// Mute Value, used when buffer is empty
output	wire	[NO_IN_CH-1:0]		in_overflow_o,	// IN Buffer Overflow (Codec to SoC)
output	reg	[(NO_IN_CH*2)-1:0]	in_level_o,	// IN Buffer fill Level (Codec to SoC)
output	reg	[NO_OUT_CH-1:0]		out_underflow_o,// OUT Buffer Underflow (SoC to Codec)
output	reg	[(NO_OUT_CH*2)-1:0]	out_level_o,	// OUT Buffer fill Level (SoC to Codec)

	// ===============================================================================
	// Internal Buffer Interface

input	wire		bclk_i,		// Bit Clock

input	wire	[3:0]	in_sel_i,	// IN FIFO Select  (Codec to SoC)
input	wire		we_i,		// IN FIFO Write Enable  (Codec to SoC)
input	wire	[31:0]	din_i,		// IN FIFO Data input  (Codec to SoC)

input	wire	[3:0]	out_sel_i,	// OUT FIFO Select  (SoC to Codec)
input	wire		re_i,		// OUT FIFO Read Enable  (SoC to Codec)
output	wire	[31:0]	dout_o		// OUT FIFO Data output  (SoC to Codec)
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
localparam integer in_ch_l2 = ilog2(NO_IN_CH);
localparam integer in_mem_depth  = MAX_FIFO_DEPTH + in_ch_l2;
localparam integer out_mem_depth = MAX_FIFO_DEPTH + out_ch_l2;

// ---------------------------------------------------------------------------------------
// Wires and Registers

genvar		n;
integer		i;

wire		reset = (rst_i == `MPAC_ACR);

// ---------------------------------------------------------------------------------------
// IN Buffer (Codec to SoC)

// -------------------------------------------------
// AXI Side (clk_i)

reg		fsm_re;
reg	[2:0]	state, next_state;
wire	[in_mem_depth-1:0]	in_rd_addr;
wire	[(MAX_FIFO_DEPTH*NO_IN_CH)-1:0]	ififo_rd_addr_all;
wire	[NO_IN_CH-1:0]		axi_empty;
wire	[NO_IN_CH-1:0]		axi_re;
reg	[MAX_FIFO_DEPTH-1:0]	ififo_rd_addr_sel;
wire	[(NO_IN_CH*2)-1:0]	in_rd_level;
reg				axs_in_tvalid_d;
reg				axs_in_tready_r;

assign axs_in_tlast_o = 1'b0;
assign in_rd_addr = {axs_in_tid_i[in_ch_l2-1:0], ififo_rd_addr_sel};

always @(posedge clk_i) in_level_o <= #1 in_rd_level;

// Select the FIFO controller address based on the current input channel
// selection (axs_in_tid_i)
always @(ififo_rd_addr_all or axs_in_tid_i)
   begin
	ififo_rd_addr_sel = (axs_in_tid_i==4'h0) ? ififo_rd_addr_all[`RG(MAX_FIFO_DEPTH,0)] : `ZERO(MAX_FIFO_DEPTH) ;
	for(i=1;i<NO_OUT_CH;i=i+1)
		ififo_rd_addr_sel = ififo_rd_addr_sel | ((axs_in_tid_i==i[3:0]) ? ififo_rd_addr_all[`RG(MAX_FIFO_DEPTH,i)] : `ZERO(MAX_FIFO_DEPTH));
   end

// Select the empty signal for the current channel id
always @(axi_empty or axs_in_tid_i)
   begin
	axs_in_tvalid_d = (axs_in_tid_i==4'h0) ? !axi_empty[0] : 1'b0;
	for(i=1;i<NO_OUT_CH;i=i+1)
		axs_in_tvalid_d = axs_in_tvalid_d | ((axs_in_tid_i==i[3:0]) ? !axi_empty[i] : 1'b0);
   end

always @(posedge clk_i)	axs_in_tready_r <= #1 axs_in_tready_i;

// This is a little kludge ... because it takes two cycles to get first valid
// data after read enable is asserted. One cycle for the FIFO controller and
// one cycle for the memory ....
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	state <= #1 3'h0;
	else
	if(soft_rst_i)		state <= #1 3'h0;
	else			state <= #1 next_state;

always @(*)
   begin

	fsm_re = 1'b0;
	axs_in_tvalid_o = 1'b0;
	next_state = state;	// default stay in current state
	case(state)

	   3'h0:
	    begin
		if(axs_in_tvalid_d && axs_in_tready_i)	next_state = 3'h1;
		else
		if(axs_in_tvalid_d)			next_state = 3'h5;
	    end

	   3'h1:
	    begin
		fsm_re = 1'b1;
		next_state = 3'h2;
	    end

	   3'h2:
	    begin
		axs_in_tvalid_o = 1'b1;
		if(!axs_in_tready_i)	next_state = 3'h4;
		else
		if(!axs_in_tvalid_d)	next_state = 3'h4;
		else	fsm_re = 1'b1;
	    end

	   3'h4:
	    begin
		next_state = 3'h0;
	    end

	   3'h5:
	    begin
		if(axs_in_tready_i && axs_in_tready_r)	axs_in_tvalid_o = 1'b1;

		if(axs_in_tready_i && axs_in_tready_r)	fsm_re = 1'b1;

		if(!axs_in_tvalid_d)	next_state = 3'h0;
		else
		if(axs_in_tready_i && axs_in_tready_r)	next_state = 3'h6;
		
	    end

	   3'h6:
	    begin
		fsm_re = 1'b1;
	   	if(axs_in_tvalid_d && axs_in_tready_i)	next_state = 3'h2;
		else
		if(axs_in_tvalid_d)			next_state = 3'h5;
		else					next_state = 3'h0;
	    end

	endcase
   end
	
// -------------------------------------------------
// Codec Side (bclk_i)

wire	[in_mem_depth-1:0]	in_wr_addr;
wire	[(MAX_FIFO_DEPTH*NO_IN_CH)-1:0]	ififo_wr_addr_all;
reg	[MAX_FIFO_DEPTH-1:0]	ififo_wr_addr_sel;
wire	[NO_IN_CH-1:0]		in_we;
wire	[NO_IN_CH-1:0]		in_overflow_d;
reg	[NO_IN_CH-1:0]		in_overflow;
wire	[NO_IN_CH-1:0]		in_full;

always @(posedge bclk_i) in_overflow <= #1 in_overflow_d;

// Synchronize overflow signal to SoC clock
mpac_sync_mb	#( .STAG(2), .WIDTH(NO_IN_CH))
		s1( .clk_i(clk_i), .data_i(in_overflow), .data_o(in_overflow_o));

assign in_wr_addr = {in_sel_i[in_ch_l2-1:0], ififo_wr_addr_sel};

// Select the FIFO controller address based on the current input channel
// selection (in_sel_i)
always @(ififo_wr_addr_all or in_sel_i)
   begin
	ififo_wr_addr_sel = (in_sel_i==4'h0) ? ififo_wr_addr_all[`RG(MAX_FIFO_DEPTH,0)] : `ZERO(MAX_FIFO_DEPTH);
	for(i=1;i<NO_OUT_CH;i=i+1)
		ififo_wr_addr_sel = ififo_wr_addr_sel | ((in_sel_i==i[3:0]) ? ififo_wr_addr_all[`RG(MAX_FIFO_DEPTH,i)] : `ZERO(MAX_FIFO_DEPTH));
   end

// -------------------------------------------------
// Modules

// For efficiency we share one memory block for all IN channels
mpac_dpram  #(
	.DW(			32			),	
	.AW(			in_mem_depth		)	
) in_mem (
	.wr_clk_i(		bclk_i			),	
	.we_i(			|in_we			),	
	.wr_addr_i(		in_wr_addr		),	
	.data_i(		din_i			),	

	.rd_clk_i(		clk_i			),	
	.rd_addr_i(		in_rd_addr		),	
	.data_o(		axs_in_tdata_o		)	
	);

// Instantiate a FIFO Controller for each IN channel
generate
begin:IN_FC
for(n=0;n<NO_IN_CH;n=n+1)
   begin

	// FIFO Controller read enable
	assign axi_re[n] = (axs_in_tid_i==n[3:0]) ? fsm_re : 1'b0;

	// FIFO Controller write enable
	assign in_we[n] = we_i & !in_full[n] & (in_sel_i==n[3:0]);

	//Overflow condition occurs when we try to write to a full FIFO
	assign in_overflow_d[n] = we_i & in_full[n];

	// Out Buffer (SoC to Codec)
	mpac_fifo_dc_ext_mem  #(
		.aw(			MAX_FIFO_DEPTH				)	
	)  in_fifo (
		.reset_i(		~reset					),	
		.clear_i(		soft_rst_i				),	

		.wr_clk_i(		bclk_i					),	
		.we_i(			in_we[n]				),	// wr_clk
		.wr_addr_o(		ififo_wr_addr_all[`RG(MAX_FIFO_DEPTH,n)]),	// wr_clk
		.wr_level_o(							),	// wr_clk
		.full_o(		in_full[n]				),	// wr_clk

		.rd_clk_i(		clk_i					),	
		.re_i(			axi_re[n]				),	// rd_clk
		.rd_addr_o(		ififo_rd_addr_all[`RG(MAX_FIFO_DEPTH,n)]),	// rd_clk
		.rd_level_o(		in_rd_level[`RG(2,n)]			),	// rd_clk
		.empty_o(		axi_empty[n]				)	// rd_clk
		);
   end
end
endgenerate

// ---------------------------------------------------------------------------------------
// Out Buffer (SoC to Codec)

// -------------------------------------------------
// AXI Side (clk_i)

wire	[out_mem_depth-1:0]	out_wr_addr;
wire	[NO_OUT_CH-1:0]		axi_we;
wire	[NO_OUT_CH-1:0]		axi_full;
wire	[(NO_OUT_CH*MAX_FIFO_DEPTH)-1:0]	ofifo_wr_addr_all;
reg	[MAX_FIFO_DEPTH-1:0]	ofifo_wr_addr_sel;
wire	[NO_OUT_CH-1:0]		out_underflow_d;

always @(posedge clk_i) out_underflow_o <= #1 out_underflow_d;

assign out_wr_addr = {axs_out_tid_i[out_ch_l2-1:0], ofifo_wr_addr_sel};

// Select the FIFO controller address based on the current output channel
// selection (axs_out_tid_i)
always @(ofifo_wr_addr_all or axs_out_tid_i)
   begin
	ofifo_wr_addr_sel = (axs_out_tid_i==4'h0) ? ofifo_wr_addr_all[`RG(MAX_FIFO_DEPTH,0)] : `ZERO(MAX_FIFO_DEPTH);
	for(i=1;i<NO_OUT_CH;i=i+1)
		ofifo_wr_addr_sel = ofifo_wr_addr_sel | ((axs_out_tid_i==i[3:0]) ? ofifo_wr_addr_all[`RG(MAX_FIFO_DEPTH,i)] : `ZERO(MAX_FIFO_DEPTH));
   end

always @(axi_full or axs_out_tid_i)
   begin
	axs_out_tready_o = (axs_out_tid_i==4'h0) ? ~axi_full[0] : 1'b0;
	for(i=1;i<NO_OUT_CH;i=i+1)
		axs_out_tready_o = axs_out_tready_o | ((axs_out_tid_i==i[3:0]) ? ~axi_full[i] : 1'b0);
   end

// -------------------------------------------------
// Codec Side (bclk_i)

wire	[out_mem_depth-1:0]	out_rd_addr;
wire	[(NO_OUT_CH*MAX_FIFO_DEPTH)-1:0]	ofifo_rd_addr_all;

wire	[NO_OUT_CH-1:0]		re;
wire	[NO_OUT_CH-1:0]		out_empty;
reg	[MAX_FIFO_DEPTH-1:0]	ofifo_rd_addr_sel;
wire	[(NO_OUT_CH*2)-1:0]	out_rd_level;
wire	[31:0]			dout_d;
reg				out_empty_sel;

assign out_rd_addr = {out_sel_i[out_ch_l2-1:0], ofifo_rd_addr_sel};

always @(posedge clk_i) out_level_o <= #1 out_rd_level;

// Select the FIFO controller read address for current channel (out_sel_i)
always @(ofifo_rd_addr_all or out_sel_i)
   begin
	ofifo_rd_addr_sel = (out_sel_i==4'h0) ? ofifo_rd_addr_all[`RG(MAX_FIFO_DEPTH,0)] : `ZERO(MAX_FIFO_DEPTH);
	for(i=1;i<NO_OUT_CH;i=i+1)
		ofifo_rd_addr_sel = ofifo_rd_addr_sel | ((out_sel_i==i[3:0]) ? ofifo_rd_addr_all[`RG(MAX_FIFO_DEPTH,i)] : `ZERO(MAX_FIFO_DEPTH));
   end

// Select 'empty' for current channel (out_sel_i)
always @(out_empty or out_sel_i)
   begin
	out_empty_sel = (out_sel_i==4'h0) ? out_empty[0] : 1'b0;
	for(i=1;i<NO_OUT_CH;i=i+1)
		out_empty_sel = out_empty_sel | ((out_sel_i==i[3:0]) ? out_empty[i] : 1'b0);
   end

// Return mute value when the FIFO is empty
assign dout_o = out_empty_sel ? mute_val_i : dout_d;

// -------------------------------------------------
// Modules

// For efficiency we share one memory block for all OUT channels
mpac_dpram  #(
	.DW(			32			),	
	.AW(			out_mem_depth		)	
) out_mem (
	.wr_clk_i(		clk_i			),	
	.we_i(			|axi_we			),	
	.wr_addr_i(		out_wr_addr		),	
	.data_i(		axs_out_tdata_i		),	

	.rd_clk_i(		bclk_i			),	
	.rd_addr_i(		out_rd_addr		),	
	.data_o(		dout_d			)	
	);

// Instantiate a FIFO Controller for each OUT channel
generate
begin:OUT_FC
for(n=0;n<NO_OUT_CH;n=n+1)
   begin
	// FIFO controller write enable
	assign axi_we[n] = axs_out_tvalid_i & axs_out_tready_o & (axs_out_tid_i == n[3:0]) & !axi_full[n];

	// FIFO controller read enable
	assign re[n] = re_i & (out_sel_i==n[3:0]) & !out_empty[n];

	// Underflow condition occures when we try to read from an empty FIFO
	assign out_underflow_d[n] = re_i & (out_sel_i==n[3:0]) & out_empty[n];

	// Out Buffer FIFO controllers (SoC to Codec)
	mpac_fifo_dc_ext_mem  #(
		.aw(			MAX_FIFO_DEPTH				)	
	)  out_fifo (
		.reset_i(		~reset					),	
		.clear_i(		soft_rst_i				),	

		.wr_clk_i(		clk_i					),	
		.we_i(			axi_we[n]				),	// wr_clk
		.wr_addr_o(		ofifo_wr_addr_all[`RG(MAX_FIFO_DEPTH,n)]),	// wr_clk
		.wr_level_o(							),	// wr_clk
		.full_o(		axi_full[n]				),	// wr_clk

		.rd_clk_i(		bclk_i					),	
		.re_i(			re[n]					),	// rd_clk
		.rd_addr_o(		ofifo_rd_addr_all[`RG(MAX_FIFO_DEPTH,n)]),	// rd_clk
		.rd_level_o(		out_rd_level[`RG(2,n)]			),	// rd_clk
		.empty_o(		out_empty[n]				)	// rd_clk
		);
   end
end
endgenerate

endmodule
