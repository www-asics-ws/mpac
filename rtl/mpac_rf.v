/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Register file                                              ////
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

module mpac_rf #(
parameter integer	NO_IN_CH	= 8,	// Max number of input channels supported by hardware (1-16)
parameter integer	NO_OUT_CH	= 8,	// Max number of output channels supported by hardware (1-16)
parameter integer	MAX_FIFO_DEPTH	= 10	// FIFO (buffer) depth for each channel (as Log2) in DW
) (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// SoC Clock
input	wire		rst_i,		// SoC Reset

	// ===============================================================================
	// Register File Interface

input	wire	[31:0]	rf_wr_addr_i,
input	wire	[31:0]	rf_wr_data_i,
input	wire		rf_we_i,

input	wire	[31:0]	rf_rd_addr_i,
output	wire	[31:0]	rf_rd_data_o,
input	wire		rf_re_i,

output	reg		int_o,

	// ===============================================================================
	// Internal Interface

output	reg		soft_rst_o,	// Software Reset
output	wire		enable_o,	// Enable IP Core

output	wire	[3:0]	no_ich_o,	// Number of input channels
					// Must be less than or equal to NO_IN_CH
output	wire	[3:0]	no_och_o,	// Number of output channels
					// Must be less than or equal to NO_OUT_CH
output	reg	[31:0]	mute_val_o,	// Mute Value
output	wire	[15:0]	mute_och_o,	// Mute output channel (each bit corresponds to one channel)
output	wire	[15:0]	mute_ich_o,	// Mute input channel (each bit corresponds to one channel)

output	wire	[2:0]	prot_sel_o,	// Protocol Select
					// 0 - I2S (2 channel only)
					// 1 - TDM
					// 2 - AC97
					// 3-7 RESERVED

output	wire	[2:0]	sample_size_o,	// Sample size from SoC
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED

output	wire		sample_align_o,	// Sample alignment
					// 0 - right (lsb)
					// 1 - left (msb)

output	wire		sample_pac_o,	// Samples are packed
					// 0 - no
					// 1 - yes

output	wire	[2:0]	ser_slot_width_o, // Serial Slot Width
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED

output	wire		ser_msb_first_o,// Shift out MSB first

output	wire	[1:0]	sync_start_o,	// Sync start
					// 0 - sync with serial data
					// 1 - one clock before serial data
					// 2-3 RESERVED

output	wire	[1:0]	sync_dur_o,	// Sync duration
					// 0 - One cycle active at the beginning
					// 1 - 50% duty cycle
					// 2 - One cycle in-active at the end
					// 3 RESERVED

output wire		sync_inv_o,	// Invert Sync
					// 0 - not inverted (first high than low)
					// 1 - inverted (first low than high)

input	wire	[NO_IN_CH-1:0]		in_overflow_i,	// IN Buffer Overflow (Codec to SoC)
input	wire	[NO_OUT_CH-1:0]		out_underflow_i,// OUT Buffer Underflow (SoC to Codec)
input	wire	[(NO_IN_CH*2)-1:0]	in_level_i,	// IN Buffer fill Level (Codec to SoC)
input	wire	[(NO_OUT_CH*2)-1:0]	out_level_i	// OUT Buffer fill Level (SoC to Codec)

);


// ---------------------------------------------------------------------------------------
// Local Parameters


// ---------------------------------------------------------------------------------------
// Wires and Registers

genvar		n;
integer		i;

wire	[31:0]	version;
wire	[31:0]	csr;
reg	[30:0]	csr_r;
reg	[2:0]	soft_reset_r;
reg	[31:0]	mute_ch_r;
reg	[31:0]	cfg1_r;
wire	[31:0]	ib_stat;
wire	[31:0]	ob_stat;
reg	[31:0]	buf_ovfl;

wire		bo_int, bu_int;
wire	[7:0]	isrc_set;
reg	[7:0]	int_src_r;
wire	[31:0]	int_src;
reg	[7:0]	int_clr_delay;
reg	[31:0]	bo_int_en;
reg	[31:0]	int_mask;

reg	[31:0]	t_data;
reg		timer_int;

// ---------------------------------------------------------------------------------------
// Alises

assign no_och_o = csr_r[15:12];
assign no_ich_o = csr_r[11:8];
assign enable_o	= csr_r[0];

assign csr = {1'b0, csr_r};

assign mute_ich_o = mute_ch_r[31:16];
assign mute_och_o = mute_ch_r[15:0];

// ------------ Configuration Register ------------
// cfg1_r[31:27]				// 31:27  RESERVED
assign prot_sel_o	= cfg1_r[26:24];	// 26:24  Protocol Select
// cfg1_r[23:21]				// 23:21  RESERVED
assign sample_size_o	= cfg1_r[20:18];	// 20:18  Sample size from SoC
assign sample_align_o	= cfg1_r[17];		// 17	  Sample alignment
assign sample_pac_o	= cfg1_r[16];		// 16	  Samples are packed
// cfg1_r[15:11]				// 15:12  RESERVED
assign ser_msb_first_o  = cfg1_r[11];		// 11     Shift out MSB first
assign ser_slot_width_o	= cfg1_r[10:8];		// 10:8	  Serial Slot Width
// cfg1_r[7:5]					// 7:5    RESERVED
assign sync_start_o	= cfg1_r[4:3];		// 4:3	  Sync start
assign sync_dur_o	= cfg1_r[2:1];		// 2:1	  Sync duration
assign sync_inv_o	= cfg1_r[0];		// 0	  Invert Sync

// Buffer Status
assign ib_stat =  {{16-NO_IN_CH{2'b0}}, in_level_i};
assign ob_stat =  {{16-NO_OUT_CH{2'b0}}, out_level_i};

// ---------------------------------------------------------------------------------------
// Version and configuration

`ifdef MPAC_HAVE_TIMER
wire	have_timer = 1'b1;
`else
wire	have_timer = 1'b0;
`endif

wire	[3:0]	no_in_ch_hw = NO_IN_CH-1;	// Max number of input channels supported by hardware (1-16)
wire	[3:0]	no_out_ch_hw = NO_OUT_CH-1;	// Max number of output channels supported by hardware (1-16)
wire	[3:0]	max_fifo_dept_hw = NO_OUT_CH;	// FIFO (buffer) depth for each channel (as Log2) in DW

assign version = {
		7'h0,			// 31:25  RESERVED
		have_timer,		// 24     Indicated whether the timer module is present or not
		no_out_ch_hw,		// 23:20  Number of max input channels supported bu HW
		no_in_ch_hw,		// 19:16  Number of max input channels supported bu HW
		max_fifo_dept_hw,	// 15:12  FIFO Size (log 2)
		`VER			// 11:0   IP Core Version
		};

// ---------------------------------------------------------------------------------------
// Register Read


// Read Data Mux
assign rf_rd_data_o =	((rf_rd_addr_i[5:2] == 4'h00) ? csr               : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h01) ? version           : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h02) ? mute_ch_r         : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h03) ? mute_val_o        : 32'h00) |

			((rf_rd_addr_i[5:2] == 4'h04) ? cfg1_r            : 32'h00) |

			((rf_rd_addr_i[5:2] == 4'h06) ? ib_stat           : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h07) ? ob_stat           : 32'h00) |

			((rf_rd_addr_i[5:2] == 4'h08) ? buf_ovfl          : 32'h00) |

			((rf_rd_addr_i[5:2] == 4'h09) ? bo_int_en         : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h0a) ? int_mask          : 32'h00) |
			((rf_rd_addr_i[5:2] == 4'h0b) ? int_src           : 32'h00) |


			((rf_rd_addr_i[5:2] == 4'h0d) ? t_data            : 32'h00) |
			32'h00;


// ---------------------------------------------------------------------------------------
// Register Write

// -------------------------------------------------
// Write Enables

wire	csr_we		= rf_we_i && (rf_wr_addr_i[5:2] == 4'h00);

wire	mute_ch_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h02);
wire	mute_val_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h03);

wire	cfg1_we		= rf_we_i && (rf_wr_addr_i[5:2] == 4'h04);

wire	b_ovfl_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h08);

wire	bo_int_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h09);
wire	int_mask_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h0a);
wire	isrc_we		= rf_we_i && (rf_wr_addr_i[5:2] == 4'h0b);

wire	timer_we	= rf_we_i && (rf_wr_addr_i[5:2] == 4'h0d);

// -------------------------------------------------
// Actual registers

// CSR register
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	csr_r <= #1 31'h0;
	else
	if(csr_we)		csr_r <= #1 rf_wr_data_i[30:0];

// Soft Reset
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)		soft_rst_o <= #1 1'b0;
	else
	if(csr_we && rf_wr_data_i[31])	soft_rst_o <= #1 1'b1;
	else
	if(&soft_reset_r)		soft_rst_o <= #1 1'b0;

// Make sure soft reset is asserted for 8 clock cycles
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	soft_reset_r <= #1 3'h0;
	else
	if(!soft_rst_o)		soft_reset_r <= #1 3'h0;
	else			soft_reset_r <= #1 soft_reset_r + 3'h1;

// Mute channel register
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	mute_ch_r <= #1 32'h0;
	else
	if(mute_ch_we)		mute_ch_r <= #1 rf_wr_data_i[31:0];

// Mute Value register
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	mute_val_o <= #1 32'h0;
	else
	if(mute_val_we)		mute_val_o <= #1 rf_wr_data_i[31:0];

// Configuration register
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	cfg1_r <= #1 32'h0;
	else
	if(cfg1_we)		cfg1_r <= #1 rf_wr_data_i[31:0];

// Buffer overflow/undeflow register
generate
begin:OUT_OVERFLOW
for(n=0;n<16;n=n+1)
   begin
	if(n>NO_OUT_CH-1)
		always @(posedge clk_i)		buf_ovfl[n] <= #1 1'b0;
	else
		always @(posedge clk_i `MPAC_ACRT)
			if(rst_i == `MPAC_ACR)			buf_ovfl[n] <= #1 1'b0;
			else
			if(b_ovfl_we && rf_wr_data_i[n])	buf_ovfl[n] <= #1 1'b0;
			else
			if(out_underflow_i[n])			buf_ovfl[n] <= #1 1'b0;
   end
end
endgenerate

generate
begin:IN_OVERFLOW
for(n=0;n<16;n=n+1)
   begin
	if(n>NO_IN_CH-1)
		always @(posedge clk_i)		buf_ovfl[16+n] <= #1 1'b0;
	else
		always @(posedge clk_i `MPAC_ACRT)
			if(rst_i == `MPAC_ACR)			buf_ovfl[16+n] <= #1 1'b0;
			else
			if(b_ovfl_we && rf_wr_data_i[16+n])	buf_ovfl[16+n] <= #1 1'b0;
			else
			if(in_overflow_i[n])			buf_ovfl[16+n] <= #1 1'b0;
   end
end
endgenerate

// ---------------------------------------------------------------------------------------
// Interrupts

// Buffer over/under flow interrupt enable
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	bo_int_en <= #1 32'h0;
	else
	if(bo_int_we)		bo_int_en <= #1 rf_wr_data_i[31:0];

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	int_mask <= #1 32'h0;
	else
	if(int_mask_we)		int_mask <= #1 rf_wr_data_i[31:0];

assign bo_int = |(bo_int_en[31:16] & buf_ovfl[31:16]);
assign bu_int = |(bo_int_en[15:0]  & buf_ovfl[15:0]);

assign isrc_set = {
		7'h0,
		timer_int
		};

generate
begin:ISRC
for(n=0;n<8;n=n+1)
   begin
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)		int_src_r[n] <= #1 1'b0;
	else
	if(soft_rst_o)			int_src_r[n] <= #1 1'b0;
	else
	if(isrc_set[n])			int_src_r[n] <= #1 1'b1;
	else
	if(isrc_we && rf_wr_data_i[n])	int_src_r[n] <= #1 1'b0;
   end
end
endgenerate

assign int_src = {
		16'h0,		// 31:16
		6'h0,		// 15:10
		bo_int,		// 9
		bu_int,		// 8
		int_src_r	// 7:0
		};

// Interrupt clear delay guarantees that the interrupt line goes low every
// time we clear an interrupt, without loosing interrupts
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)		int_clr_delay <= #1 8'h0;
	else				int_clr_delay <= {int_clr_delay[6:0], isrc_we & |rf_wr_data_i};

// Interrupt Output
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	int_o <= #1 1'b0;
	else			int_o <= #1 (|(int_src & int_mask)) & !(|int_clr_delay);

// ---------------------------------------------------------------------------------------
// Simple Timer Module

`ifdef MPAC_HAVE_TIMER
reg	[34:0]	timer_r;
reg	[31:0]	t_ctrl;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	timer_r <= #1 35'h0;
	else
	if(timer_we)		timer_r <= #1 35'h0;
	else			timer_r <= #1 timer_r + 35'h1;

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)	t_ctrl <= #1 32'h0;
	else
	if(timer_we)		t_ctrl <= #1 rf_wr_data_i;
	else
	if(timer_int)		t_ctrl[31] <= #1 1'b0;

always @(*)
	case(t_ctrl[29:28])
	   2'h0: t_data = timer_r[31:0];
	   2'h1: t_data = timer_r[32:1];
	   2'h2: t_data = timer_r[33:2];
	   2'h3: t_data = timer_r[34:3];
	endcase

always @(posedge clk_i)	timer_int <= #1 t_ctrl[31] & (t_data[27:0] == t_ctrl[27:0]);
`else
always @(posedge clk_i) t_data <= #1 32'h0;
always @(posedge clk_i) timer_int <= #1 1'h0;
`endif

endmodule
