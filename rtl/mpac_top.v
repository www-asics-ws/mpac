/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Top Level                                                  ////
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

module mpac_top #(
parameter integer	NO_IN_CH	= 8,	// Max number of input channels supported by hardware (1-16)
parameter integer	NO_OUT_CH	= 8,	// Max number of output channels supported by hardware (1-16)
parameter integer	MAX_FIFO_DEPTH	= 10	// FIFO (buffer) depth for each channel (as Log2) in DW
						// e.g. 8 = 2^8 = 256 DW = 1024 bytes
) (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// SoC Clock
input	wire		rst_i,		// SoC Reset

	// ===============================================================================
	// AXIL Slave Interface

output	wire		int_o,		// Interrupt output

		// =======================================================================
		// Write Address Channel Signals.
input	wire	[31:0]	axl_awaddr_i,	// Master Write address.
input	wire		axl_awvalid_i,	// Master Write address valid.
output	wire		axl_awready_o,	// Slave Write address ready.

			// Write Data Channel Signals.
input	wire	[31:0]	axl_wdata_i,	// Master Write data.
input	wire	[3:0]	axl_wstrb_i,	// Master Write strobes.
input	wire		axl_wvalid_i,	// Master Write valid.
output	wire		axl_wready_o,	// Slave Write ready.

			// Write Response Channel Signals.
output	wire	[1:0]	axl_bresp_o,	// Slave Write response.
output	wire		axl_bvalid_o,	// Slave Write response valid.
input	wire		axl_bready_i,	// Master Response ready.

		// =======================================================================
		// Read Address Channel Signals.
input	wire	[31:0]	axl_araddr_i,	// Master Read address.
input	wire		axl_arvalid_i,	// Master Read address valid.
output	wire		axl_arready_o,	// Slave Read address ready.

			// Read Data Channel Signals.
output	wire	[31:0]	axl_rdata_o,	// Slave Read data.
output	wire	[1:0]	axl_rresp_o,	// Slave Read response.
output	wire		axl_rvalid_o,	// Slave Read valid.
input	wire		axl_rready_i,	// Master Read ready.


	// ===============================================================================
	// AXI Stream Interface.

		// AXI-Stream IN (Codec to SoC) Data Channel Signals
input	wire	[3:0]	axs_in_tid_i,		// Receive Data channel ID
output	wire		axs_in_tvalid_o,	// Transmit data channel valid
input	wire		axs_in_tready_i,	// Transmit data channel ready
output	wire		axs_in_tlast_o,		// Transmit data channel last word
output	wire	[31:0]	axs_in_tdata_o,		// Transmit data channel data

		// AXI-Stream OUT (SoC to Codec) Data Channel Signals
input	wire	[3:0]	axs_out_tid_i,		// Receive Data channel ID
input	wire		axs_out_tvalid_i,	// Receive Data channel valid
output	wire		axs_out_tready_o,	// Receive Data channel ready
input	wire		axs_out_tlast_i,	// Receive Data channel last word
input	wire	[31:0]	axs_out_tdata_i,	// Receive Data channel data

	// ===============================================================================
	// I2S Interface.

input	wire		i2s_bclk_i,		// I2S Bit Clock
output	wire		i2s_wc_sync_o,		// I2S Word Clock/Sync
output	wire		i2s_dout_o,		// I2S serial data output
input	wire		i2s_din_i		// I2S serial data input
);

// ---------------------------------------------------------------------------------------
// Local Parameters


// ---------------------------------------------------------------------------------------
// Wires and Registers

wire		soft_rst;		// Software reset
wire		enable;			// Enable IP Core
wire		i2s_rst;		// i2s Reset
wire		i2s_soft_rst;		// i2s Soft Reset

wire	[31:0]	rf_wr_addr;		// RF Write Address
wire	[31:0]	rf_wr_data;		// RF Write Data
wire		rf_we;			// RF Write Enable

wire	[31:0]	rf_rd_addr;		// RF Read Address
wire	[31:0]	rf_rd_data;		// RF Read Data
wire		rf_re;			// RF Read Enable

wire	[NO_IN_CH-1:0]		in_overflow;	// IN Buffer Overflow (Codec to SoC)
wire	[(NO_IN_CH*2)-1:0]	in_level;	// IN Buffer fill Level (Codec to SoC)
wire	[NO_OUT_CH-1:0]		out_underflow;	// OUT Buffer Underflow (SoC to Codec))
wire	[(NO_OUT_CH*2)-1:0]	out_level;	// OUT Buffer fill Level (SoC to Codec

wire	[3:0]	buf_in_sel;		// IN FIFO Select  (Codec to SoC)
wire		buf_we;			// IN FIFO Write Enable  (Codec to SoC)
wire	[31:0]	buf_din;		// IN FIFO Data input  (Codec to SoC)
wire	[3:0]	buf_out_sel;		// OUT FIFO Select  (SoC to Codec)
wire		buf_re;			// OUT FIFO Read Enable  (SoC to Codec)
wire	[31:0]	buf_dout;		// OUT FIFO Data output  (SoC to Codec)

wire	[31:0]	ser_din;		// Input data, right (lsb aligned)
wire	[31:0]	ser_dout;		// Output data, right (lsb aligned)

wire	[3:0]	no_ich;			// Number of input channels
wire	[3:0]	no_och;			// Number of output channels
wire	[31:0]	mute_val;		// Mute Value
wire	[15:0]	mute_och;		// Mute output channel (each bit corresponds to one channel)
wire	[15:0]	mute_ich;		// Mute input channel (each bit corresponds to one channel)
wire	[2:0]	prot_sel;		// Protocol Select
wire	[2:0]	sample_size;		// Sample size from SoC
wire		sample_align;		// Sample alignment
wire		sample_pac;		// Samples are packed
wire	[2:0]	ser_slot_width;		// Serial Slot Width
wire		ser_msb_first;		// Shift out MSB first
wire	[1:0]	sync_start;		// Sync start
wire	[1:0]	sync_dur;		// Sync duration
wire		sync_inv;		// Invert Sync
wire		ser_out_ld;		// Output Serializer load
wire		ser_in_ld;		// Input Serializer Latch Enable

wire		enable_s;		// IP Core enabled
wire		sync_inv_s;		// Invert Sync
wire		sample_align_s;		// Sample alignment
wire		sample_pac_s;		// Samples are packed
wire		ser_msb_first_s;	// Shift out MSB first

wire	[3:0]	no_ich_s;		// Number of input channels
wire	[3:0]	no_och_s;		// Number of output channels
wire	[2:0]	prot_sel_s;		// Protocol Select
wire	[2:0]	sample_size_s;		// Sample size from SoC
wire	[2:0]	ser_slot_width_s;	// Serial Slot Width
wire	[1:0]	sync_start_s;		// Sync start
wire	[1:0]	sync_dur_s;		// Sync duration
wire	[31:0]	mute_val_s;		// Mute Value
wire	[15:0]	mute_och_s;		// Mute output channel (each bit corresponds to one channel)
wire	[15:0]	mute_ich_s;		// Mute input channel (each bit corresponds to one channel)

// ---------------------------------------------------------------------------------------
// Synchronizers

mpac_sync_1b #( .STAG(2) ) s0 (
	.clk_i(		i2s_bclk_i		),
	.data_i(	rst_i			),
	.data_o(	i2s_rst			)
	);

mpac_sync_1b #( .STAG(2) ) s1 (
	.clk_i(		i2s_bclk_i		),
	.data_i(	soft_rst		),
	.data_o(	i2s_soft_rst		)
	);

mpac_sync_1b #( .STAG(2) ) s2 (
	.clk_i(		clk_i			),
	.data_i(	enable			),
	.data_o(	enable_s		)
	);

mpac_sync_1b #( .STAG(2) ) s3 (
	.clk_i(		clk_i			),
	.data_i(	sync_inv		),
	.data_o(	sync_inv_s		)
	);

mpac_sync_1b #( .STAG(2) ) s4 (
	.clk_i(		clk_i			),
	.data_i(	sample_align		),
	.data_o(	sample_align_s		)
	);

mpac_sync_1b #( .STAG(2) ) s5 (
	.clk_i(		clk_i			),
	.data_i(	sample_pac		),
	.data_o(	sample_pac_s		)
	);

mpac_sync_1b #( .STAG(2) ) s6 (
	.clk_i(		clk_i			),
	.data_i(	ser_msb_first		),
	.data_o(	ser_msb_first_s		)
	);


mpac_sync_mb	#( .STAG(2), .WIDTH(4) ) sw1(
	.clk_i(		clk_i			),
	.data_i(	no_ich			),
	.data_o(	no_ich_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(4) ) sw2(
	.clk_i(		clk_i			),
	.data_i(	no_och			),
	.data_o(	no_och_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(3) ) sw3(
	.clk_i(		clk_i			),
	.data_i(	prot_sel		),
	.data_o(	prot_sel_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(3) ) sw4(
	.clk_i(		clk_i			),
	.data_i(	sample_size		),
	.data_o(	sample_size_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(3) ) sw5(
	.clk_i(		clk_i			),
	.data_i(	ser_slot_width		),
	.data_o(	ser_slot_width_s	)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(2) ) sw6(
	.clk_i(		clk_i			),
	.data_i(	sync_start		),
	.data_o(	sync_start_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(2) ) sw7(
	.clk_i(		clk_i			),
	.data_i(	sync_dur		),
	.data_o(	sync_dur_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(32) ) sw8(
	.clk_i(		clk_i			),
	.data_i(	mute_val		),
	.data_o(	mute_val_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(16) ) sw9(
	.clk_i(		clk_i			),
	.data_i(	mute_och		),
	.data_o(	mute_och_s		)
	);

mpac_sync_mb	#( .STAG(2), .WIDTH(16) ) sw10(
	.clk_i(		clk_i			),
	.data_i(	mute_ich		),
	.data_o(	mute_ich_s		)
	);


// ---------------------------------------------------------------------------------------
// Module Instances

mpac_axl_if axi_if (

	.clk_i(			clk_i			),	
	.rst_i(			rst_i			),	

		// ===============================================================================
		// Write Address Channel Signals.

	.axl_awaddr_i(		axl_awaddr_i		),	// Master Write address.
	.axl_awvalid_i(		axl_awvalid_i		),	// Master Write address valid.
	.axl_awready_o(		axl_awready_o		),	// Slave Write address ready.

	.axl_wdata_i(		axl_wdata_i		),	// Master Write data.
	.axl_wstrb_i(		axl_wstrb_i		),	// Master Write strobes.
	.axl_wvalid_i(		axl_wvalid_i		),	// Master Write valid.
	.axl_wready_o(		axl_wready_o		),	// Slave Write ready.

	.axl_bresp_o(		axl_bresp_o		),	// Slave Write response.
	.axl_bvalid_o(		axl_bvalid_o		),	// Slave Write response valid.
	.axl_bready_i(		axl_bready_i		),	// Master Response ready.

		// ===============================================================================
		// Read Address Channel Signals.

	.axl_araddr_i(		axl_araddr_i		),	// Master Read address.
	.axl_arvalid_i(		axl_arvalid_i		),	// Master Read address valid.
	.axl_arready_o(		axl_arready_o		),	// Slave Read address ready.

	.axl_rdata_o(		axl_rdata_o		),	// Slave Read data.
	.axl_rresp_o(		axl_rresp_o		),	// Slave Read response.
	.axl_rvalid_o(		axl_rvalid_o		),	// Slave Read valid.
	.axl_rready_i(		axl_rready_i		),	// Master Read ready.

		// ===============================================================================
		// RF Write Interface

	.rf_wr_addr_o(		rf_wr_addr		),	// RF Write Address
	.rf_wr_data_o(		rf_wr_data		),	// RF Write Data
	.rf_wr_be_o(					),	// RF Write Byte Enable
	.rf_we_o(		rf_we			),	// RF Write Enable
	.rf_wr_wait_i(		1'b0			),	// RF Write Wait

		// ===============================================================================
		// RF Read Interface

	.rf_rd_addr_o(		rf_rd_addr		),	// RF Read Address
	.rf_rd_data_i(		rf_rd_data		),	// RF Read Data
	.rf_re_o(		rf_re			),	// RF Read Enable
	.rf_rd_wait_i(		1'b0			)	// RF Read Wait
	);


mpac_rf  #(
	.NO_IN_CH(		NO_IN_CH		),	// Max number of input channels supported by hardware (1-16
	.NO_OUT_CH(		NO_OUT_CH		),	// Max number of output channels supported by hardware (1-16
	.MAX_FIFO_DEPTH(	MAX_FIFO_DEPTH		)	// FIFO (buffer depth for each channel (as Log2 in DW
) rf (
			// ===============================================================================
			// General Signals

	.clk_i(			clk_i			),	// SoC Clock
	.rst_i(			rst_i			),	// SoC Reset

			// ===============================================================================
			// Register File Interface

	.rf_wr_addr_i(		rf_wr_addr		),	
	.rf_wr_data_i(		rf_wr_data		),	
	.rf_we_i(		rf_we			),	

	.rf_rd_addr_i(		rf_rd_addr		),	
	.rf_rd_data_o(		rf_rd_data		),	
	.rf_re_i(		rf_re			),	

	.int_o(			int_o			),	

			// ===============================================================================
			// Internal Interface

	.soft_rst_o(		soft_rst		),	// Software Reset
	.enable_o(		enable			),	// Enable IP Core

	.no_ich_o(		no_ich			),	// Number of input channels
	.no_och_o(		no_och			),	// Number of output channels
	.mute_val_o(		mute_val		),	// Mute Value
	.mute_och_o(		mute_och		),	// Mute output channel (each bit corresponds to one channel)
	.mute_ich_o(		mute_ich		),	// Mute input channel (each bit corresponds to one channel)
	.prot_sel_o(		prot_sel		),	// Protocol Select
	.sample_size_o(		sample_size		),	// Sample size from SoC
	.sample_align_o(	sample_align		),	// Sample alignment
	.sample_pac_o(		sample_pac		),	// Samples are packed
	.ser_slot_width_o(	ser_slot_width		),	// Serial Slot Width
	.ser_msb_first_o(	ser_msb_first		),	// Shift out MSB first
	.sync_start_o(		sync_start		),	// Sync start
	.sync_dur_o(		sync_dur		),	// Sync duration
	.sync_inv_o(		sync_inv		),	// Invert Sync

	.in_overflow_i(		in_overflow		),	// IN Buffer Overflow (Codec to SoC)
	.in_level_i(		in_level		),	// IN Buffer fill Level (Codec to SoC)
	.out_underflow_i(	out_underflow		),	// OUT Buffer Underflow (SoC to Codec)
	.out_level_i(		out_level		)	// OUT Buffer fill Level (SoC to Codec)
	);


mpac_pctrl  #(
	.NO_IN_CH(		NO_IN_CH		),	// Max number of input channels supported by hardware (1-16
	.NO_OUT_CH(		NO_OUT_CH		),	// Max number of output channels supported by hardware (1-16
	.MAX_FIFO_DEPTH(	MAX_FIFO_DEPTH		)	// FIFO (buffer depth for each channel (as Log2 in DW
)  pctrl (
			// ===============================================================================
			// General Signals

	.clk_i(			i2s_bclk_i		),	// i2s Clock
	.rst_i(			i2s_rst			),	// i2s Reset
	.soft_rst_i(		i2s_soft_rst		),	// i2s Software Reset

			// ===============================================================================
			// Internal Interface

	.ser_out_ld_o(		ser_out_ld		),	// Output Serializer load
	.out_ch_sel_o(		buf_out_sel		),	// Output channel select

	.ser_in_ld_o(		ser_in_ld		),	// Input Serializer load
	.in_ch_sel_o(		buf_in_sel		),	// Input channel select

			// ===============================================================================
			// Register File Interface

	.enable_i(		enable_s		),	// IP Core enabled
	.no_ich_i(		no_ich_s		),	// Number of input channels
	.no_och_i(		no_och_s		),	// Number of output channels

	.prot_sel_i(		prot_sel_s		),	// Protocol Select
	.sample_size_i(		sample_size_s		),	// Sample size from SoC
	.ser_slot_width_i(	ser_slot_width_s	),	// Serial Slot Width
	.sync_start_i(		sync_start_s		),	// Sync start
	.sync_dur_i(		sync_dur_s		),	// Sync duration
	.sync_inv_i(		sync_inv_s		),	// Invert Sync

			// ===============================================================================
			// i2s Interface

	.i2s_wc_sync_o(		i2s_wc_sync_o		)	// I2S Word Clock/Sync
	);


mpac_buffer  #(
	.NO_IN_CH(		NO_IN_CH		),	// Max number of input channels supported by hardware (1-16
	.NO_OUT_CH(		NO_OUT_CH		),	// Max number of output channels supported by hardware (1-16
	.MAX_FIFO_DEPTH(	MAX_FIFO_DEPTH		)	// FIFO (buffer depth for each channel (as Log2 in DW
) buf_i (
		// ===============================================================================
		// General Signals

	.clk_i(			clk_i			),	// SoC Clock
	.rst_i(			rst_i			),	// SoC Reset
	.soft_rst_i(		soft_rst		),	// SoC Soft Reset

		// ===============================================================================
		// AXI Stream Signals.

				// AXI-Stream IN (Codec to SoC) Data Channel Signals
	.axs_in_tid_i(		axs_in_tid_i		),	// Receive Data channel ID
	.axs_in_tvalid_o(	axs_in_tvalid_o		),	// Transmit data channel valid
	.axs_in_tready_i(	axs_in_tready_i		),	// Transmit data channel ready
	.axs_in_tlast_o(	axs_in_tlast_o		),	// Transmit data channel last word
	.axs_in_tdata_o(	axs_in_tdata_o		),	// Transmit data channel data

				// AXI-Stream OUT (SoC to Codec) Data Channel Signals
	.axs_out_tid_i(		axs_out_tid_i		),	// Receive Data channel ID
	.axs_out_tvalid_i(	axs_out_tvalid_i	),	// Receive Data channel valid
	.axs_out_tready_o(	axs_out_tready_o	),	// Receive Data channel ready
	.axs_out_tlast_i(	axs_out_tlast_i		),	// Receive Data channel last word
	.axs_out_tdata_i(	axs_out_tdata_i		),	// Receive Data channel data

		// ===============================================================================
		// Register File Interface

	.mute_val_i(		mute_val_s		),	// Mute Value,  used when buffer is empty
	.in_overflow_o(		in_overflow		),	// IN Buffer Overflow (Codec to SoC)
	.in_level_o(		in_level		),	// IN Buffer fill Level (Codec to SoC)
	.out_underflow_o(	out_underflow		),	// OUT Buffer Underflow (SoC to Codec)
	.out_level_o(		out_level		),	// OUT Buffer fill Level (SoC to Codec)

		// ===============================================================================
		// Internal Buffer Interface

	.bclk_i(		i2s_bclk_i		),	// Bit Clock

	.in_sel_i(		buf_in_sel		),	// IN FIFO Select  (Codec to SoC)
	.we_i(			buf_we			),	// IN FIFO Write Enable  (Codec to SoC)
	.din_i(			buf_din			),	// IN FIFO Data input  (Codec to SoC)

	.out_sel_i(		buf_out_sel		),	// OUT FIFO Select  (SoC to Codec)
	.re_i(			buf_re			),	// OUT FIFO Read Enable  (SoC to Codec)
	.dout_o(		buf_dout		)	// OUT FIFO Data output  (SoC to Codec)
	);


mpac_data_mux #(
	.NO_IN_CH(		NO_IN_CH		),	// Max number of input channels supported by hardware (1-16
	.NO_OUT_CH(		NO_OUT_CH		)	// Max number of output channels supported by hardware (1-16
) dmux (
			// ===============================================================================
			// General Signals

	.clk_i(			i2s_bclk_i		),	// i2s Clock
	.rst_i(			i2s_rst			),	// i2s Reset
	.soft_rst_i(		i2s_soft_rst		),	// i2s Soft Reset

			// ===============================================================================
			// Register File Interface

	.enable_i(		enable_s		),	// Enable IP Core
	.no_ich_i(		no_ich_s		),	// Number of input channels
	.no_och_i(		no_och_s		),	// Number of output channels
	.mute_val_i(		mute_val_s		),	// Mute Value
	.mute_och_i(		mute_och_s		),	// Mute output channel (each bit corresponds to one channel)
	.mute_ich_i(		mute_ich_s		),	// Mute input channel (each bit corresponds to one channel)
	.sample_size_i(		sample_size_s		),	// Sample size from SoC
	.sample_align_i(	sample_align_s		),	// Sample alignment
	.sample_pac_i(		sample_pac_s		),	// Samples are packed

			// ===============================================================================
			// Internal Buffer Interface

	.ser_in_le_i(		ser_in_ld		),	// Input Latch Enable (dout le)
	.buf_in_we_o(		buf_we			),	// Input buffer write enable
	.in_ch_sel_i(		buf_in_sel		),	// Input channel select (Codec to SoC)

	.ser_din_i(		ser_din			),	// Serial Data input  (Codec to SoC)
	.buf_dout_o(		buf_din			),	// Buffer Data output  (Codec to SoC)


	.ser_out_ld_i(		ser_out_ld		),	// Output Serializer load
	.buf_out_rd_o(		buf_re			),	// Output buffer read
	.out_ch_sel_i(		buf_out_sel		),	// Output channel select

	.buf_din_i(		buf_dout		),	// Buffer Data input  (SoC to Codec)
	.ser_dout_o(		ser_dout		)	// Serial Data output  (SoC to Codec)
	);

mpac_ser_io ser_io (
		// ===============================================================================
		// General Signals

	.clk_i(			i2s_bclk_i		),	// bit Clock

		// ===============================================================================
		// Register File Interface

	.ser_slot_width_i(	ser_slot_width_s	),	// Serial Slot Width
	.msb_first_i(		ser_msb_first_s		),	// Shift out MSB first

		// ===============================================================================
		// Internal Interface

	.din_i(			ser_dout		),	// Input data, right (lsb aligned)
	.dout_o(		ser_din			),	// Output data, right (lsb aligned)

	.out_le_i(		ser_out_ld		),	// Output Latch Enable (din le)
	.in_le_i(		ser_in_ld		),	// Input Latch Enable (dout le)

		// ===============================================================================
		// I2S Signals.

	.i2s_dout_o(		i2s_dout_o		),	// I2S serial data output
	.i2s_din_i(		i2s_din_i		)	// I2S serial data input
	);

endmodule

