/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Top Level Test Bench                                       ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
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



module test;

///////////////////////////////////////////////////////////////////
//
// Local IOs and Vars
//

reg		clock;
reg		reset;
wire		int;			// interrupt request output

wire	[31:0]	axl_awaddr;		// Master Write address.
wire		axl_awvalid;		// Master Write address valid.
wire		axl_awready;		// Slave Write address ready.
wire	[31:0]	axl_wdata;		// Master Write data.
wire	[3:0]	axl_wstrb;		// Master Write strobes.
wire		axl_wvalid;		// Master Write valid.
wire		axl_wready;		// Slave Write ready.
wire	[1:0]	axl_bresp;		// Slave Write response.
wire		axl_bvalid;		// Slave Write response valid.

wire		axl_bready;		// Master Response ready.
wire	[31:0]	axl_araddr;		// Master Read address.
wire		axl_arvalid;		// Master Read address valid.
wire		axl_arready;		// Slave Read address ready.

wire	[31:0]	axl_rdata;		// Slave Read data.
wire	[1:0]	axl_rresp;		// Slave Read response.
wire		axl_rvalid;		// Slave Read valid.
wire		axl_rready;		// Master Read ready.

wire	[3:0]	axs_in_tid;		// Transmit Data channel ID
wire		axs_in_tvalid;		// Transmit data channel valid
wire		axs_in_tready;		// Transmit data channel ready
wire		axs_in_tlast;		// Transmit data channel last word
wire	[31:0]	axs_in_tdata;		// Transmit data channel data

wire	[3:0]	axs_out_tid;		// Receive Data channel ID
wire		axs_out_tvalid;		// Receive Data channel valid
wire		axs_out_tready;		// Receive Data channel ready
wire		axs_out_tlast;		// Receive Data channel last word
wire	[31:0]	axs_out_tdata;		// Receive Data channel data

reg		i2s_bclk;		// I2S Bit Clock
wire		i2s_wc_sync;		// I2S Word Clock/Sync
wire		i2s_dout;		// I2S serial data output
wire		i2s_din;		// I2S serial data input

///////////////////////////////////////////////////////////////////
//
// Test Definitions
//

`define	CSR		32'h0000
`define	VERS		32'h0004
`define	MUTE		32'h0008
`define	MVAL		32'h000c
`define	CFG		32'h0010

`define	IB_STAT		32'h0018
`define	OB_STAT		32'h001c
`define	BF_OVUN		32'h0020
`define	BO_INT_EN	32'h0024
`define	INT_MASK	32'h0028
`define	INT_SRC		32'h002c

`define	TIMER		32'h0034

///////////////////////////////////////////////////////////////////
//
// Misc test Development vars
//

real		SYS_CLK_PER, BCLK_PER;
integer		error_count;
reg	[31:0]	tmp;

///////////////////////////////////////////////////////////////////
//
// Initial Startup and Simulation Begin
//

initial
   begin
	SYS_CLK_PER = 5;
	BCLK_PER = 20;

	$timeformat (-9, 1, " ns", 12);
	$display("\n\n");
	$display("****************************************************");
	$display("***     MPAC IP Core Test Bench Started          ***");
	$display("****************************************************");

`ifdef WAVES
  	$shm_open("waves");
	$shm_probe("AS",test,"AS");
	$display("INFO: Signal dump enabled ...");
`endif

   	repeat(5)	@(posedge clock);
	error_count = 0;
   	reset = `MPAC_ACR;
   	repeat(10)	@(posedge clock);
	#4;
	reset = !reset;
   	repeat(10)	@(posedge clock);

	$display("\n\n");
	$display("+++ SoC clock period: %f, I2S clock period: %f", SYS_CLK_PER, BCLK_PER);
	$display("\n\n");
	mpac_read(`VERS, tmp);
	$display("IP HW Version: %x", tmp & 32'h000fff);
	$display("HW FIFO Depth: %0d", (tmp>>12) & 32'h00f);
	$display("HW NO ICH: %0d", (tmp>>16) & 32'h00f);
	$display("HW NO OCH: %0d", (tmp>>20) & 32'h00f);
	$display("Have Timer: %0d", (tmp>>24) & 32'h001);

   	repeat(10)	@(posedge clock);

	if(1)			// Regression Run
	   begin
		test1;
		test_pack_16;
		test_pack_18;
		test_pack_20;
		test_pack_24;
	   end
	else
	if(1)			// Test Debug Area
	   begin
		test1;
		//test_pack_16;
		//test_pack_18;
		//test_pack_20;
		//test_pack_24;
	   end

   	repeat(2000)	@(posedge clock);

	$display("\n\n");
	$display("****************************************************");
	$display("***     Test Bench Finished. Total ERRORS: %0d", error_count);
	$display("****************************************************");
	$display("\n\n");

   	$finish;
   end


///////////////////////////////////////////////////////////////////
//
// Watchdog Timer
//



///////////////////////////////////////////////////////////////////
//
// Clock generation
//

initial
   begin
	#SYS_CLK_PER;
	clock = 0;
	while(1)	#(SYS_CLK_PER) clock = !clock;
   end

initial
   begin
	#BCLK_PER;
	i2s_bclk = 0;
	while(1)	#(BCLK_PER) i2s_bclk = !i2s_bclk;
   end

///////////////////////////////////////////////////////////////////
//
// Module Instantiations
//

mpac_top  #(
	.NO_IN_CH(		16			),	// Max number of input channels supported by hardware (0-15)
	.NO_OUT_CH(		16			),	// Max number of output channels supported by hardware (0-15)
	.MAX_FIFO_DEPTH(	9			)	// FIFO (buffer depth for each channel (as Log2)
) mpac (
			// ===============================================================================
			// General Signals

	.clk_i(			clock			),	// SoC Clock
	.rst_i(			reset			),	// SoC Reset

			// ===============================================================================
			// AXIL Slave Interface

	.int_o(			int			),	

				// =======================================================================
				// Write Address Channel Signals.
	.axl_awaddr_i(		axl_awaddr		),	// Master Write address.
	.axl_awvalid_i(		axl_awvalid		),	// Master Write address valid.
	.axl_awready_o(		axl_awready		),	// Slave Write address ready.

					// Write Data Channel Signals.
	.axl_wdata_i(		axl_wdata		),	// Master Write data.
	.axl_wstrb_i(		axl_wstrb		),	// Master Write strobes.
	.axl_wvalid_i(		axl_wvalid		),	// Master Write valid.
	.axl_wready_o(		axl_wready		),	// Slave Write ready.

					// Write Response Channel Signals.
	.axl_bresp_o(		axl_bresp		),	// Slave Write response.
	.axl_bvalid_o(		axl_bvalid		),	// Slave Write response valid.
	.axl_bready_i(		axl_bready		),	// Master Response ready.

				// =======================================================================
				// Read Address Channel Signals.
	.axl_araddr_i(		axl_araddr		),	// Master Read address.
	.axl_arvalid_i(		axl_arvalid		),	// Master Read address valid.
	.axl_arready_o(		axl_arready		),	// Slave Read address ready.

					// Read Data Channel Signals.
	.axl_rdata_o(		axl_rdata		),	// Slave Read data.
	.axl_rresp_o(		axl_rresp		),	// Slave Read response.
	.axl_rvalid_o(		axl_rvalid		),	// Slave Read valid.
	.axl_rready_i(		axl_rready		),	// Master Read ready.

			// ===============================================================================
			// AXI Stream Signals.

				// AXI-Stream IN (Codec to SoC) Data Channel Signals
	.axs_in_tid_i(		axs_in_tid		),	// Receive Data channel ID
	.axs_in_tvalid_o(	axs_in_tvalid		),	// Transmit data channel valid
	.axs_in_tready_i(	axs_in_tready		),	// Transmit data channel ready
	.axs_in_tlast_o(	axs_in_tlast		),	// Transmit data channel last word
	.axs_in_tdata_o(	axs_in_tdata		),	// Transmit data channel data

				// AXI-Stream OUT (SoC to Codec) Data Channel Signals
	.axs_out_tid_i(		axs_out_tid		),	// Receive Data channel ID
	.axs_out_tvalid_i(	axs_out_tvalid		),	// Receive Data channel valid
	.axs_out_tready_o(	axs_out_tready		),	// Receive Data channel ready
	.axs_out_tlast_i(	axs_out_tlast		),	// Receive Data channel last word
	.axs_out_tdata_i(	axs_out_tdata		),	// Receive Data channel data

			// ===============================================================================
			// AXI Stream Signals.

	.i2s_bclk_i(		i2s_bclk		),	// I2S Bit Clock
	.i2s_wc_sync_o(		i2s_wc_sync		),	// I2S Word Clock/Sync
	.i2s_dout_o(		i2s_dout		),	// I2S serial data output
	.i2s_din_i(		i2s_din			)	// I2S serial data input
	);

i2s_bfm i2s (
	.clk_i(			i2s_bclk		),	// I2S Bit Clock
	.i2s_sync_i(		i2s_wc_sync		),	// I2S Word Clock/Sync
	.i2s_dout_o(		i2s_din			),	// I2S serial data output
	.i2s_din_i(		i2s_dout		)	// I2S serial data input
	);

axl_mast_bfm axl(
	.clk_i(			clock			),
	.reset_i(		reset			),

	// ===============================================================================
	// AXI 4 Light Write Slave Channel

	// Write Address Channel Signals.
	.axl_awaddr_o(		axl_awaddr		),	// Master Write address. 
	.axl_awvalid_o(		axl_awvalid		),	// Master Write address valid.
	.axl_awready_i(		axl_awready		),	// Slave Write address ready.

	// Write Data Channel Signals.
	.axl_wdata_o(		axl_wdata		),	// Master Write data.
	.axl_wstrb_o(		axl_wstrb		),	// Master Write strobes.
	.axl_wvalid_o(		axl_wvalid		),	// Master Write valid.
	.axl_wready_i(		axl_wready		),	// Slave Write ready.

	// Write Response Channel Signals.
	.axl_bresp_i(		axl_bresp		),	// Slave Write response.
	.axl_bvalid_i(		axl_bvalid		),	// Slave Write response valid. 
	.axl_bready_o(		axl_bready		),	// Master Response ready.

	// ===============================================================================
	// AXI 4 Light Read Slave Channel

	// Read Address Channel Signals.
	.axl_araddr_o(		axl_araddr		),	// Master Read address.
	.axl_arvalid_o(		axl_arvalid		),	// Master Read address valid.
	.axl_arready_i(		axl_arready		),	// Slave Read address ready.

	// Read Data Channel Signals.
	.axl_rdata_i(		axl_rdata		),	// Slave Read data.
	.axl_rresp_i(		axl_rresp		),	// Slave Read response.
	.axl_rvalid_i(		axl_rvalid		),	// Slave Read valid.
	.axl_rready_o(		axl_rready		)	// Master Read ready.
	);

axi_istr_bfm istr(
	.clock_i(		clock			),	// Clock Input
	.reset_i(		reset			),	// Reset Input

	// ===============================================================================
	// AXI 4 Stream Receive Channel
	.tid_o(			axs_out_tid		),	// Receive Data channel ID
	.tvalid_o(		axs_out_tvalid		),	// Receive Data channel valid
	.tready_i(		axs_out_tready		),	// Receive Data channel ready
	.tlast_o(		axs_out_tlast		),	// Receive Data channel last word
	.tdata_o(		axs_out_tdata		)
	);

axi_ostr_bfm ostr(
	.clock_i(		clock			),	// Clock Input
	.reset_i(		reset			),	// Reset Input

	// ===============================================================================
	// AXI 4 Stream Transmit Channel
	.tid_o(			axs_in_tid		),	// Transmit Data channel ID
	.tvalid_i(		axs_in_tvalid		),	// Transmit data channel valid
	.tready_o(		axs_in_tready		),	// Transmit data channel ready
	.tlast_i(		axs_in_tlast		),	// Transmit data channel last word
	.tstrb_i(		4'hf			),	// Transmit data channel byte strobes
	.tkeep_i(		4'hf			),	// Transmit data channel byte strobes
	.tdata_i(		axs_in_tdata		)	// Transmit data channel data
	);

///////////////////////////////////////////////////////////////////
//
// Test and test lib Includes
//

`include "tests.v"

endmodule

