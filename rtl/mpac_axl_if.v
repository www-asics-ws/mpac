/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  AXI Light wrapper                                          ////
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

module mpac_axl_if (

input	wire		clk_i,
input	wire		rst_i,

	// ===============================================================================
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
output	reg		axl_bvalid_o,	// Slave Write response valid.
input	wire		axl_bready_i,	// Master Response ready.

	// ===============================================================================
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
	// RF Write Interface
output	reg	[31:0]	rf_wr_addr_o,	// RF Write Address
output	reg	[31:0]	rf_wr_data_o,	// RF Write Data
output	reg	[3:0]	rf_wr_be_o,	// RF Write Byte Enable
output	reg		rf_we_o,	// RF Write Enable
input	wire		rf_wr_wait_i,	// RF Write Wait

	// ===============================================================================
	// RF Read Interface

output	wire	[31:0]	rf_rd_addr_o,	// RF Read Address
input	wire	[31:0]	rf_rd_data_i,	// RF Read Data
output	wire		rf_re_o,	// RF Read Enable
input	wire		rf_rd_wait_i	// RF Read Wait
);

////////////////////////////////////////////////////////////
//
// Read Transfers
//

`define AXL_RD_SINGLE_CYCLE

reg		rd_busy;
reg		axl_rvalid_r;

// ------------------------------------------------------------------
// Address Phase
assign axl_arready_o = !rd_busy | (axl_rvalid_o & axl_rready_i);

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)			rd_busy <= #1 1'b0;
	else
	if(axl_arvalid_i)			rd_busy <= #1 1'b1;
	else
	if(axl_rvalid_o && axl_rready_i)	rd_busy <= #1 1'b0;

// ------------------------------------------------------------------
// DATA Phase

wire	[31:0]	rd_addr;
reg	[31:0]	rd_addr_r;

always @(posedge clk_i) if(axl_arvalid_i)	rd_addr_r <= #1 axl_araddr_i;
assign  rd_addr = axl_arvalid_i ? axl_araddr_i : rd_addr_r;

`ifdef AXL_RD_SINGLE_CYCLE	// ----- Single Cycle Data Phase

wire data_ready = axl_arvalid_i;
wire re = axl_arvalid_i && axl_arready_o;

`else				// ----- Two Cycle Data Phase
reg		data_ready, re;

always @(posedge clk_i) data_ready <= #1 axl_arvalid_i & (!rd_busy | (axl_rvalid_o & axl_rready_i));
always @(posedge clk_i) re <= #1 axl_arvalid_i & axl_arready_o;

`endif				// -------------------------------

assign rf_rd_addr_o = rd_addr;
assign rf_re_o = axl_rvalid_o & axl_rready_i;

// ------------------------------------------------------------------
// Response Logic
always @(posedge clk_i `MPAC_ACRT)	// RVALID MUST wait for ARVALID
	if(rst_i == `MPAC_ACR)			axl_rvalid_r <= #1 1'b0;
	else
	if(data_ready)				axl_rvalid_r <= #1 1'b1;
	else
	if(axl_rready_i && !rf_rd_wait_i)	axl_rvalid_r <= #1 1'b0;

assign axl_rvalid_o = axl_rvalid_r & !rf_rd_wait_i;

assign axl_rresp_o  = `AXL_RESP_OK;

reg	[31:0]	rd_data_r;

always @(posedge clk_i) if(axl_arvalid_i && axl_arready_o)	rd_data_r <= #1 rf_rd_data_i;

assign axl_rdata_o = rd_data_r;

////////////////////////////////////////////////////////////
//
// Write Transfers
//

reg		wr_busy;
wire		we_d;

// ------------------------------------------------------------------
// Address Phase

// Latch Read Address if no reads are pending
always @(posedge clk_i)
	if(axl_awready_o && axl_awvalid_i)	rf_wr_addr_o <= #1 axl_awaddr_i;

// Signal Address Ready
assign axl_awready_o = axl_awvalid_i & (!wr_busy | (axl_bready_i & axl_bvalid_o));

always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)			wr_busy <= #1 1'b0;
	else
	if(axl_awvalid_i)			wr_busy <= #1 1'b1;
	else
	if(axl_bready_i && axl_bvalid_o)	wr_busy <= #1 1'b0;

// ------------------------------------------------------------------
// Data Phase
assign we_d = axl_wvalid_i & !rf_wr_wait_i;
assign axl_wready_o = we_d;
always @(posedge clk_i)	rf_we_o <= #1 we_d;
always @(posedge clk_i)	if(axl_wvalid_i)	rf_wr_data_o <= #1 axl_wdata_i;
always @(posedge clk_i)	if(axl_wvalid_i)	rf_wr_be_o <= #1 axl_wstrb_i;

// ------------------------------------------------------------------
// Status Phase
always @(posedge clk_i `MPAC_ACRT)
	if(rst_i == `MPAC_ACR)			axl_bvalid_o <= #1 1'b0;
	else
	if(we_d)				axl_bvalid_o <= #1 1'b1;
	else
	if(axl_bready_i)			axl_bvalid_o <= #1 1'b0;

assign axl_bresp_o  = `AXL_RESP_OK;

endmodule
