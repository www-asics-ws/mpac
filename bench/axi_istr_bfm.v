/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Test Bench                                                 ////
////                                                             ////
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


`timescale 1ns / 100ps

module axi_istr_bfm(

input	wire		clock_i,	// Clock Input
input	wire		reset_i,	// Reset Input

	// ===============================================================================
	// AXI 4 Stream Transmit Channel
output	reg	[3:0]	tid_o,
output	wire		tvalid_o,	// Receive Data channel valid
input	wire		tready_i,	// Receive Data channel ready
output	wire		tlast_o,	// Receive Data channel last word
output	wire	[31:0]	tdata_o		// Receive Data channel data
);


// ---------------------------------------------------------------------------------------
// Local Parameters

localparam BUF_WI_L2	= 12;		// Number of bits to address buffer depth
localparam NO_CH_L2	= 4;

localparam buf_size = (1<<NO_CH_L2) * (1<<BUF_WI_L2);	// Number of buffers * Buffer depth

// ---------------------------------------------------------------------------------------
// Wires and Registers

reg [15:0] rdy_del;
initial rdy_del = 0;

reg	[31:0]	mem	[0:buf_size];
reg	[31:0]	last_cnt, dcnt;
reg	[15:0]	tvalid_r;
wire	[15:0]	tvalid_d;
reg		en = 0;
initial		tid_o = 0;

assign tdata_o = tvalid_o ? mem[ {tid_o[3:0], dcnt[BUF_WI_L2-1:0]} ] : 32'hBAD1_XXXX;

assign tlast_o = (dcnt == last_cnt);

always @(posedge clock_i)
	if(tvalid_o && tready_i && tlast_o)	en <= #1 1'b0;

always @(posedge clock_i)
	if(reset_i)				dcnt <= #1 32'h0;
	else
	if(tvalid_o && tready_i)		dcnt <= #1 dcnt +  32'h01;

always @(posedge clock_i)
	if(reset_i)				tvalid_r <= #1 16'h0;
	else
	if(tvalid_o && tready_i)		tvalid_r <= #1 16'h0;
	else					tvalid_r <= #1 {tvalid_r[14:0], tready_i & !tvalid_o & en};

assign tvalid_d = {tvalid_r[14:0], 1'b1};
assign tvalid_o = tvalid_d[rdy_del] & en;

task init;
input	id;
input	mode;
input	count;

integer i, count, mode, id;
reg	[31:0]	tmp;
begin
@(posedge clock_i);
#1;
dcnt = 0;
en = 1;
tvalid_r = 0;
tid_o = id;
last_cnt = count-1;

if(mode != 99)
for(i=0;i<count;i=i+1)
   begin
	tmp = $random;
	case(mode)
	  default:	mem[{tid_o[3:0], i[BUF_WI_L2-1:0]}] = i+1;
	  1:		mem[{tid_o[3:0], i[BUF_WI_L2-1:0]}] = tmp;
	  2:		mem[{tid_o[3:0], i[BUF_WI_L2-1:0]}] = {tid_o[3:0], i[27:0]};
	  3:		mem[{tid_o[3:0], i[BUF_WI_L2-1:0]}] = {tid_o[3:0], tmp[27:0]};
	  4:		mem[{tid_o[3:0], i[BUF_WI_L2-1:0]}] = {16'h00, 4'h0, tid_o[3:0], i[7:0]};

	endcase
   end

@(posedge clock_i);
while(en)	@(posedge clock_i);
@(posedge clock_i);

end
endtask

endmodule

