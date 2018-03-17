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

module axi_ostr_bfm(

input	wire		clock_i,	// Clock Input
input	wire		reset_i,	// Reset Input

	// ===============================================================================
	// AXI 4 Stream Transmit Channel
output	reg	[3:0]	tid_o,
input	wire		tvalid_i,	// Transmit data channel valid
output	wire		tready_o,	// Transmit data channel ready
input	wire		tlast_i,	// Transmit data channel last word
input	wire	[3:0]	tstrb_i,	// Transmit data channel byte strobes
input	wire	[3:0]	tkeep_i,	// Transmit data channel byte strobes
input	wire	[31:0]	tdata_i		// Transmit data channel data

);

// ---------------------------------------------------------------------------------------
// Local Parameters

localparam BUF_WI_L2	= 12;		// Number of bits to address buffer depth
localparam NO_CH_L2	= 4;

localparam buf_size = (1<<NO_CH_L2) * (1<<BUF_WI_L2);	// Number of buffers * Buffer depth

// ---------------------------------------------------------------------------------------
// Wires and Registers

integer rdy_del;

reg	[31:0]	mem	[0:buf_size];
reg	[31:0]	cnt, last_cnt;
reg	[15:0]	tready_r;
wire	[15:0]	tready_d;
reg	[15:0]	en_cnt;
wire		enable;

initial
   begin
	tid_o = 0;
	rdy_del = 0;
   end

always @(posedge clock_i)
	if(tready_o && tvalid_i)		mem[{tid_o[NO_CH_L2-1:0], cnt[BUF_WI_L2-1:0]}] <= #1 tdata_i;

always @(posedge clock_i)
	if(reset_i)				cnt <= #1 32'h0;
	else
	if(tready_o && tvalid_i)		cnt <= #1 cnt + 32'h1;

always @(posedge clock_i)
	if(tready_o && tvalid_i && tlast_i)	last_cnt <= #1 cnt + 32'h1;

always @(posedge clock_i)
	if(reset_i)				tready_r <= #1 16'h0;
	else
	if(tvalid_i && tready_o)		tready_r <= #1 16'h0;
	else
	if(!enable)				tready_r <= #1 16'h0;
	else					tready_r <= #1 {tready_r[14:0], tvalid_i & !tready_o};

assign tready_d = {tready_r[14:0], 1'b1};
assign tready_o = tready_d[rdy_del] & enable;

// ===============================================================================
// Transfer complete tracking
reg	rx_busy;

/*
always @(posedge clock_i)
	if(reset_i)				rx_busy <= #1 1'b0;
	else
	if(tlast_i && tready_o && tvalid_i)	rx_busy <= #1 1'b0;
	else
	if(tvalid_i)				rx_busy <= #1 1'b1;

task wait_rx_done;
begin
while(rx_busy)	@(posedge clock_i);
@(posedge clock_i);
end
endtask
*/

task wait_idle;

begin
while(!enable)	@(posedge clock_i);
while(enable)	@(posedge clock_i);
repeat(3)	@(posedge clock_i);

end
endtask

// ===============================================================================
// Delayed Module Enable

always @(posedge clock_i)
	if(reset_i)				en_cnt <= #1 16'h0;
	else
	if(enable && tready_o && tvalid_i)	en_cnt <= #1 en_cnt - 16'h01;

assign enable = en_cnt != 16'h0;

// ===============================================================================
// Tasks

task init;
input	id;
input	mode;
input	count;

integer i, count, mode, id, tmp;

begin
@(posedge clock_i);
#1;
cnt = 0;
tid_o = id;
last_cnt = 32'h0;
tready_r = 0;
en_cnt = count;

for(i=0;i<count;i=i+1)
   begin
	tmp = $random;
	case(mode)
	  default:	mem[{tid_o[3:0], i[BUF_WI_L2:0]}] = i+1;
	  1:		mem[{tid_o[3:0], i[BUF_WI_L2:0]}] = tmp;
	  2:		mem[{tid_o[3:0], i[BUF_WI_L2:0]}] = {tid_o[NO_CH_L2-1:0], i[27:0]};
	  3:		mem[{tid_o[3:0], i[BUF_WI_L2:0]}] = {tid_o[NO_CH_L2-1:0], tmp[27:0]};
	  4:		mem[{tid_o[3:0], i[BUF_WI_L2:0]}] = {16'h00, tid_o[NO_CH_L2-1:0], i[11:0]};
	endcase
   end
end
endtask

endmodule

