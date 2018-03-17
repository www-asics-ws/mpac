/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  I2S BFM                                                    ////
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


module i2s_bfm (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// bit Clock


	// ===============================================================================
	// I2S Signals.

output	wire		i2s_sync_i,		// I2S sync input
output	wire		i2s_dout_o,		// I2S serial data output
input	wire		i2s_din_i		// I2S serial data input
);

// ---------------------------------------------------------------------------------------
// Local Parameters


localparam BUF_WI_L2	= 12;		// Number of bits to address buffer depth
localparam NO_CH_L2	= 4;

localparam buf_size = (1<<NO_CH_L2) * (1<<BUF_WI_L2);	// Number of buffers * Buffer depth

// ---------------------------------------------------------------------------------------
// Wires and Registers

integer		slot_width  = 16;
integer		msb_first = 0;
integer		slot_count = 4;

reg	[31:0]	in_buf[0:buf_size];
reg	[31:0]	out_buf[0:buf_size];

wire	[31:0]	din_i;
reg	[31:0]	dout_o;

reg	[31:0]	dout_r;
reg	[31:0]	din_r;
reg		sdata_in_r;

reg		i2s_sync_r;
wire		start, slot;
reg	[7:0]	bit_cnt = 0, s_cnt = 0;
reg		in_buf_we;
reg	[3:0]	in_buf_ch;
reg	[BUF_WI_L2-1:0]	in_buf_addr = 0;
reg		next_entry;
reg		enabled = 1'b0;
integer		max = 16;
reg		en = 0;

wire		dout_next;
reg		dout_next_r;
reg	[3:0]	out_slot;
reg	[BUF_WI_L2-1:0]	out_addr = 0;


task init;
input	mode;
input	sc;
input	max_tx;
input	sw;

integer	mode, sc, max_tx, n, m, sw;
reg	[31:0]	tmp;

begin

max = max_tx;
en =1;
slot_count = sc;
slot_width = sw;

for(m=0;m<16;m=m+1)
for(n=0;n<max;n=n+1)
   begin

	tmp = $random;
	case(mode)
	  default:	out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}] = n+1;
	  1:		out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}] = tmp;
	  3:		out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}] = {m[NO_CH_L2-1:0], tmp[27:0]};
	  2:		out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}] = {m[NO_CH_L2-1:0], n[27:0]};
	  99:		out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}] = out_buf[{m[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}];
	endcase

   end

end
endtask


task wait_idle;

begin
while(!enabled)	@(posedge clk_i);
while(enabled)	@(posedge clk_i);
repeat(3)	@(posedge clk_i);

end
endtask


task dump_in_buf;
input	[3:0]	slot;
input	[7:0]	count;

integer n, tmp;
begin

$display("\nIN_BUF for slot %0d", slot);

for(n=0;n<count;n=n+1)
   begin
	$display("%0d: %x", n, in_buf[{slot[NO_CH_L2-1:0], n[BUF_WI_L2-1:0]}]);
   end

$display("\n");
end
endtask

// ---------------------------------------------------------------------------------------
// Control Logic

always @(posedge clk_i) i2s_sync_r <= #1 i2s_sync_i;
assign start = i2s_sync_i & !i2s_sync_r;

always @(posedge clk_i)
	if(start)	bit_cnt <= #1 8'h0;
	else
	if(!enabled)	bit_cnt <= #1 8'h0;
	else
	if(slot)	bit_cnt <= #1 8'h0;
	else		bit_cnt <= #1 bit_cnt + 8'h1;

assign slot = bit_cnt == (slot_width-1);

always @(posedge clk_i)
	if(start)	s_cnt <= #1 8'h0;
	else
	if(!enabled)	s_cnt <= #1 8'h0;
	else
	if(slot)	s_cnt <= #1 s_cnt + 8'h1;

always @(posedge clk_i)
	if(start && en)		enabled <= #1 1'b1;
	else
	if((out_addr >= (max-1)) && (out_slot>= (slot_count-1)) && slot)
	   begin
		enabled <= #1 1'b0;
		en <= #1 1'b0;
	   end




reg		enable_out_r = 1'b0;

always @(posedge clk_i)
	if(enabled)		enable_out_r <= #1 1'b1;
	else
	if(start)		enable_out_r <= #1 1'b0;

wire	enable_out = enabled | start | enable_out_r;

reg		en_r;
always @(posedge clk_i)	en_r <= #1 en;

// ---------------------------------------------------------------------------------------
// In Buffer Control

always @(posedge clk_i) in_buf_we <= #1 slot & enabled;
always @(posedge clk_i) in_buf_ch <= #1 s_cnt[3:0];

always @(posedge clk_i)
	if(in_buf_we)
	   begin
		//$display("IN_BUF[%0d][%0d] = %x", in_buf_ch, in_buf_addr, dout_o);
		in_buf[{in_buf_ch[NO_CH_L2-1:0], in_buf_addr[BUF_WI_L2-1:0]}] <= #1 dout_o;
	   end

always @(posedge clk_i) next_entry <= #1 start & (in_buf_ch!=0);

always @(posedge clk_i)
	if(!enabled)	in_buf_addr <= #1 0;
	else
	if(next_entry)	in_buf_addr <= #1 in_buf_addr + 1;

// ---------------------------------------------------------------------------------------
// Out Buffer Control

assign din_i = out_buf[{out_slot[NO_CH_L2-1:0], out_addr[BUF_WI_L2-1:0]}];

always @(posedge clk_i)
	if(dout_next_r)
	   begin
		//$display("OUT_BUF[%0d][%0d] = %x", out_slot, out_addr, din_i);
	   end

assign dout_next = bit_cnt == (slot_width - 3);
always @(posedge clk_i) dout_next_r <= #1 (dout_next & enable_out) || (en & ! en_r);

always @(posedge clk_i)
	if(!enable_out)	out_slot <= #1 0;
	else
	if(start)	out_slot <= #1 0;
	else
	if(dout_next)
	   begin
		if(!enable_out)				out_slot <= #1 4'h0;
		else
		if((slot_count - 1) == out_slot)	out_slot <= #1 4'h0;
		else					out_slot <= #1 out_slot + 1;
	   end

always @(posedge clk_i)
	if(!enable_out)		out_addr <= #1 0;
	else
	if(dout_next && ((slot_count - 1) == out_slot))
				out_addr <= #1 out_addr + 1;

// ---------------------------------------------------------------------------------------
// Shift Logic

// Serial Output
assign	i2s_dout_o = dout_r[31];

always @(posedge clk_i)
	//if(dout_next_r || (!enable_out & !i2s_sync_i))
	if(dout_next_r)
	   begin
		if(msb_first)
		   begin
			case(slot_width)	// Align output data to be left (msb) aligned
			   16:		dout_r <= #1 {din_i[15:0], 16'h0};
			   18:		dout_r <= #1 {din_i[17:0], 14'h0};
			   20:		dout_r <= #1 {din_i[19:0], 12'h0};
			   24:		dout_r <= #1 {din_i[23:0], 8'h0};
			   default:	dout_r <= #1 din_i;
			endcase
		   end
		else
		   begin
			// Reverse Input data
			case(slot_width)	// Align output data to be left (msb) aligned
			   16:		dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], 16'h0};
	
			   18:		dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							14'h0};
	
			   20:		dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							din_i[18], din_i[19], 12'h0};
	
			   24:		dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							din_i[18], din_i[19], din_i[20], din_i[21], din_i[22], din_i[23],
							8'h0};
	
			   default:	dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							din_i[18], din_i[19], din_i[20], din_i[21], din_i[22], din_i[23], 
							din_i[24], din_i[25], din_i[26], din_i[27], din_i[28], din_i[29], 
							din_i[30], din_i[31]};
			endcase
		   end
	   end
	else
	if(enable_out)	dout_r <= #1 {dout_r[30:0], 1'b0};


// Serial Input
always @(negedge clk_i)
	sdata_in_r <= #1 i2s_din_i;

always @(posedge clk_i)
	din_r <= #1 {din_r[30:0], sdata_in_r };

always @(posedge clk_i)	// Input data is already right (lsb) aligned
	if(slot)
	   begin
		if(msb_first)
		   begin
			case(slot_width)	// Align output data to be left (msb) aligned
			   16:		dout_o <= #1 {16'h0, din_r[15:0]};
			   18:		dout_o <= #1 {14'h0, din_r[17:0]};
			   20:		dout_o <= #1 {12'h0, din_r[19:0]};
			   24:		dout_o <= #1 {8'h0, din_r[23:0]};
			   default:	dout_o <= #1 din_r;
			endcase
		   end
		else
		   begin
			// Reverse Input data
			case(slot_width)	// Align output data to be left (msb) aligned
			   16:		dout_o <= #1 {	16'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15]};
	
			   18:		dout_o <= #1 {	14'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17]};
	
			   20:		dout_o <= #1 {	12'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17],
							din_r[18], din_r[19]};
	
			   24:		dout_o <= #1 {	8'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17],
							din_r[18], din_r[19], din_r[20], din_r[21], din_r[22], din_r[23]};
	
			   default:	dout_o <= #1 {	din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17],
							din_r[18], din_r[19], din_r[20], din_r[21], din_r[22], din_r[23], 
							din_r[24], din_r[25], din_r[26], din_r[27], din_r[28], din_r[29], 
							din_r[30], din_r[31]};
			endcase
		   end
	   end



endmodule
