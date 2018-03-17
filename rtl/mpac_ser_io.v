/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Serializer                                                 ////
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

module mpac_ser_io (
	// ===============================================================================
	// General Signals

input	wire		clk_i,		// bit Clock

	// ===============================================================================
	// Register File Interface

input	wire	[2:0]	ser_slot_width_i, // Serial Slot Width
					// 0 - 16 bit
					// 1 - 18 bit
					// 2 - 20 bit
					// 3 - 24 bit
					// 4 - 32 bit
					// 5-7 RESERVED
input	wire		msb_first_i,	// Shift out MSB first

	// ===============================================================================
	// Internal Interface

input	wire	[31:0]	din_i,		// Input data, right (lsb) aligned
output	reg	[31:0]	dout_o,		// Output data , right (lsb) aligned

input	wire		out_le_i,	// Output Latch Enable
input	wire		in_le_i,	// Input Latch Enable

	// ===============================================================================
	// I2S Signals.

output	wire		i2s_dout_o,		// I2S serial data output
input	wire		i2s_din_i		// I2S serial data input
);

// ---------------------------------------------------------------------------------------
// Local Parameters

// ---------------------------------------------------------------------------------------
// Wires and Registers

reg	[31:0]	dout_r;
reg	[31:0]	din_r;

// ---------------------------------------------------------------------------------------
// Shift Logic

// Serial Output
assign	i2s_dout_o = dout_r[31];

always @(posedge clk_i)
	if(out_le_i)
	   begin
		if(msb_first_i)
		   begin
			case(ser_slot_width_i)	// Align output data to be left (msb) aligned
			   3'h0:	dout_r <= #1 {din_i[15:0], 16'h0};
			   3'h1:	dout_r <= #1 {din_i[17:0], 14'h0};
			   3'h2:	dout_r <= #1 {din_i[19:0], 12'h0};
			   3'h3:	dout_r <= #1 {din_i[23:0], 8'h0};
			   default:	dout_r <= #1 din_i;
			endcase
		   end
		else
		   begin
			// Reverse Input data
			case(ser_slot_width_i)	// Align output data to be left (msb) aligned
			   3'h0:	dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], 16'h0};
	
			   3'h1:	dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							14'h0};
	
			   3'h2:	dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
							din_i[6], din_i[7], din_i[8], din_i[9], din_i[10], din_i[11],
							din_i[12], din_i[13], din_i[14], din_i[15], din_i[16], din_i[17],
							din_i[18], din_i[19], 12'h0};
	
			   3'h3:	dout_r <= #1 {	din_i[0], din_i[1], din_i[2], din_i[3], din_i[4], din_i[5],
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
	else		dout_r <= #1 {dout_r[30:0], 1'b0};


// Serial Input
always @(posedge clk_i)
	din_r <= #1 {din_r[30:0], i2s_din_i };

always @(posedge clk_i)	// Input data is already right (lsb) aligned
	if(in_le_i)
	   begin
		if(msb_first_i)
		   begin
			case(ser_slot_width_i)	// Align output data to be left (msb) aligned
			   3'h0:	dout_o <= #1 {16'h0, din_r[15:0]};
			   3'h1:	dout_o <= #1 {14'h0, din_r[17:0]};
			   3'h2:	dout_o <= #1 {12'h0, din_r[19:0]};
			   3'h3:	dout_o <= #1 {8'h0, din_r[23:0]};
			   default:	dout_o <= #1 din_r;
			endcase
		   end
		else
		   begin
			// Reverse Input data
			case(ser_slot_width_i)	// Align output data to be left (msb) aligned
			   3'h0:	dout_o <= #1 {	16'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15]};
	
			   3'h1:	dout_o <= #1 {	14'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17]};
	
			   3'h2:	dout_o <= #1 {	12'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
							din_r[6], din_r[7], din_r[8], din_r[9], din_r[10], din_r[11],
							din_r[12], din_r[13], din_r[14], din_r[15], din_r[16], din_r[17],
							din_r[18], din_r[19]};
	
			   3'h3:	dout_o <= #1 {	8'h0, din_r[0], din_r[1], din_r[2], din_r[3], din_r[4], din_r[5],
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
