/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  DPRAM                                                      ////
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

module mpac_dpram #(
parameter integer DW = 8,
parameter integer AW = 8
) (
input	wire			wr_clk_i,
input	wire			rd_clk_i,
input	wire			we_i,
input	wire	[AW-1:0]	wr_addr_i,
input	wire	[AW-1:0]	rd_addr_i,
input	wire	[DW-1:0]	data_i,
output	reg	[DW-1:0]	data_o
);

parameter MD = 1<<AW;
reg [DW-1:0] mem_array [MD-1:0];

always @(posedge rd_clk_i) data_o <= #1 mem_array[rd_addr_i];

always @(posedge wr_clk_i) if(we_i) mem_array[wr_addr_i] <= #1 data_i;

endmodule

