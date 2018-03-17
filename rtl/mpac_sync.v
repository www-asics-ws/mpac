/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Synchronizers                                              ////
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


module mpac_sync_mb #(
parameter	STAG = 2,	// Number of stages
parameter	WIDTH = 1	// Signal Width
) (
input	wire			clk_i,
input	wire	[WIDTH-1:0]	data_i,
output	wire	[WIDTH-1:0]	data_o
);

genvar n;

generate
   begin
	for(n=0;n<WIDTH;n=n+1)
		mpac_sync_1b	#(.STAG(STAG))	sync( .clk_i(clk_i), .data_i(data_i[n]), .data_o(data_o[n]));
   end
endgenerate

endmodule


module mpac_sync_1b #(
parameter	STAG = 2	// Number of stages
) (
input	wire		clk_i,
input	wire		data_i,
output	wire		data_o
);

genvar n;
`MPAC_ASYNCH_REG	reg	[STAG-1:0]	cdc_sync_reg;

generate
begin
if(STAG==1)
   begin
	always @(posedge clk_i)	cdc_sync_reg[0] <= #1 data_i;
	assign data_o = cdc_sync_reg[0];
   end
else
   begin
	always @(posedge clk_i)	cdc_sync_reg[0] <= #1 data_i;
	for(n=0;n<STAG-1;n=n+1)
		always @(posedge clk_i)	cdc_sync_reg[n+1] <= #1 cdc_sync_reg[n];
	assign data_o = cdc_sync_reg[STAG-1];
   end
end
endgenerate

endmodule







