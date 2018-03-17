////////////////////////////////////////////////////////////////////
////                                                             ////
////  AXI Light Master BFM                                       ////
////                                                             ////
////                                                             ////
////  Author: Rudolf Usselmann                                   ////
////           (rudi@asics.ws)                                   ////
////                                                             ////
////  Proprietary and Confidential Information of                ////
////  ASICS World Services, LTD                                  ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2000-2011 ASICS World Services, LTD.          ////
////                         www.asics.ws                        ////
////                         info@asics.ws                       ////
////                                                             ////
//// This software is provided under license and contains        ////
//// proprietary and confidential material which is the          ////
//// property of ASICS.ws.                                       ////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

//  $Id:        $

`timescale 1ns / 100ps

module axl_mast_bfm (

input	wire		clk_i,
input	wire		reset_i,

	// ===============================================================================
	// Write Address Channel Signals.
output	reg	[31:0]	axl_awaddr_o,	// Master Write address. 
output	reg		axl_awvalid_o,	// Master Write address valid.
input	wire		axl_awready_i,	// Slave Write address ready.

	// Write Data Channel Signals.
output	reg	[31:0]	axl_wdata_o,	// Master Write data.
output	reg	[3:0]	axl_wstrb_o,	// Master Write strobes.
output	reg		axl_wvalid_o,	// Master Write valid.
input	wire		axl_wready_i,	// Slave Write ready.

	// Write Response Channel Signals.
input	wire	[1:0]	axl_bresp_i,	// Slave Write response.
input	wire		axl_bvalid_i,	// Slave Write response valid. 
output	wire		axl_bready_o,	// Master Response ready.

	// ===============================================================================
	// Read Address Channel Signals.
output	reg	[31:0]	axl_araddr_o,	// Master Read address.
output	reg		axl_arvalid_o,	// Master Read address valid.
input	wire		axl_arready_i,	// Slave Read address ready.

	// Read Data Channel Signals.
input	wire	[31:0]	axl_rdata_i,	// Slave Read data.
input	wire	[1:0]	axl_rresp_i,	// Slave Read response.
input	wire		axl_rvalid_i,	// Slave Read valid.
output	reg		axl_rready_o	// Master Read ready.
);

////////////////////////////////////////////////////////////
//
// Local Wires and registers
//

reg	[7:0]	axl_rvalid_r;
reg	[7:0]	axl_bvalid_r;
integer		rr_del, wr_ddel, wr_del;

initial
   begin
	rr_del = 0;	// Master RREADY delay
	wr_ddel = 0;	// Master Write Data Phase delay
	wr_del = 0;	// Master BREADY delay

	axl_araddr_o = 32'hxxxxxxxx;
	axl_arvalid_o = 1'b0;

	axl_wdata_o = 32'hxxxx;
	axl_wstrb_o = 4'hx;
	axl_wvalid_o = 1'b0;

	#1;
	$display("\nINFO: AXI MASTER BFM INSTANTIATED (%m)\n");
   end


////////////////////////////////////////////////////////////
//
// Read Transfers
//

wire r_start = axl_arvalid_o & axl_arready_i;

wire r_start_d = (rr_del==0 || ((rr_del-1)==0)) ? r_start : axl_rvalid_r[rr_del-2];

always @(posedge clk_i)
	if(r_start_d)		axl_rvalid_r <= #1 8'h0;
	else			axl_rvalid_r <= #1 {axl_rvalid_r[6:0], r_start};


always @(posedge r_start_d)
	if(rr_del==-1 || rr_del==0)		axl_rready_o <= #1 1'b1; 

always @(posedge clk_i)
	if(reset_i)				axl_rready_o <= #1 1'b0;
	else
	if(r_start_d)				axl_rready_o <= #1 1'b1;
	else
	if(axl_rvalid_i && rr_del!=-1)		axl_rready_o <= #1 1'b0;



task read;
input [31:0] a;
begin

	#1;
	axl_araddr_o = a;
	axl_arvalid_o = 1'b1;
	#1;
	@(posedge clk_i);
	while(axl_arready_i == 1'b0)	@(posedge clk_i);

	#1;
	axl_araddr_o = 32'hxxxxxxxx;
	axl_arvalid_o = 1'b0;

end
endtask


task get_data;
output [31:0] d;
output [1:0]  r;
begin

@(posedge clk_i);
while((axl_rvalid_i == 1'b0) || (axl_rready_o == 1'b0))	@(posedge clk_i);
assign d = axl_rdata_i;
assign r = axl_rresp_i;
end
endtask


////////////////////////////////////////////////////////////
//
// Write Transfers
//
reg	[3:0]	ss, sss;
reg	[31:0]	dd, ddd;
reg		axl_bready_r;

wire w_start = axl_wvalid_o & axl_wready_i;

wire w_start_d = (wr_del==0) ? w_start : axl_bvalid_r[wr_del-1];

always @(posedge clk_i)
	if(w_start_d)		axl_bvalid_r <= #1 8'h0;
	else			axl_bvalid_r <= #1 {axl_bvalid_r[6:0], w_start};

always @(posedge clk_i)
	if(reset_i)				axl_bready_r <= #1 1'b0;
	else
	if(w_start_d)				axl_bready_r <= #1 1'b1;
	else
	if(axl_bvalid_i)			axl_bready_r <= #1 1'b0;


assign axl_bready_o = axl_bready_r | (wr_del>7) | (wr_del<0);


task write;
input [31:0] a;
input [3:0] s;
input [31:0] d;

begin

	#1;
	axl_awaddr_o = a;
	axl_awvalid_o = 1'b1;
	ss = s;
	dd = d;

	@(posedge clk_i);
	while(axl_awready_i == 1'b0)	@(posedge clk_i);

	#1;
	axl_awaddr_o = 32'hxxxxxxxx;
	axl_awvalid_o = 1'b0;

end
endtask


always @(posedge clk_i)
   begin
	#3;
	if(axl_awready_i && axl_awvalid_o)
	   begin
		ddd = dd;
		sss = ss;
	   end

   end

always @(posedge axl_awready_i)
  begin
	repeat(wr_ddel)	@(posedge clk_i);

	#1;
	axl_wvalid_o = 1'b1;
	#3;
	axl_wdata_o = ddd;
	axl_wstrb_o = sss;

	@(posedge clk_i);
	while(axl_wready_i == 1'b0)	@(posedge clk_i);

	#1;
	axl_wdata_o = 32'hxxxx;
	axl_wstrb_o = 4'hx;
	axl_wvalid_o = 1'b0;
   end


endmodule


