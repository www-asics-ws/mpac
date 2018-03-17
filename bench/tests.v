
/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Test Bench                                                 ////
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


task mpac_read;
input   [31:0]  addr;
output  [31:0]  data;

reg     [1:0]   reply;
reg     [31:0]  data;
begin
fork
     axl.read(addr);
     axl.get_data(data, reply);
join
if(reply != `AXL_RESP_OK)
        $display("ERROR: Read %x failed with reply %x (%t)", addr, reply, $time);
end
endtask


task mpac_write;
input   [31:0]  addr;
input   [31:0]  data;
begin
axl.write(addr, 4'hf, data);
@(posedge clock);
end
endtask


task show_errors;
begin

$display("\n");
$display("     +--------------------+");
$display("     |  Total ERRORS: %0d   |", error_count);
$display("     +--------------------+");
end
endtask


task config_mpac;

input	[3:0]	no_ich;		// Number of input channels
				// Must be less than or equal to NO_IN_CH
input	[3:0]	no_och;		// Number of output channels
				// Must be less than or equal to NO_OUT_CH
input	[31:0]	mute_val;	// Mute Value

input	[2:0]	prot_sel;	// Protocol Select
				// 0 - I2S (2 channel only)
				// 1 - TDM
				// 2 - AC97
				// 3-7 RESERVED

input	[2:0]	sample_size;	// Sample size from SoC
				// 0 - 16 bit
				// 1 - 18 bit
				// 2 - 20 bit
				// 3 - 24 bit
				// 4 - 32 bit
				// 5-7 RESERVED

input		sample_align;	// Sample alignment
				// 0 - right (lsb)
				// 1 - left (msb)

input		sample_pac;	// Samples are packed
				// 0 - no
				// 1 - yes

input		ser_msb_first;	// Serial MSB first

input	[2:0]	ser_slot_width; // Serial Slot Width
				// 0 - 16 bit
				// 1 - 18 bit
				// 2 - 20 bit
				// 3 - 24 bit
				// 4 - 32 bit
				// 5-7 RESERVED

input	[1:0]	sync_start;	// Sync start
				// 0 - sync with serial data
				// 1 - one clock before serial data
				// 2-3 RESERVED

input	[1:0]	sync_dur;	// Sync duration
				// 0 - One cycle active at the beginning
				// 1 - 50% duty cycle
				// 2 - One cycle in-active at the end
				// 3 RESERVED

input 		sync_inv;	// Invert Sync
				// 0 - not inverted (first high than low)
				// 1 - inverted (first low than high)

reg	[31:0]	cfg;
reg	[31:0]	csr;
begin

// ------------ Configuration Register ------------
cfg[31:27] = 0;			// 31:27  RESERVED
cfg[26:24] = prot_sel;		// 26:24  Protocol Select
cfg[23:21] = 0;			// 23:21  RESERVED
cfg[20:18] = sample_size;	// 20:18  Sample size from SoC
cfg[17] = sample_align;		// 17	  Sample alignment
cfg[16] = sample_pac;		// 16	  Samples are packed
cfg[15:12] = 0;			// 15:12  RESERVED
cfg[11] = ser_msb_first;	// 11   Serial MSB first
cfg[10:8] = ser_slot_width;	// 10:8	  Serial Slot Width
cfg[7:5] = 0;			// 7:5    RESERVED
cfg[4:3] = sync_start;		// 4:3	  Sync start
cfg[2:1] = sync_dur;		// 2:1	  Sync duration
cfg[0] = sync_inv;		// 0	  Invert Sync

mpac_write(`CFG, cfg);
mpac_write(`MVAL, mute_val);

csr = { 16'h0,
	no_ich,
	no_ich,
	7'h0,
	1'b1
	};

mpac_write(`CSR, csr);

end
endtask


task test1;

reg	[31:0]  tmp;
integer		s, n, max_s, tx_cnt, mode, s_wi, samp_wi;
reg	[31:0]	src, dst, mask;
integer		debug, i, msb_first;

begin
$display("\n");
$display("*********************************************");
$display("***  Test 1                               ***");
$display("*********************************************\n");

debug=0;
tx_cnt = 64;
max_s = 4;
mode = 1;
s_wi = 0;
msb_first = 0;

if(mode != 1)
	$display("WARNING: mode is set to %0d, not using random data.", mode);

// FIX_ME
// - Add variation of System Sample width, vs Slot Width
// - Add variation of Sync Width

for(msb_first=0;msb_first<2;msb_first=msb_first + 1)
for(max_s=2;max_s<17;max_s=max_s + 2)
for(s_wi=0;s_wi<5;s_wi=s_wi+1)
   begin
	mpac_write(`CSR, 32'h80000000);
   	repeat(10)	@(posedge clock);	// Must wait 10 clock cycles after soft reset

	case(s_wi)
	   0: samp_wi = 16;
	   1: samp_wi = 18;
	   2: samp_wi = 20;
	   3: samp_wi = 24;
	   4: samp_wi = 32;
	endcase

	$display("Running I2S/TDM Test, sample width: %0d bits, max slots: %0d, MSB first %0d", samp_wi, max_s, msb_first);
	
	if(samp_wi<32)	mask = (1<<samp_wi) -1;
	else		mask = 32'hffffffff;
	
	i2s.msb_first = msb_first;
	for(s=0;s<max_s;s=s+1)
		istr.init(s, mode, tx_cnt);	// tid, mode, count

	config_mpac(	max_s-1,// Number of input channels
			max_s-1,// Number of output channels
			0,	// Mute Value
			0,	// Protocol Select
			s_wi,	// Sample size from SoC
			0,	// Sample alignment
			0,	// Samples are packed
	
			msb_first,// Serial MSB first
			s_wi,	// Serial Slot Width
			0,	// Sync start
			1,	// Sync duration
			0	// Invert Sync
		);

	i2s.init(mode, max_s, tx_cnt, samp_wi);
	i2s.wait_idle;

	repeat(200)	@(posedge clock);	// Must wait to let all transfers complete
	mpac_write(`CSR, 32'h00000000);		// Disable IP

	// Read out internal buffers
	for(s=0;s<max_s;s=s+1)
	   begin
		ostr.init(s, mode, tx_cnt);
		ostr.wait_idle;
	   end
	
	// --------------------------------------------------------
	// Check SoC to Codec Data

	for(s=0;s<max_s;s=s+1)
	  for(n=0;n<tx_cnt-1;n=n+1)	// SoC to Codec
	    begin
	
		src =   istr.mem[{s[3:0], n[istr.BUF_WI_L2-1:0]}] & mask;
		dst = i2s.in_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;
	
		if(debug)	$display("BFM IN BUF[%0d][%0d]: %x", s, n, dst );
		if(debug)	$display("  ISTR BUF[%0d][%0d]: %x", s, n, src );
	
		if(dst !== src)
		   begin
			$display("IN Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end

	for(s=0;s<max_s;s=s+1)
	  for(n=0;n<tx_cnt;n=n+1)	// codec to SoC
	    begin
	
		dst =   ostr.mem[{s[3:0], n[istr.BUF_WI_L2-1:0]}]  & mask;
		src = i2s.out_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}]  & mask;
	
		if(debug)	$display("BFM OUT BUF[%0d][%0d]: %x", s, n, src );
		if(debug)	$display("  OSTR BUF[%0d][%0d]: %x", s, n, dst );
	
		if(dst !== src)
		   begin
			$display("OUT Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	
	    end
	repeat(10)	@(posedge clock);
   end


show_errors;
$display("*********************************************");
$display("*** Test DONE ...                         ***");
$display("*********************************************\n\n");
end
endtask


task test_pack_16;

reg	[31:0]  tmp;
integer		s, n, max_s, tx_cnt, mode, s_wi, samp_wi;
reg	[31:0]	src, dst, mask;
integer		debug, i, msb_first;

begin
$display("\n");
$display("*********************************************");
$display("***  Pack 16 Test                         ***");
$display("*********************************************\n");

debug=0;
tx_cnt = 64;
max_s = 2;
mode = 1;
s_wi = 0;
msb_first = 0;

if(mode != 1)
	$display("WARNING: mode is set to %0d, not using random data.", mode);

for(msb_first=0;msb_first<2;msb_first=msb_first + 1)
for(max_s=2;max_s<17;max_s=max_s + 2)
//for(s_wi=0;s_wi<5;s_wi=s_wi+1)	FIXED
   begin
   	repeat(20)	@(posedge clock);
	mpac_write(`CSR, 32'h80000000);
   	repeat(10)	@(posedge clock);	// Must wait 10 clock cycles after soft reset

	case(s_wi)
	   0: samp_wi = 16;
	   1: samp_wi = 18;
	   2: samp_wi = 20;
	   3: samp_wi = 24;
	   4: samp_wi = 32;
	endcase

	$display("Running I2S/TDM Pack 16 Test, sample width: %0d bits, max slots: %0d, MSB first %0d", samp_wi, max_s, msb_first);

	if(samp_wi<32)	mask = (1<<samp_wi) -1;
	else		mask = 32'hffffffff;

	i2s.msb_first = msb_first;
	// packing yields 1/2 compression
	// tx_cnt = tx_cnt * 1/2;
	for(s=0;s<max_s;s=s+1)
		istr.init(s, 1, tx_cnt /2);	// tid, mode, count

	config_mpac(	max_s-1,// Number of input channels
			max_s-1,// Number of output channels
			0,	// Mute Value
			0,	// Protocol Select
			s_wi,	// Sample size from SoC
			0,	// Sample alignment
			1,	// Samples are packed
	
			msb_first,// Serial MSB first
			s_wi,	// Serial Slot Width
			0,	// Sync start
			1,	// Sync duration
			0	// Invert Sync
		);

	i2s.init(mode, max_s, tx_cnt, samp_wi);
	i2s.wait_idle;

	repeat(100)	@(posedge clock);	// Must wait to let all transfers complete
	mpac_write(`CSR, 32'h00000000);		// Disable IP

	// Read out internal buffers
	// packing yields 1/2 compression
	// tx_cnt = tx_cnt * 1/2;
	for(s=0;s<max_s;s=s+1)
	   begin
		ostr.init(s, mode, tx_cnt /2);
		ostr.wait_idle;
	   end

	// --------------------------------------------------------
	// Check SoC to Codec Data

	for(s=0;s<max_s;s=s+1)
	  for(n=0;n<tx_cnt-1;n=n+1)	// SoC to Codec
	    begin
	
		tmp =   istr.mem[{s[3:0], 1'b0, n[istr.BUF_WI_L2-1:1]}];
		dst = i2s.in_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		src = n[0] ? {16'h0, tmp[31:16]} : {16'h0, tmp[15:0]};
	
		if(debug)	$display("BFM IN BUF[%0d][%0d]: %x", s, n, dst );
		if(debug)	$display("  ISTR BUF[%0d][%0d]: %x", s, n, tmp );
	
		if(dst !== src)
		   begin
			$display("IN Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end

	for(s=0;s<max_s;s=s+1)
	  for(n=0;n<tx_cnt;n=n+1)	// codec to SoC
	    begin
		tmp =   ostr.mem[{s[3:0], 1'b0, n[istr.BUF_WI_L2-1:1]}];
		src = i2s.out_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		dst = n[0] ? {16'h0, tmp[31:16]} : {16'h0, tmp[15:0]};

		if(debug)	$display("BFM OUT BUF[%0d][%0d]: %x", s, n, src );
		if(debug)	$display("  OSTR BUF[%0d][%0d]: %x", s, n, dst );

		if(dst !== src)
		   begin
			$display("OUT Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end

	    end
	repeat(10)	@(posedge clock);
   end


show_errors;
$display("*********************************************");
$display("*** Test DONE ...                         ***");
$display("*********************************************\n\n");
end
endtask





task test_pack_18;

reg	[31:0]  tmp;
reg	[31:0]  st;
integer		s, n, max_s, tx_cnt, mode, s_wi, samp_wi;
reg	[31:0]	src, dst, mask;
integer		debug, i, msb_first;

begin
$display("\n");
$display("*********************************************");
$display("***  Pack 18 Test                         ***");
$display("*********************************************\n");

debug=0;
tx_cnt = 144;
max_s = 2;
mode = 1;
s_wi = 1;
msb_first = 0;

if(mode != 1)
	$display("WARNING: mode is set to %0d, not using random data.", mode);

for(msb_first=0;msb_first<2;msb_first=msb_first + 1)
for(max_s=2;max_s<17;max_s=max_s + 2)
//for(s_wi=0;s_wi<5;s_wi=s_wi+1)	FIXED !!!
   begin
   	repeat(20)	@(posedge clock);
	mpac_write(`CSR, 32'h80000000);
   	repeat(10)	@(posedge clock);	// Must wait 10 clock cycles after soft reset

	case(s_wi)
	   0: samp_wi = 16;
	   1: samp_wi = 18;
	   2: samp_wi = 20;
	   3: samp_wi = 24;
	   4: samp_wi = 32;
	endcase

	$display("Running I2S/TDM Pack 18 Test, sample width: %0d bits, max slots: %0d, MSB first %0d", samp_wi, max_s, msb_first);

	if(samp_wi<32)	mask = (1<<samp_wi) -1;
	else		mask = 32'hffffffff;

	i2s.msb_first = msb_first;
	// packing yields 9/16 compression
	// tx_cnt = tx_cnt * 9/16;
	for(s=0;s<max_s;s=s+1)
		istr.init(s, 1, tx_cnt * 9/16);	// tid, mode, count

	config_mpac(	max_s-1,// Number of input channels
			max_s-1,// Number of output channels
			0,	// Mute Value
			0,	// Protocol Select
			s_wi,	// Sample size from SoC
			0,	// Sample alignment
			1,	// Samples are packed
	
			msb_first,// Serial MSB first
			s_wi,	// Serial Slot Width
			0,	// Sync start
			1,	// Sync duration
			0	// Invert Sync
		);

	i2s.init(mode, max_s, tx_cnt, samp_wi);
	i2s.wait_idle;

	repeat(100)	@(posedge clock);	// Must wait to let all transfers complete
	mpac_write(`CSR, 32'h00000000);		// Disable IP

	// Read out internal buffers
	// packing yields 9/16 compression
	// tx_cnt = tx_cnt * 9/16;
	for(s=0;s<max_s;s=s+1)
	   begin
		ostr.init(s, mode, tx_cnt * 9/16);
		ostr.wait_idle;
	   end

	repeat(20)	@(posedge clock);	// Must wait to let all transfers complete
	// --------------------------------------------------------
	// Check SoC to Codec Data

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	begin
	  i=0;
	  for(n=0;n<tx_cnt-1;n=n+1)	// SoC to Codec
	    begin
	
		src = i2s.in_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;
		tmp =   istr.mem[{s[3:0],  i[istr.BUF_WI_L2-1:0]}];

		case(n[3:0])
		   0:
		     begin
			dst = tmp[17:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[3:0], st[31:18]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = st[21:4];
		     end

		   3:
		     begin
			dst = {tmp[7:0], st[31:22]};
			st = tmp;
			i = i + 1;
		     end

		   4:
		     begin
			dst = st[25:8];
		     end

		   5:
		     begin
			dst = {tmp[11:0], st[31:26]};
			st = tmp;
			i = i + 1;
		     end

		   6:
		     begin
			dst = st[29:12];
		     end

		   7:
		     begin
			dst = {tmp[15:0], st[31:30]};
			st = tmp;
			i = i + 1;
		     end

		   8:
		     begin
			dst = {tmp[1:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   9:
		     begin
			dst = st[19:2];
		     end

		   10:
		     begin
			dst = {tmp[5:0], st[31:20]};
			st = tmp;
			i = i + 1;
		     end

		   11:
		     begin
			dst = st[23:6];
		     end

		   12:
		     begin
			dst = {tmp[9:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   13:
		     begin
			dst = st[27:10];
		     end

		   14:
		     begin
			dst = {tmp[13:0], st[31:28]};
			st = tmp;
			i = i + 1;
		     end

		   15:
		     begin
			dst = st[31:14];
		     end
		endcase

		if(debug)	$display("BFM IN BUF[%0d][%0d]: %x", s, n, dst );
		if(debug)	$display("  ISTR BUF[%0d][%0d]: %x", s, n, src );
	
		if(dst !== src)
		   begin
			$display("IN Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end
	  end

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	  begin
	  i=0;
	  for(n=0;n<tx_cnt;n=n+1)	// codec to SoC
	    begin

		tmp =   ostr.mem[{s[3:0],  i[ostr.BUF_WI_L2-1:0]}];

		case(n[3:0])
		   0:
		     begin
			dst = tmp[17:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[3:0], st[31:18]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = st[21:4];
		     end

		   3:
		     begin
			dst = {tmp[7:0], st[31:22]};
			st = tmp;
			i = i + 1;
		     end

		   4:
		     begin
			dst = st[25:8];
		     end

		   5:
		     begin
			dst = {tmp[11:0], st[31:26]};
			st = tmp;
			i = i + 1;
		     end

		   6:
		     begin
			dst = st[29:12];
		     end

		   7:
		     begin
			dst = {tmp[15:0], st[31:30]};
			st = tmp;
			i = i + 1;
		     end

		   8:
		     begin
			dst = {tmp[1:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   9:
		     begin
			dst = st[19:2];
		     end

		   10:
		     begin
			dst = {tmp[5:0], st[31:20]};
			st = tmp;
			i = i + 1;
		     end

		   11:
		     begin
			dst = st[23:6];
		     end

		   12:
		     begin
			dst = {tmp[9:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   13:
		     begin
			dst = st[27:10];
		     end

		   14:
		     begin
			dst = {tmp[13:0], st[31:28]};
			st = tmp;
			i = i + 1;
		     end

		   15:
		     begin
			dst = st[31:14];
		     end
		endcase

		src = i2s.out_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		if(debug)	$display("BFM OUT BUF[%0d][%0d]: %x", s, n, src );
		if(debug)	$display("  OSTR BUF[%0d][%0d]: %x", s, n, dst );

		if(dst !== src)
		   begin
			$display("OUT Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	     end
	   end

	repeat(10)	@(posedge clock);
   end

show_errors;
$display("*********************************************");
$display("*** Test DONE ...                         ***");
$display("*********************************************\n\n");
end
endtask


task test_pack_20;

reg	[31:0]  tmp;
reg	[31:0]  st;
integer		s, n, max_s, tx_cnt, mode, s_wi, samp_wi;
reg	[31:0]	src, dst, mask;
integer		debug, i, msb_first;

begin
$display("\n");
$display("*********************************************");
$display("***  Pack 20 Test                         ***");
$display("*********************************************\n");

debug=0;
tx_cnt = 144;
max_s = 2;
mode = 1;
s_wi = 2;
msb_first = 0;

if(mode != 1)
	$display("WARNING: mode is set to %0d, not using random data.", mode);

for(msb_first=0;msb_first<2;msb_first=msb_first + 1)
for(max_s=2;max_s<17;max_s=max_s + 2)
//for(s_wi=0;s_wi<5;s_wi=s_wi+1)	FIXED !!!
   begin
   	repeat(10)	@(posedge clock);
	mpac_write(`CSR, 32'h80000000);
   	repeat(10)	@(posedge clock);	// Must wait 10 clock cycles after soft reset

	case(s_wi)
	   0: samp_wi = 16;
	   1: samp_wi = 18;
	   2: samp_wi = 20;
	   3: samp_wi = 24;
	   4: samp_wi = 32;
	endcase

	$display("Running I2S/TDM Pack 20 Test, sample width: %0d bits, max slots: %0d, MSB first %0d", samp_wi, max_s, msb_first);

	if(samp_wi<32)	mask = (1<<samp_wi) -1;
	else		mask = 32'hffffffff;

	i2s.msb_first = msb_first;
	// packing yields 5/8 compression
	// tx_cnt = tx_cnt * 5/8
	for(s=0;s<max_s;s=s+1)
		istr.init(s, mode, tx_cnt * 5/8);	// tid, mode, count

	config_mpac(	max_s-1,// Number of input channels
			max_s-1,// Number of output channels
			0,	// Mute Value
			0,	// Protocol Select
			s_wi,	// Sample size from SoC
			0,	// Sample alignment
			1,	// Samples are packed

			msb_first,// Serial MSB first
			s_wi,	// Serial Slot Width
			0,	// Sync start
			1,	// Sync duration
			0	// Invert Sync
		);

	i2s.init(mode, max_s, tx_cnt, samp_wi);
	i2s.wait_idle;

	repeat(100)	@(posedge clock);	// Must wait to let all transfers complete
	mpac_write(`CSR, 32'h00000000);		// Disable IP

	// Read out internal buffers
	// packing yields 5/8 compression
	// tx_cnt = tx_cnt * 5/8
	for(s=0;s<max_s;s=s+1)
	   begin
		ostr.init(s, mode, tx_cnt * 5/8);
		ostr.wait_idle;
	   end

	repeat(20)	@(posedge clock);	// Must wait to let all transfers complete
	// --------------------------------------------------------
	// Check SoC to Codec Data

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	  begin
	  i=0;
	  for(n=0;n<tx_cnt-1;n=n+1)	// SoC to Codec
	    begin

		tmp =   istr.mem[{s[3:0],  i[istr.BUF_WI_L2-1:0]}];

		case(n[2:0])
		   0:
		     begin
			dst = tmp[19:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[7:0], st[31:20]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = st[27:8];
		     end

		   3:
		     begin
			dst = {tmp[15:0], st[31:28]};
			st = tmp;
			i = i + 1;
		     end

		   4:
		     begin
			dst = {tmp[3:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   5:
		     begin
			dst = st[23:4];
		     end

		   6:
		     begin
			dst = {tmp[11:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   7:
		     begin
			dst = st[31:12];
		     end

		endcase

		src = i2s.in_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		if(debug)	$display("BFM IN BUF[%0d][%0d]: %x", s, n, dst );
		if(debug)	$display("  ISTR BUF[%0d][%0d]: %x", s, n, src );
	
		if(dst !== src)
		   begin
			$display("IN Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end
	  end

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	  begin
	  i=0;
	  for(n=0;n<tx_cnt;n=n+1)	// codec to SoC
	    begin

		tmp =   ostr.mem[{s[3:0],  i[ostr.BUF_WI_L2-1:0]}];

		case(n[2:0])
		   0:
		     begin
			dst = tmp[19:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[7:0], st[31:20]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = st[27:8];
		     end

		   3:
		     begin
			dst = {tmp[15:0], st[31:28]};
			st = tmp;
			i = i + 1;
		     end

		   4:
		     begin
			dst = {tmp[3:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   5:
		     begin
			dst = st[23:4];
		     end

		   6:
		     begin
			dst = {tmp[11:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   7:
		     begin
			dst = st[31:12];
		     end

		endcase

		src = i2s.out_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		if(debug)	$display("BFM OUT BUF[%0d][%0d]: %x", s, n, src );
		if(debug)	$display("  OSTR BUF[%0d][%0d]: %x", s, n, dst );

		if(dst !== src)
		   begin
			$display("OUT Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end
	repeat(10)	@(posedge clock);
   end
 end

show_errors;
$display("*********************************************");
$display("*** Test DONE ...                         ***");
$display("*********************************************\n\n");
end
endtask




task test_pack_24;

reg	[31:0]  tmp;
reg	[31:0]  st;
integer		s, n, max_s, tx_cnt, mode, s_wi, samp_wi;
reg	[31:0]	src, dst, mask;
integer		debug, i, msb_first;

begin
$display("\n");
$display("*********************************************");
$display("***  Pack 24 Test                         ***");
$display("*********************************************\n");

debug=0;
tx_cnt = 144;
max_s = 2;
mode = 1;
s_wi = 3;
msb_first = 0;

if(mode != 1)
	$display("WARNING: mode is set to %0d, not using random data.", mode);

for(msb_first=0;msb_first<2;msb_first=msb_first + 1)
for(max_s=2;max_s<17;max_s=max_s + 2)
//for(s_wi=0;s_wi<5;s_wi=s_wi+1)	FIXED !!!
   begin
   	repeat(10)	@(posedge clock);
	mpac_write(`CSR, 32'h80000000);
   	repeat(10)	@(posedge clock);	// Must wait 10 clock cycles after soft reset

	case(s_wi)
	   0: samp_wi = 16;
	   1: samp_wi = 18;
	   2: samp_wi = 20;
	   3: samp_wi = 24;
	   4: samp_wi = 32;
	endcase

	$display("Running I2S/TDM Pack 24 Test, sample width: %0d bits, max slots: %0d, MSB first %0d", samp_wi, max_s, msb_first);

	if(samp_wi<32)	mask = (1<<samp_wi) -1;
	else		mask = 32'hffffffff;

	i2s.msb_first = msb_first;
	// packing yields 3/4 compression
	// tx_cnt = tx_cnt * 3/4
	for(s=0;s<max_s;s=s+1)
		istr.init(s, mode, tx_cnt * 3/4);	// tid, mode, count

	config_mpac(	max_s-1,// Number of input channels
			max_s-1,// Number of output channels
			0,	// Mute Value
			0,	// Protocol Select
			s_wi,	// Sample size from SoC
			0,	// Sample alignment
			1,	// Samples are packed
	
			msb_first,// Serial MSB first
			s_wi,	// Serial Slot Width
			0,	// Sync start
			1,	// Sync duration
			0	// Invert Sync
		);

	i2s.init(mode, max_s, tx_cnt, samp_wi);
	i2s.wait_idle;

	repeat(100)	@(posedge clock);	// Must wait to let all transfers complete
	mpac_write(`CSR, 32'h00000000);		// Disable IP

	// Read out internal buffers
	// packing yields 3/4 compression
	// tx_cnt = tx_cnt * 3/4
	for(s=0;s<max_s;s=s+1)
	   begin
		ostr.init(s, mode, tx_cnt * 3/4);
		ostr.wait_idle;
	   end

	repeat(20)	@(posedge clock);	// Must wait to let all transfers complete
	// --------------------------------------------------------
	// Check SoC to Codec Data

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	  begin
	  i=0;
	  for(n=0;n<tx_cnt-1;n=n+1)	// SoC to Codec
	    begin

		src = i2s.in_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;
		tmp =   istr.mem[{s[3:0],  i[istr.BUF_WI_L2-1:0]}];

		case(n[1:0])
		   0:
		     begin
			dst = tmp[23:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[15:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = {tmp[7:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   3:
		     begin
			dst = st[31:8];
		     end

		endcase

		if(debug)	$display("BFM IN BUF[%0d][%0d]: %x", s, n, dst );
		if(debug)	$display("  ISTR BUF[%0d][%0d]: %x", s, n, src );

		if(dst !== src)
		   begin
			$display("IN Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end
	  end

	i=0;
	s=0;
	for(s=0;s<max_s;s=s+1)
	  begin
	  i=0;
	  for(n=0;n<tx_cnt;n=n+1)	// codec to SoC
	    begin

		tmp =   ostr.mem[{s[3:0],  i[ostr.BUF_WI_L2-1:0]}];

		case(n[1:0])
		   0:
		     begin
			dst = tmp[23:0];
			st = tmp;
			i = i + 1;
		     end

		   1:
		     begin
			dst = {tmp[15:0], st[31:24]};
			st = tmp;
			i = i + 1;
		     end

		   2:
		     begin
			dst = {tmp[7:0], st[31:16]};
			st = tmp;
			i = i + 1;
		     end

		   3:
		     begin
			dst = st[31:8];
		     end

		endcase

		src = i2s.out_buf[{s[3:0], n[ i2s.BUF_WI_L2-1:0]}] & mask;

		if(debug)	$display("BFM OUT BUF[%0d][%0d]: %x", s, n, src );
		if(debug)	$display("  OSTR BUF[%0d][%0d]: %x", s, n, dst );

		if(dst !== src)
		   begin
			$display("OUT Slot %0d, Word %0d Mismatch. Expected: %x, Got: %x",
					s, n, src, dst );
			error_count = error_count + 1;
		   end
	    end
	repeat(10)	@(posedge clock);
   end
 end

show_errors;
$display("*********************************************");
$display("*** Test DONE ...                         ***");
$display("*********************************************\n\n");
end
endtask


