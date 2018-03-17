/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Multi Protocol Audio Controller (MPAC)                     ////
////  Configuration & defines include file                       ////
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


// ==============================================================
// Make sure we are included only once
`ifndef MPAC_CONFIG_INC
`define MPAC_CONFIG_INC


`timescale 1ns / 100ps
`default_nettype none		// Enfoce clean code


`define VER	12'h001

// ==============================================================
// Configuration

/* MPAC_ASYNC_RESET
Per default the MPAC core implements synchronous reset for all flip flops.
Define MPAC_ASYNC_RESET if asynchronous reset is needed.
*/
`define MPAC_ASYNC_RESET

/* MPAC_RESET_ACTIVE_HIGH
Per default the MPAC core implements an active low reset level.
Define MPAC_RESET_ACTIVE_HIGH if active high reset is needed.
*/
`define MPAC_RESET_ACTIVE_HIGH


/* MPAC_HAVE_TIMER
Define MPAC_HAVE_TIMER to enable a simple timer implemented in hardware
The timer can be used to generate interrupts at fixed intervals.
*/
`define MPAC_HAVE_TIMER


// ==============================================================
// Definitions



// ---------------------------------------------
// Reset Specification
// ---------------------------------------------
`ifdef MPAC_RESET_ACTIVE_HIGH
`define MPAC_ACR 1'b1
`else
`define MPAC_ACR 1'b0
`endif

`ifdef MPAC_ASYNC_RESET
`ifdef MPAC_RESET_ACTIVE_HIGH
`define MPAC_ACRT or posedge rst_i
`else
`define MPAC_ACRT or negedge rst_i
`endif
`else
`define MPAC_ACRT 	// no trigger
`endif


// ---------------------------------------------
// Miasc Macros
// ---------------------------------------------
`define RGS(a,b)		(((b+1)*a)-1):(b*a)
`define RG(a,b)			a*b+:a

`define ZERO(WIDTH)		{WIDTH{1'b0}}
`define ONE(WIDTH)		({{WIDTH-1{1'b0}},1'b1})

// ---------------------------------------------
// Synthesis Directives
// ---------------------------------------------

// For registers to be debugged with chipscope or similar
`define MPAC_DBG    		(* mark_debug = "TRUE", dont_touch = "TRUE" *)

// Registers that are part of CDC
`define MPAC_ASYNCH_REG		(* shreg_extract = "no", async_reg = "true" *)

// To force State encoding to "GRAY"
`define MPAC_GRAY		(* fsm_encoding = "gray" *)

// To force State encoding to "GRAY" on registers that are part of CDC
`define MPAC_GRAY_ASYC		(* fsm_encoding = "gray", async_reg = "true"

// ---------------------------------------------
// AXI Response Codes
// ---------------------------------------------
`define AXL_RESP_OK     2'h0    // Normal access okay indicates if a normal access has been
                                // successful. Can also indicate an exclusive access failure.
`define AXL_RESP_XOK    2'h1    // Exclusive access okay indicates that either the read or
                                // write portion of an exclusive access has been successful.
`define AXL_RESP_SERR   2'h2    // Slave error is used when the access has reached the slave
                                // successfully, but the slave wishes to return an error
                                // condition to the originating master.
`define AXL_RESP_DERR   2'h3    // Slave error is used when the access has reached the slave
                                // successfully, but the slave wishes to return an error
                                // condition to the originating master.

// ---------------------------------------------
// TTY Coloring directives (for debugging)
// ---------------------------------------------
`define         RED             "\033[0;41m %s \033[0m"
`define         GREEN           "\033[0;42m %s \033[0m"
`define         BRAUN           "\033[0;43m %s \033[0m"
`define         BLUE            "\033[0;44m %s \033[0m"
`define         PURPLE          "\033[0;45m %s \033[0m"
`define         LBLUE           "\033[0;46m %s \033[0m"
`define         GRAY            "\033[0;47m %s \033[0m"

`define         ERR             "\033[0;41mERROR\033[0m:"


// ==============================================================
// End of File
`endif
