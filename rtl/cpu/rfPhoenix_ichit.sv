// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_ichit.sv
//
// BSD 3-Clause License
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//                                                                          
// ============================================================================

import rfPhoenixPkg::*;
import rfPhoenixMmupkg::*;

module rfPhoenix_ichit(clk, ip, tag, valid, ihit, rway, vtag, icv);
parameter LINES=128;
parameter WAYS=4;
parameter AWID=32;
input clk;
input [AWID-1:0] ip;
input [AWID-1:6] tag [0:3];
input [LINES-1:0] valid [0:WAYS-1];
output reg ihit;
output reg [1:0] rway;
output reg [AWID-7:0] vtag;	// victim tag
output reg icv;

reg [AWID-7:0] prev_vtag = 'd0;
reg [1:0] prev_rway = 'd0;
reg [WAYS-1:0] ihit1;
reg ihit2;
reg icv2, icv1;

integer k;
always_ff @(posedge clk)
begin
	for (k = 0; k < WAYS; k = k + 1)
	  ihit1[k] = tag[k[1:0]]==ip[AWID-1:6] && valid[k][ip[12:6]]==1'b1;
end

integer k1;
always_comb
begin
	icv2 = 1'b0;
	for (k1 = 0; k1 < WAYS; k1 = k1 + 1)
	  icv2 = icv2 | valid[k1][ip[12:6]]==1'b1;
end

integer n;
always_comb
begin
	rway = prev_rway;
	for (n = 0; n < WAYS; n = n + 1)	
		if (ihit1[n]) rway = n;
end

// For victim cache update
integer m;
always_comb
begin
	vtag = prev_vtag;
	for (m = 0; m < WAYS; m = m + 1)
		if (ihit1[m]) vtag = tag[m[1:0]];
end


always_ff @(posedge clk)
	prev_rway <= rway;

always_ff @(posedge clk)
	ihit = |ihit1;
//always_ff @(posedge clk)
//	ihit = #1 ihit2 & |ihit1;

always_ff @(posedge clk)
	prev_vtag <= vtag;
always_ff @(posedge clk)
	icv1 <= icv2;
always_ff @(posedge clk)
	icv <= icv1;	

endmodule
