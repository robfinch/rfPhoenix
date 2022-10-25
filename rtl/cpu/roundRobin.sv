`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2005-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
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
//
module roundRobin(rst, clk, ce, req, lock, sel, sel_enc);
parameter N = 8;
input rst;				// reset
input clk;				// clock
input ce;				// clock enable
input [N-1:0] req;		// request
input [N-1:0] lock;		// lock selection
output reg [N-1:0] sel;		// select, one hot
output reg [$clog2(N):0] sel_enc;	// select, encoded

integer n;
reg [N-1:0] nextGrant;	// unrotated value of grant
reg [N*2-1:0] rgrnts;
reg [N*2-1:0] reqs;
reg [N*2-1:0] sels;
reg [N-1:0] base;

always_comb
	reqs = {req,req};
always_comb
	rgrnts = reqs & ~(reqs-base);
// nextGrant should be one-hot
always_comb
	nextGrant = {rgrnts[N*2-1:N]|rgrnts[N-1:0]};

always_ff @(posedge clk)
	if (rst)
		base <= 2'd1;
	else begin
		if ((lock & sel)=='d0)
			base <= {base[N-2:0],base[N-1]};
	end

// Assign the next owner, if isn't locked
always_ff @(posedge clk)
	if (rst)
		sel <= 'h0;
	else if (ce)
		if ((lock & sel)=='d0) begin
			sel <= nextGrant;
		end

always_ff @(posedge clk)
	if (rst)
		sel_enc <= 'd0;
	else if (ce)
		if ((lock & sel)=='d0) begin
			sel_enc <= {$clog2(N)+1{1'b1}};
			for (n = 0; n < N; n = n + 1)
				if (nextGrant[n])
					sel_enc <= n;
			end

endmodule
