// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_round_robin_select.sv
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

module rfPhoenix_round_robin_select(rst, clk, i, o, ov);
input rst;
input clk;
input [NTHREADS-1:0] i;
output Tid o;
output reg ov;

reg [2:0] amt;
reg [15:0] ishift;
reg [NTHREADS*2-1:0] irot;
wire [3:0] ffoo;
genvar g;

always_comb
	ishift = i << amt;
generate begin : gRot
always_comb
	irot = ishift[NTHREADS*2-1:NTHREADS]|ishift[NTHREADS-1:0];
end
endgenerate

ffo12 uffo1 (.i({12'd0,irot}), .o(ffoo));

always_ff @(posedge clk)
	o = ffoo[2:0] - amt;
always_ff @(posedge clk)
	ov = ffoo!=4'd15;

always_ff @(posedge clk)
if (rst)
	amt <= 'd0;
else begin
	if (ffoo!=4'd15) begin
		amt <= amt + 3'd1;
		if (amt >= NTHREADS-1)
			amt <= 'd0;
	end
end

endmodule
