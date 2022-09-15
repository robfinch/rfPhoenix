// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_icvalid.sv
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

module rfPhoenix_icvalid(rst, clk, invce, ip, adr, wr, way, invline, invall, valid);
parameter LINES=128;
parameter WAYS=4;
input rst;
input clk;
input invce;
input CodeAddress ip;
input CodeAddress adr;		// physical address
input wr;
input [1:0] way;
input invline;
input invall;
output reg [LINES-1:0] valid [0:WAYS-1];

integer n, m;
integer g;

initial begin
for (m = 0; m < WAYS; m = m + 1) begin
  for (n = 0; n < LINES; n = n + 1)
    valid[m][n] = 1'b0;
end
end

always_ff @(posedge clk, posedge rst)
if (rst) begin
	for (g = 0; g < WAYS; g = g + 1)
		valid[g] <= 'd0;
end
else begin
	if (wr)
		valid[way][ip[13:7]] <= 1'b1;
	else if (invce) begin
		for (g = 0; g < WAYS; g = g + 1) begin
			if (invline)
				valid[g][adr[13:7]] <= 1'b0;
			else if (invall)
				valid[g] <= 'd0;
		end
	end
end

endmodule

