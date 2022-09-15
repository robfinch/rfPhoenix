// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_ictag.sv
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

module rfPhoenix_ictag(rst, clk, wr, ipo, way, rclk, ndx, tag);
parameter LINES=128;
parameter WAYS=4;
parameter AWID=32;
input rst;
input clk;
input wr;
input [AWID-1:0] ipo;
input [1:0] way;
input rclk;
input [6:0] ndx;
(* ram_style="block" *)
output [AWID-1:6] tag [0:3];

(* ram_style="block" *)
reg [AWID-1:6] tags [0:WAYS*LINES-1];
reg [6:0] rndx;

integer g,g1;
integer n,n1;

initial begin
for (g = 0; g < WAYS; g = g + 1) begin
  for (n = 0; n < LINES; n = n + 1)
    tags[g * LINES + n] = 32'd1;
end
end

always_ff @(posedge clk)
// Resetting all the tags will force implementation with FF's. Since tag values
// do not matter to synthesis it is simply omitted.
`ifdef IS_SIM
if (rst) begin
	for (g1 = 0; g1 < WAYS; g1 = g1 + 1) begin
	  for (n1 = 0; n1 < LINES; n1 = n1 + 1)
	    tags[g1 * LINES + n1] = 32'd1;
	end
end
else
`endif
begin
	if (wr)
		tags[way * LINES + ipo[12:6]] <= ipo[AWID-1:6];
end

always_ff @(posedge rclk)
	rndx <= ndx;
assign tag[0] = tags[{2'b00,rndx}];
assign tag[1] = tags[{2'b01,rndx}];
assign tag[2] = tags[{2'b10,rndx}];
assign tag[3] = tags[{2'b11,rndx}];

endmodule
