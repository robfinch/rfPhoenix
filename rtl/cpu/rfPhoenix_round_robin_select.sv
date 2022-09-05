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

Tid prev;
Tid sel1;
Tid sel2;
Tid sel3;
Tid sel4;
Tid sel5;
Tid sel6;
Tid sel7;
Tid sel0;

/*
always_comb
	sel1 = prev + 3'd1;
always_comb
	sel2 = prev + 3'd2;
always_comb
	sel3 = prev + 3'd3;
always_comb
	sel4 = prev + 3'd4;
always_comb
	sel5 = prev + 3'd5;
always_comb
	sel6 = prev + 3'd6;
always_comb
	sel7 = prev + 3'd7;
*/
reg [2:0] amt;
reg [15:0] ishift;
reg [7:0] irot;
wire [3:0] ffoo;

always_comb
	ishift = i << amt;
always_comb
	irot = ishift[15:8]|ishift[7:0];
ffo12 uffo1 (.i({4'd0,irot}), .o(ffoo));

always_ff @(posedge clk)
	o = ffoo[2:0] - amt;
always_ff @(posedge clk)
	ov = ffoo!=4'd15;

always_ff @(posedge clk)
if (rst)
	amt = 'd0;
else begin
	if (ffoo!=4'd15)
		amt <= amt + 3'd1;
end

/*
task tAdd;
begin
	sel0 <= sel0 + 3'd1;
	sel1 <= sel1 + 3'd1;
	sel2 <= sel2 + 3'd1;
	sel3 <= sel3 + 3'd1;
	sel4 <= sel4 + 3'd1;
	sel5 <= sel5 + 3'd1;
	sel6 <= sel6 + 3'd1;
	sel7 <= sel7 + 3'd1;
end
endtask

always_ff @(posedge clk)
if (rst) begin
	prev <= 'd0;
	o <= 'd0;
	ov <= 'd0;
	sel0 <= 3'd0;
	sel1 <= 3'd1;
	sel2 <= 3'd2;
	sel3 <= 3'd3;
	sel4 <= 3'd4;
	sel5 <= 3'd5;
	sel6 <= 3'd6;
	sel7 <= 3'd7;
end
else begin
	if (i[sel1]) begin
		tAdd();
		prev <= o;
		o <= sel1;
		ov <= 1'b1;
	end
	else if (i[sel2]) begin
		tAdd();
		prev <= o;
		o <= sel2;
		ov <= 1'b1;
	end
	else if (i[sel3]) begin
		tAdd();
		prev <= o;
		o <= sel3;
		ov <= 1'b1;
	end
	else if (i[sel4]) begin
		tAdd();
		prev <= o;
		o <= sel4;
		ov <= 1'b1;
	end
	else if (i[sel5]) begin
		tAdd();
		prev <= o;
		o <= sel5;
		ov <= 1'b1;
	end
	else if (i[sel6]) begin
		tAdd();
		prev <= o;
		o <= sel6;
		ov <= 1'b1;
	end
	else if (i[sel7]) begin
		tAdd();
		prev <= o;
		o <= sel7;
		ov <= 1'b1;
	end
	
	else if (i[sel0]) begin
		prev <= o;
		o <= prev;
		ov <= 1'b1;
	end
	
	else begin
		o <= 'd0;
		ov <= 1'b0;
	end
end
*/
endmodule
