`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_insn_fifo.sv
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
import rfPhoenixPkg::*;

module rfPhoenix_insn_fifo(rst, clk, wr, decin, ifbin, rd, decout, ifbout, cnt, almost_full, full, empty, v);
parameter DEP=16;
input rst;
input clk;
input wr;
input DecodeBus decin;
output DecodeBus decout;
input InstructionFetchbuf ifbin;
output InstructionFetchbuf ifbout;
input rd;
output reg [$clog2(DEP)-1:0] cnt = 'd0;
output reg almost_full = 'd0;
output reg full = 'd0;
output reg empty = 'd0;
output reg v = 'd0;

reg [$clog2(DEP)-1:0] wr_ptr;
reg [$clog2(DEP)-1:0] rd_ptr;
(* ram_style = "distributed" *)
DecodeBus [DEP-1:0] decmem;
(* ram_style = "distributed" *)
InstructionFetchbuf [DEP-1:0] ifbmem;
integer n;

always_ff @(posedge clk)
	if (rst) begin
		wr_ptr <= 'd0;
		rd_ptr <= 'd0;
		for (n = 0; n < DEP; n = n + 1) begin
			decmem[n] <= 'd0;
			ifbmem[n] <= 'd0;
		end
	end
	else begin
		if (rd & wr) begin
			decmem[wr_ptr] <= decin;
			ifbmem[wr_ptr] <= ifbin;
			wr_ptr <= wr_ptr + 2'd1;
			rd_ptr <= rd_ptr + 2'd1;
		end
		else if (wr) begin
			decmem[wr_ptr] <= decin;
			ifbmem[wr_ptr] <= ifbin;
			wr_ptr <= wr_ptr + 2'd1;
		end
		else if (rd) begin
			if (!empty)
				rd_ptr <= rd_ptr + 2'd1;
		end
	end
always_ff @(posedge clk)
	if (rst)
		decout <= 'd0;
	else
		decout <= decmem[rd_ptr];
always_ff @(posedge clk)
	if (rst)
		ifbout <= 'd0;
	else
		ifbout <= ifbmem[rd_ptr];
always_comb
	if (wr_ptr >= rd_ptr)
		cnt = wr_ptr - rd_ptr;
	else
		cnt = (DEP + wr_ptr) - rd_ptr;
always_comb
	almost_full = cnt > DEP - 7;
always_comb
	full = cnt==DEP-1;
always_comb
	empty = cnt=='d0;
always_comb
	v = cnt > 'd0;

endmodule
