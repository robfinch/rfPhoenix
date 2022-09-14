`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_fifo.sv
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

module rfPhoenix_mem_resp_fifo(rst, clk, wr, di, rd, dout, cnt, full, empty, v, 
	rollback, rollback_bitmaps);
parameter WID=3;
parameter DEP=16;
input rst;
input clk;
input wr;
input MemoryArg_t di;
input rd;
output MemoryArg_t dout;
output reg [$clog2(DEP)-1:0] cnt;
output reg full;
output reg empty;
output reg v;
input [NTHREADS-1:0] rollback;
output reg [127:0] rollback_bitmaps [0:NTHREADS-1];

reg [$clog2(DEP)-1:0] wr_ptr;
reg [$clog2(DEP)-1:0] rd_ptr;
(* ram_style = "distributed" *)
MemoryArg_t  [DEP-1:0] mem;
integer n,n2;

always_ff @(posedge clk, posedge rst)
	if (rst) begin
		wr_ptr <= 'd0;
		rd_ptr <= 'd0;
`ifdef IS_SIM		
		for (n = 0; n < DEP; n = n + 1)
			mem[n] <= 'd0;		
`endif
	end
	else begin
		if (rd & wr) begin
			mem[wr_ptr] <= di;
			wr_ptr <= wr_ptr + 2'd1;
			rd_ptr <= rd_ptr + 2'd1;
		end
		else if (wr) begin
			mem[wr_ptr] <= di;
			wr_ptr <= wr_ptr + 2'd1;
		end
		else if (rd) begin
			rd_ptr <= rd_ptr + 2'd1;
		end
		dout <= mem[rd_ptr];
		for (n = 0; n < DEP; n = n + 1)
			if (rollback[mem[n].thread])
				mem[n].v <= 1'b0;
	end

always_comb
	if (wr_ptr >= rd_ptr)
		cnt = wr_ptr - rd_ptr;
	else
		cnt = wr_ptr + (DEP - rd_ptr);

always_ff @(posedge clk, posedge rst)
	if (rst) begin
		for (n2 = 0; n2 < NTHREADS; n2 = n2 + 1)
			rollback_bitmaps[n2] <= 'd0;
	end
	else begin
		if (rd & wr) begin
			rollback_bitmaps[di.thread][di.tgt] <= 1'b1;
		end
		else if (wr) begin
			rollback_bitmaps[di.thread][di.tgt] <= 1'b1;
		end
		else if (rd) begin
			rollback_bitmaps[dout.thread][dout.tgt] <= 1'b0;
		end
		for (n2 = 0; n2 < NTHREADS; n2 = n2 + 1)
			if (rollback[n2])
				rollback_bitmaps[n2] <= 'd0;
	end

always_comb
	full = cnt==DEP-1;
always_comb
	empty = cnt=='d0;
always_comb
	v = cnt > 'd0;

endmodule
