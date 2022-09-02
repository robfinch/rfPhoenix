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
	rollback, rollback_thread, rollback_bitmap);
parameter WID=3;
parameter DEP=16;
input rst;
input clk;
input wr;
input MemoryResponse di;
input rd;
output MemoryResponse dout;
output reg [5:0] cnt;
output reg full;
output reg empty;
output reg v;
input rollback;
input [3:0] rollback_thread;
output reg [127:0] rollback_bitmap;

reg [127:0] rollback_bitmaps [0:NTHREADS];

reg [5:0] wr_ptr;
reg [5:0] rd_ptr;
MemoryResponse  [DEP-1:0] mem;
integer n,n2;

always_ff @(posedge clk)
	if (rst) begin
		for (n = 0; n < NTHREADS; n = n + 1)
			rollback_bitmaps[n] <= 'd0;
		wr_ptr <= 'd0;
		rd_ptr <= 'd0;
		for (n = 0; n < DEP; n = n + 1)
			mem[n] <= 'd0;		
	end
	else begin
		if (rd & wr) begin
			mem[wr_ptr] <= di;
			rollback_bitmaps[di.thread][di.tgt] <= 1'b1;
		end
		else if (wr) begin
			mem[wr_ptr] <= di;
			rollback_bitmaps[di.thread][di.tgt] <= 1'b1;
			wr_ptr <= wr_ptr + 2'd1;
		end
		else if (rd) begin
			rd_ptr <= rd_ptr + 2'd1;
			rollback_bitmaps[dout.thread][dout.tgt] <= 1'b0;
		end
		dout <= mem[rd_ptr[5:0]];
		if (rollback) begin
			for (n = 0; n < DEP; n = n + 1)
				if (mem[n].thread==rollback_thread)	
					mem[n].v <= 1'b0;
			rollback_bitmaps[rollback_thread] <= 'd0;
		end
	end
always_comb
	if (wr_ptr >= rd_ptr)
		cnt = wr_ptr - rd_ptr;
	else
		cnt = wr_ptr + (DEP - rd_ptr);

always_comb
	full = cnt==DEP-1;
always_comb
	empty = cnt=='d0;
always_comb
	v = cnt > 'd0;

always_comb
	rollback_bitmap = rollback_bitmaps[rollback_thread];

endmodule
