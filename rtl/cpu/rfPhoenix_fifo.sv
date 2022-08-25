`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	Thor2022_fifo.sv
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
module Thor2022_fifo(rst, clk, wr, di, rd, dout, cnt, full);
parameter WID=3;
input rst;
input clk;
input wr;
input [WID-1:0] di;
input rd;
output reg [WID-1:0] dout;
output reg [3:0] cnt;
output reg full;

reg [3:0] wr_ptr;
reg [3:0] rd_ptr;
reg [WID-1:0] mem [0:15];
integer n;

always_ff @(posedge clk)
	if (rst) begin
		wr_ptr <= 'd0;
		rd_ptr <= 'd0;
		for (n = 0; n < 16; n = n + 1)
			mem[n] <= 'd0;		
	end
	else begin
		if (rd & wr)
			;
		else if (wr) begin
			mem[wr_ptr] <= di;
			wr_ptr <= wr_ptr + 2'd1;
		end
		else if (rd) begin
			rd_ptr <= rd_ptr + 2'd1;
		end
		dout <= mem[rd_ptr[3:0]];
	end
always_comb
	if (wr_ptr >= rd_ptr)
		cnt = wr_ptr - rd_ptr;
	else
		cnt = wr_ptr + (5'd16 - rd_ptr);

always_comb
	full = cnt==4'd15;

endmodule
