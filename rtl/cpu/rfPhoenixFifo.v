`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
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

import rfPhoenixPkg::*;

module rfPhoenixFifo(wrst, wclk, wr, di, rrst, rclk, rd, dout, cnt);
parameter WID=128;
input wrst;
input wclk;
input wr;
input [WID-1:0] di;
input rrst;
input rclk;
input rd;
output [WID-1:0] dout;
output [7:0] cnt;
reg [7:0] cnt;

reg [7:0] wr_ptr;
reg [7:0] rd_ptr,rrd_ptr;
reg [WID-1:0] mem [0:255];

wire [7:0] wr_ptr_p1 = wr_ptr + 8'd1;
wire [7:0] rd_ptr_p1 = rd_ptr + 8'd1;
reg [7:0] rd_ptrs;

always @(posedge wclk)
	if (wrst)
		wr_ptr <= 8'd0;
	else if (wr) begin
		mem[wr_ptr] <= di;
		wr_ptr <= wr_ptr_p1;
	end
always @(posedge wclk)		// synchronize read pointer to wclk domain
	rd_ptrs <= rd_ptr;

always @(posedge rclk)
	if (rrst)
		rd_ptr <= 8'd0;
	else if (rd)
		rd_ptr <= rd_ptr_p1;
always @(posedge rclk)
	rrd_ptr <= rd_ptr;

assign dout = mem[rrd_ptr[7:0]];

always @(wr_ptr or rd_ptrs)
	if (rd_ptrs > wr_ptr)
		cnt <= wr_ptr + (9'd256 - rd_ptrs);
	else
		cnt <= wr_ptr - rd_ptrs;

endmodule
