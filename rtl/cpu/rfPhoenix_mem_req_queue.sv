// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_mem_req_queue.sv
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

module rfPhoenix_mem_req_queue(rst, clk, wr0, wr_ack0, i0, wr1, wr_ack1, i1,
	rd, o, valid, empty, ldo0, found0, ldo1, found1, full,
	rollback, rollback_bitmaps);
parameter AWID = 32;
parameter QDEP = 8;
input rst;
input clk;
input wr0;
output reg wr_ack0;
input memory_arg_t i0;
input wr1;
output reg wr_ack1;
input memory_arg_t i1;
input rd;
output memory_arg_t o;
output reg valid;
output reg empty;
output memory_arg_t ldo0;
output reg found0;
output memory_arg_t ldo1;
output reg found1;
output reg full;
input [NTHREADS-1:0] rollback;
output reg [127:0] rollback_bitmaps [0:NTHREADS-1];

integer n3, n5, n6;
reg [4:0] qndx = 'd0;
memory_arg_t [QDEP-1:0] que;
reg [QDEP-1:0] valid_bits = 'd0;
reg [63:0] isel0, isel1;
reg [63:0] qsel [0:QDEP-1];
reg [255:0] imask0, imask1;
reg [255:0] dat10, dat11;
reg [31:0] sx0, sx1;
reg [7:0] last_tid;
reg overlapping_store;

initial begin
	for (n5 = 0; n5 < QDEP; n5 = n5 + 1) begin
		que[n5] = 'd0;
	end
end

// Align select lines.
reg [31:0] i0_sel, i1_sel;
function [31:0] fnSel;
input [2:0] sz;
case(sz)
byt:	fnSel = 32'h00000001;
wyde:	fnSel = 32'h00000003;
tetra:	fnSel = 32'h0000000F;
octa:	fnSel = 32'h000000FF;
//hexi:	fnSel = 32'h0000FFFF;
//hexipair:	fnSel = 32'hFFFFFFFF;
default:	fnSel = 32'h000000FF;
endcase
endfunction

always_comb
	i0_sel = fnSel(i0.sz);
always_comb
	i1_sel = fnSel(i1.sz);

always_comb
	isel0 = i0_sel << i0.adr[3:0];
always_comb
	isel1 = i1_sel << i1.adr[3:0];

// Generate a mask for the load data.

genvar g1;
generate begin
for (g1 = 0; g1 < 32; g1 = g1 + 1)
begin
	always_comb
		if (i0_sel[g1])
			imask0[g1*8+7:g1*8] = 8'hFF;
		else
			imask0[g1*8+7:g1*8] = 8'h00;
	always_comb
		if (i0_sel[g1])
			sx0[g1] = dat10[g1*8+7];
		else
			sx0[g1] = 1'b0;
	always_comb
		if (i1_sel[g1])
			imask1[g1*8+7:g1*8] = 8'hFF;
		else
			imask1[g1*8+7:g1*8] = 8'h00;
	always_comb
		if (i1_sel[g1])
			sx1[g1] = dat11[g1*8+7];
		else
			sx1[g1] = 1'b0;
end
end
endgenerate

// Search the queue for a matching store. If more than one store matches the
// most recently added store is the chosen one.

reg foundst0, foundst1;
always_comb
begin
	tSearch(MR_LOAD,MR_LOADZ,i0,isel0,sx0,imask0,ldo0,found0);
	tSearch(MR_LOAD,MR_LOADZ,i1,isel1,sx1,imask1,ldo1,found1);
	tSearch(MR_STORE,MR_STORE,i0,isel0,sx0,imask0,ldo0,foundst0);
	tSearch(MR_STORE,MR_STORE,i1,isel1,sx1,imask1,ldo1,foundst1);
end

task tSearch;
input [3:0] func1;
input [3:0] func2;
input memory_arg_t i;
input [63:0] isel;
input [31:0] sx;
input [255:0] imask;
output memory_arg_t ldo;
output found;
integer n2;
reg [255:0] dat1;
begin
	ldo = i;
	found = 1'b0;
	if (i.func==func1 || i.func==func2) begin
		for (n2 = 0; n2 < QDEP; n2 = n2 + 1) begin
			if (i.adr[AWID-1:4]==que[n2].adr[AWID-1:4] && valid_bits[n2]) begin
				if ((isel & qsel[n2])==isel) begin
					found = 1'b1;
					// Align the data with the load address
					if (i.adr > que[n2].adr)
						dat1 = que[n2].res >> {i.adr - que[n2].adr,3'b0};
					else
						dat1 = que[n2].res << {que[n2].adr - i.adr,3'b0};
					// For a LOAD sign extend value to machine width.
					if (i.func==MR_LOAD) begin
						ldo.res = /*|sx ? (dat1 & imask) | ~imask :*/ (dat1 & imask);
						case(i.sz)
						byt:	ldo.res = {{32{ldo.res[7]}},ldo.res[7:0]};
						wyde:	ldo.res = {{16{ldo.res[15]}},ldo.res[15:0]};
						tetra:ldo.res = ldo.res[31:0];
						default:	ldo.res = ldo.res[31:0];
						endcase
					end
					else
						ldo.res = dat1 & imask;
				end
			end
		end
	end
end
endtask

/* under construction */
/*
task tMergeStore2;
input memory_arg_t2 i1;
input memory_arg_t2 i2;
output memory_arg_t2 o;
reg [31:0] sel1, sel2;
integer g1, g2;
begin
	sel1 = i1.sel << i1.adr[4:0];
	sel2 = i2.sel << i2.adr[4:0];
	o.dat = 'd0;
	for (g1 = 0; g1 < 32; g1 = g1 + 1) begin
		g2 = g1 + i2.adr[4:0];
		if (i2.sel[g1])
			o.dat[g2] = i2.dat[g1];
		g2 = g1 + i1.adr[4:0];
		if (i1.sel[g1])
			o.dat[g2] = i1.dat[g1];
	end
	o.sel = sel1 | sel2;
end
endtask

task tMergeStore;
integer n;
begin
	for (n = 0; n < QDEP; n = n + 1) begin
		if (que[n].adr[AWID-1:5]==i.adr[AWID-1:5] && i.func==MR_STORE && que[n].func==MR_STORE)
			tMergeStore2(i, que[n], que[n]);
	end
end
endtask
*/

/*
always_comb
begin
	overlapping_store = 1'b0;
	for (n6 = 0; n6 < QDEP; n6 = n6 + 1)
		if (que[n6].adr[AWID-1:5]==i0.adr[AWID-1:5] && i0.func==MR_STORE && que[n6].func==MR_STORE)
			overlapping_store = 1'b1;
end
*/
always_comb
	overlapping_store = 1'b0;

always_ff @(posedge clk, posedge rst)
if (rst) begin
	valid_bits <= 'd0;
	wr_ack0 <= 1'b0;
	wr_ack1 <= 1'b0;
	last_tid <= 8'd255;
	qndx <= 'd0;
	for (n3 = 0; n3 < NTHREADS; n3 = n3 + 1)
		rollback_bitmaps[n3] <= 'd0;
end
else begin
	wr_ack0 <= 1'b0;
	wr_ack1 <= 1'b0;
//	o <= que[0];
	if (wr0 && found0)
		wr_ack0 <= 1'b1;
	if (wr1 && found1)
		wr_ack1 <= 1'b1;
	// Port #0 take precedence.
	if ((rd & ~empty) & (wr0 & ~overlapping_store) & !foundst0) begin
		for (n3 = 1; n3 < QDEP; n3 = n3 + 1) begin
			que[n3-1] <= que[n3];
			qsel[n3-1] <= qsel[n3];
			valid_bits[n3-1] <= valid_bits[n3];
		end
		valid_bits[QDEP-1] <= 1'b0;
		wr_ack0 <= 1'b1;
		if (last_tid != i0.tid) begin
			rollback_bitmaps[que[0].thread][que[0].tgt] <= 1'b0;
			rollback_bitmaps[i0.thread][i0.tgt] <= 1'b1;
			que[qndx-1] <= i0;
			qsel[qndx-1] <= fnSel(i0.sz) << i0.adr[3:0];
			last_tid <= i0.tid;
			valid_bits[qndx-1] <= 1'b1;
		end
		else
			qndx <= qndx - 2'd1;
	end
	else if ((rd & ~empty) & wr1 & !foundst1) begin
		for (n3 = 1; n3 < QDEP; n3 = n3 + 1) begin
			que[n3-1] <= que[n3];
			qsel[n3-1] <= qsel[n3];
			valid_bits[n3-1] <= valid_bits[n3];
		end
		valid_bits[QDEP-1] <= 1'b0;
		wr_ack1 <= 1'b1;
		if (last_tid != i1.tid) begin
			rollback_bitmaps[que[0].thread][que[0].tgt] <= 1'b0;
			rollback_bitmaps[i1.thread][i1.tgt] <= 1'b1;
			que[qndx-1] <= i1;
			qsel[qndx-1] <= fnSel(i1.sz) << i1.adr[3:0];
			valid_bits[qndx-1] <= 1'b1;
			last_tid <= i1.tid;
		end
		else
			qndx <= qndx - 2'd1;
	end
	else if (wr0 && !overlapping_store && !foundst0) begin
		if (qndx < QDEP) begin
			if (last_tid != i0.tid) begin
				rollback_bitmaps[i0.thread][i0.tgt] <= 1'b1;
				que[qndx] <= i0;
				qsel[qndx] <= fnSel(i0.sz) << i0.adr[3:0];
				valid_bits[qndx] <= 1'b1;
				qndx <= qndx + 2'd1;
				last_tid <= i0.tid;
			end
			wr_ack0 <= 1'b1;
		end
	end
	else if (wr1 & !foundst1) begin
		if (qndx < QDEP) begin
			if (last_tid != i1.tid) begin
				rollback_bitmaps[i1.thread][i1.tgt] <= 1'b1;
				que[qndx] <= i1;
				qsel[qndx] <= fnSel(i1.sz) << i1.adr[3:0];
				valid_bits[qndx] <= 1'b1;
				qndx <= qndx + 2'd1;
				last_tid <= i1.tid;
			end
			wr_ack1 <= 1'b1;
		end
	end
	else if (rd & ~empty) begin
		if (|qndx)
			qndx <= qndx - 2'd1;
		for (n3 = 1; n3 < QDEP; n3 = n3 + 1) begin
			que[n3-1] <= que[n3];
			qsel[n3-1] <= qsel[n3];
			valid_bits[n3-1] <= valid_bits[n3];
			rollback_bitmaps[que[0].thread][que[0].tgt] <= 1'b0;
		end
		valid_bits[QDEP-1] <= 1'b0;
	end
	if (|rollback) begin
		for (n3 = 0; n3 < QDEP; n3 = n3 + 1)
			if (rollback[que[n3].thread]) begin
				que[n3].v <= 1'b0;
				rollback_bitmaps[que[n3].thread] <= 'd0;
			end
	end
end

always_comb
	empty = ~|valid_bits;
always_comb
	o = que[0];
always_comb
	valid = valid_bits[0];
always_comb
	full = qndx==QDEP-1 || overlapping_store;

endmodule
