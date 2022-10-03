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

module rfPhoenixMcVecAlu(rst, clk, ir, a, b, c, imm, i, o, done, ridi, rido);
input rst;
input clk;
input instruction_t ir;
input vector_value_t a;
input vector_value_t b;
input vector_value_t c;
input value_t imm;
input pipeline_reg_t i;
output pipeline_reg_t o;
output reg done;
input tid_t ridi;
output tid_t rido;

pipeline_reg_t wbo;
vector_value_t o1;
integer n;
genvar g;
reg [NLANES-1:0] don;

vector_quad_value_t ab, bb, cb, ob1;
vector_double_value_t ao, bo, co, oo1;
vector_half_value_t aw, bw, cw, ow1;	// wydes

assign ab = a;
assign bb = b;
assign cb = c;
assign aw = a;
assign bw = b;
assign cw = c;
assign ao = a;
assign bo = b;
assign co = c;

generate begin
	for (g = 0; g < NLANES; g = g + 1)
		rfPhoenixMcAlu ualu1(
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(a[g]),
			.b(b[g]),
			.c(c[g]),
			.imm(imm),
			.o(o1[g]),
			.done(don[g]),
			.ridi('d0),
			.rido()
		);
end
endgenerate

`ifdef SUPPORT_16BIT_OPS
generate begin
	for (g = 0; g < NLANES*2; g = g + 1)
		rfPhoenixMcAlu16 ualu1(
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(aw[g]),
			.b(bw[g]),
			.c(cw[g]),
			.imm(imm[15:0]),
			.o(ow1[g]),
			.done(),
			.ridi('d0),
			.rido()
		);
end
endgenerate
`endif

`ifdef SUPPORT_64BIT_OPS
generate begin
	for (g = 0; g < NLANES/2; g = g + 1)
		rfPhoenixMcAlu64 ualu1(
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(ao[g]),
			.b(bo[g]),
			.c(co[g]),
			.imm(imm[63:0]),
			.o(oo1[g]),
			.done(),
			.ridi('d0),
			.rido()
		);
end
endgenerate
`endif

`ifdef SUPPORT_128BIT_OPS
generate begin
	for (g = 0; g < NLANES/4; g = g + 1)
		rfPhoenixMcAlu128 ualu1(
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(ab[g]),
			.b(bb[g]),
			.c(cb[g]),
			.o(ob1[g]),
			.done(),
			.ridi('d0),
			.rido()
		);
end
endgenerate
`endif

//ft_delay #(.WID(4), .DEP(7)) uftd1 (.clk(clk), .ce(1'b1), .i(ridi), .o(rido));
vtdl #(.WID($bits(pipeline_reg_t)), .DEP(16)) uvtdl1 (.clk(clk), .ce(1'b1), .a(4'd8), .d(i), .q(wbo));

always_ff @(posedge clk)
begin
	o <= wbo;
	case(wbo.dec.prc)
	PRC16:	o.res <= ow1;
	PRC64:	o.res <= oo1;
	PRC128:	o.res <= ob1;
	default:	o.res <= o1;
	endcase
end

always_comb
	done <= &don;

endmodule
