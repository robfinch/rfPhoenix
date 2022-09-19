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

module rfPhoenixMcAlu16(rst, clk, ir, a, b, c, imm, o, done, ridi, rido);
parameter NPIPE = 8;
input rst;
input clk;
input instruction_t ir;
input half_value_t a;
input half_value_t b;
input half_value_t c;
input half_value_t imm;
output half_value_t o;
output reg done;
input tid_t ridi;
output tid_t rido;

integer n;

wire fms = ir.any.opcode==OP_FMS16 || ir.any.opcode==OP_FNMS16;
wire fnm = ir.any.opcode==OP_FNMA16 || ir.any.opcode==OP_FNMS16;
wire fadd = ir.any.opcode==OP_R2 && ir.r2.func==OP_FADD;
wire fsub = ir.any.opcode==OP_R2 && ir.r2.func==OP_FSUB;
wire mul = ir.any.opcode==OP_R2 && ir.r2.func==OP_MUL;

half_value_t fma_o, fma_o1;
half_value_t fcmp_o;
half_value_t i2f_o,i2f1_o;
half_value_t f2i_o,f2i1_o;
half_value_t ftrunc_o,ftrunc1_o;
half_value_t frsqrte_o,frsqrte1_o;
half_value_t fres_o,fres1_o;
half_value_t fsig_o,fsig1_o;
value_t muli_o,muli1_o,muli2_o;

mult16x16combo uimul1(
	.a(a),
	.b(mul ? b : imm),
	.o(muli2_o)
);

// Multiply takes only zero cycles, add eight cycles.
vtdl #(.WID($bits(value_t)), .DEP(16)) udly1 (.clk(clk), .ce(1'b1), .a(4'd8), .d(muli2_o), .q(muli_o));

i2f16 ui2f1
(
	.clk(clk),
	.ce(1'b1),
	.op(),
	.rm(3'b001),
	.i(a),
	.o(i2f1_o)
);

f2i16 uf2i1
(
	.clk(clk),
	.ce(1'b1),
	.op(1'b0),
	.i(a),
	.o(f2i1_o),
	.overflow()
);

fpTrunc16 utrnc1
(
	.clk(clk),
	.ce(1'b1),
	.i(a),
	.o(ftrunc1_o),
	.overflow()
);

/*
fpRsqrte16 ufrsqrte1
(
	.clk(clk),
	.ce(1'b1),
	.ld(1'b0),	// This signal not used
	.a(a),
	.o(frsqrte1_o)
);

fpRes16 ufres1
(
	.clk(clk),
	.ce(1'b1),
	.a(a),
	.o(fres1_o)
);

`ifdef SIGMOID
fpSigmoid16 ufsig1
(
	.clk(clk),
	.ce(1'b1),
	.a(a),
	.o(sig1_o)
);

vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl5 (.clk(clk), .ce(1'b1), .a(4'd5), .d(fsig1_o), .q(fsig_o));
`endif
*/
/*
	uftd1 used 13,624 LUTs!
ft_delay #(.WID($bits(half_value_t)), .DEP(6)) uftd0 (.clk(clk), .ce(1'b1), .i(i2f1_o), .o(i2f_o));
ft_delay #(.WID($bits(half_value_t)), .DEP(6)) uftd1 (.clk(clk), .ce(1'b1), .i(f2i1_o), .o(f2i_o));
ft_delay #(.WID($bits(half_value_t)), .DEP(6)) uftd2 (.clk(clk), .ce(1'b1), .i(ftrunc1_o), .o(ftrunc_o));
ft_delay #(.WID($bits(half_value_t)), .DEP(4)) uftd3 (.clk(clk), .ce(1'b1), .i(frsqrte1_o), .o(frsqrte_o));
ft_delay #(.WID($bits(half_value_t)), .DEP(4)) uftd4 (.clk(clk), .ce(1'b1), .i(fres1_o), .o(fres_o));
ft_delay #(.WID($bits(half_value_t)), .DEP(4)) uftd5 (.clk(clk), .ce(1'b1), .i(fsig1_o), .o(fsig_o));
*/

vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl0 (.clk(clk), .ce(1'b1), .a(4'd7), .d(i2f1_o), .q(i2f_o));
vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl1 (.clk(clk), .ce(1'b1), .a(4'd7), .d(f2i1_o), .q(f2i_o));
vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl2 (.clk(clk), .ce(1'b1), .a(4'd7), .d(ftrunc1_o), .q(ftrunc_o));
vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl3 (.clk(clk), .ce(1'b1), .a(4'd5), .d(frsqrte1_o), .q(frsqrte_o));
vtdl #(.WID($bits(half_value_t)), .DEP(16)) uvtdl4 (.clk(clk), .ce(1'b1), .a(4'd5), .d(fres1_o), .q(fres_o));


fpFMA16nrL8 ufma1 (
	.clk(clk),
	.ce(1'b1),
	.op(fms|fsub),
	.rm(ir.f3.rm),
	.a(a ^ {fnm,{$bits(half_value_t)-1{1'b0}}}),
	.b(fadd|fsub ? 16'h3E00 : b),	// multiply by one for FADD/FSUB
	.c(fadd|fsub ? b : c),
	.o(fma_o),
	.inf(),
	.zero(),
	.overflow(),
	.underflow(),
	.inexact()
);

/*
always_ff @(posedge clk)
if (rst)
	for (n = 0; n < NPIPE - 1; n = n + 1) begin
		fma_pipe[n+1] <= 'd0;
	end
else begin
	for (n = 0; n < NPIPE - 1; n = n + 1) begin
		fma_pipe[n+1] <= fma_pipe[n];
	end
	fma_pipe[0] <= fma_o1;
end
*/
vtdl #(.WID($bits(tid_t)), .DEP(16)) uvtdl7 (.clk(clk), .ce(1'b1), .a(4'd8), .d(ridi), .q(rido));

always_comb
case(ir.any.opcode)
OP_R2:
	case(ir.r2.func)
	OP_R1:
		case(ir.r2.Rb)
		OP_I2F:	o = i2f_o;
		OP_F2I:	o = f2i_o;
		OP_FTRUNC:	o = ftrunc_o;
		OP_FRSQRTE:	o = frsqrte_o;
		OP_FRES:		o = fres_o;
		OP_FSIGMOID:	o = fsig_o;
		default:	o = 'd0;
		endcase
	OP_MUL:	o = muli_o[31:0];
	OP_FADD,OP_FSUB:	o = fma_o;
	default:	o = 'd0;
	endcase
OP_MULI:	o = muli_o[31:0];
OP_FMA16,OP_FMS16,OP_FNMA16,OP_FNMS16:	o = fma_o;
default:	o = 'd0;
endcase

endmodule
