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

module rfPhoenixMcAlu(rst, clk, ir, a, b, c, imm, o, done, ridi, rido);
parameter NPIPE = 8;
input rst;
input clk;
input Instruction ir;
input Value a;
input Value b;
input Value c;
input Value imm;
output Value o;
output reg done;
input [3:0] ridi;
output [3:0] rido;

integer n;

wire fms = ir.any.opcode==FMS || ir.any.opcode==FNMS;
wire fnm = ir.any.opcode==FNMA || ir.any.opcode==FNMS;
wire fmul = ir.any.opcode==R2 && ir.r2.func==FMUL;
wire fadd = ir.any.opcode==R2 && ir.r2.func==FADD;
wire fsub = ir.any.opcode==R2 && ir.r2.func==FSUB;

Value fma_o, fma_o1;
Value fcmp_o;
Value i2f_o,i2f1_o;
Value f2i_o,f2i1_o;
Value frunc_o,ftrunc1_o;
Value frsqrte_o,frsqrte1_o;
Value fres_o,fres1_o;
Value fsig_o,fsig1_o;
DoubleValue muli_o,muli1_o;

mult32x32 uimul1(
	.clk(clk),
	.ce(1'b1),
	.a(a),
	.b(imm),
	.o(muli1_o)
);

// Multiply takes only six cycles, add a cycle.
always_ff @(posedge clk)
	muli_o <= muli1_o;

i2f32 ui2f1
(
	.clk(clk),
	.ce(1'b1),
	.op(),
	.rm(3'b001),
	.i(a),
	.o(i2f1_o)
);

f2i32 uf2i1
(
	.clk(clk),
	.ce(1'b1),
	.op(1'b0),
	.i(a),
	.o(f2i_o),
	.overflow()
);

fpTrunc32 utrnc1
(
	.clk(clk),
	.ce(1'b1),
	.i(a),
	.o(ftrunc1_o),
	.overflow()
);

fpRsqrte32 ufrsqrte1
(
	.clk(clk),
	.ce(1'b1),
	.ld(1'b0),	// This signal not used
	.a(a),
	.o(frsqrte1_o)
);

fpRes32 ufres1
(
	.clk(clk),
	.ce(1'b1),
	.a(a),
	.o(fres1_o)
);

fpSigmoid32 ufsig1
(
	.clk(clk),
	.ce(1'b1),
	.a(a),
	.o(sig1_o)
);

ft_delay #(.WID($bits(Value)), .DEP(6)) uftd0 (.clk(clk), .ce(1'b1), .i(i2f1_o), .o(i2f_o));
ft_delay #(.WID($bits(Value)), .DEP(6)) uftd1 (.clk(clk), .ce(1'b1), .i(f2i1_o), .o(f2i_o));
ft_delay #(.WID($bits(Value)), .DEP(6)) uftd2 (.clk(clk), .ce(1'b1), .i(ftrunc1_o), .o(ftrunc_o));
ft_delay #(.WID($bits(Value)), .DEP(4)) uftd3 (.clk(clk), .ce(1'b1), .i(frsqrte1_o), .o(frsqrte_o));
ft_delay #(.WID($bits(Value)), .DEP(4)) uftd4 (.clk(clk), .ce(1'b1), .i(fres1_o), .o(fres_o));
ft_delay #(.WID($bits(Value)), .DEP(4)) uftd5 (.clk(clk), .ce(1'b1), .i(fsig1_o), .o(fsig_o));

fpFMA32nrL7 ufma1 (
	.clk(clk),
	.ce(1'b1),
	.op(fms|fsub),
	.rm(ir.f3.rm),
	.a(a ^ {fnm,{$bits(Value)-1{1'b0}}}),
	.b(fadd|fsub ? 32'h3F800000 : b),	// multiply by one for FADD/FSUB
	.c(fmul ? 32'h0 : fadd|fsub ? b : c),
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
ft_delay #(.WID(4), .DEP(7)) uftd7 (.clk(clk), .ce(1'b1), .i(ridi), .o(rido));

always_comb
case(ir.any.opcode)
R2:
	case(ir.r2.func)
	R1:
		case(ir.r2.Rb)
		I2F:	o = i2f_o;
		F2I:	o = f2i_o;
		FTRUNC:	o = ftrunc_o;
		FRSQRTE:	o = frsqrte_o;
		FRES:		o = fres_o;
		FSIGMOID:	o = fsig_o;
		default:	o = 'd0;
		endcase
	FADD,FSUB,FMUL:	o = fma_o;
	default:	o = 'd0;
	endcase
MULI:	o = muli_o[31:0];
FMA,FMS,FNMA,FNMS:	o = fma_o;
default:	o = 'd0;
endcase

endmodule
