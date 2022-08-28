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

Value fma_o, fma_o1;
Value fcmp_o;
Value i2f_o,i2f1_o;
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
	if (ce) muli_o <= muli1_o;

i2f32 ui2f1
(
	.clk(clk),
	.ce(ce),
	.op(),
	.rm(3'b001),
	.i(a),
	.o(i2f1_o)
);

ft_delay #(.WID($bits(Value)), .DEP(6)) uftd0 (.clk(clk), .ce(1'b1), .i(i2f1_o), .o(i2f_o));

fpFMA32nrL7 ufma1 (
	.clk(clk),
	.ce(1'b1),
	.op(fms),
	.rm(ir.f3.rm),
	.a(a ^ {fnm,{$bits(Value)-1{1'b0}}}),
	.b(b),
	.c(c),
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

always
case(ir.any.opcode)
R2:
	case(ir.r2.func)
	I2F:	o = i2f_o;
	default:	o = 'd0;
	endcase
MULI:	o = muli_o[31:0];
FMA,FMS,FNMA,FNMS:	o = fma_o;
default:	o = 'd0;
endcase

endmodule
