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

module rfPhoenixVecAlu(ir, a, b, c, t, Ta, Tb, Tt, imm, asid, hmask, o);
input Instruction ir;
input VecValue a;
input VecValue b;
input VecValue c;
input VecValue t;
input Ta;
input Tb;
input Tt;
input Value imm;
input ASID asid;
input Value hmask;
output VecValue o;

VecValue o1;

integer n;
genvar g;
generate begin
	for (g = 0; g < NLANES; g = g + 1)
		rfPhoenixAlu ualu (
			.ir(ir),
			.a(a[g]),
			.b(b[g]),
			.c(c[g]),
			.t(t[g]),
			.imm(imm),
			.asid(asid),
			.hmask(hmask),
			.o(o1[g])
		);
end
endgenerate

reg [31:0] ptendx;
integer n2;
always_comb begin
	ptendx = 32'hFFFFFFFF;
//for (n2 = 0; n2 < 8; n2 = n2 + 1)
//		if (a[0][31:16]==b[n2*2+1][15:0] && (asid==b[n2*2+1][31:22]||b[n2*2+1][21]))
//			ptendx = n2;
end

always_comb
	case(ir.any.opcode)
	R2:
		case (ir.r2.func)
		FCMP_EQ,FCMP_NE,FCMP_LT,FCMP_GE,FCMP_LE,FCMP_GT,
		CMP_EQ,CMP_NE,CMP_LT,CMP_GE,CMP_LE,CMP_GT,
		CMP_LTU,CMP_GEU,CMP_LEU,CMP_GTU:
			if (Tt)
				o = o1;
			else if (Ta|Tb) begin
				o = 'd0;
				for (n = 0; n < NLANES; n = n + 1)
					o[0][n] = o1[n];
			end
			else
				o = o1;
		VEX:	o = {NLANES{a[imm[3:0]]}};
//		VEINS:
		VSHUF:
			for (n = 0; n < NLANES; n = n + 1)
				o[n] = a[b[n][3:0]];
		VSLLV:		o = a << {b[0][3:0],5'd0};
		VSRLV:		o = a >> {b[0][3:0],5'd0};
		VSLLVI:		o = a << {imm[3:0],5'd0};
		VSRLVI:		o = a >> {imm[3:0],5'd0};
		SHPTENDX:	o = {NLANES{ptendx}};
		default:	o = o1;
		endcase
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI,
	CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		if (Tt)
			o = o1;
		else if (Ta) begin
			o = 'd0;
			for (n = 0; n < NLANES; n = n + 1)
				o[0][n] = o1[n];
		end
		else
			o = o1;
	default:	o = o1;
	endcase

endmodule
