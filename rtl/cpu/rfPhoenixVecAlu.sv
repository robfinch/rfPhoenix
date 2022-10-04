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

module rfPhoenixVecAlu(ir, prc, a, b, c, t, Ta, Tb, Tt, imm, asid, hmask, 
	trace_dout, trace_empty, trace_valid, trace_count,
	o);
input instruction_t ir;
input prec_t prc;
input vector_value_t a;
input vector_value_t b;
input vector_value_t c;
input vector_value_t t;
input Ta;
input Tb;
input Tt;
input value_t imm;
input ASID asid;
input value_t hmask;
input Address trace_dout;
input trace_empty;
input trace_valid;
input [10:0] trace_count;
output vector_value_t o;

vector_value_t o1,o2;

vector_quad_value_t ab, bb, cb, ob1, ob2;
vector_half_value_t ac, bc, cc, ob3, ob4;
vector_double_value_t ad, bd, cd, ob5, ob6;

assign ab = a;
assign bb = b;
assign cb = c;
assign ac = a;
assign bc = b;
assign cc = c;
assign ad = a;
assign bd = b;
assign cd = c;

integer n;
genvar g;
generate begin
	for (g = 0; g < NLANES; g = g + 1)
		rfPhoenixAlu ualu1 (
			.ir(ir),
			.a(a[g]),
			.b(b[g]),
			.c(c[g]),
			.t(t[g]),
			.imm(imm),
			.asid(asid),
			.hmask(hmask),
			.trace_dout(trace_dout),
			.trace_empty(trace_empty),
			.trace_valid(trace_valid),
			.trace_count(trace_count),
			.o(o1[g])
		);
`ifdef SUPPORT_16BIT_OPS
	for (g = 0; g < NLANES*2; g = g + 1)
		rfPhoenixAlu16 ualu2 (
			.ir(ir),
			.a(ac[g]),
			.b(bc[g]),
			.c(cc[g]),
			.imm(imm),
			.o(ob3[g])
		);
`endif
`ifdef SUPPORT_64BIT_OPS
	for (g = 0; g < NLANES/2; g = g + 1)
		rfPhoenixAlu64 ualu3 (
			.ir(ir),
			.a(ad[g]),
			.b(bd[g]),
			.c(cd[g]),
			.imm(imm),
			.o(ob5[g])
		);
`endif
`ifdef SUPPORT_128BIT_OPS
	for (g = 0; g < NLANES/4; g = g + 1)
		rfPhoenixAlu128 ualu4 (
			.ir(ir),
			.a(ab[g]),
			.b(bb[g]),
			.c(cb[g]),
			.o(ob1[g])
		);
`endif

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

// 16-bit precision
`ifdef SUPPORT_16BIT_OPS
always_comb
	case(ir.any.opcode)
	OP_R2:
		case (ir.r2.func)
		OP_VEX:	
			ob4 = {NLANES{ac[imm[4:0]]}};
//		VEINS:
		OP_VSHUF:
			for (n = 0; n < NLANES*2; n = n + 1)
				ob4[n] = ac[bc[n][4:0]];
		OP_VSLLV:
			ob4 = ac << {bc[0][4:0],2'd0};
		OP_VSRLV:		
			ob4 = ac >> {bc[0][4:0],2'd0};
		OP_VSLLVI:
			ob4 = ac << {imm[4:0],2'd0};
		OP_VSRLVI:
			ob4 = ac >> {imm[4:0],2'd0};
		default:	ob4 = ob3;
		endcase
	OP_FCMP,OP_CMP:
		if (Tt)
			ob4 = ob3;
		else if (Ta|Tb) begin
			ob4 = 'd0;
			for (n = 0; n < NLANES*2; n = n + 1)
				ob4[0][n] = ob3[n[4:0]];
		end
		else
			ob4 = ob3;
	OP_FCMPI16,OP_CMPI16:
		if (Tt)
			ob4 = ob3;
		else if (Ta) begin
			ob4 = 'd0;
			for (n = 0; n < NLANES/4; n = n + 1)
				ob4[0][n] = ob3[n[3:0]];
		end
		else
			ob4 = ob3;
	default:	ob4 = ob3;
	endcase
`endif

// 64-bit precision
`ifdef SUPPORT_64BIT_OPS
always_comb
	case(ir.any.opcode)
	OP_R2:
		case (ir.r2.func)
		OP_VEX:	
			ob6 = {NLANES{ac[imm[4:0]]}};
//		VEINS:
		OP_VSHUF:
			for (n = 0; n < NLANES*2; n = n + 1)
				ob6[n] = ac[bc[n][4:0]];
		OP_VSLLV:
			ob6 = ac << {bc[0][4:0],2'd0};
		OP_VSRLV:		
			ob6 = ac >> {bc[0][4:0],2'd0};
		OP_VSLLVI:
			ob6 = ac << {imm[4:0],2'd0};
		OP_VSRLVI:
			ob6 = ac >> {imm[4:0],2'd0};
		default:	ob6 = ob5;
		endcase
	OP_FCMPI64,OP_CMPI64:
		if (Tt)
			ob6 = ob5;
		else if (Ta) begin
			ob6 = 'd0;
			for (n = 0; n < NLANES/4; n = n + 1)
				ob6[0][n] = ob5[n[3:0]];
		end
		else
			ob6 = ob5;
	OP_FCMP,OP_CMP:
		if (Tt)
			ob6 = ob5;
		else if (Ta|Tb) begin
			ob6 = 'd0;
			for (n = 0; n < NLANES*2; n = n + 1)
				ob6[0][n] = ob5[n[4:0]];
		end
		else
			ob6 = ob5;
	default:	ob6 = ob5;
	endcase
`endif

`ifdef SUPPORT_128BIT_OPS
// 128-bit precision
always_comb
	case(ir.any.opcode)
	OP_R2:
		case (ir.r2.func)
		OP_VEX:	
			ob2 = {NLANES{ab[imm[1:0]]}};
//		VEINS:
		OP_VSHUF:
			for (n = 0; n < NLANES/4; n = n + 1)
				ob2[n] = ab[bb[n][1:0]];
		OP_VSLLV:
			ob2 = ab << {bb[0][1:0],7'd0};
		OP_VSRLV:		
			ob2 = ab >> {bb[0][1:0],7'd0};
		OP_VSLLVI:
			ob2 = ab << {imm[1:0],7'd0};
		OP_VSRLVI:
			ob2 = ab >> {imm[1:0],7'd0};
		default:	ob2 = ob1;
		endcase
	OP_FCMP,OP_CMP:
		if (Tt)
			ob2 = ob1;
		else if (Ta|Tb) begin
			ob2 = 'd0;
			for (n = 0; n < NLANES/4; n = n + 1)
				ob2[0][n] = ob1[n[1:0]];
		end
		else
			ob2 = ob1;
	OP_FCMPI128,OP_CMPI128:
		if (Tt)
			ob2 = ob1;
		else if (Ta) begin
			ob2 = 'd0;
			for (n = 0; n < NLANES/4; n = n + 1)
				ob2[0][n] = ob1[n[3:0]];
		end
		else
			ob2 = ob1;
	default:	ob2 = ob1;
	endcase
`endif

// 32-bit precision
always_comb
	case(ir.any.opcode)
	OP_R2:
		case (ir.r2.func)
		OP_VEX:	
			o2 = {NLANES{a[imm[3:0]]}};
//		VEINS:
		OP_VSHUF:
			for (n = 0; n < NLANES; n = n + 1)
				o2[n] = a[b[n][3:0]];
		OP_VSLLV:
			o2 = a << {b[0][3:0],5'd0};
		OP_VSRLV:		
			o2 = a >> {b[0][3:0],5'd0};
		OP_VSLLVI:
			o2 = a << {imm[3:0],5'd0};
		OP_VSRLVI:
			o2 = a >> {imm[3:0],5'd0};
		OP_SHPTENDX:	o2 = {NLANES{ptendx}};
		default:	o2 = o1;
		endcase
	OP_FCMP,OP_CMP:
		if (Tt)
			o2 = o1;
		else if (Ta|Tb) begin
			o2 = 'd0;
			for (n = 0; n < NLANES; n = n + 1)
				o2[0][n] = o1[n[3:0]];
		end
		else
			o2 = o1;
	OP_FCMPI32,OP_CMPI32:
		if (Tt)
			o2 = o1;
		else if (Ta) begin
			o2 = 'd0;
			for (n = 0; n < NLANES; n = n + 1)
				o2[0][n] = o1[n[3:0]];
		end
		else
			o2 = o1;
	default:	o2 = o1;
	endcase

always_comb
	case(prc)
	PRC16:	o = ob4;
	PRC64:	o = ob6;
	PRC128:	o = ob2;
	default:	o = o2;
	endcase

endmodule
