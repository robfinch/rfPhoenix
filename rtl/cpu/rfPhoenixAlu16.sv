`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2013-2022  Robert Finch, Waterloo
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

module rfPhoenixAlu16(ir, a, b, c, imm, o);
input instruction_t ir;
input half_value_t a;
input half_value_t b;
input half_value_t c;
input half_value_t imm;
output half_value_t o;

integer n;

half_value_t fcmp_o, fcmpi_o;
half_value_t fclass_o;
wire [7:0] exp;
wire inf, xz, vz, snan, qnan, xinf;

fpDecomp16 udc1
(
	.i(a),
	.sgn(),
	.exp(exp),
	.man(),
	.fract(),
	.xz(xz),
	.mz(),
	.vz(vz),
	.inf(inf),
	.xinf(xinf),
	.qnan(),
	.snan(snan),
	.nan(qnan)
);

assign fclass_o[0] = a[15] & inf;	// negative infinity
assign fclass_o[1] = a[15];				// negative number
assign fclass_o[2] = a[15] & xz & ~vz;		// negative subnormal
assign fclass_o[3] = a[15] & vz;	// negative zero
assign fclass_o[4] = ~a[15] & vz;	// positive zero
assign fclass_o[5] = ~a[15] & xz & ~vz;		// positive subnormal
assign fclass_o[6] = ~a[15];			// positive number
assign fclass_o[7] = ~a[15] & inf;	// positive infinity
assign fclass_o[8] = snan;				// signalling nan
assign fclass_o[9] = qnan;				// quiet nan
assign fclass_o[14:10] = 'd0;
assign fclass_o[15] = a[15];

fpCompare16 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

fpCompare16 ucmp2
(
	.a(a),
	.b(imm),
	.o(fcmpi_o),
	.nan(),
	.snan()
);

wire [4:0] cntlz_o;
cntlz16 ucntlz1 (.i(a), .o(cntlz_o));

always_comb
case(ir.any.opcode)
OP_R2:
	case(ir.r2.func)
	OP_R1:
		case(ir.r2.Rb)
		OP_CNTLZ:			o = {11'd0,cntlz_o};
		OP_FABS:			o = {1'b0,a[14:0]};
		OP_FNABS:			o = {1'b1,a[14:0]};
		OP_FNEG:			o = {~a[15],a[14:0]};
		OP_FCLASS:		o = fclass_o;
		OP_FSIGN:		o = vz ? 16'h0 : a[15] ? 16'hBE00 : 16'h3E00;
		OP_FFINITE:	o = {15'd0,~xinf};
		OP_SEXTB:		o = {{8{a[7]}},a[7:0]};
		default:	o = 'd0;
		endcase
	OP_ADD:		o = a + b;
	OP_SUB:		o = a - b;
//	OP_MUL:		o = a * b;
	OP_AND:		o = a & b;
	OP_OR:			o = a | b;
	OP_XOR:		o = a ^ b;
	/*
	OP_CMP:
		begin
			o[0] = a == b;
			o[1] = $signed(a) < $signed(b);
			o[2] = $signed(a) <= $signed(b);
			o[4:3] = 2'b0;
			o[5] = a < b;
			o[6] = a <= b;
			o[7] = 1'b0;
			o[8] = a != b;
			o[9] = $signed(a) >= $signed(b);
			o[10] = $signed(a) > $signed(b);
			o[12:11] = 2'b0;
			o[13] = a >= b;
			o[14] = a > b;
			o[15] = 1'b0;
			o[16] = fcmp_o[0];	// ==
			o[17] = fcmp_o[1];	// <
			o[18] = fcmp_o[2];	// <=
			o[19] = fcmp_o[3];
			o[20] = fcmp_o[4];
			o[21] = fcmp_o[8];	// <>
			o[22] = fcmp_o[9];	// >=
			o[23] = fcmp_o[10];	// >
			o[24] = fcmp_o[11];	// mag >=
			o[25] = fcmp_o[12];	// ordered
			o[31:26] = 6'd0;
		end
	*/
	OP_CMP_EQ:	o = a == b;
	OP_CMP_NE:	o = a != b;
	OP_CMP_LT:	o = $signed(a) < $signed(b);
	OP_CMP_GE:	o = $signed(a) >= $signed(b);
	OP_CMP_LE: o = $signed(a) <= $signed(b);
	OP_CMP_GT:	o = $signed(a) > $signed(b);
	OP_CMP_LTU:	o = a < b;
	OP_CMP_GEU:	o = a >= b;
	OP_CMP_LEU:	o = a <= b;
	OP_CMP_GTU:	o = a > b;
	OP_FCMP_EQ:	o = fcmp_o[0];
	OP_FCMP_NE:	o = fcmp_o[8]|fcmp_o[4];	// return 1 if Nan
	OP_FCMP_LT:	o = fcmp_o[1];
	OP_FCMP_LE:	o = fcmp_o[2];
	OP_FCMP_GT:	o = fcmp_o[10];
	OP_FCMP_GE:	o = fcmp_o[9];
	default:	o = 'd0;
	endcase
OP_ADDI:		o = a + imm;
OP_SUBFI:		o = imm - a;
//OP_MULI:		o = a * imm;
OP_ANDI:		o = a & imm;
OP_ORI:			o = a | imm;
OP_XORI:		o = a ^ imm;
OP_CMP_EQI:	o = a == imm;
OP_CMP_NEI:	o = a != imm;
OP_CMP_LTI:	o = $signed(a) < $signed(imm);
OP_CMP_GEI:	o = $signed(a) >= $signed(imm);
OP_CMP_LEI: o = $signed(a) <= $signed(imm);
OP_CMP_GTI:	o = $signed(a) > $signed(imm);
OP_CMP_LTUI:	o = a < imm;
OP_CMP_GEUI:	o = a >= imm;
OP_CMP_LEUI:	o = a <= imm;
OP_CMP_GTUI:	o = a > imm;
OP_FCMP_EQI:	o = fcmpi_o[0];
OP_FCMP_NEI:	o = fcmpi_o[8]|fcmpi_o[4];	// return 1 if Nan
OP_FCMP_LTI:	o = fcmpi_o[1];
OP_FCMP_LEI:	o = fcmpi_o[2];
OP_FCMP_GTI:	o = fcmpi_o[10];
OP_FCMP_GEI:	o = fcmpi_o[9];
default:	o = 'd0;
endcase

endmodule
