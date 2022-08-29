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

module rfPhoenixAlu(ir, a, b, c, imm, o);
input Instruction ir;
input Value a;
input Value b;
input Value c;
input Value imm;
output Value o;

integer n;

Value fcmp_o, fcmpi_o;
Value fclass_o;
wire [7:0] exp;
wire inf, xz, vz, snan, qnan, xinf;

fpDecomp32 udc1
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

assign fclass_o[0] = a[31] & inf;	// negative infinity
assign fclass_o[1] = a[31];				// negative number
assign fclass_o[2] = a[31] & xz & ~vz;		// negative subnormal
assign fclass_o[3] = a[31] & vz;	// negative zero
assign fclass_o[4] = ~a[31] & vz;	// positive zero
assign fclass_o[5] = ~a[31] & xz & ~vz;		// positive subnormal
assign fclass_o[6] = ~a[31];			// positive number
assign fclass_o[7] = ~a[31] & inf;	// positive infinity
assign fclass_o[8] = snan;				// signalling nan
assign fclass_o[9] = qnan;				// quiet nan
assign fclass_o[30:10] = 'd0;
assign fclass_o[31] = a[31];

fpCompare32 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

fpCompare32 ucmp2
(
	.a(a),
	.b(imm),
	.o(fcmpi_o),
	.nan(),
	.snan()
);

wire [5:0] cntlz_o;
cntlz32 ucntlz1 (.i(a), .o(cntlz_o));

wire [5:0] cntpop_o;
cntpop32 ucntpop1 (.i(a), .o(cntpop_o));

always_comb
case(ir.any.opcode)
R2:
	case(ir.r2.func)
	R1:
		case(ir.r2.Rb)
		CNTLZ:		o = {26'd0,cntlz_o};
		CNTPOP:		o = {26'd0,cntpop_o};
		FCLASS:		o = fclass_o;
		FSIGN:		o = vz ? 32'h0 : a[31] ? 32'hBF800000 : 32'h3F800000;
		FFINITE:	o = {31'd0,~xinf};
		default:	o = 'd0;
		endcase
	ADD:		o = a + b;
	SUB:		o = a - b;
	AND:		o = a & b;
	OR:			o = a | b;
	XOR:		o = a ^ b;
	CMP_EQ:	o = a == b;
	CMP_NE:	o = a != b;
	CMP_LT:	o = $signed(a) < $signed(b);
	CMP_GE:	o = $signed(a) >= $signed(b);
	CMP_LE: o = $signed(a) <= $signed(b);
	CMP_GT:	o = $signed(a) > $signed(b);
	CMP_LTU:	o = a < b;
	CMP_GEU:	o = a >= b;
	CMP_LEU:	o = a <= b;
	CMP_GTU:	o = a > b;
	FCMP_EQ:	o = fcmp_o[0];
	FCMP_NE:	o = fcmp_o[8]|fcmp_o[4];	// return 1 if Nan
	FCMP_LT:	o = fcmp_o[1];
	FCMP_LE:	o = fcmp_o[2];
	FCMP_GT:	o = fcmp_o[10];
	FCMP_GE:	o = fcmp_o[9];
	SLLI:			o = a << imm[4:0];
	SRLI:			o = a >> imm[4:0];
	SRAI:			o = {{32{a[31]}},a} >> imm[4:0];
	SLL:			o = a << b[4:0];
	SRL:			o = a >> b[4:0];
	SRA:			o = {{32{a[31]}},a} >> b[4:0];
	default:	o = 'd0;
	endcase
ADDI:			o = a + imm;
SUBFI:		o = imm - a;
ANDI:			o = a & imm;
ORI:			o = a | imm;
XORI:			o = a ^ imm;
CMP_EQI:	o = a == imm;
CMP_NEI:	o = a != imm;
CMP_LTI:	o = $signed(a) < $signed(imm);
CMP_GEI:	o = $signed(a) >= $signed(imm);
CMP_LEI:	o = $signed(a) <= $signed(imm);
CMP_GTI:	o = $signed(a) > $signed(imm);
CMP_LTUI:	o = a < imm;
CMP_GEUI:	o	= a >= imm;
CMP_LEUI:	o = a <= imm;
CMP_GTUI:	o = a > imm;
FCMP_EQI:	o = fcmpi_o[0];
FCMP_NEI:	o = fcmpi_o[8]|fcmpi_o[4];	// return 1 if Nan
FCMP_LTI:	o = fcmpi_o[1];
FCMP_LEI:	o = fcmpi_o[2];
FCMP_GTI:	o = fcmpi_o[10];
FCMP_GEI:	o = fcmpi_o[9];
default:	o = 'd0;
endcase


endmodule
