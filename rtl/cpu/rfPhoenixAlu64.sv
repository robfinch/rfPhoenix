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
input double_value_t a;
input double_value_t b;
input double_value_t c;
input double_value_t imm;
output double_value_t o;

integer n;

double_value_t o2;
double_value_t fcmp_o, fcmpi_o;
double_value_t fclass_o;
wire [10:0] exp;
wire inf, xz, vz, snan, qnan, xinf;

fpDecomp64 udc1
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

assign fclass_o[0] = a[63] & inf;	// negative infinity
assign fclass_o[1] = a[63];				// negative number
assign fclass_o[2] = a[63] & xz & ~vz;		// negative subnormal
assign fclass_o[3] = a[63] & vz;	// negative zero
assign fclass_o[4] = ~a[63] & vz;	// positive zero
assign fclass_o[5] = ~a[63] & xz & ~vz;		// positive subnormal
assign fclass_o[6] = ~a[63];			// positive number
assign fclass_o[7] = ~a[63] & inf;	// positive infinity
assign fclass_o[8] = snan;				// signalling nan
assign fclass_o[9] = qnan;				// quiet nan
assign fclass_o[14:10] = 'd0;
assign fclass_o[15] = a[63];

fpCompare64 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

fpCompare64 ucmp2
(
	.a(a),
	.b(imm),
	.o(fcmpi_o),
	.nan(),
	.snan()
);

wire [4:0] cntlz_o;
cntlz64 ucntlz1 (.i(a), .o(cntlz_o));

always_comb
case(ir.any.opcode)
OP_R2:
	case(ir.r2.func)
	OP_R1:
		case(ir.r2.Rb)
		OP_CNTLZ:			o = {57'd0,cntlz_o};
		OP_FABS:			o = {1'b0,a[62:0]};
		OP_FNABS:			o = {1'b1,a[62:0]};
		OP_FNEG:			o = {~a[63],a[62:0]};
		OP_FCLASS:		o = fclass_o;
		OP_FSIGN:		o = vz ? 64'h0 : a[63] ? 64'hBFF0000000000000 : 64'h3FF0000000000000;
		OP_FFINITE:	o = {63'd0,~xinf};
		OP_SEXTB:		o = {{56{a[7]}},a[7:0]};
		default:	o = 'd0;
		endcase
	OP_ADD:		o = a + b;
	OP_SUB:		o = a - b;
//	OP_MUL:		o = a * b;
	OP_AND:		o = a & b;
	OP_OR:			o = a | b;
	OP_XOR:		o = a ^ b;
	default:	o = 'd0;
	endcase
OP_ADDI:		o = a + imm;
OP_SUBFI:		o = imm - a;
//OP_MULI:		o = a * imm;
OP_ANDI:		o = a & imm;
OP_ORI:			o = a | imm;
OP_XORI:		o = a ^ imm;
OP_CMP:
	begin
		o2 = 'd0;
		o2[0] = a == b;
		o2[1] = $signed(a) < $signed(b);
		o2[2] = $signed(a) <= $signed(b);
		o2[4:3] = 2'b0;
		o2[5] = a < b;
		o2[6] = a <= b;
		o2[7] = 1'b0;
		o2[8] = a != b;
		o2[9] = $signed(a) >= $signed(b);
		o2[10] = $signed(a) > $signed(b);
		o2[12:11] = 2'b0;
		o2[13] = a >= b;
		o2[14] = a > b;
		o2[15] = 1'b0;
		o = o2;
		/*
		case(ir.r2.func[3:0])
		4'd15:	o = o2;
		default:	o = {63'd0,o2[ir.r2.func[3:0]]};
		endcase
		*/
	end
OP_CMPI:
	begin
		o2 = 'd0;
		o2[0] = a == imm;
		o2[1] = $signed(a) < $signed(imm);
		o2[2] = $signed(a) <= $signed(imm);
		o2[4:3] = 2'b0;
		o2[5] = a < imm;
		o2[6] = a <= imm;
		o2[7] = 1'b0;
		o2[8] = a != imm;
		o2[9] = $signed(a) >= $signed(imm);
		o2[10] = $signed(a) > $signed(imm);
		o2[12:11] = 2'b0;
		o2[13] = a >= imm;
		o2[14] = a > imm;
		o2[15] = 1'b0;
		o = o2;
		/*
		case(ir.cmpi.N)
		4'd15:	o = o2;
		default:	o = {63'd0,o2[ir.cmpi.N]};
		endcase
		*/
	end
OP_FCMP:
	begin
		o2 = 'd0;
		o2[0] = fcmp_o[0];	// ==
		o2[1] = fcmp_o[1];	// <
		o2[2] = fcmp_o[2];	// <=
		o2[3] = fcmp_o[3];
		o2[4] = fcmp_o[4];
		o2[5] = fcmp_o[8]|fcmp_o[4];	// <>
		o2[9] = fcmp_o[9];	// >=
		o2[10] = fcmp_o[10];	// >
		o2[11] = fcmp_o[11];	// mag >=
		o2[12] = fcmp_o[12];	// ordered
		o = o2;
		/*
		case(ir.r2.func[3:0])
		4'd15:	o = o2;
		default:	o = {63'd0,o2[ir.r2.func[3:0]]};
		endcase
		*/
	end
OP_FCMPI:
	begin
		o2 = 'd0;
		o2[0] = fcmpi_o[0];	// ==
		o2[1] = fcmpi_o[1];	// <
		o2[2] = fcmpi_o[2];	// <=
		o2[3] = fcmpi_o[3];
		o2[4] = fcmpi_o[4];
		o2[5] = fcmpi_o[8]|fcmpi_o[4];	// <>	return 1 if Nan
		o2[9] = fcmpi_o[9];	// >=
		o2[10] = fcmpi_o[10];	// >
		o2[11] = fcmpi_o[11];	// mag >=
		o2[12] = fcmpi_o[12];	// ordered
		o = o2;
		/*
		case(ir.r2.func[3:0])
		4'd15:	o = o2;
		default:	o = {63'd0,o2[ir.r2.func[3:0]]};
		endcase
		*/
	end
default:	o = 'd0;
endcase

endmodule
