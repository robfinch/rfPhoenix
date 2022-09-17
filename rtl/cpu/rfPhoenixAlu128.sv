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

module rfPhoenixAlu128(ir, a, b, c, o);
input instruction_t ir;
input quad_value_t a;
input quad_value_t b;
input quad_value_t c;
output quad_value_t o;

integer n;

quad_value_t fcmp_o, fcmpi_o;
quad_value_t fclass_o;
wire [7:0] exp;
wire inf, xz, vz, snan, qnan, xinf;

fpDecomp128 udc1
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

assign fclass_o[0] = a[127] & inf;	// negative infinity
assign fclass_o[1] = a[127];				// negative number
assign fclass_o[2] = a[127] & xz & ~vz;		// negative subnormal
assign fclass_o[3] = a[127] & vz;	// negative zero
assign fclass_o[4] = ~a[127] & vz;	// positive zero
assign fclass_o[5] = ~a[127] & xz & ~vz;		// positive subnormal
assign fclass_o[6] = ~a[127];			// positive number
assign fclass_o[7] = ~a[127] & inf;	// positive infinity
assign fclass_o[8] = snan;				// signalling nan
assign fclass_o[9] = qnan;				// quiet nan
assign fclass_o[30:10] = 'd0;
assign fclass_o[31] = a[127];

fpCompare128 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

always_comb
case(ir.any.opcode)
OP_R2:
	case(ir.r2.func)
	OP_R1:
		case(ir.r2.Rb)
		OP_FABS:			o = {1'b0,a[126:0]};
		OP_FNABS:		o = {1'b1,a[126:0]};
		OP_FNEG:			o = {~a[127],a[126:0]};
		OP_FCLASS:		o = fclass_o;
		OP_FSIGN:		o = vz ? 128'h0 : a[127] ? 32'hBFFF0000000000000000000000000000 : 128'h3FFF0000000000000000000000000000;
		OP_FFINITE:	o = {127'd0,~xinf};
		default:	o = 'd0;
		endcase
	OP_ADD:		o = a + b;
	OP_SUB:		o = a - b;
	OP_AND:		o = a & b;
	OP_OR:			o = a | b;
	OP_XOR:		o = a ^ b;
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
default:	o = 'd0;
endcase


endmodule
