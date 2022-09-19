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

module rfPhoenixAlu(ir, a, b, c, t, imm, asid, hmask, trace_dout, trace_empty, trace_valid, trace_count, o);
input instruction_t ir;
input value_t a;
input value_t b;
input value_t c;
input value_t t;
input value_t imm;
input ASID asid;
input value_t hmask;
input Address trace_dout;
input trace_empty;
input trace_valid;
input [10:0] trace_count;
output value_t o;

integer n;

value_t fcmp_o, fcmpi_o;
value_t fclass_o;
wire [7:0] exp;
wire inf, xz, vz, snan, qnan, xinf;

DoubleValue sllr, slli;
DoubleValue srlr, srli;
DoubleValue srar, srai;
always_comb
	sllr = {a,{$bits(value_t){ir[36]}}} << b[5:0];
always_comb
	slli = {a,{$bits(value_t){ir[36]}}} << imm[5:0];
always_comb
	srlr = {{$bits(value_t){ir[36]}},a} >> b[5:0];
always_comb
	srli = {{$bits(value_t){ir[36]}},a} >> imm[5:0];
always_comb
	srar = {{$bits(value_t){a[31]}},a} >> b[5:0];
always_comb
	srai = {{$bits(value_t){a[31]}},a} >> imm[5:0];
	 
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

wire [15:0] hash;
rfPhoenix_ipt_hash uhash1
(
	.asid(asid),
	.adr(a),
	.mask(hmask),
	.hash(hash)
);

always_comb
case(ir.any.opcode)
OP_R2:
	case(ir.r2.func)
	OP_R1:
		case(ir.r2.Rb)
		OP_CNTLZ:		o = {26'd0,cntlz_o};
		OP_CNTPOP:		o = {26'd0,cntpop_o};
		OP_PTGHASH:	o = {16'h0,hash};
		OP_FABS:			o = {1'b0,a[30:0]};
		OP_FNABS:		o = {1'b1,a[30:0]};
		OP_FNEG:			o = {~a[31],a[30:0]};
		OP_FCLASS:		o = fclass_o;
		OP_FSIGN:		o = vz ? 32'h0 : a[31] ? 32'hBF800000 : 32'h3F800000;
		OP_FFINITE:	o = {31'd0,~xinf};
		OP_SEXTB:		o = {{24{a[7]}},a[7:0]};
		OP_SEXTW:		o = {{16{a[15]}},a[15:0]};
		OP_PEEKQ:		
			case(imm[3:0])
			4'd15:	o = trace_dout;
			default:	o = 'd0;
			endcase
		OP_POPQ:
			case(imm[3:0])
			4'd15:	o = trace_dout;
			default:	o = 'd0;
			endcase
		OP_STATQ:		
			case(imm[3:0])
			4'd15:	o = {trace_empty,trace_valid,19'd0,trace_count};
			default:	o = 'd0;
			endcase
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
	OP_SLLI:			o = slli[63:32];
	OP_SRLI:			o = srli[31: 0];
	OP_SRAI:			o = srai[31: 0];
	OP_SLL:			o = sllr[63:32];
	OP_SRL:			o = srlr[31 :0];
	OP_SRA:			o = srar[31 :0];
	OP_REMASK:
		if (ir.r2.Rb.num[5]) begin	// same bit as E xpand
			case(ir.r2.Rb.num[4:0])
			5'd1: o = a;		// 1:1 map
			5'd2:	o = {{2{a[7]}},{2{a[6]}},{2{a[5]}},{2{a[4]}},{2{a[3]}},{2{a[2]}},{2{a[1]}},{2{a[0]}}};	// 1:2
			5'd4: o = {{4{a[3]}},{4{a[2]}},{4{a[1]}},{4{a[0]}}};	// 1:4 map (converting 128-bit lanes to 32-bit lanes)
			default:	o = a;
			endcase
		end
		else begin
			case(ir.r2.Rb.num[4:0])
			5'd1: o = a;		// 1:1 map
			5'd2:	o = {a[14],a[12],a[10],a[8],a[6],a[4],a[2],a[0]};
			5'd4:	o = {a[12],a[8],a[4],a[0]};
			default:	o = a;
			endcase
		end
	default:	o = 'd0;
	endcase
OP_ADDI:			o = a + imm;
OP_SUBFI:		o = imm - a;
OP_ANDI:			o = a & imm;
OP_ORI:			o = a | imm;
OP_XORI:			o = a ^ imm;
OP_CMPI:
	begin
		o[0] = a == imm;
		o[1] = $signed(a) < $signed(imm);
		o[2] = $signed(a) <= $signed(imm);
		o[4:3] = 2'b0;
		o[5] = a < imm;
		o[6] = a <= imm;
		o[7] = 1'b0;
		o[8] = a != imm;
		o[9] = $signed(a) >= $signed(imm);
		o[10] = $signed(a) > $signed(imm);
		o[12:11] = 2'b0;
		o[13] = a >= imm;
		o[14] = a > imm;
		o[15] = 1'b0;
		o[16] = fcmpi_o[0];
		o[17] = fcmpi_o[1];
		o[18] = fcmpi_o[2];
		o[19] = fcmpi_o[3];
		o[20] = fcmpi_o[4];
		o[21] = fcmpi_o[8];	// <>
		o[22] = fcmpi_o[9];	// >=
		o[23] = fcmpi_o[10];	// >
		o[24] = fcmpi_o[11];	// mag >=
		o[25] = fcmpi_o[12];	// ordered
		o[31:26] = 6'd0;
	end
OP_CMP_EQI:	o = a == imm;
OP_CMP_NEI:	o = a != imm;
OP_CMP_LTI:	o = $signed(a) < $signed(imm);
OP_CMP_GEI:	o = $signed(a) >= $signed(imm);
OP_CMP_LEI:	o = $signed(a) <= $signed(imm);
OP_CMP_GTI:	o = $signed(a) > $signed(imm);
OP_CMP_LTUI:	o = a < imm;
OP_CMP_GEUI:	o	= a >= imm;
OP_CMP_LEUI:	o = a <= imm;
OP_CMP_GTUI:	o = a > imm;
OP_FCMP_EQI:	o = fcmpi_o[0];
OP_FCMP_NEI:	o = fcmpi_o[8]|fcmpi_o[4];	// return 1 if Nan
OP_FCMP_LTI:	o = fcmpi_o[1];
OP_FCMP_LEI:	o = fcmpi_o[2];
OP_FCMP_GTI:	o = fcmpi_o[10];
OP_FCMP_GEI:	o = fcmpi_o[9];
OP_CSR:			o = c;
OP_RET:			o = t + imm;
default:	o = 'd0;
endcase


endmodule
