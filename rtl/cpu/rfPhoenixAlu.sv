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

integer m,n;

value_t fcmp_o, fcmpi_o;
value_t fclass_o;
wire [7:0] exp;
wire inf, xz, vz, snan, qnan, xinf;
value_t o2;

DoubleValue sllr, slli;
DoubleValue srlr, srli;
DoubleValue srar, srai;
always_comb
	sllr = {a,{$bits(value_t){ir[33]}}} << b[5:0];
always_comb
	slli = {a,{$bits(value_t){ir[33]}}} << imm[5:0];
always_comb
	srlr = {{$bits(value_t){ir[33]}},a} >> b[5:0];
always_comb
	srli = {{$bits(value_t){ir[33]}},a} >> imm[5:0];
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
		OP_MCMPRSS:
			begin
				o = 'd0;
				m = 0;
				for (n = 0; n < 32; n = n + 1)
					if (a[n]) begin
						o[m] = 1'b1;
						m = m + 1;
					end
			end
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
		default:	o = {31'd0,o2[ir.r2.func[3:0]]};
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
		default:	o = {31'd0,o2[ir.cmpi.N]};
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
		default:	o = {31'd0,o2[ir.r2.func[3:0]]};
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
		default:	o = {31'd0,o2[ir.r2.func[3:0]]};
		endcase
		*/
	end
OP_CSR:			o = c;
OP_RET:	
	if (ir[39])
		o = b;
	else
		o = t + imm;
default:	o = 'd0;
endcase


endmodule
