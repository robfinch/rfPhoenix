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

import const_pkg::*;
import rfPhoenixPkg::*;

module rfPhoenix_decoder(ifb, deco);
input instruction_fetchbuf_t ifb;
output decode_bus_t deco;

reg pfx;
reg op16,op32,op64,op128;
reg ret;
reg [2:0] sp_sel;

always_comb
begin

	sp_sel = ifb.sp_sel;
	pfx = ifb.pfx.opcode==OP_PFX;
	ret = ifb.insn.any.opcode==OP_RET;
	deco.v = ifb.v;

	deco.rti = ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_R1 && ifb.insn.r2.Rb==OP_RTI;
	deco.flt = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==FLT;
	deco.brk = ifb.insn.any.opcode==OP_BRK;
	deco.irq = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==;
	deco.rex = ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_R1 && ifb.insn.r2.Rb==OP_REX;
	deco.Ra = ifb.insn.r2.Ra;
	deco.Rb = ret ? 7'd59 : ifb.insn.r2.Rb;
	deco.Rm = {3'b110,ifb.insn.r2.Rm};
	deco.Ta = ifb.insn.r2.Ra.vec;
	deco.Tb = ifb.insn.r2.Rb.vec;

	// Rt
	case(ifb.insn.any.opcode)
	OP_R2:	
		case(ifb.insn.r2.func)
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		OP_SLL,OP_SRL,OP_SRA,OP_SLLI,OP_SRLI,OP_SRAI:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_CMP,OP_FCMP:
		begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_CMPI16,OP_CMPI32,OP_CMPI64:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_FCMPI16,OP_FCMPI32,OP_FCMPI64:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec;end
	OP_FMA,OP_FMS,OP_FNMA,OP_FNMS:	begin deco.Rt = ifb.insn.f3.Rt; deco.Rt.vec = ifb.insn.f3.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_NOP:
		begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	OP_CALL:	begin deco.Rt = ifb.insn.call.Rt==2'b00 ? 7'd0 : {4'b1100,ifb.insn.call.Rt}; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	OP_BSR:	begin deco.Rt = ifb.insn.call.Rt==2'b00 ? 7'd0 : {4'b1100,ifb.insn.call.Rt}; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	OP_RET:	begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT:	begin deco.Rt = ifb.insn.ls.Rt; deco.Rt.vec = ifb.insn.ls.Rt.vec; deco.Tt = ifb.insn.ls.Rt.vec; end
	OP_STB,OP_STW,OP_STT:	begin deco.Rt = ifb.insn.ls.Rt; deco.Rt.vec = ifb.insn.ls.Rt.vec; deco.Tt = ifb.insn.ls.Rt.vec; end
	OP_CSR:	begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.ri.Rt.vec; end
	OP_Bcc,OP_FBcc:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	endcase
	
	// Register file writes	
	deco.rfwr = 'd0;
	deco.vrfwr = 'd0;
	case(ifb.insn.any.opcode)
	OP_R2:	
		case(ifb.insn.r2.func)
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		OP_SLL,OP_SRL,OP_SRA,OP_SLLI,OP_SRLI,OP_SRAI:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		default:	begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_CMP,OP_CMPI16,OP_CMPI32,OP_CMPI64:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_FCMP,OP_FCMPI16,OP_FCMPI32,OP_FCMPI64:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_FMA,OP_FMS,OP_FNMA,OP_FNMS:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_NOP:
		begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	OP_CALL:	begin deco.rfwr = ifb.insn.call.Rt!='d0; end
	OP_BSR:	begin deco.rfwr = ifb.insn.call.Rt!='d0; end
	OP_RET: 	begin deco.rfwr = ifb.insn.r2.Rt != 'd0; end
	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_CSR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	default:	begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	endcase

	deco.multicycle = 'd0;
	case(ifb.insn.any.opcode)
	OP_R2:
		case(ifb.insn.r2.func)
		OP_LDBX,OP_LDBUX,OP_LDWX,OP_LDWUX,OP_LDTX,
		OP_STBX,OP_STWX,OP_STTX:	deco.multicycle = 1'b0;
		default:	deco.multicycle = 'd0;
		endcase
	OP_FMA,OP_FMS,OP_FNMA,OP_FNMS:	deco.multicycle = 1'b1;
	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT,
	OP_STB,OP_STW,OP_STT:	deco.multicycle = 1'b0;
	default:	deco.multicycle = 'd0;
	endcase

	deco.imm = 'd0;
	case(ifb.insn.any.opcode)
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_PEEKQ,OP_POPQ,OP_STATQ,OP_RESETQ:	deco.imm = deco.Ra[3:0];
			default:	deco.imm = 'd0;
			endcase
		OP_PUSHQ:	deco.imm = deco.Ra[3:0];
		default:	deco.imm = 'd0;
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		deco.imm = {{64{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	OP_CMPI16,OP_CMPI32,OP_CMPI64:
		deco.imm = {{67{ifb.insn.cmpi.imm[12]}},ifb.insn.cmpi.imm};
	OP_FCMPI16,OP_FCMPI32,OP_FCMPI64:
		deco.imm = {{67{ifb.insn.cmpi.imm[12]}},ifb.insn.cmpi.imm};
	OP_CALL:	deco.imm = ifb.insn.call.target;
	OP_BSR:	deco.imm = ifb.insn.call.target;
	OP_Bcc,OP_FBcc:	deco.imm = {{64{ifb.insn.br.disp[15]}},ifb.insn.br.disp};
	OP_RET:	deco.imm = {{64{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT,
	OP_STB,OP_STW,OP_STT:
		deco.imm = {{64{ifb.insn.ls.disp[15]}},ifb.insn.ls.disp};
	OP_CSR:	deco.imm = {{64{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	default:	deco.imm = 'd0;
	endcase
	// Handle postfixes	
	if (pfx) begin
		deco.imm[127:13] = {{83{ifb.pfx.imm[31]}},ifb.pfx.imm};
`ifdef SUPPORT_64BIT_OPS		
		if (ifb.pfx2.opcode==OP_PFX) begin
			deco.imm[127:45] = {{51{ifb.pfx2.imm[31]}},ifb.pfx.imm};
			/*
			if (ifb.pfx3.opcode==OP_PFX) begin
				deco.imm[127:80] = {{16{ifb.pfx3.imm[31]}},ifb.pfx.imm};
				if (ifb.pfx4.opcode==OP_PFX) begin
					deco.imm[127:112] = ifb.pfx4.imm[15:0];
			end
			*/
		end
`endif		
	end

	// Figure 16-bit ops
	op16 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_CNTLZ:		op16 = ifb.insn[38:37]==PRC16;
			OP_FCLASS:	op16 = ifb.insn[38:37]==PRC16;
			OP_FFINITE:	op16 = ifb.insn[38:37]==PRC16;
			OP_I2F:	op16 = ifb.insn[38:37]==PRC16;
			OP_F2I:	op16 = ifb.insn[38:37]==PRC16;
			OP_FTRUNC:	op16 = ifb.insn[38:37]==PRC16;
			OP_FABS:	op16 = ifb.insn[38:37]==PRC16;
			OP_FNABS:	op16 = ifb.insn[38:37]==PRC16;
			OP_FNEG:	op16 = ifb.insn[38:37]==PRC16;
			OP_FSIGN:	op16 = ifb.insn[38:37]==PRC16;
			OP_SEXTB:	op16 = ifb.insn[38:37]==PRC16;
			default:	;
			endcase
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:
			op16 = ifb.insn[38:37]==PRC16;
		OP_FADD:		op16 = ifb.insn[36]==1'd0;
		OP_FSUB:		op16 = ifb.insn[36]==1'd0;
		default:	;
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		op16 = ifb.insn[39]==1'd0;
	OP_CMP:	op16=ifb.insn.cmp.sz==2'b00;
	OP_CMPI16:	op16 = TRUE;
	OP_FCMP:	op16=ifb.insn.cmp.sz==2'b00;
	OP_FCMPI16:	op16 = TRUE;
	OP_FMA16,OP_FMS16,OP_FNMA16,OP_FNMS16:
		op16 = TRUE;
	default:	;
	endcase

	// Figure 32-bit ops
	/* This decoder redundant. The default is to assume 32-bit ops if not 16 or
	   128-bit.

	op32 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_FCLASS:	op32 = ifb.insn[38:37]==2'd1;
			OP_FFINITE:	op32 = ifb.insn[38:37]==2'd1;
			OP_I2F:	op32 = ifb.insn[38:37]==2'd1;
			OP_F2I:	op32 = ifb.insn[38:37]==2'd1;
			OP_FTRUNC:	op32 = ifb.insn[38:37]==2'd1;
			OP_FABS:	op32 = ifb.insn[38:37]==2'd1;
			OP_FNABS:	op32 = ifb.insn[38:37]==2'd1;
			OP_FNEG:	op32 = ifb.insn[38:37]==2'd1;
			OP_FSIGN:	op32 = ifb.insn[38:37]==2'd1;
			default:	op32 = TRUE;
			default:	;
			endcase
		OP_FCMP_EQ:	op32 = ifb.insn[38:37]==2'd1;
		OP_FCMP_NE:	op32 = ifb.insn[38:37]==2'd1;
		OP_FCMP_LT:	op32 = ifb.insn[38:37]==2'd1;
		OP_FCMP_GE:	op32 = ifb.insn[38:37]==2'd1;
		OP_FCMP_LE:	op32 = ifb.insn[38:37]==2'd1;
		OP_FCMP_GT:	op32 = ifb.insn[38:37]==2'd1;
		OP_FADD:		op32 = ifb.insn[36]==1'd1;
		OP_FSUB:		op32 = ifb.insn[36]==1'd1;
		default:	op32 = TRUE;
		default:	;
		endcase
	OP_FMA,OP_FMS,OP_FNMA,OP_FNMS:
		op32 = TRUE;
	default:	op32 = TRUE;
	default:	;
	endcase
	*/

	// Figure 64-bit ops
	op64 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_CNTLZ:		op64 = ifb.insn[38:37]==PRC64;
			OP_FCLASS:	op64 = ifb.insn[38:37]==PRC64;
			OP_FFINITE:	op64 = ifb.insn[38:37]==PRC64;
			OP_I2F:	op64 = ifb.insn[38:37]==PRC64;
			OP_F2I:	op64 = ifb.insn[38:37]==PRC64;
			OP_FTRUNC:	op64 = ifb.insn[38:37]==PRC64;
			OP_FABS:	op64 = ifb.insn[38:37]==PRC64;
			OP_FNABS:	op64 = ifb.insn[38:37]==PRC64;
			OP_FNEG:	op64 = ifb.insn[38:37]==PRC64;
			OP_FSIGN:	op64 = ifb.insn[38:37]==PRC64;
			OP_SEXTB:	op64 = ifb.insn[38:37]==PRC64;
			default:	;
			endcase
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:
			op64 = ifb.insn[38:37]==PRC64;
	//	OP_FADD:		op64 = ifb.insn[36]==1'd0;
//		OP_FSUB:		op64 = ifb.insn[36]==1'd0;
		default:	;
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		op64 = pfx && ifb.pfx.prc==PRC64;
	OP_CMP:	op64=ifb.insn.cmp.sz==PRC64;
	OP_CMPI64:	op64 = TRUE;
	OP_FCMP:	op64=ifb.insn.cmp.sz==PRC64;
	OP_FCMPI64:	op64 = TRUE;
	OP_FMA64,OP_FMS64,OP_FNMA64,OP_FNMS64:
		op64 = TRUE;
	default:	;
	endcase

	// Figure 128-bit ops
	op128 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_CNTLZ:		op128 = ifb.insn[38:37]==PRC128;
			OP_FCLASS:	op128 = ifb.insn[38:37]==PRC128;
			OP_FFINITE:	op128 = ifb.insn[38:37]==PRC128;
			OP_I2F:	op128 = ifb.insn[38:37]==PRC128;
			OP_F2I:	op128 = ifb.insn[38:37]==PRC128;
			OP_FTRUNC:	op128 = ifb.insn[38:37]==PRC128;
			OP_FABS:	op128 = ifb.insn[38:37]==PRC128;
			OP_FNABS:	op128 = ifb.insn[38:37]==PRC128;
			OP_FNEG:	op128 = ifb.insn[38:37]==PRC128;
			OP_FSIGN:	op128 = ifb.insn[38:37]==PRC128;
			OP_SEXTB:	op128 = ifb.insn[38:37]==PRC128;
			default:	;
			endcase
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:
			op128 = ifb.insn[38:37]==PRC128;
//		OP_FADD:		op128 = ifb.insn[36]==1'd0;
//		OP_FSUB:		op128 = ifb.insn[36]==1'd0;
		default:	;
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		op128 = pfx && ifb.pfx.prc==PRC128;
	OP_CMP:	op128=ifb.insn.cmp.sz==PRC128;
	OP_FCMP:	op128=ifb.insn.cmp.sz==PRC128;
	OP_FMA128,OP_FMS128,OP_FNMA128,OP_FNMS128:
		op128 = TRUE;
	default:	;
	endcase

	// Set precision for ops
	if (op16)
		deco.prc = PRC16;
	else if (op64)
		deco.prc = PRC64;
	else if (op128)
		deco.prc = PRC128;
	else
		deco.prc = PRC32;
	if (pfx)
		deco.prc = ifb.pfx.prc;

	deco.storer = 'd0;
	deco.storen = 'd0;
	deco.loadr = 'd0;
	deco.loadn = 'd0;
	case(ifb.insn.any.opcode)
	OP_R2:
		case(ifb.insn.r2.func)
		OP_LDBX,OP_LDBUX,OP_LDWX,OP_LDWUX,OP_LDTX:	deco.loadn = 1'b1;
		OP_STBX,OP_STWX,OP_STTX:	deco.storen = 1'b1;
		default:	;
		endcase
	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT:	deco.loadr = 1'b1;
	OP_STB,OP_STW,OP_STT:	deco.storer = 1'b1;
	default:	;
	endcase

	deco.br = ifb.insn.any.opcode==OP_Bcc || ifb.insn.any.opcode==OP_FBcc;
	deco.cjb = ifb.insn.any.opcode==OP_CALL || ifb.insn.any.opcode==OP_BSR;
	deco.store = deco.storer|deco.storen;
	deco.stcr = ifb.insn.any.opcode==OP_STCR || (ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_STCRX);
	deco.loadu = ifb.insn.any.opcode==OP_LDBU||ifb.insn.any.opcode==OP_LDWU || (ifb.insn.any.opcode==OP_R2 && (ifb.insn.r2.func==OP_LDBUX || ifb.insn.r2.func==OP_LDWUX));
	deco.load = deco.loadr|deco.loadn;
	deco.ldsr = ifb.insn.any.opcode==OP_LDSR || (ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_LDSRX);
	deco.mem = deco.store|deco.load|deco.stcr|deco.ldsr;
	deco.popq = ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_R1 && ifb.insn.r1.func1==OP_POPQ;

	deco.Rc = ifb.insn.f3.Rc;

	// Stack pointer spec mux
	if (deco.Ra==7'd31)
		case(sp_sel)
		3'd1:	deco.Ra = 7'd60;
		3'd2:	deco.Ra = 7'd61;
		3'd3:	deco.Ra = 7'd62;
		3'd4:	deco.Ra = 7'd63;
		default:	;
		endcase

	if (deco.Rb==7'd31)
		case(sp_sel)
		3'd1:	deco.Rb = 7'd60;
		3'd2:	deco.Rb = 7'd61;
		3'd3:	deco.Rb = 7'd62;
		3'd4:	deco.Rb = 7'd63;
		default:	;
		endcase

	if (deco.Rc==7'd31)
		case(sp_sel)
		3'd1:	deco.Rc = 7'd60;
		3'd2:	deco.Rc = 7'd61;
		3'd3:	deco.Rc = 7'd62;
		3'd4:	deco.Rc = 7'd63;
		default:	;
		endcase
	
	if (deco.Rt==7'd31)
		case(sp_sel)
		3'd1:	deco.Rt = 7'd60;
		3'd2:	deco.Rt = 7'd61;
		3'd3:	deco.Rt = 7'd62;
		3'd4:	deco.Rt = 7'd63;
		default:	;
		endcase

	if (deco.br|deco.store)
		deco.Rc = deco.Rt;

	// Memory operation sizes
	case(ifb.insn.any.opcode)
	OP_LDB,OP_LDBU,OP_STB:	deco.memsz = byt;
	OP_LDW,OP_LDWU,OP_STW:	deco.memsz = wyde;
	OP_LDT,OP_STT:	deco.memsz = tetra;
	OP_R2:
		case (ifb.insn.r2.func)
		OP_LDBX,OP_LDBUX,OP_STBX:	deco.memsz = byt;
		OP_LDWX,OP_LDWUX,OP_STWX:	deco.memsz = wyde;
		OP_LDTX,OP_STTX:	deco.memsz = tetra;
		default:	deco.memsz = tetra;
		endcase
	default:	deco.memsz = tetra;
	endcase
	case(ifb.insn.any.opcode)
	OP_R2:
		case (ifb.insn.r2.func)
		OP_LDCX,OP_STCX:	deco.compress = 1'b1;
		default:	deco.compress = 1'b0;
		endcase
	OP_LDC,OP_STC:	deco.compress = 1'b1;
	default:	deco.compress = 1'b0;
	endcase

	deco.pfx = ifb.insn.any.opcode==OP_PFX;

	deco.csr = ifb.insn.any.opcode==OP_CSR;
	deco.csrrd = ifb.insn.any.opcode==OP_CSR && ifb.insn.csr.func==2'd0;
	deco.csrrw = ifb.insn.any.opcode==OP_CSR && ifb.insn.csr.func==2'd1;
	deco.csrrc = ifb.insn.any.opcode==OP_CSR && ifb.insn.csr.func==2'd2;
	deco.csrrs = ifb.insn.any.opcode==OP_CSR && ifb.insn.csr.func==2'd3;

	deco.hasRa = ifb.insn.any.opcode!=OP_PFX && !deco.cjb;
	deco.hasRb = (ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func!=OP_R1) ||
								ifb.insn.any.opcode==OP_FMA16 ||
								ifb.insn.any.opcode==OP_FMS16 ||
								ifb.insn.any.opcode==OP_FNMA16 ||
								ifb.insn.any.opcode==OP_FNMS16 ||
								ifb.insn.any.opcode==OP_FMA64 ||
								ifb.insn.any.opcode==OP_FMS64 ||
								ifb.insn.any.opcode==OP_FNMA64 ||
								ifb.insn.any.opcode==OP_FNMS64 ||
								ifb.insn.any.opcode==OP_FMA128 ||
								ifb.insn.any.opcode==OP_FMS128 ||
								ifb.insn.any.opcode==OP_FNMA128 ||
								ifb.insn.any.opcode==OP_FNMS128 ||
								ifb.insn.any.opcode==OP_FMA ||
								ifb.insn.any.opcode==OP_FMS ||
								ifb.insn.any.opcode==OP_FNMA ||
								ifb.insn.any.opcode==OP_FNMS
								;
	deco.hasRc = 	deco.br || deco.store ||
								ifb.insn.any.opcode==OP_FMA16 ||
								ifb.insn.any.opcode==OP_FMS16 ||
								ifb.insn.any.opcode==OP_FNMA16 ||
								ifb.insn.any.opcode==OP_FNMS16 ||
								ifb.insn.any.opcode==OP_FMA64 ||
								ifb.insn.any.opcode==OP_FMS64 ||
								ifb.insn.any.opcode==OP_FNMA64 ||
								ifb.insn.any.opcode==OP_FNMS64 ||
								ifb.insn.any.opcode==OP_FMA128 ||
								ifb.insn.any.opcode==OP_FMS128 ||
								ifb.insn.any.opcode==OP_FNMA128 ||
								ifb.insn.any.opcode==OP_FNMS128 ||
								ifb.insn.any.opcode==OP_FMA ||
								ifb.insn.any.opcode==OP_FMS ||
								ifb.insn.any.opcode==OP_FNMA ||
								ifb.insn.any.opcode==OP_FNMS
								;
	deco.hasRm =  !deco.cjb && !deco.br && !deco.pfx;
	deco.hasRt =	!deco.pfx && !deco.br;

	deco.is_vector = (deco.hasRt & deco.Rt.vec) |
									(deco.hasRa & deco.Ra.vec) |
									(deco.hasRb & deco.Rb.vec) |
									(deco.hasRc & deco.Rc.vec) ;

	deco.Rtsrc = 	1'b0;//deco.hasRt & (deco.br|deco.store);

	if ((deco.hasRa & deco.Ra.vec) | ((deco.loadn|deco.storen) & (deco.hasRb & deco.Rb.vec)) | (deco.hasRt & deco.Rt.vec)) deco.memsz = vect;
	deco.need_steps = deco.memsz==vect && !((deco.loadr|deco.storer) && !deco.Ra.vec);
end

endmodule
