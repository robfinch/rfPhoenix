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
// modification, are permiRt[6]ed provided that the following conditions are met:
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
//    this software without specific prior wriRt[6]en permission.
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

module rfPhoenix_decoder(ifb, sp_sel, deco);
input InstructionFetchbuf ifb;
input [2:0] sp_sel;
output DecodeBus deco;

reg pfx;
reg op16,op32,op128;

always_comb
begin

	pfx = ifb.pfx.opcode==OP_PFX;

	deco.v = ifb.v;

	deco.rti = ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_R1 && ifb.insn.r2.Rb==OP_RTI;
	deco.flt = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==FLT;
	deco.brk = ifb.insn.any.opcode==OP_BRK;
	deco.irq = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==;
	deco.rex = ifb.insn.any.opcode==OP_R2 && ifb.insn.r2.func==OP_R1 && ifb.insn.r2.Rb==OP_REX;
	deco.Ra = ifb.insn.r2.Ra;
	deco.Rb = ifb.insn.r2.Rb;
	deco.Rm = {3'b110,ifb.insn.r2.Rm};
	deco.Ta = ifb.insn.r2.Ra.vec;
	deco.Tb = ifb.insn.r2.Rb.vec;

	// Rt
	case(ifb.insn.any.opcode)
	OP_R2:	
		case(ifb.insn.r2.func)
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		OP_CMPI,OP_CMP_EQI,OP_CMP_NEI,OP_CMP_LTI,OP_CMP_GEI,OP_CMP_LEI,OP_CMP_GTI,
		OP_CMP_LTUI,OP_CMP_GEUI,OP_CMP_LEUI,OP_CMP_GTUI:
			begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		OP_SLL,OP_SRL,OP_SRA,OP_SLLI,OP_SRLI,OP_SRAI:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_CMP_EQI,OP_CMP_NEI,OP_CMP_LTI,OP_CMP_GEI,OP_CMP_LEI,OP_CMP_GTI,
	OP_CMP_LTUI,OP_CMP_GEUI,OP_CMP_LEUI,OP_CMP_GTUI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	OP_FCMP_EQI,OP_FCMP_NEI,OP_FCMP_LTI,OP_FCMP_GEI,OP_FCMP_LEI,OP_FCMP_GTI:
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

	// Register file writes	
	deco.rfwr = 'd0;
	deco.vrfwr = 'd0;
	case(ifb.insn.any.opcode)
	OP_R2:	
		case(ifb.insn.r2.func)
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		OP_CMP,OP_CMP_EQ,OP_CMP_NE,OP_CMP_LT,OP_CMP_GE,OP_CMP_LE,OP_CMP_GT,
		OP_CMP_LTU,OP_CMP_GEU,OP_CMP_LEU,OP_CMP_GTU:
			begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		OP_SLL,OP_SRL,OP_SRA,OP_SLLI,OP_SRLI,OP_SRAI:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		default:	begin deco.Rt.num = 'd0; deco.Rt.vec = 1'b0; end
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_CMP_EQI,OP_CMP_NEI,OP_CMP_LTI,OP_CMP_GEI,OP_CMP_LEI,OP_CMP_GTI,
	OP_CMP_LTUI,OP_CMP_GEUI,OP_CMP_LEUI,OP_CMP_GTUI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	OP_FCMP_EQI,OP_FCMP_NEI,OP_FCMP_LTI,OP_FCMP_GEI,OP_FCMP_LEI,OP_FCMP_GTI:
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
	// Disable writing r0.
	if (deco.Rt=='d0)
		deco.rfwr = 1'b0;

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
	OP_CMPI,OP_CMP_EQI,OP_CMP_NEI,OP_CMP_LTI,OP_CMP_GEI,OP_CMP_LEI,OP_CMP_GTI,
	OP_CMP_LTUI,OP_CMP_GEUI,OP_CMP_LEUI,OP_CMP_GTUI:
		deco.imm = {{64{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	OP_FCMP_EQI,OP_FCMP_NEI,OP_FCMP_LTI,OP_FCMP_GEI,OP_FCMP_LEI,OP_FCMP_GTI:
		deco.imm = {{64{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
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
		deco.imm[31:16] = ifb.pfx.imm;
		//if (ifb.pfx2.opcode==OP_PFX)
		//	deco.imm[79:50] = ifb.pfx.imm;
	end


	// Figure 16-bit ops
	op16 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_CNTLZ:		op16 = ifb.insn[38:37]==2'd0;
			OP_FCLASS:	op16 = ifb.insn[38:37]==2'd0;
			OP_FFINITE:	op16 = ifb.insn[38:37]==2'd0;
			OP_I2F:	op16 = ifb.insn[38:37]==2'd0;
			OP_F2I:	op16 = ifb.insn[38:37]==2'd0;
			OP_FTRUNC:	op16 = ifb.insn[38:37]==2'd0;
			OP_FABS:	op16 = ifb.insn[38:37]==2'd0;
			OP_FNABS:	op16 = ifb.insn[38:37]==2'd0;
			OP_FNEG:	op16 = ifb.insn[38:37]==2'd0;
			OP_FSIGN:	op16 = ifb.insn[38:37]==2'd0;
			default:	;
			endcase
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:
			op16 = ifb.insn[38:37]==2'd0;
		OP_CMP,OP_CMP_EQ,OP_CMP_NE,OP_CMP_LT,OP_CMP_GE,OP_CMP_LE,OP_CMP_GT,
		OP_CMP_LTU,OP_CMP_GEU,OP_CMP_LEU,OP_CMP_GTU:
			op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_EQ:	op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_NE:	op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_LT:	op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_GE:	op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_LE:	op16 = ifb.insn[38:37]==2'd0;
		OP_FCMP_GT:	op16 = ifb.insn[38:37]==2'd0;
		OP_FADD:		op16 = ifb.insn[36]==1'd0;
		OP_FSUB:		op16 = ifb.insn[36]==1'd0;
		default:	;
		endcase
	OP_ADDI,OP_SUBFI,OP_ANDI,OP_ORI,OP_XORI:
		op16 = ifb.insn[39]==1'd0;
	OP_CMP_EQI,OP_CMP_NEI,OP_CMP_LTI,OP_CMP_GEI,OP_CMP_LEI,OP_CMP_GTI,
	OP_CMP_LTUI,OP_CMP_GEUI,OP_CMP_LEUI,OP_CMP_GTUI:
		op16 = ifb.insn[39]==1'b0;
	OP_FCMP_EQI,OP_FCMP_NEI,OP_FCMP_LTI,OP_FCMP_GEI,OP_FCMP_LEI,OP_FCMP_GTI:
		op16 = ifb.insn[39]==1'd0;
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

	// Figure 128-bit ops
	op128 = FALSE;
	case(ifb.insn.any.opcode)		
	OP_R2:
		case(ifb.insn.r2.func)
		OP_R1:
			case(ifb.insn.r1.func1)
			OP_FCLASS:	op128 = ifb.insn[38:37]==2'd2;
			OP_FFINITE:	op128 = ifb.insn[38:37]==2'd2;
			OP_I2F:	op128 = ifb.insn[38:37]==2'd2;
			OP_F2I:	op128 = ifb.insn[38:37]==2'd2;
			OP_FTRUNC:	op128 = ifb.insn[38:37]==2'd2;
			OP_FABS:	op128 = ifb.insn[38:37]==2'd2;
			OP_FNABS:	op128 = ifb.insn[38:37]==2'd2;
			OP_FNEG:	op128 = ifb.insn[38:37]==2'd2;
			OP_FSIGN:	op128 = ifb.insn[38:37]==2'd2;
			default:	;
			endcase
		OP_ADD,OP_SUB,OP_AND,OP_OR,OP_XOR:
			op128 = ifb.insn[38:37]==2'd2;
		OP_CMP,OP_CMP_EQ,OP_CMP_NE,OP_CMP_LT,OP_CMP_GE,OP_CMP_LE,OP_CMP_GT,
		OP_CMP_LTU,OP_CMP_GEU,OP_CMP_LEU,OP_CMP_GTU:
			op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_EQ:	op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_NE:	op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_LT:	op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_GE:	op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_LE:	op128 = ifb.insn[38:37]==2'd2;
		OP_FCMP_GT:	op128 = ifb.insn[38:37]==2'd2;
		OP_FADD128:	op128 = TRUE;
		OP_FSUB128:	op128 = TRUE;
		default:	;
		endcase
	OP_FMA128,OP_FMS128,OP_FNMA128,OP_FNMS128:
		op128 = TRUE;
	default:	;
	endcase

	// Set precision for ops
	if (op16)
		deco.prc = PRC16;
	else if (op128)
		deco.prc = PRC128;
	else
		deco.prc = PRC32;

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

	if (deco.br|deco.store)
		deco.Rc = ifb.insn.r2.Rt;
	else
		deco.Rc = ifb.insn.f3.Rc;

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
								ifb.insn.any.opcode==OP_FMA ||
								ifb.insn.any.opcode==OP_FMS ||
								ifb.insn.any.opcode==OP_FNMA ||
								ifb.insn.any.opcode==OP_FNMS
								;
	deco.hasRc = 	ifb.insn.any.opcode==OP_FMA ||
								ifb.insn.any.opcode==OP_FMS ||
								ifb.insn.any.opcode==OP_FNMA ||
								ifb.insn.any.opcode==OP_FNMS
								;
	deco.hasRm =  !deco.cjb && !deco.br && !deco.pfx;
	deco.hasRt =	!deco.pfx;

	deco.is_vector = (deco.hasRt & deco.Rt.vec) |
									(deco.hasRa & deco.Ra.vec) |
									(deco.hasRb & deco.Rb.vec) |
									(deco.hasRc & deco.Rc.vec) ;

	deco.Rtsrc = 	1'b0;//deco.hasRt & (deco.br|deco.store);

	if ((deco.hasRa & deco.Ra.vec) | ((deco.loadn|deco.storen) & (deco.hasRb & deco.Rb.vec)) | (deco.hasRt & deco.Rt.vec)) deco.memsz = vect;
	deco.need_steps = deco.memsz==vect && !((deco.loadr|deco.storer) && !deco.Ra.vec);
end

endmodule
