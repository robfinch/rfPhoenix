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

always_comb
begin

	deco.v = ifb.v;

	deco.rti = ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==RTI;
	deco.flt = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==FLT;
	deco.brk = ifb.insn.any.opcode==BRK;
	deco.irq = 1'b0;//ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==;
	deco.rex = ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r2.Rb==REX;
	deco.Ra = ifb.insn.r2.Ra;
	deco.Rb = ifb.insn.r2.Rb;
	deco.Rm = {3'b100,ifb.insn.r2.Rm};
	deco.Ta = ifb.insn.r2.Ra.vec;
	deco.Tb = ifb.insn.r2.Rb.vec;

	// Rt
	case(ifb.insn.any.opcode)
	R2:	
		case(ifb.insn.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		CMPI,CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
		CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
			begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		SLL,SRL,SRA,SLLI,SRLI,SRAI:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
		default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec;end
	FMA,FMS,FNMA,FNMS:	begin deco.Rt = ifb.insn.f3.Rt; deco.Rt.vec = ifb.insn.f3.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	NOP:
		begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	CALL:	begin deco.Rt = ifb.insn.call.Rt==2'b00 ? 7'd0 : {4'b1100,ifb.insn.call.Rt}; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	BSR:	begin deco.Rt = ifb.insn.call.Rt==2'b00 ? 7'd0 : {4'b1100,ifb.insn.call.Rt}; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	RET:	begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	LDB,LDBU,LDW,LDWU,LDT:	begin deco.Rt = ifb.insn.ls.Rt; deco.Rt.vec = ifb.insn.ls.Rt.vec; deco.Tt = ifb.insn.ls.Rt.vec; end
	STB,STW,STT:	begin deco.Rt = ifb.insn.ls.Rt; deco.Rt.vec = ifb.insn.ls.Rt.vec; deco.Tt = ifb.insn.ls.Rt.vec; end
	CSR:	begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.ri.Rt.vec; end
	Bcc,FBcc:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	endcase
	
	// Stack pointer spec mux
	if (deco.Ra==7'd31)
		case(sp_sel)
		3'd1:	deco.Ra = 7'd52;
		3'd2:	deco.Ra = 7'd53;
		3'd3:	deco.Ra = 7'd54;
		3'd4:	deco.Ra = 7'd55;
		default:	;
		endcase

	if (deco.Rb==7'd31)
		case(sp_sel)
		3'd1:	deco.Rb = 7'd52;
		3'd2:	deco.Rb = 7'd53;
		3'd3:	deco.Rb = 7'd54;
		3'd4:	deco.Rb = 7'd55;
		default:	;
		endcase

	if (deco.Rc==7'd31)
		case(sp_sel)
		3'd1:	deco.Rc = 7'd52;
		3'd2:	deco.Rc = 7'd53;
		3'd3:	deco.Rc = 7'd54;
		3'd4:	deco.Rc = 7'd55;
		default:	;
		endcase
	
	if (deco.Rt==7'd31)
		case(sp_sel)
		3'd1:	deco.Rt = 7'd52;
		3'd2:	deco.Rt = 7'd53;
		3'd3:	deco.Rt = 7'd54;
		3'd4:	deco.Rt = 7'd55;
		default:	;
		endcase

	// Register file writes	
	deco.rfwr = 'd0;
	deco.vrfwr = 'd0;
	case(ifb.insn.any.opcode)
	R2:	
		case(ifb.insn.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		CMPI,CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
		CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
			begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		SLL,SRL,SRA,SLLI,SRLI,SRAI:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
		default:	begin deco.Rt.num = 'd0; deco.Rt.vec = 1'b0; end
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	FMA,FMS,FNMA,FNMS:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	NOP:
		begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	CALL:	begin deco.rfwr = ifb.insn.call.Rt!='d0; end
	BSR:	begin deco.rfwr = ifb.insn.call.Rt!='d0; end
	RET: 	begin deco.rfwr = ifb.insn.r2.Rt != 'd0; end
	LDB,LDBU,LDW,LDWU,LDT:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	CSR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	default:	begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	endcase
	// Disable writing r0.
	if (deco.Rt=='d0)
		deco.rfwr = 1'b0;

	deco.multicycle = 'd0;
	case(ifb.insn.any.opcode)
	R2:
		case(ifb.insn.r2.func)
		LDBX,LDBUX,LDWX,LDWUX,LDTX,
		STBX,STWX,STTX:	deco.multicycle = 1'b0;
		default:	deco.multicycle = 'd0;
		endcase
	FMA,FMS,FNMA,FNMS:	deco.multicycle = 1'b1;
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:	deco.multicycle = 1'b0;
	default:	deco.multicycle = 'd0;
	endcase

	deco.imm = 'd0;
	case(ifb.insn.any.opcode)
	R2:
		case(ifb.insn.r2.func)
		R1:
			case(ifb.insn.r1.func1)
			PEEKQ,POPQ,STATQ,RESETQ:	deco.imm = deco.Ra[3:0];
			default:	deco.imm = 'd0;
			endcase
		PUSHQ:	deco.imm = deco.Ra[3:0];
		default:	deco.imm = 'd0;
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	CMPI,CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	CALL:	deco.imm = ifb.insn.call.target;
	BSR:	deco.imm = ifb.insn.call.target;
	Bcc,FBcc:	deco.imm = {{16{ifb.insn.br.disp[15]}},ifb.insn.br.disp};
	RET:	deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:
		deco.imm = {{16{ifb.insn.ls.disp[15]}},ifb.insn.ls.disp};
	CSR:	deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	default:	deco.imm = 'd0;
	endcase	
	if (ifb.pfx.opcode==PFX)
		deco.imm[31:16] = ifb.pfx.imm;

	deco.storer = 'd0;
	deco.storen = 'd0;
	deco.loadr = 'd0;
	deco.loadn = 'd0;
	case(ifb.insn.any.opcode)
	R2:
		case(ifb.insn.r2.func)
		LDBX,LDBUX,LDWX,LDWUX,LDTX:	deco.loadn = 1'b1;
		STBX,STWX,STTX:	deco.storen = 1'b1;
		default:	;
		endcase
	LDB,LDBU,LDW,LDWU,LDT:	deco.loadr = 1'b1;
	STB,STW,STT:	deco.storer = 1'b1;
	default:	;
	endcase

	deco.br = ifb.insn.any.opcode==Bcc || ifb.insn.any.opcode==FBcc;
	deco.cjb = ifb.insn.any.opcode==CALL || ifb.insn.any.opcode==BSR;
	deco.store = deco.storer|deco.storen;
	deco.stcr = ifb.insn.any.opcode==STCR || (ifb.insn.any.opcode==R2 && ifb.insn.r2.func==STCRX);
	deco.loadu = ifb.insn.any.opcode==LDBU||ifb.insn.any.opcode==LDWU || (ifb.insn.any.opcode==R2 && (ifb.insn.r2.func==LDBUX || ifb.insn.r2.func==LDWUX));
	deco.load = deco.loadr|deco.loadn;
	deco.ldsr = ifb.insn.any.opcode==LDSR || (ifb.insn.any.opcode==R2 && ifb.insn.r2.func==LDSRX);
	deco.mem = deco.store|deco.load|deco.stcr|deco.ldsr;
	deco.popq = ifb.insn.any.opcode==R2 && ifb.insn.r2.func==R1 && ifb.insn.r1.func1==POPQ;

	if (deco.br|deco.store)
		deco.Rc = ifb.insn.r2.Rt;
	else
		deco.Rc = ifb.insn.f3.Rc;

	// Memory operation sizes
	case(ifb.insn.any.opcode)
	LDB,LDBU,STB:	deco.memsz = byt;
	LDW,LDWU,STW:	deco.memsz = wyde;
	LDT,STT:	deco.memsz = tetra;
	R2:
		case (ifb.insn.r2.func)
		LDBX,LDBUX,STBX:	deco.memsz = byt;
		LDWX,LDWUX,STWX:	deco.memsz = wyde;
		LDTX,STTX:	deco.memsz = tetra;
		default:	deco.memsz = tetra;
		endcase
	default:	deco.memsz = tetra;
	endcase

	deco.pfx = ifb.insn.any.opcode==PFX;

	deco.csr = ifb.insn.any.opcode==CSR;
	deco.csrrd = ifb.insn.any.opcode==CSR && ifb.insn.csr.func==2'd0;
	deco.csrrw = ifb.insn.any.opcode==CSR && ifb.insn.csr.func==2'd1;
	deco.csrrc = ifb.insn.any.opcode==CSR && ifb.insn.csr.func==2'd2;
	deco.csrrs = ifb.insn.any.opcode==CSR && ifb.insn.csr.func==2'd3;

	deco.hasRa = ifb.insn.any.opcode!=PFX && !deco.cjb;
	deco.hasRb = (ifb.insn.any.opcode==R2 && ifb.insn.r2.func!=R1) ||
								ifb.insn.any.opcode==FMA ||
								ifb.insn.any.opcode==FMS ||
								ifb.insn.any.opcode==FNMA ||
								ifb.insn.any.opcode==FNMS
								;
	deco.hasRc = 	ifb.insn.any.opcode==FMA ||
								ifb.insn.any.opcode==FMS ||
								ifb.insn.any.opcode==FNMA ||
								ifb.insn.any.opcode==FNMS
								;
	deco.hasRm =  ifb.insn.r2.m && !deco.cjb && !deco.br && !deco.pfx;
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
