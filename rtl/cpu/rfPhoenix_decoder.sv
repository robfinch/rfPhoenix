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

module rfPhoenix_decoder(ifb, sp_sel, rz, deco);
input InstructionFetchbuf ifb;
input [2:0] sp_sel;
input rz;
output DecodeBus deco;

always_comb
begin

	deco.rti = ifb.insn.any.opcode==R2 && ifb.insn.r2.Ra==6'd63 && ifb.insn.r2.func==VSRLVI && ifb.insn.r2.Rb==6'd1;
	deco.flt = ifb.insn.any.opcode==R2 && ifb.insn.r2.Ra==6'd63 && ifb.insn.r2.func==VSLLVI && ifb.insn.r2.Rb==6'd1 && ifb.insn.r2.pad==3'b000;
	deco.brk = ifb.insn.any.opcode==R2 && ifb.insn.r2.Ra==6'd63 && ifb.insn.r2.func==VSLLVI && ifb.insn.r2.Rb==6'd1 && ifb.insn.r2.pad==3'b010;
	deco.irq = ifb.insn.any.opcode==R2 && ifb.insn.r2.Ra==6'd63 && ifb.insn.r2.func==VSLLVI && ifb.insn.r2.Rb==6'd1 && ifb.insn.r2.pad==3'b100;
	deco.Ra = ifb.insn.r2.Ra;
	deco.Rb = ifb.insn.r2.Rb;
	deco.Rc = ifb.insn.f3.Rc;
	deco.Rm = {3'b100,ifb.insn.r2.Rm};
	deco.Ta = ifb.insn.r2.Ra.vec;
	deco.Tb = ifb.insn.r2.Rb.vec;

	// Rt
	case(ifb.insn.any.opcode)
	R2:	
		case(ifb.insn.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
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
	CALLA,CALLR:
		begin deco.Rt = {ifb.insn.call.Rt==2'b0} ? 'd0 : {4'b1010,ifb.insn.call.Rt}; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	STB,STW,STT:	begin deco.Rt = ifb.insn.ls.Rt; deco.Rt.vec = ifb.insn.ls.Rt.vec; deco.Tt = ifb.insn.ls.Rt.vec; end
	CSR:	begin deco.Rt = ifb.insn.ri.Rt; deco.Rt.vec = ifb.insn.ri.Rt.vec; deco.Tt = ifb.insn.ri.Rt.vec; end
	Bcc,FBcc:	begin deco.Rt = ifb.insn.r2.Rt; deco.Rt.vec = ifb.insn.r2.Rt.vec; deco.Tt = ifb.insn.r2.Rt.vec; end
	default:	begin deco.Rt = 'd0; deco.Rt.vec = 1'b0; deco.Tt = 1'b0; end
	endcase
	
	// Stack pointer spec mux
	if (deco.Ra==7'd31)
		case(sp_sel)
		3'd1:	deco.Ra = 7'd44;
		3'd2:	deco.Ra = 7'd45;
		3'd3:	deco.Ra = 7'd46;
		3'd4:	deco.Ra = 7'd47;
		default:	;
		endcase

	if (deco.Rb==7'd31)
		case(sp_sel)
		3'd1:	deco.Rb = 7'd44;
		3'd2:	deco.Rb = 7'd45;
		3'd3:	deco.Rb = 7'd46;
		3'd4:	deco.Rb = 7'd47;
		default:	;
		endcase

	if (deco.Rc==7'd31)
		case(sp_sel)
		3'd1:	deco.Rc = 7'd44;
		3'd2:	deco.Rc = 7'd45;
		3'd3:	deco.Rc = 7'd46;
		3'd4:	deco.Rc = 7'd47;
		default:	;
		endcase
	
	if (deco.Rt==7'd31)
		case(sp_sel)
		3'd1:	deco.Rt = 7'd44;
		3'd2:	deco.Rt = 7'd45;
		3'd3:	deco.Rt = 7'd46;
		3'd4:	deco.Rt = 7'd47;
		default:	;
		endcase

	deco.multicycle = 'd0;
	case(ifb.insn.any.opcode)
	FMA,FMS,FNMA,FNMS:	deco.multicycle = 1'b1;
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:	deco.multicycle = 1'b1;
	default:	deco.multicycle = 'd0;
	endcase

	// Register file writes	
	deco.rfwr = 'd0;
	deco.vrfwr = 'd0;
	case(ifb.insn.any.opcode)
	R2:	
		case(ifb.insn.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
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
	CALLA,CALLR:
		begin deco.rfwr = ifb.insn.call.Rt!=2'b00; end
	LDB,LDBU,LDW,LDWU,LDT:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	CSR:	begin deco.vrfwr = ifb.insn.r2.Rt.vec; deco.rfwr = ~ifb.insn.r2.Rt.vec; end
	default:	begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	endcase
	// Disable writing r0 if the rz flag is set.
	if (deco.rfwr && deco.Rt=='d0)
		deco.rfwr = ~rz;

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
	ADDI,SUBFI,ANDI,ORI,XORI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	CMPI,CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	Bcc,FBcc:	deco.imm = {{16{ifb.insn.br.disp[15]}},ifb.insn.br.disp};
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:
		deco.imm = {{16{ifb.insn.ls.disp[15]}},ifb.insn.ls.disp};
	CSR:	deco.imm = {{16{ifb.insn.ri.imm[15]}},ifb.insn.ri.imm};
	default:	deco.imm = 'd0;
	endcase	
	if (ifb.pfx.opcode==PFX)
		deco.imm[31:16] = ifb.pfx.imm;

	deco.storen = 'd0;
	deco.loadn = 'd0;
	case(ifb.insn.any.opcode)
	R2:
		case(ifb.insn.r2.func)
		LDBX,LDBUX,LDWX,LDWUX,LDTX:	deco.loadn = 1'b1;
		STBX,STWX,STTX:	deco.storen = 1'b1;
		default:	;
		endcase
	default:	;
	endcase

	deco.br = ifb.insn.any.opcode==Bcc || ifb.insn.any.opcode==FBcc;
	deco.cjb = ifb.insn.any.opcode==CALLA || ifb.insn.any.opcode==CALLR || ifb.insn.any.opcode==JMPR;
	deco.storer = ifb.insn.any.opcode==STB || ifb.insn.any.opcode==STW || ifb.insn.any.opcode==STT;
	deco.store = deco.storer|deco.storen;
	deco.stc = ifb.insn.any.opcode==STC || (ifb.insn.any.opcode==R2 && ifb.insn.r2.func==STCX);
	deco.loadr = ifb.insn.any.opcode==LDB || ifb.insn.any.opcode==LDBU || ifb.insn.any.opcode==LDW || ifb.insn.any.opcode==LDWU || ifb.insn.any.opcode==LDT;
	deco.loadu = ifb.insn.any.opcode==LDBU||ifb.insn.any.opcode==LDWU || (ifb.insn.any.opcode==R2 && (ifb.insn.r2.func==LDBUX || ifb.insn.r2.func==LDWUX));
	deco.load = deco.loadr|deco.loadn;
	deco.ldr = ifb.insn.any.opcode==LDR || (ifb.insn.any.opcode==R2 && ifb.insn.r2.func==LDRX);
	deco.mem = deco.store|deco.load|deco.stc|deco.ldr;

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
	if (deco.Rt[6]) deco.memsz = vect;
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
	deco.hasRt =	!deco.cjb && !deco.pfx;

	deco.is_vector = deco.Rt[6]|deco.Ra[6]|deco.Rb[6]|deco.Rc[6];

end

endmodule
