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

import rfPhoenixPkg::*;

module rfPhoenix_decoder(ir, pfx, rz, deco);
input Instruction ir;
input Postfix pfx;
input rz;
output sDecodeBus deco;

always_comb
begin

	deco.Ra = ir.r2.Ra;
	deco.Ta = ir.r2.Ta;
	deco.Rb = ir.r2.Rb;
	deco.Tb = ir.r2.Tb;
	deco.Rc = ir.f3.Rc;
	deco.Tc = ir.f3.Tc;

	// Rt
	case(ir.any.opcode)
	R2:	
		case(ir.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.Rt = ir.r2.Rt; deco.Tt = ir.r2.Tt; end
		default:	begin deco.Rt = 'd0; deco.Tt = 1'b0; end
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		begin deco.Rt = ir.ri.Rt; deco.Tt = ir.ri.Tt; end
	CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		begin deco.Rt = ir.ri.Rt; deco.Tt = ir.ri.Tt; end
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		begin deco.Rt = ir.ri.Rt; deco.Tt = ir.ri.Tt; end
	FMA,FMS,FNMA,FNMS:	begin deco.Rt = ir.f3.Rt; deco.Tt = ir.f3.Tt; end
	NOP:
		begin deco.Rt = 'd0; deco.Tt = 1'b0; end
	CALLA,CALLR,JMP,BRA:
		begin deco.Rt = {ir.call.Rt==2'b0} ? 'd0 : {4'b1010,ir.call.Rt}; deco.Tt = 1'b0; end
	STB,STW,STT:	begin deco.Rt = ir.ls.Rt; deco.Tt = ir.ls.Tt; end
	CSR:	begin deco.Rt = ir.ri.Rt; deco.Tt = ir.ri.Tt; end
	default:	begin deco.Rt = 'd0; deco.Tt = 1'b0; end
	endcase

	deco.multicycle = 'd0;
	case(ir.any.opcode)
	FMA,FMS,FNMA,FNMS:	deco.multicycle = 1'b1;
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:	deco.multicycle = 1'b1;
	default:	deco.multicycle = 'd0;
	endcase

	// Register file writes	
	deco.rfwr = 'd0;
	deco.vrfwr = 'd0;
	case(ir.any.opcode)
	R2:	
		case(ir.r2.func)
		ADD,SUB,AND,OR,XOR:	begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
		default:	begin deco.Rt = 'd0; deco.Tt = 1'b0; end
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	FMA,FMS,FNMA,FNMS:	begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	NOP:
		begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	CALLA,CALLR,JMP,BRA:
		begin deco.rfwr = ir.call.Rt!=2'b00; end
	LDB,LDBU,LDW,LDWU,LDT:	begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	CSR:	begin deco.vrfwr = ir.r2.Tt; deco.rfwr = ~ir.r2.Tt; end
	default:	begin deco.rfwr = 'd0; deco.vrfwr = 'd0; end
	endcase
	// Disable writing r0 if the rz flag is set.
	if (deco.rfwr && deco.Rt=='d0)
		deco.rfwr = ~rz;

	deco.multicycle = 'd0;
	case(ir.any.opcode)
	R2:
		case(ir.r2.func)
		LDBX,LDBUX,LDWX,LDWUX,LDTX,
		STBX,STWX,STTX:	deco.multicycle = 1'b1;
		default:	deco.multicycle = 'd0;
		endcase
	FMA,FMS,FNMA,FNMS:	deco.multicycle = 1'b1;
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:	deco.multicycle = 1'b1;
	default:	deco.multicycle = 'd0;
	endcase

	deco.imm = 'd0;
	case(ir.any.opcode)
	Bcc:	deco.imm = {{15{ir.br.disp[16]}},ir.br.disp};
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:
		deco.imm = {{16{ir.ls.disp[15]}},ir.ls.disp};
	default:	deco.imm = 'd0;
	endcase	
	if (pfx.opcode==PFX)
		deco.imm[31:16] = pfx.imm;

	deco.storen = 'd0;
	deco.loadn = 'd0;
	case(ir.any.opcode)
	R2:
		case(ir.r2.func)
		LDBX,LDBUX,LDWX,LDWUX,LDTX:	deco.loadn = 1'b1;
		STBX,STWX,STTX:	deco.storen = 1'b1;
		default:	;
		endcase
	default:	;
	endcase

	deco.br = ir.any.opcode==Bcc || ir.any.opcode==FBcc;
	deco.cjb = ir.any.opcode==CALLA || ir.any.opcode==CALLR || ir.any.opcode==JMP || ir.any.opcode==BRA;
	deco.storer = ir.any.opcode==STB || ir.any.opcode==STW || ir.any.opcode==STT;
	deco.store = deco.storer|deco.storen;
	deco.loadr = ir.any.opcode==LDB || ir.any.opcode==LDBU || ir.any.opcode==LDW || ir.any.opcode==LDWU || ir.any.opcode==LDT;
	deco.loadu = ir.any.opcode==LDBU||ir.any.opcode==LDWU || (ir.any.opcode==R2 && (ir.r2.func==LDBUX || ir.r2.func==LDWUX));
	deco.load = deco.loadr|deco.loadn;

	// Memory operation sizes
	case(ir.any.opcode)
	LDB,LDBU,STB:	deco.memsz = byt;
	LDW,LDWU,STW:	deco.memsz = wyde;
	LDT,STT:	deco.memsz = tetra;
	R2:
		case (ir.r2.func)
		LDBX,LDBUX,STBX:	deco.memsz = byt;
		LDWX,LDWUX,STWX:	deco.memsz = wyde;
		LDTX,STTX:	deco.memsz = tetra;
		default:	deco.memsz = tetra;
		endcase
	default:	deco.memsz = tetra;
	endcase
	if (deco.Tt) deco.memsz = vect;

	deco.csrrd = ir.any.opcode==CSR && ir.csr.func==2'd0;
	deco.csrrw = ir.any.opcode==CSR && ir.csr.func==2'd1;
	deco.csrrc = ir.any.opcode==CSR && ir.csr.func==2'd2;
	deco.csrrs = ir.any.opcode==CSR && ir.csr.func==2'd3;
end

endmodule
