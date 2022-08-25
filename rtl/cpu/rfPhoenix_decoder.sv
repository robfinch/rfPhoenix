import rfPhoenixPkg::*;

module rfPhoenix_decoder(ir, deco);
input Instruction ir;
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
	FMA,FMS,FNMA,FNMS:	begin deco.Rt = ir.f3.Rt; deco.Tt = ir.f3.Tt; end
	NOP:
		begin deco.Rt = 'd0; deco.Tt = 1'b0; end
	CALLA,CALLR,JMP,BRA:
		begin deco.Rt = {ir.call.Rt==2'b0} ? 'd0 : {4'b1010,ir.call.Rt}; deco.Tt = 1'b0; end
	STB,STW,STT:	begin deco.Rt = ir.ls.Rt; deco.Tt = ir.ls.Tt; end
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
		ADD,SUB,AND,OR,XOR:	begin deco.vrfwr <= ir.r2.Tt; deco.rfwr <= ~ir.r2.Tt; end
		default:	begin deco.Rt = 'd0; deco.Tt = 1'b0; end
		endcase
	FMA,FMS,FNMA,FNMS:	begin deco.vrfwr <= ir.r2.Tt; deco.rfwr <= ~ir.r2.Tt; end
	NOP:
		begin deco.rfwr <= 'd0; deco.vrfwr <= 'd0; end
	CALLA,CALLR,JMP,BRA:
		begin deco.rfwr <= ir.call.Rt!=2'b00; end
	LDB,LDBU,LDW,LDWU,LDT:	begin deco.vrfwr <= ir.r2.Tt; deco.rfwr <= ~ir.r2.Tt; end
	default:	begin deco.rfwr <= 'd0; deco.vrfwr <= 'd0; end
	endcase

	deco.multicycle = 'd0;
	case(ir.any.opcode)
	FMA,FMS,FNMA,FNMS:	deco.multicycle = 1'b1;
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:	deco.multicycle = 1'b1;
	default:	deco.multicycle = 'd0;
	endcase

	deco.imm = 'd0;
	case(ir.any.opcode)
	Bcc:	deco.imm = {{15{ir.br.disp[16]}},ir.br.disp};
	default:	deco.imm = 'd0;
	endcase	

	deco.br = ir.any.opcode==Bcc || ir.any.opcode==FBcc;
	deco.cjb = ir.any.opcode==CALLA || ir.any.opcode==CALLR || ir.any.opcode==JMP || ir.any.opcode==BRA;
	deco.storer = ir.any.opcode==STB || ir.any.opcode==STW || ir.any.opcode==STT;
	deco.storen = ir.any.opcode==STX;
	deco.store = deco.storer|deco.storen;
	deco.loadr = ir.any.opcode==LDB || ir.any.opcode==LDBU || ir.any.opcode==LDW || ir.any.opcode==LDWU || ir.any.opcode==LDT;
	deco.loadn = ir.any.opcode==LDX;
	deco.loadu = ir.any.opcode==LDBU||ir.any.opcode==LDWU || (ir.any.opcode==LDX && (ir.r2.func==LDBU || ir.r2.func==LDWU));
	deci.load = deco.loadr|deco.loadn;

	// Memory operation sizes
	case(ir.any.opcode)
	LDB,LDBU,STB:	deco.memsz = byt;
	LDW,LDWU,STW:	deco.memsz = wyde;
	LDT,STT:	deco.memsz = tetra;
	LDX,STX:
		case (ir.r2.func)
		LDB,LDBU,STB:	deco.memsz = byt;
		LDW,LDWU,STW:	deco.memsz = wyde;
		LDT,STT:	deco.memsz = tetra;
		endcase
	default:	deco.memsz = tetra;
	endcase
	if (deco.Tt) deco.memsz = vect;

end

endmodule
