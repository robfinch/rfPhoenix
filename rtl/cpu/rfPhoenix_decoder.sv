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
end

endmodule
