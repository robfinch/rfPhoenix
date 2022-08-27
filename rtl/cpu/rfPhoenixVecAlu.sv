import rfPhoenixPkg::*;

module rfPhoenixVecAlu(rst, clk, ir, a, b, c, imm, o);
input rst;
input clk;
input Instruction ir;
input VecValue a;
input VecValue b;
input VecValue c;
input Value imm;
output VecValue o;

VecValue o1;

integer n;
genvar g;
generate begin
	for (g = 0; g < NLANES; g = g + 1)
		rfPhoenixAlu ualu (
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(a[g]),
			.b(b[g]),
			.c(c[g]),
			.imm(imm),
			.o(o1[g])
		);
end
endgenerate

always_comb
	case(ir.any.opcode)
	R2:
		case (ir.r2.func)
		VEX:	o = {NLANES{a[imm[3:0]]}};
//		VEINS:
		VSHUF:
			for (n = 0; n < NLANES; n = n + 1)
				o[n] = a[b[n][3:0]];
		VSLLV:		o <= a << {b[0][3:0],5'd0};
		VSRLV:		o <= a >> {b[0][3:0],5'd0};
		VSLLVI:		o <= a << {imm[3:0],5'd0};
		VSRLVI:		o <= a >> {imm[3:0],5'd0};
		default:	o = o1;
		endcase
	default:	o = o1;
	endcase

endmodule
