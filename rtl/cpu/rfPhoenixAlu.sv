
import rfPhoenixPkg::*;

module rfPhoenixAlu(rst, clk, ir, a, b, c, imm, o);
parameter NPIPE = 8;
input rst;
input clk;
input Instruction ir;
input Value a;
input Value b;
input Value c;
input Value imm;
output Value o;

integer n;
Value [NPIPE-1:0] fma_pipe;

wire fms = ir.any.opcode==FMS || ir.any.opcode==FNMS;
wire fnm = ir.any.opcode==FNMA || ir.any.opcode==FNMS;

Value fma_o, fma_o1;
Value fcmp_o;

fpCompare32 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

always
case(ir.any.opcode)
R2:
	case(ir.r2.func)
	ADD:		o = a + b;
	SUB:		o = a - b;
	AND:		o = a & b;
	OR:			o = a | b;
	XOR:		o = a ^ b;
	CMP_EQ:	o = a == b;
	CMP_NE:	o = a != b;
	CMP_LT:	o = $signed(a) < $signed(b);
	CMP_GE:	o = $signed(a) >= $signed(b);
	CMP_LE: o = $signed(a) <= $signed(b);
	CMP_GT:	o = $signed(a) > $signed(b);
	CMP_LTU:	o = a < b;
	CMP_GEU:	o = a >= b;
	CMP_LEU:	o = a <= b;
	CMP_GTU:	o = a > b;
	SLLI:			o = a << imm[4:0];
	SRLI:			o = a >> imm[4:0];
	SRAI:			o = {{32{a[31]}},a} >> imm[4:0];
	SLL:			o = a << b[4:0];
	SRL:			o = a >> b[4:0];
	SRA:			o = {{32{a[31]}},a} >> b[4:0];
	default:	o = 'd0;
	endcase
ADDI:			o = a + imm;
SUBFI:		o = imm - a;
ANDI:			o = a & imm;
ORI:			o = a | imm;
XORI:			o = a ^ imm;
CMP_EQI:	o = a == imm;
CMP_NEI:	o = a != imm;
CMP_LTI:	o = $signed(a) < $signed(imm);
CMP_GEI:	o = $signed(a) >= $signed(imm);
CMP_LEI:	o = $signed(a) <= $signed(imm);
CMP_GTI:	o = $signed(a) > $signed(imm);
CMP_LTUI:	o = a < imm;
CMP_GEUI:	o	= a >= imm;
CMP_LEUI:	o = a <= imm;
CMP_GTUI:	o = a > imm;
FCMP_EQ:	o = fcmp_o[0];
FCMP_NE:	o = fcmp_o[8]|fcmp_o[4];	// return 1 if Nan
FCMP_LT:	o = fcmp_o[1];
FCMP_LE:	o = fcmp_o[2];
FCMP_GT:	o = fcmp_o[10];
FCMP_GE:	o = fcmp_o[9];
default:	o = 'd0;
endcase

endmodule
