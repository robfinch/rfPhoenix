
import rfPhoenixPkg::*;

module rfPhoenixMcAlu(rst, clk, ir, a, b, c, o, done);
parameter NPIPE = 8;
input rst;
input clk;
input Instruction ir;
input Value a;
input Value b;
input Value c;
output Value o;
output reg done;

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

fpFMA32nrCombo ufma1 (
	.op(fms),
	.rm(ir.f3.rm),
	.a(a ^ {fnm,{$bits(Value)-1{1'b0}}}),
	.b(b),
	.c(c),
	.o(fma_o1),
	.inf(),
	.zero(),
	.overflow(),
	.underflow(),
	.inexact()
);

always_ff @(posedge clk)
if (rst)
	for (n = 0; n < NPIPE - 1; n = n + 1) begin
		fma_pipe[n+1] <= 'd0;
	end
else begin
	for (n = 0; n < NPIPE - 1; n = n + 1) begin
		fma_pipe[n+1] <= fma_pipe[n];
	end
	fma_pipe[0] <= fma_o1;
end

always
case(ir.any.opcode)
FMA,FMS,FNMA,FNMS:	o = fma_pipe[NPIPE-1];
FCMP_EQ:	o = fcmp_o[0];
FCMP_NE:	o = fcmp_o[8]|fcmp_o[4];	// return 1 if Nan
FCMP_LT:	o = fcmp_o[1];
FCMP_LE:	o = fcmp_o[2];
FCMP_GT:	o = fcmp_o[10];
FCMP_GE:	o = fcmp_o[9];
default:	o = 'd0;
endcase

endmodule
