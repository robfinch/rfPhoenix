
import rfPhoenixPkg::*;

module rfPhoenixMcAlu(rst, clk, ir, a, b, c, imm, o, done, ridi, rido);
parameter NPIPE = 8;
input rst;
input clk;
input Instruction ir;
input Value a;
input Value b;
input Value c;
input Value imm;
output Value o;
output reg done;
input [3:0] ridi;
output [3:0] rido;

integer n;
Value [NPIPE-1:0] fma_pipe;

wire fms = ir.any.opcode==FMS || ir.any.opcode==FNMS;
wire fnm = ir.any.opcode==FNMA || ir.any.opcode==FNMS;

Value fma_o, fma_o1;
Value fcmp_o;

fpFMA32nrL7 ufma1 (
	.clk(clk),
	.ce(1'b1),
	.op(fms),
	.rm(ir.f3.rm),
	.a(a ^ {fnm,{$bits(Value)-1{1'b0}}}),
	.b(b),
	.c(c),
	.o(fma_o),
	.inf(),
	.zero(),
	.overflow(),
	.underflow(),
	.inexact()
);

/*
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
*/
ft_delay #(.WID(4), .DEP(7)) (.clk(clk), .ce(1'b1), .i(ridi), .o(rido));

always
case(ir.any.opcode)
FMA,FMS,FNMA,FNMS:	o = fma_o;
default:	o = 'd0;
endcase

endmodule
