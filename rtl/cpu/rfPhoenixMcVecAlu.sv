
import rfPhoenixPkg::*;

module rfPhoenixMcVecAlu(rst, clk, ir, a, b, c, imm, o, done, ridi, rido);
input rst;
input clk;
input Instruction ir;
input VecValue a;
input VecValue b;
input VecValue c;
input Value imm;
output VecValue o;
output reg done;
input [3:0] ridi;
output [3:0] rido;

integer n;
genvar g;
reg [NLANES-1:0] don;

generate begin
	for (g = 0; g < NLANES; g = g + 1)
		rfPhoenixMcAlu ualu1(
			.rst(rst),
			.clk(clk),
			.ir(ir),
			.a(a[g]),
			.b(b[g]),
			.c(c[g]),
			.imm(imm),
			.o(o[g]),
			.done(don[g])
		);
end
endgenerate

ft_delay #(.WID(4), .DEP(7)) (.clk(clk), .ce(1'b1), .i(ridi), .o(rido));

always_comb
	done <= &don;

endmodule
