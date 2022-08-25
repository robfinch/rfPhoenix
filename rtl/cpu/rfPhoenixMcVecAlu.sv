
import rfPhoenixPkg::*;

module rfPhoenixMcVecAlu(rst, clk, ir, a, b, c, o, done);
input rst;
input clk;
input Instruction ir;
input VecValue a;
input VecValue b;
input VecValue c;
output VecValue o;
output reg done;

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
			.o(o[g]),
			.done(don[g])
		);
end
endgenerate

always_comb
	done <= &don;

endmodule
