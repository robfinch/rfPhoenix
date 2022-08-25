import rfPhoenixPkg::*;

module rfPhoenixVecAlu(rst, clk, ir, a, b, c, o);
input rst;
input clk;
input Instruction ir;
input VecValue a;
input VecValue b;
input VecValue c;
output VecValue o;

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
			.o(o[g])
		);
end
endgenerate



endmodule
