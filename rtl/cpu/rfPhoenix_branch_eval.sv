module rfPhoenix_branch_eval(ir, a, b, o);
input Instruction ir;
input Value a;
input Value b;
output reg o;

Value fcmp_o;

fpCompare32 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

always_comb
case(ir.any.opcode)
Bcc:
	case(ir.br.cnd)
	default:	o <= 1'b0;
	3'd0:		o <= $signed(a) <  $signed(b);
	3'd1:		o <= $signed(a) >= $signed(b);
	3'd2:		o <= $signed(a) <= $signed(b);
	3'd3:		o <= $signed(a) >  $signed(b);
	3'd6:		o <= a==b;
	3'd7:		o <= a!=b;
	endcase
FBcc:
	case(ir.br.cnd)
	default:	o <= 1'b0;
	3'd0:		o <= fcmp_o[0];
	3'd1:		o <= fcmp_o[8]|fcmp_o[4];	// matches NE if Nan
	3'd2:		o <= fcmp_o[1];
	3'd3:		o <= fcmp_o[2];
	3'd6:		o <= fcmp_o[10];
	3'd7:		o <= fcmp_o[9];
	endcase
default:	o <= 1'b0;
endcase

endmodule
