module rfPhoenix_branch_eval(ir, a, b, o);
input Instruction ir;
input Value a;
input Value b;
output reg o;

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
default:	o <= 1'b0;
endcase

endmodule
