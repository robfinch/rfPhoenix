
// Find first zero
module ffz6(i, o);
input [5:0] i;
output reg [2:0] o;
always_comb
casez(i)
6'b0?????:  o <= 3'd5;
6'b10????:  o <= 3'd4;
6'b110???:  o <= 3'd3;
6'b1110??:  o <= 3'd2;
6'b11110?:  o <= 3'd1;
6'b111110:  o <= 3'd0;
default:    o <= 3'd7;
endcase
endmodule

