
// Find first zero
module ffz12(i, o);
input [11:0] i;
output reg [3:0] o;

wire [2:0] o1,o2;
ffz6 u1 (i[11:6],o1);
ffz6 u2 (i[5:0],o2);
always_comb
if (o1==3'd7 && o2==3'd7)
    o <= 4'd15;
else if (o1==3'd7)
    o <= o2;
else
    o <= 3'd6 + o1;

endmodule

