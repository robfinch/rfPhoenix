
// Find last one
module flo6(i, o);
input [5:0] i;
output reg [2:0] o;
always @*
casex(i)
6'bxxxxx1:  o <= 3'd0;
6'bxxxx10:  o <= 3'd1;
6'bxxx100:  o <= 3'd2;
6'bxx1000:  o <= 3'd3;
6'bx10000:  o <= 3'd4;
6'b100000:  o <= 3'd5;
default:    o <= 3'd7;
endcase
endmodule

module flo12(i, o);
input [11:0] i;
output reg [3:0] o;

wire [2:0] o1,o2;
flo6 u1 (i[5:0],o1);
flo6 u2 (i[11:6],o2);
always @*
if (o1==3'd7 && o2==3'd7)
    o <= 4'd15;
else if (o1==3'd7)
    o <= 3'd6 + o2;
else
    o <= o1;

endmodule

module flo24(i, o);
input [23:0] i;
output reg [4:0] o;

wire [3:0] o1,o2;
flo12 u1 (i[23:12],o1);
flo12 u2 (i[11:0],o2);
always @*
if (o1==4'd15 && o2==4'd15)
    o <= 5'd31;
else if (o2==4'd15)
    o <= 4'd12 + o1;
else
    o <= o2;

endmodule

module flo48(i, o);
input [47:0] i;
output reg [5:0] o;

wire [4:0] o1,o2;
flo24 u1 (i[47:24],o1);
flo24 u2 (i[23:0],o2);
always @*
if (o1==5'd31 && o2==5'd31)
    o <= 6'd63;
else if (o2==5'd31)
    o <= 5'd24 + o1;
else
    o <= o2;

endmodule

module flo96(i, o);
input [95:0] i;
output reg [6:0] o;

wire [5:0] o1,o2;
flo48 u1 (i[95:48],o1);
flo48 u2 (i[47:0],o2);
always @*
if (o1==6'd63 && o2==6'd63)
    o <= 7'd127;
else if (o2==6'd63)
    o <= 6'd48 + o1;
else
    o <= o2;

endmodule

