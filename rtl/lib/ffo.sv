// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	ffo.sv
//
// BSD 3-Clause License
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//                                                                          
// ============================================================================

// Find first one
module ffo6(i, o);
input [5:0] i;
output reg [2:0] o;
always_comb
casez(i)
6'b1?????:  o <= 3'd5;
6'b01????:  o <= 3'd4;
6'b001???:  o <= 3'd3;
6'b0001??:  o <= 3'd2;
6'b00001?:  o <= 3'd1;
6'b000001:  o <= 3'd0;
default:    o <= 3'd7;
endcase
endmodule

module ffo12(i, o);
input [11:0] i;
output reg [3:0] o;

wire [2:0] o1,o2;
ffo6 u1 (i[11:6],o1);
ffo6 u2 (i[5:0],o2);
always @*
if (o1==3'd7 && o2==3'd7)
    o <= 4'd15;
else if (o1==3'd7)
    o <= o2;
else
    o <= 3'd6 + o1;

endmodule

module ffo24(i, o);
input [23:0] i;
output reg [4:0] o;

wire [3:0] o1,o2;
ffo12 u1 (i[23:12],o1);
ffo12 u2 (i[11:0],o2);
always @*
if (o1==4'd15 && o2==4'd15)
    o <= 5'd31;
else if (o1==4'd15)
    o <= o2;
else
    o <= 4'd12 + o1;

endmodule

module ffo48(i, o);
input [47:0] i;
output reg [5:0] o;

wire [4:0] o1,o2;
ffo24 u1 (i[47:24],o1);
ffo24 u2 (i[23:0],o2);
always @*
if (o1==5'd31 && o2==5'd31)
    o <= 6'd63;
else if (o1==5'd31)
    o <= o2;
else
    o <= 5'd24 + o1;

endmodule

module ffo96(i, o);
input [95:0] i;
output reg [6:0] o;

wire [5:0] o1,o2;
ffo48 u1 (i[95:48],o1);
ffo48 u2 (i[47:0],o2);
always @*
if (o1==6'd63 && o2==6'd63)
    o <= 7'd127;
else if (o1==6'd63)
    o <= o2;
else
    o <= 6'd48 + o1;

endmodule

module ffo144(i, o);
input [143:0] i;
output reg [7:0] o;

wire [5:0] o1,o2,o3;
ffo48 u1 (i[143:96],o1);
ffo48 u2 (i[95:48],o2);
ffo48 u3 (i[47:0],o3);
always_comb
if (o1==6'd63 && o2==6'd63 && o3==6'd63)
	o <= 8'd255;
else if (o1==6'd63 && o2==6'd63)
	o <= o3;
else if (o1==6'd63)
  o <= 8'd48 + o2;
else
  o <= 8'd96 + o1;

endmodule
