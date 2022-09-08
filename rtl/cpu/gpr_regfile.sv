// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	gpr_regfile.sv
//
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

//import const_pkg::*;
import rfPhoenixPkg::*;

module gpr_regfile(clk, wr, wa, i, ra, o);
input clk;
input wr;
input [5+TidMSB+1:0] wa;
input Value i;
input [5+TidMSB+1:0] ra;
output Value o;

integer k;



(* ram_style = "block" *)
Value [NTHREADS*NREGS-1:0] mem;
initial begin
	for (k = 0; k < NTHREADS*NREGS; k = k + 1)
		mem[k] <= 32'd0;
end
reg [5+TidMSB+1:0] rar;
always_ff @(posedge clk)
	rar <= ra;
always_ff @(posedge clk)
	if (wr) mem[wa] <= i;
always_ff @(posedge clk)
	o <= mem[rar];

/*

generate begin : gRegfile
case(NTHREADS)
1,2,3,4:
//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG
blk_mem256x32 bmem0 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .douta(),  // output wire [31 : 0] douta
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .web(1'b0),      // input wire [0 : 0] web
  .addrb(ra),  // input wire [7 : 0] addrb
  .dinb(32'd0),    // input wire [31 : 0] dinb
  .doutb(o)  // output wire [31 : 0] doutb
);
5,6,7,8:
blk_mem512x32 bmem1 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .douta(),  // output wire [31 : 0] douta
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .web(1'b0),      // input wire [0 : 0] web
  .addrb(ra),  // input wire [7 : 0] addrb
  .dinb(32'd0),    // input wire [31 : 0] dinb
  .doutb(o)  // output wire [31 : 0] doutb
);
9,10,11,12,13,14,15,16:
blk_mem1024x32 bmem2 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .douta(),  // output wire [31 : 0] douta
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .web(1'b0),      // input wire [0 : 0] web
  .addrb(ra),  // input wire [7 : 0] addrb
  .dinb(32'd0),    // input wire [31 : 0] dinb
  .doutb(o)  // output wire [31 : 0] doutb
);
endcase
end
endgenerate
*/

endmodule
