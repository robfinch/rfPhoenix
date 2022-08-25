// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_gp_regfile.sv
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

module rfPhoenix_gp_regfile(rst, clk, wr, wthread, wa, i,
	rthread, ra0, ra1, ra2, ra3, ra4, o0, o1, o2, o3, o4);
input rst;
input clk;
input wr;
input [3:0] wthread;
input Regspec wa;
input Value i;
input [3:0] rthread;
input Regspec ra0;
input Regspec ra1;
input Regspec ra2;
input Regspec ra3;
input Regspec ra4;
output Value o0;
output Value o1;
output Value o2;
output Value o3;
output Value o4;

integer n,k;
Regspec ra0r, ra1r, ra2r, ra3r, ra4r;
reg [3:0] rthreadr;

(* ram_style = "block" *)
Value regfile [0:NREGS*16-1];

initial begin
	for (k = 0; k < 16; k = k + 1) begin
		for (n = 0; n < NREGS; n = n + 1) begin
			regfile[k*NREGS+n] <= 32'd0;
		end
	end
end

always_ff @(posedge clk)
	if (wr)	regfile[{wthread,wa}] <= i;
always_ff @(posedge clk)
	rthreadr <= rthread;
always_ff @(posedge clk)
	ra0r <= ra0;
always_ff @(posedge clk)
	ra1r <= ra1;
always_ff @(posedge clk)
	ra2r <= ra2;
always_ff @(posedge clk)
	ra3r <= ra3;
always_ff @(posedge clk)
	ra4r <= ra4;
	
always_comb
	o0 <= regfile[{rthreadr,ra0r}];
always_comb
	o1 <= regfile[{rthreadr,ra1r}];
always_comb
	o2 <= regfile[{rthreadr,ra2r}];
always_comb
	o3 <= regfile[{rthreadr,ra3r}];
always_comb
	o4 <= regfile[{rthreadr,ra4r}];

endmodule
