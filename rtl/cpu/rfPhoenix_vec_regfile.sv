// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_vec_regfile.sv
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

module rfPhoenix_vec_regfile(rst, clk, wr, wthread, wmask, wa, i,
	rthread, ra0, ra1, ra2, ra3, ra4, o0, o1, o2, o3, o4);
input rst;
input clk;
input wr;
input Tid wthread;
input [63:0] wmask;
input Regspec wa;
input VecValue i;
input Tid rthread;
input Regspec ra0;
input Regspec ra1;
input Regspec ra2;
input Regspec ra3;
input Regspec ra4;
output VecValue o0;
output VecValue o1;
output VecValue o2;
output VecValue o3;
output VecValue o4;

genvar g;
generate begin
	for (g = 0; g < NLANES; g = g + 1) begin
		gpr_regfile urf0(clk, {4{wr}} & wmask[g*4+3:g*4], {wthread,wa}, i[g], {rthread,ra0}, o0[g]);
		gpr_regfile urf1(clk, {4{wr}} & wmask[g*4+3:g*4], {wthread,wa}, i[g], {rthread,ra1}, o1[g]);
		gpr_regfile urf2(clk, {4{wr}} & wmask[g*4+3:g*4], {wthread,wa}, i[g], {rthread,ra2}, o2[g]);
		gpr_regfile urf3(clk, {4{wr}} & wmask[g*4+3:g*4], {wthread,wa}, i[g], {rthread,ra3}, o3[g]);
		gpr_regfile urf4(clk, {4{wr}} & wmask[g*4+3:g*4], {wthread,wa}, i[g], {rthread,ra4}, o4[g]);
	end
end
endgenerate

endmodule
