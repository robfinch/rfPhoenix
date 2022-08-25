// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	Thor2022_tlbram.sv
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

module Thor2022_tlbram(clk, wr, wa, i, ra, wo, o);
parameter WID = 160;
input clk;
input wr;
input [5:0] wa;
input [WID-1:0] i;
input [5:0] ra;
output [WID-1:0] wo;
output [WID-1:0] o;

genvar g;
generate begin : gRam
for (g = 0; g < WID; g = g + 1)
	// 2 LUTs per bit
	RAM64X1D u1 (
		.WE(wr),
		.D(i[g]),
		.WCLK(clk),
		.A0(wa[0]),
		.A1(wa[1]),
		.A2(wa[2]),
		.A3(wa[3]),
		.A4(wa[4]),
		.A5(wa[5]),
		.DPRA0(ra[0]),
		.DPRA1(ra[1]),
		.DPRA2(ra[2]),
		.DPRA3(ra[3]),
		.DPRA4(ra[4]),
		.DPRA5(ra[5]),
		.SPO(wo[g]),
		.DPO(o[g])
	);
end
endgenerate

endmodule
