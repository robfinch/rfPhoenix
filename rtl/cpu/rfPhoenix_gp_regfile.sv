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

module rfPhoenix_gp_regfile(rst, clk, ce, wr, wthread, wa, i,
	rthread, ra0, ra1, ra2, ra3, ra4, o0, o1, o2, o3, o4);
input rst;
input clk;
input ce;
input [3:0] wr;
input tid_t wthread;
input regspec_t wa;
input value_t i;
input tid_t rthread;
input regspec_t ra0;
input regspec_t ra1;
input regspec_t ra2;
input regspec_t ra3;
input regspec_t ra4;
output value_t o0;
output value_t o1;
output value_t o2;
output value_t o3;
output value_t o4;

gpr_regfile #(.ZERO_BYPASS(1)) ugpr0 (.clk(clk), .ce(ce), .wr(wr), .wa({wthread,wa.num}), .i(i), .ra({rthread,ra0.num}), .o(o0));
gpr_regfile #(.ZERO_BYPASS(1)) ugpr1 (.clk(clk), .ce(ce), .wr(wr), .wa({wthread,wa.num}), .i(i), .ra({rthread,ra1.num}), .o(o1));
gpr_regfile #(.ZERO_BYPASS(1)) ugpr2 (.clk(clk), .ce(ce), .wr(wr), .wa({wthread,wa.num}), .i(i), .ra({rthread,ra2.num}), .o(o2));
gpr_regfile #(.ZERO_BYPASS(1)) ugpr3 (.clk(clk), .ce(ce), .wr(wr), .wa({wthread,wa.num}), .i(i), .ra({rthread,ra3.num}), .o(o3));
gpr_regfile #(.ZERO_BYPASS(1)) ugpr4 (.clk(clk), .ce(ce), .wr(wr), .wa({wthread,wa.num}), .i(i), .ra({rthread,ra4.num}), .o(o4));

endmodule

