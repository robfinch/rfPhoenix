`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2013-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
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

import rfPhoenixPkg::*;

module rfPhoenix_branch_eval(ir, a, b, o);
input Instruction ir;
input Value a;
input Value b;
output reg o;

Value fcmp_o;

fpCompare32 ucmp1
(
	.a(a),
	.b(b),
	.o(fcmp_o),
	.nan(),
	.snan()
);

always_comb
case(ir.any.opcode)
Bcc:
	case(ir.br.cnd)
	default:	o <= 1'b0;
	3'd0:		o <= $signed(a) <  $signed(b);
	3'd1:		o <= $signed(a) >= $signed(b);
	3'd2:		o <= a < b;
	3'd3:		o <= a >= b;
	3'd4:		o <= a[ir.br.Rb];
	3'd5:		o <= 1'b0;
	3'd6:		o <= a==b;
	3'd7:		o <= a!=b;
	endcase
FBcc:
	case(ir.br.cnd)
	default:	o <= 1'b0;
	3'd0:		o <= fcmp_o[0];
	3'd1:		o <= fcmp_o[8]|fcmp_o[4];	// matches NE if Nan
	3'd2:		o <= fcmp_o[1];
	3'd3:		o <= fcmp_o[2];
	3'd4:		o <= 1'b0;
	3'd5:		o <= 1'b0;
	3'd6:		o <= fcmp_o[10];
	3'd7:		o <= fcmp_o[9];
	endcase
default:	o <= 1'b0;
endcase

endmodule
