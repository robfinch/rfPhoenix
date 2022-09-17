// ============================================================================
//        __
//   \\__/ o\    (C) 2005-2022  Robert Finch, Waterloo
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
//
module roundRobin(rst, clk, ce, req, lock, sel, sel_enc);
input rst;				// reset
input clk;				// clock
input ce;				// clock enable
input [7:0] req;		// request
input [7:0] lock;		// lock selection
output [7:0] sel;		// select, one hot
output reg [2:0] sel_enc;	// select, encoded

reg [7:0] sel;

reg [2:0] rot;			// forward rotate applied to request lines
reg [2:0] amt;			// how much to rotate forward after a grant
reg [7:0] rgrnt;		// rotated value of grant
reg [7:0] nextGrant;	// unrotated value of grant
reg [7:0] rreq;		// rotated request

// rotate the request lines to set priority
reg [15:0] reqs;
always_comb
 	reqs = {req,8'h00} >> rot;
always_comb
	rreq = reqs[15:8]|reqs[7:0];

// rotate the rotated grant value back into place
reg [15:0] rgrnts;
always_comb
	rgrnts = {8'h00,rgrnt} << rot;
always_comb
	nextGrant = {rgrnts[15:8]|rgrnts[7:0]};

// If there is a request, determine how far the request
// lines should be rotated when there is a grant
// This should synthesize to a 3 bit wide ROM. (3 LUTs)
always_comb
	casez (rreq)
	8'b???????1:	amt <= 1;
	8'b??????1?:	amt <= 2;
	8'b?????1??:	amt <= 3;
	8'b????1???:	amt <= 4;
	8'b???1????:	amt <= 5;
	8'b??1?????:	amt <= 6;
	8'b?1??????:	amt <= 7;
	8'b1???????:	amt <= 0;
	default:		amt <= 0;
	endcase

// set grant (if request present) based on which request
// was honored.
// This should synthesize to a 8 bit wide ROM. (8 LUTs)
always_comb
	case ({~|rreq,amt})
	4'd1:	rgrnt <= 8'b00000001;
	4'd2:	rgrnt <= 8'b00000010;
	4'd3:	rgrnt <= 8'b00000100;
	4'd4:	rgrnt <= 8'b00001000;
	4'd5:	rgrnt <= 8'b00010000;
	4'd6:	rgrnt <= 8'b00100000;
	4'd7:	rgrnt <= 8'b01000000;
	4'd0:	rgrnt <= 8'b10000000;
	default:	rgrnt <= 8'd0;
	endcase

// rotate the priorities on a grant
always_ff @(posedge clk)
	if (rst)
		rot <= 'd0;
	else if (ce)
		if (!(lock & sel))
			rot <= rot + amt;

// Assign the next owner, if isn't locked
always_ff @(posedge clk)
	if (rst)
		sel <= 'd0;
	else if (ce)
		if (!(lock & sel)) begin
			sel <= nextGrant;
			// nextGrant should be one-hot
			casez(nextGrant)
			8'b???????1:	sel_enc <= 3'd0;
			8'b??????1?:	sel_enc <= 3'd1;
			8'b?????1??:	sel_enc <= 3'd2;
			8'b????1???:	sel_enc <= 3'd3;
			8'b???1????:	sel_enc <= 3'd4;
			8'b??1?????:	sel_enc <= 3'd5;
			8'b?1??????:	sel_enc <= 3'd6;
			8'b1???????:	sel_enc <= 3'd7;
			default:			sel_enc <= 3'd0;
			endcase
		end

endmodule
