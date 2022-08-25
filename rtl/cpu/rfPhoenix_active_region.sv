// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_active_region.sv
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
import rfPhoenixMmupkg::*;

module rfPhoenix_active_region(clk, wr, rwa, i, o, adr, region_num, region, err);
input clk;
input wr;
input [5:0] rwa;
input Value i;
output Value o;
input Address adr;
output reg [3:0] region_num;
output REGION region;
output reg err;

integer n;
REGION [7:0] pma_regions;

initial begin
	// ROM
	pma_regions[7].start = 32'hFFFD0000;
	pma_regions[7].nd 	= 32'hFFFFFFFF;
	pma_regions[7].pmt	= 32'h00000000;
	pma_regions[7].cta	= 32'h00000000;
	pma_regions[7].at 	= 20'h0000D;		// rom, byte addressable, cache-read-execute

	// IO
	pma_regions[6].start = 32'hFF800000;
	pma_regions[6].nd = 32'hFF9FFFFF;
	pma_regions[6].pmt	 = 32'h00000300;
	pma_regions[6].cta	= 32'h00000000;
	pma_regions[6].at = 20'h00206;		// io, (screen) byte addressable, read-write

	// Vacant
	pma_regions[5].start = 32'hFFFFFFFF;
	pma_regions[5].nd = 32'hFFFFFFFF;
	pma_regions[5].pmt	 = 32'h00000000;
	pma_regions[5].cta	= 32'h00000000;
	pma_regions[5].at = 20'h0FF00;		// no access

	// Scratchpad RAM
	pma_regions[4].start = 32'hFFFC0000;
	pma_regions[4].nd = 32'hFFFCFFFF;
	pma_regions[4].pmt	 = 32'h00002300;
	pma_regions[4].cta	= 32'h00000000;
	pma_regions[4].at = 20'h0020F;		// byte addressable, read-write-execute cacheable

	// vacant
	pma_regions[3].start = 32'hFFFFFFFF;
	pma_regions[3].nd = 32'hFFFFFFFF;
	pma_regions[3].pmt	 = 32'h00000000;
	pma_regions[3].cta	= 32'h00000000;
	pma_regions[3].at = 20'h0FF00;		// no access

	// vacant
	pma_regions[2].start = 32'hFFFFFFFF;
	pma_regions[2].nd = 32'hFFFFFFFF;
	pma_regions[2].pmt	 = 32'h00000000;
	pma_regions[2].cta	= 32'h00000000;
	pma_regions[2].at = 20'h0FF00;		// no access

	// DRAM
	pma_regions[1].start = 32'h00000000;
	pma_regions[1].nd = 32'h1FFFFFFF;
	pma_regions[1].pmt	 = 32'h00002400;
	pma_regions[1].cta	= 32'h00000000;
	pma_regions[1].at = 20'h0010F;	// ram, byte addressable, cache-read-write-execute

	// vacant
	pma_regions[0].start = 32'hFFFFFFFF;
	pma_regions[0].nd = 32'hFFFFFFFF;
	pma_regions[0].pmt	 = 32'h00000000;
	pma_regions[0].cta	= 32'h00000000;
	pma_regions[0].at = 20'h0FF00;		// no access

end

always_ff @(posedge clk)
	if (wr) begin
		case(rwa[2:0])
		3'd0:	pma_regions[rwa[5:3]].start <= i;
		3'd1:	pma_regions[rwa[5:3]].nd <= i;
		3'd2: pma_regions[rwa[5:3]].pmt <= i;
		3'd3: pma_regions[rwa[5:3]].cta <= i;
		3'd4:	pma_regions[rwa[5:3]].at <= i;
		default:	;
		endcase
	end
always_ff @(posedge clk)
	case(rwa[2:0])
	3'd0:	o <= pma_regions[rwa[5:3]].start;
	3'd1:	o <= pma_regions[rwa[5:3]].nd;
	3'd2:	o <= pma_regions[rwa[5:3]].pmt;
	3'd3:	o <= pma_regions[rwa[5:3]].cta;
	3'd4:	o <= pma_regions[rwa[5:3]].at;
	default:	;
	endcase

always_comb
begin
	err = 1'b1;
	region_num = 4'd0;
	region = pma_regions[0];
  for (n = 0; n < 8; n = n + 1)
    if (adr[31:4] >= pma_regions[n].start[31:4] && adr[31:4] <= pma_regions[n].nd[31:4]) begin
    	region = pma_regions[n];
    	region_num = n;
    	err = 1'b0;
  	end
end    	
    	
endmodule

