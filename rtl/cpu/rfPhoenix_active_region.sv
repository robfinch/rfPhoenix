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
input [6:0] rwa;
input value_t i;
output value_t o;
input physical_address_t adr;
output reg [3:0] region_num;
output REGION region;
output reg err;

integer n;
REGION [7:0] pma_regions;

initial begin
	// ROM
	pma_regions[7].start = 48'hFFFD0000;
	pma_regions[7].nd 	= 48'hFFFFFFFF;
	pma_regions[7].pmt	= 48'h00000000;
	pma_regions[7].cta	= 48'h00000000;
	pma_regions[7].at 	= 20'h0000D;		// rom, byte address table, cache-read-execute
	pma_regions[7].lock = "LOCK";

	// IO
	pma_regions[6].start = 48'hFF800000;
	pma_regions[6].nd = 48'hFF9FFFFF;
	pma_regions[6].pmt	 = 48'h00000300;
	pma_regions[6].cta	= 48'h00000000;
	pma_regions[6].at = 20'h00206;		// io, (screen) byte address table, read-write
	pma_regions[6].lock = "LOCK";

	// Vacant
	pma_regions[5].start = 48'hFFFFFFFF;
	pma_regions[5].nd = 48'hFFFFFFFF;
	pma_regions[5].pmt	 = 48'h00000000;
	pma_regions[5].cta	= 48'h00000000;
	pma_regions[5].at = 20'h0FF00;		// no access
	pma_regions[5].lock = "LOCK";

	// Scratchpad RAM
	pma_regions[4].start = 48'hFFFC0000;
	pma_regions[4].nd = 48'hFFFCFFFF;
	pma_regions[4].pmt	 = 48'h00002300;
	pma_regions[4].cta	= 48'h00000000;
	pma_regions[4].at = 20'h0020F;		// byte address table, read-write-execute cacheable
	pma_regions[4].lock = "LOCK";

	// vacant
	pma_regions[3].start = 48'hFFFFFFFF;
	pma_regions[3].nd = 48'hFFFFFFFF;
	pma_regions[3].pmt	 = 48'h00000000;
	pma_regions[3].cta	= 48'h00000000;
	pma_regions[3].at = 20'h0FF00;		// no access
	pma_regions[3].lock = "LOCK";

	// vacant
	pma_regions[2].start = 48'hFFFFFFFF;
	pma_regions[2].nd = 48'hFFFFFFFF;
	pma_regions[2].pmt	 = 48'h00000000;
	pma_regions[2].cta	= 48'h00000000;
	pma_regions[2].at = 20'h0FF00;		// no access
	pma_regions[2].lock = "LOCK";

	// DRAM
	pma_regions[1].start = 48'h00000000;
	pma_regions[1].nd = 48'h1FFFFFFF;
	pma_regions[1].pmt	 = 48'h00002400;
	pma_regions[1].cta	= 48'h00000000;
	pma_regions[1].at = 20'h0010F;	// ram, byte address table, cache-read-write-execute
	pma_regions[1].lock = "LOCK";

	// vacant
	pma_regions[0].start = 48'hFFFFFFFF;
	pma_regions[0].nd = 48'hFFFFFFFF;
	pma_regions[0].pmt	 = 48'h00000000;
	pma_regions[0].cta	= 48'h00000000;
	pma_regions[0].at = 20'h0FF00;		// no access
	pma_regions[0].lock = "LOCK";

end

always_ff @(posedge clk)
	if (wr && (pma_regions[rwa[6:4]].lock=="UNLK" || rwa[3:0]==4'hE)) begin
		case(rwa[3:0])
		4'd0:	pma_regions[rwa[6:4]].start[31: 0] <= i;
		4'd1:	pma_regions[rwa[6:4]].start[47:32] <= i[15:0];
		4'd2:	pma_regions[rwa[6:4]].nd[31:0] <= i;
		4'd3:	pma_regions[rwa[6:4]].nd[47:32] <= i[15:0];
		4'd4: pma_regions[rwa[6:4]].pmt[31:0] <= i;
		4'd5: pma_regions[rwa[6:4]].pmt[47:32] <= i[15:0];
		4'd6: pma_regions[rwa[6:4]].cta[31:0] <= i;
		4'd7: pma_regions[rwa[6:4]].cta[47:32] <= i[15:0];
		4'd8:	pma_regions[rwa[6:4]].at <= i;
		4'd14: pma_regions[rwa[6:4]].lock <= i;
		default:	;
		endcase
	end
always_ff @(posedge clk)
	case(rwa[3:0])
	4'd0:	o <= pma_regions[rwa[6:4]].start[31:0];
	4'd1:	o <= pma_regions[rwa[6:4]].start[47:32];
	4'd2:	o <= pma_regions[rwa[6:4]].nd[31:0];
	4'd3:	o <= pma_regions[rwa[6:4]].nd[47:32];
	4'd4:	o <= pma_regions[rwa[6:4]].pmt[31:0];
	4'd5:	o <= pma_regions[rwa[6:4]].pmt[47:32];
	4'd6:	o <= pma_regions[rwa[6:4]].cta[31:0];
	4'd7:	o <= pma_regions[rwa[6:4]].cta[47:32];
	4'd8:	o <= pma_regions[rwa[6:4]].at;
	4'd14:	o <= pma_regions[rwa[6:4]].lock;
	default:	;
	endcase

always_comb
begin
	err = 1'b1;
	region_num = 4'd0;
	region = pma_regions[0];
  for (n = 0; n < 8; n = n + 1)
    if (adr[47:4] >= pma_regions[n].start[47:4] && adr[47:4] <= pma_regions[n].nd[47:4]) begin
    	region = pma_regions[n];
    	region_num = n;
    	err = 1'b0;
  	end
end    	
    	
endmodule

