// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_icache.sv
//	- instruction cache
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

import wishbone_pkg::*;
import rfPhoenixPkg::*;
import rfPhoenixMmupkg::*;

module rfPhoenix_icache(rst,clk,state,
	ip,ip_o,ihit,ihite,ihito,ic_line,ic_valid,ic_tage,ic_tago,
	upd_adr,ici,ic_wway,wr_ic1,wr_ic2);
input rst;
input clk;
input [6:0] state;
input code_address_t ip;
output code_address_t ip_o;
output reg ihit;
output reg ihite;
output reg ihito;
output reg [$bits(ICacheLine)*2-1:0] ic_line;
output reg ic_valid;
output reg [$bits(address_t)-1:7] ic_tage;
output reg [$bits(address_t)-1:7] ic_tago;
input wb_address_t upd_adr;
input ICacheLine ici;
input [1:0] ic_wway;
input wr_ic1;
input wr_ic2;

parameter FALSE = 1'b0;

ICacheLine ic_eline, ic_oline;
reg [1:0] ic_rwaye,ic_rwayo,ic_wway;
reg icache_wre, icache_wro;
always_comb icache_wre = wr_ic2 && !upd_adr[5];
always_comb icache_wro = wr_ic2 &&  upd_adr[5];
reg ic_invline,ic_invall;
code_address_t ip2,ip3,ip4,ip5;
wire [$bits(code_address_t)-1:14] ictage [0:3];
wire [$bits(code_address_t)-1:14] ictago [0:3];
wire [1024/4-1:0] icvalide [0:3];
wire [1024/4-1:0] icvalido [0:3];

wire ihit2;
reg ihit3;
wire ic_valid2e, ic_valid2o;
reg ic_valide, ic_valido;
reg ic_valid3e, ic_valid3o;
wire [$bits(code_address_t)-7:0] ic_tag2e, ic_tag2o;
reg [$bits(code_address_t)-7:0] ic_tag3e, ic_tag3o;

always_ff @(posedge clk)
	ip2 <= ip;
always_ff @(posedge clk)
	ip3 <= ip2;
always_ff @(posedge clk)
	ip4 <= ip3;
always_ff @(posedge clk)
	ip5 <= ip4;
// line up ihit output with cache line output.
always_ff @(posedge clk)
	ihit3 <= ihit2;
always_comb
	// *** The following causes the hit to tend to oscillate between hit
	//     and miss.
	// If cannot cross cache line can match on either odd or even.
	if (FALSE && ip2[4:0] < 5'd22)
		ihit <= ip2[5] ? ihit2o : ihit2e;
	// Might span lines, need hit on both even and odd lines
	else
		ihit <= ihit2e&ihit2o;
always_comb
	// *** The following causes the hit to tend to oscillate between hit
	//     and miss.
	// If cannot cross cache line can match on either odd or even.
	// If we do not need the even cache line, mark as a hit.
	if (FALSE && ip2[4:0] < 6'd22)
		ihite <= ip2[5] ? 1'b1 : ihit2e;
	// Might span lines, need hit on both even and odd lines
	else
		ihite <= ihit2e;
always_comb
	// *** The following causes the hit to tend to oscillate between hit
	//     and miss.
	// If cannot cross cache line can match on either odd or even.
	// If we do not need the odd cache line, mark as a hit.
	if (FALSE && ip2[4:0] < 5'd22)
		ihito <= ip2[5] ? ihit2o : 1'b1;
	// Might span lines, need hit on both even and odd lines
	else
		ihito <= ihit2o;

always_ff @(posedge clk)
	ic_valid3e <= ic_valid2e;
always_ff @(posedge clk)
	ic_valid3o <= ic_valid2o;
always_ff @(posedge clk)
	ic_valide <= ic_valid2e;
always_ff @(posedge clk)
	ic_valido <= ic_valid2o;
assign ip_o = ip3;
always_ff @(posedge clk)
	ic_tag3o <= ic_tag2o;
always_ff @(posedge clk)
	ic_tago <= ic_tag3o;
always_ff @(posedge clk)
	ic_tag3e <= ic_tag2e;
always_ff @(posedge clk)
	ic_tage <= ic_tag3e;

always_ff @(posedge clk)
	// If cannot cross cache line can match on either odd or even.
	if (FALSE && ip2[4:0] < 5'd22)
		ic_valid <= ip2[5] ? ic_valid2o : ic_valid2e;
	else
		ic_valid <= ic_valid2o & ic_valid2e;

// 256 wide x 1024 deep, 1 cycle read latency.
sram_256x1024_1r1w uicme
(
	.rst(rst),
	.clk(clk),
	.wr(icache_wre),
	.wadr({ic_wway,upd_adr[13:6]}),//+upd_adr[5]}),
	.radr({ic_rwaye,ip2[13:6]+ip2[5]}),
	.i(ici),
	.o(ic_eline)
);

sram_256x1024_1r1w uicmo
(
	.rst(rst),
	.clk(clk),
	.wr(icache_wro),
	.wadr({ic_wway,upd_adr[13:6]}),
	.radr({ic_rwayo,ip2[13:6]}),
	.i(ici),
	.o(ic_oline)
);

always_comb
	case(ip3[5])
	1'b0:	ic_line = {ic_oline.data,ic_eline.data};
	1'b1:	ic_line = {ic_eline.data,ic_oline.data};
	endcase

rfPhoenix_ictag 
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
uictage
(
	.rst(rst),
	.clk(clk),
	.wr(icache_wre),
	.ipo(upd_adr),
	.way(ic_wway),
	.rclk(clk),
	.ndx(ip2[13:6]+ip2[5]),	// virtual index (same bits as physical address)
	.tag(ictage)
);

rfPhoenix_ictag 
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
uictago
(
	.rst(rst),
	.clk(clk),
	.wr(icache_wro),
	.ipo(upd_adr),
	.way(ic_wway),
	.rclk(clk),
	.ndx(ip2[13:6]),		// virtual index (same bits as physical address)
	.tag(ictago)
);

rfPhoenix_ichit
#(
	.LINES(256),
	.WAYS(4)
)
uichite
(
	.clk(clk),
	.ip(ip),
	.ndx(ip[13:6]+ip[5]),
	.tag(ictage),
	.valid(icvalide),
	.ihit(ihit2e),
	.rway(ic_rwaye),
	.vtag(ic_tag2e),
	.icv(ic_valid2e)
);

rfPhoenix_ichit
#(
	.LINES(256),
	.WAYS(4)
)
uichito
(
	.clk(clk),
	.ip(ip),
	.ndx(ip[13:6]),
	.tag(ictago),
	.valid(icvalido),
	.ihit(ihit2o),
	.rway(ic_rwayo),
	.vtag(ic_tag2o),
	.icv(ic_valid2o)
);

rfPhoenix_icvalid 
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
uicvale
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4),
	.ip(upd_adr),
	.adr(upd_adr),
	.wr(icache_wre),
	.way(ic_wway),
	.invline(ic_invline),
	.invall(ic_invall),
	.valid(icvalide)
);

rfPhoenix_icvalid 
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
uicvalo
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4),
	.ip(upd_adr),
	.adr(upd_adr),
	.wr(icache_wro),
	.way(ic_wway),
	.invline(ic_invline),
	.invall(ic_invall),
	.valid(icvalido)
);

endmodule
