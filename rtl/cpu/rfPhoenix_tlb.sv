// ============================================================================
//        __
//   \\__/ o\    (C) 2020-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_tlb.sv
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

module rfPhoenix_tlb(rst_i, clk_i, clock, al_i, rdy_o, asid_i, sys_mode_i,xlaten_i,
	we_i,stptr_i,
	dadr_i,next_i,iacc_i,dacc_i,iadr_i,padr_o,acr_o,tlben_i,wrtlb_i,tlbadr_i,tlbdat_i,tlbdat_o,
	tlbmiss_o, tlbmiss_adr_o, tlbkey_o,
	m_cyc_o, m_ack_i, m_adr_o, m_dat_o);
parameter ASSOC = 5;	// MAX assoc = 15
parameter AWID=32;
parameter RSTIP = 32'hFFFD0000;
input rst_i;
input clk_i;
input clock;
input [1:0] al_i;
output rdy_o;
input [9:0] asid_i;
input sys_mode_i;
input xlaten_i;
input we_i;
input stptr_i;
input Address dadr_i;
input next_i;
input iacc_i;
input dacc_i;
input Address iadr_i;
output PhysicalAddress padr_o;
output reg [3:0] acr_o;
input tlben_i;
input wrtlb_i;
input [15:0] tlbadr_i;
input TLBE tlbdat_i;
output TLBE tlbdat_o;
output reg tlbmiss_o;
output Address tlbmiss_adr_o;
output reg [31:0] tlbkey_o;
output reg m_cyc_o;
input m_ack_i;
output Address m_adr_o;
output reg [127:0] m_dat_o;
parameter TRUE = 1'b1;
parameter FALSE = 1'b0;

integer n;
Address adr_i;
Address last_ladr, last_iadr;

reg [1:0] al;
reg LRU;
typedef enum logic [3:0] {
	ST_RST = 4'd0,
	ST_RUN = 4'd1,
	ST_AGE1 = 4'd2,
	ST_AGE2 = 4'd3,
	ST_AGE3 = 4'd4,
	ST_AGE4 = 4'd5,
	ST_WRITE_PTE = 4'd6
} tlb_state_t;
tlb_state_t state = ST_RST;

wire [AWID-1:0] rstip = RSTIP;
reg [3:0] randway;
TLBE tentryi [0:ASSOC-1];
TLBE tentryo [0:ASSOC-1];
TLBE tentryo2 [0:ASSOC-1];
reg stptr;
reg xlatend;
Address iadrd;

reg [ASSOC-1:0] wr;
reg wed;
reg [3:0] hit;
reg [ASSOC-1:0] wrtlb, next_wrtlb;
genvar g1;
generate begin : gWrtlb
	for (g1 = 0; g1 < ASSOC; g1 = g1 + 1)
		always_comb begin
			next_wrtlb[g1] <= 'd0;
			if (state==ST_RUN) begin
				if (LRU && tlbadr_i[2:0]!=ASSOC-1) begin
					if (g1==ASSOC-2)
						next_wrtlb[g1] <= wrtlb_i;
				end
				else begin
					if (tlbadr_i[2:0]==ASSOC-1) begin
						if (g1==ASSOC-1)
		 					next_wrtlb[g1] <= wrtlb_i;
		 			end
					else if (g1 < ASSOC-1)
		 				next_wrtlb[g1] <= (al==2'b10 ? randway==g1 : tlbadr_i[2:0]==g1) && wrtlb_i;
	 			end
 			end
 		end
end
endgenerate

TLBE tlbdato [0:ASSOC-1];
TLBE dumped_entry;
wire clk_g = clk_i;

// TLB RAM has a 1 cycle lookup latency.
// These signals need to be matched
always_ff @(posedge clk_g)
	xlatend <= xlaten_i;
always_ff @(posedge clk_g)
	iadrd <= iadr_i;

always_comb
	tlbdat_o <= tlbdato[tlbadr_i[2:0]];

always_ff @(posedge clk_g)
begin
	al <= al_i;
	LRU <= al_i==2'b01;
end

wire [ASSOC-1:0] wrtlbd;
ft_delay #(.WID(ASSOC), .DEP(3)) udlyw (.clk(clk_g), .ce(1'b1), .i(wrtlb), .o(wrtlbd));

integer n3, n4;
always_ff @(posedge clk_g)
begin
	dumped_entry <= 'd0;
	for (n3 = 0; n3 < ASSOC; n3 = n3 + 1)
		if (wrtlbd[n3])
			dumped_entry <= tlbdato[n3];
end

wire pe_xlat, ne_xlat;
edge_det u5 (
  .rst(rst_i),
  .clk(clk_g),
  .ce(1'b1),
  .i(xlaten_i),
  .pe(pe_xlat),
  .ne(ne_xlat),
  .ee()
);

// Detect a change in the page number
wire cd_dadr, cd_iadr;
change_det #(.WID($bits(Address)-16)) ucd1 (
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.i(dadr_i[$bits(Address)-1:16]),
	.cd(cd_dadr)
);

change_det #(.WID($bits(Address)-16)) ucd2 (
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.i(iadr_i[$bits(Address)-1:16]),
	.cd(cd_iadr)
);

reg [5:0] dld, dli;
always_ff @(posedge clk_g)
	if (cd_dadr)
		dld <= 6'd0;
	else
		dld <= {dld[4:0],1'b1};
always_ff @(posedge clk_g)
	if (cd_iadr)
		dli <= 6'd0;
	else
		dli <= {dli[4:0],1'b1};

TLBE tlbdat_rst;
TLBE [ASSOC-1:0] tlbdati;
TLBE tlbdati_r;
reg [9:0] tlbadri_r;
reg [5:0] count;
reg [ASSOC-1:0] tlbwrr;
reg [ASSOC-1:0] tlbwr_r;
reg tlbeni;
reg [9:0] tlbadri;
reg clock_r;

always_ff @(posedge clk_g)
if (rst_i) begin
	randway <= 'd0;
end
else begin
	if (!wrtlb_i) begin
		randway <= randway + 2'd1;
		if (randway==ASSOC-2)
			randway <= 'd0;
	end
end

reg [9:0] rcount;
wire pe_clock;
edge_det edclk (.rst(rst_i), .clk(clk_g), .ce(1'b1), .i(clock), .pe(pe_clock), .ne(), .ee());

always_ff @(posedge clk_g)
if (rst_i) begin
	state <= ST_RST;
	tlbeni <= 1'b1;		// forces ready low
	tlbwrr <= 'd0;
	count <= 6'h0;		// Map only last 256kB
	clock_r <= 1'b0;
	m_cyc_o <= 1'b0;
	m_dat_o <= 'd0;
end
else begin
tlbeni  <= 1'b0;
tlbwrr <= 'd0;
if (pe_clock)
	clock_r <= 1'b1;
case(state)
	
// Setup the last 256kB/32 pages of memory to point to the ROM BIOS.
ST_RST:
	begin
		tlbeni <= 1'b1;
		tlbwrr <= 'd0;
		case(count[5])
//		13'b000: begin tlbwr0r <= 1'b1; tlbdat_rst <= {8'h00,8'hEF,14'h0,count[11:10],12'h000,8'h00,count[11:0]};	end // Map 16MB RAM area
//		13'b001: begin tlbwr1r <= 1'b1; tlbdat_rst <= {8'h00,8'hEF,14'h1,count[11:10],12'h000,8'h00,count[11:0]};	end // Map 16MB RAM area
//		13'b010: begin tlbwr2r <= 1'b1; tlbdat_rst <= {8'h00,8'hEF,14'h2,count[11:10],12'h000,8'h00,count[11:0]};	end // Map 16MB RAM area
		1'b0:
			begin
				tlbwrr[ASSOC-1] <= 1'b1; 
				tlbdat_rst <= 'd0;
				tlbdat_rst.asid <= 'd0;
				tlbdat_rst.g <= 1'b1;
				tlbdat_rst.v <= 1'b1;
				tlbdat_rst.m <= 1'b1;
				tlbdat_rst.rwx <= 3'd7;
				tlbdat_rst.c <= 1'b1;
				// FFFC0000
				// 1111_1111_ 11_11_1100_000 0_0000_0000_0000
				tlbdat_rst.vpn <= {27'h003FFF,count[4:0]};
				tlbdat_rst.ppn <= {35'h003FFF,count[4:0]};
				rcount <= {5'h1F,count[4:0]};
			end // Map 16MB ROM/IO area
		1'b1: begin state <= ST_RUN; tlbwrr[ASSOC-1] <= 1'd1; end
		default:	;
		endcase
		count <= count + 2'd1;
	end
ST_RUN:
	begin
		wrtlb <= next_wrtlb;
		if (|next_wrtlb) begin
			;
		end
		else if (dumped_entry.m && |dumped_entry.adr) begin
			wrtlb <= 'd0;
			state <= ST_WRITE_PTE;
		end
		else if (clock_r) begin
			wrtlb <= 'd0;
			rcount <= rcount + 2'd1;
			clock_r <= 1'b0;
			state <= ST_AGE1;
		end
	end
ST_AGE1:
	begin
		tlbeni <= 1'b1;
		state <= ST_AGE2;
	end
ST_AGE2:
	begin
		tlbeni <= 1'b1;
		state <= ST_AGE3;
	end
ST_AGE3:
	begin
		tlbeni <= 1'b1;
		state <= ST_AGE4;
	end
ST_AGE4:
	begin
		tlbeni <= 1'b1;
		tlbwrr <= {ASSOC{1'b1}};
		state <= ST_RUN;
	end
ST_WRITE_PTE:
	if (|dumped_entry.adr) begin
		m_cyc_o <= 1'b1;
		m_adr_o <= dumped_entry.adr;
		m_dat_o <= dumped_entry;
		m_dat_o[55] <= 1'b0;	// modified bit
		if (m_ack_i) begin
			m_cyc_o <= 1'b0;
			state <= ST_RUN;
		end
	end
	else
		state <= ST_RUN;
default:
	state <= ST_RUN;
endcase
end
assign rdy_o = ~tlbeni;

integer n2;
always_ff @(posedge clk_g)
begin
	case(state)
	ST_RST:	
		begin
			tlbwr_r <= 'd0;
			tlbadri_r <= 'd0;
			tlbdati_r <= 'd0;
			tlbadri <= rcount;
			for (n2 = 0; n2 < ASSOC; n2 = n2 + 1)
				tlbdati[n2] <= tlbdat_rst;
		end
	ST_RUN:
		begin
			tlbadri <= tlbadr_i[15:5];
			for (n2 = 0; n2 < ASSOC; n2 = n2 + 1)
				tlbdati[n2] <= tlbdat_i;
			tlbwr_r <= next_wrtlb;
			tlbadri_r <= tlbadr_i[15:5];
			tlbdati_r <= tlbdat_i;
		end
	ST_AGE1,ST_AGE2,ST_AGE3:
		begin
			tlbadri <= rcount;
			for (n2 = 0; n2 < ASSOC; n2 = n2 + 1)
				tlbdati[n2] <= tlbdat_i;
		end
	ST_AGE4:
		begin
			tlbadri <= rcount;
			for (n2 = 0; n2 < ASSOC; n2 = n2 + 1) begin
				tlbdati[n2] <= tlbdato[n2];
			end
		end
	default:
		begin
			tlbadri <= tlbadr_i[15:5];
			for (n2 = 0; n2 < ASSOC; n2 = n2 + 1)
				tlbdati[n2] <= tlbdat_i;
		end
	endcase
	if (tlbdati[4].ppn=='d0 && tlbdati[4].vpn != 'd0) begin
		$display("PPN zero");
	end
end
always_comb
	adr_i = iadr_i;

// Dirty / Accessed bit write logic
always_ff @(posedge clk_g)
  wed <= we_i;
always_ff @(posedge clk_g)
	stptr <= stptr_i;

integer n1,j1;
always_ff @(posedge clk_g)
begin
	wr <= 'd0;
  if (ne_xlat) begin
  	for (n1 = 0; n1 < ASSOC; n1 = n1 + 1) begin
  		if (hit==n1) begin
  			if (LRU && n1 < ASSOC-1) begin
	  			wr <= {ASSOC{1'b1}};
  				for (j1 = 1; j1 < ASSOC; j1 = j1 + 1) begin
  					if (j1 <= n1)
  						tentryi[j1] <= tentryo2[j1-1];
  					else
  						tentryi[j1] <= tentryo2[j1];
  				end
	  			tentryi[0] <= tentryo2[n1];
	  			if (wed)
	  				tentryi[0].m <= 1'b1;
	  			//tentryi[0].a <= 1'b1;
//					if (stptr)
//						tentryo[0].cards[(tentryo[n1].vpn >> ({tentryo[n1].lvl-2'd1,3'd0} + 2'd3)) & 5'h1F] <= 1'b1;
  			end
  			else begin
	  			tentryi[n1] <= tentryo2[n1];
	  			if (wed)
	  				tentryi[n1].m <= 1'b1;
	  			//tentryi[n1].a <= 1'b1;
//					if (stptr)
//						tentryo[n1].cards[(tentryo[n1].vpn >> ({tentryo[n1].lvl-2'd1,3'd0} + 2'd3)) & 5'h1F] <= 1'b1;
	  			wr[n1] <= 1'b1;
  			end
  		end
  	end
  end
end

genvar g;
generate begin : gTlbRAM
for (g = 0; g < ASSOC; g = g + 1)
	rfPhoenix_TLBRam u1 (
	  .clka(clk_g),    // input wire clka
	  .ena(tlben_i|tlbeni),      // input wire ena
	  .wea(wrtlb[g]|tlbwrr[g]),      // input wire [0 : 0] wea
	  .addra(tlbadri),  // input wire [9 : 0] addra
	  .dina(tlbdati[g]),    // input wire [63 : 0] dina
	  .douta(tlbdato[g]),  // output wire [63 : 0] douta
	  .clkb(clk_g),    // input wire clkb
	  .enb(xlaten_i),      // input wire enb
	  .web(wr[g]),      // input wire [0 : 0] web
	  .addrb(adr_i[23:13]),  // input wire [9 : 0] addrb
	  .dinb(tentryi[g]),    // input wire [63 : 0] dinb
	  .doutb(tentryo[g])  // output wire [63 : 0] doutb
	);
end
endgenerate

always_ff @(posedge clk_g)
if (rst_i) begin
  padr_o[15:0] <= rstip[15:0];
  padr_o[AWID-1:16] <= rstip[AWID-1:16];
  hit <= 4'd15;
  tlbmiss_o <= FALSE;
	tlbmiss_adr_o <= 'd0;
	tlbkey_o <= 32'hFFFFFFFF;
  acr_o <= 4'hF;
end
else begin
 	padr_o <= padr_o;
  if (pe_xlat)
  	hit <= 4'd15;
	if (next_i)
		padr_o <= padr_o + 6'd32;
  else begin
		if (!xlatend) begin
	    tlbmiss_o <= FALSE;
	  	padr_o[31:0] <= iadrd[31:0];
	    acr_o <= 4'hF;
		end
		else begin
			tlbmiss_o <= dli[4] & ~cd_iadr;
			tlbmiss_adr_o <= iadrd;
			hit <= 4'd15;
			acr_o <= 4'h0;
			for (n = 0; n < ASSOC; n = n + 1) begin
				tentryo2[n] <= tentryo[n];
				if (tentryo[n].vpn[18:10]==iadrd[31:23] && (tentryo[n].asid==asid_i || tentryo[n].g) && tentryo[n].v) begin
			  	padr_o[12:0] <= iadrd[12:0];
					padr_o[31:13] <= tentryo[n].ppn[18:0];
					acr_o <= {tentryo[n].ppn < 19'h07FFF || tentryo[n].ppn > 19'h7FFE0,tentryo[n].rwx};
					tlbmiss_o <= FALSE;
					hit <= n;
				end
			end
		end
	end
end

endmodule
