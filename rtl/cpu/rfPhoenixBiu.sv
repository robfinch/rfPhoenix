// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenixBiu.sv
//	- bus interface unit
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

module rfPhoenixBiu(rst,clk,tlbclk,clock,UserMode,MUserMode,omode,ASID,bounds_chk,pe,
	ip,ihit,ifStall,ic_line, fifoToCtrl_wack,
	fifoToCtrl_i,fifoToCtrl_full_o,fifoFromCtrl_o,fifoFromCtrl_rd,fifoFromCtrl_empty,fifoFromCtrl_v,
	bok_i, bte_o, cti_o, vpa_o, vda_o, cyc_o, stb_o, ack_i, we_o, sel_o, adr_o,
	dat_i, dat_o, sr_o, cr_o, rb_i, dce, keys, arange, ptbr, ipage_fault, clr_ipage_fault,
	itlbmiss, clr_itlbmiss);
parameter AWID=32;
input rst;
input clk;
input tlbclk;
input clock;							// clock for clock algorithm
input UserMode;
input MUserMode;
input [1:0] omode;
input [11:0] ASID;
input bounds_chk;
input pe;									// protected mode enable
input Address ip;
output reg ihit;
input ifStall;
output [pL1ICacheLineSize-1:0] ic_line;
// Fifo controls
output fifoToCtrl_wack;
input sMemoryRequest fifoToCtrl_i;
output fifoToCtrl_full_o;
output sMemoryResponse fifoFromCtrl_o;
input fifoFromCtrl_rd;
output fifoFromCtrl_empty;
output fifoFromCtrl_v;
// Bus controls
input bok_i;
output reg [1:0] bte_o;
output reg [2:0] cti_o;
output reg vpa_o;
output reg vda_o;
output reg cyc_o;
output reg stb_o;
input ack_i;
output reg we_o;
output reg [15:0] sel_o;
output Address adr_o;
input [127:0] dat_i;
output reg [127:0] dat_o;
output reg sr_o;
output reg cr_o;
input rb_i;

output reg dce;							// data cache enable
input [23:0] keys [0:7];
input [2:0] arange;
input [127:0] ptbr;
output reg ipage_fault;
input clr_ipage_fault;
output reg itlbmiss;
input clr_itlbmiss;

parameter TRUE = 1'b1;
parameter FALSE = 1'b0;
parameter HIGH = 1'b1;
parameter LOW = 1'b0;

parameter IO_KEY_ADR	= 16'hFF88;

integer m,n,k;
integer n4,n5;
genvar g;

reg [5:0] shr_ma;

reg [6:0] state;
// States for hardware routine stack, five deep.
// States go at least 3 deep.
// Memory1
// PT_FETCH <on a tlbmiss>
// READ_PDE/PTE
// 
Address next_adr_o;
reg [6:0] stk_state1, stk_state2, stk_state3, stk_state4, stk_state5;

reg xlaten_stk;
reg vpa_stk;
reg vda_stk;
reg [1:0] bte_stk;
reg [2:0] cti_stk;
reg cyc_stk;
reg stb_stk;
reg we_stk;
reg [15:0] sel_stk;
Address adro_stk;
Address dadr_stk;
Address iadr_stk;
reg [127:0] dato_stk;

reg [1:0] waycnt;
reg iaccess;
reg daccess;
reg [4:0] icnt;
reg [4:0] dcnt;
Address iadr;
reg keyViolation = 1'b0;
reg xlaten;
reg [31:0] memreq_sel;

sMemoryRequest memreq,imemreq;
reg memreq_rd = 0;
sMemoryResponse memresp;
sMemoryResponse [7:0] mem_resp;	// memory pipeline
reg zero_data = 0;
Value movdat;

// 0,1: PTE
// 2,3: PMT
// 4: PTE address
// 5: PMT address
// 6: TLB update address + way
// 15: trigger read / write
reg [63:0] tlb_bucket [0:15];

Address cta;		// card table address
Address ea;
Address afilt;

always_comb
	afilt = (memreq.func==MR_MOVST) ? memreq.dat : memreq.adr;

always_comb
	ea = cta + (afilt >> shr_ma);

reg [7:0] ealow;

reg [1:0] strips;
reg [63:0] sel;
reg [63:0] nsel;
reg [511:0] dati512;
reg [255:0] dat, dati;
wire [127:0] datis;

biu_dati_align uda1
(
	.dati(mem_resp[6].res),
	.datis(datis), 
	.amt({1'b0,mem_resp[6].badAddr[3:0],3'b0})
);

`ifdef CPU_B64
reg [15:0] sel;
reg [127:0] dat, dati;
wire [63:0] datis = dati >> {ealow[2:0],3'b0};
`endif
`ifdef CPU_B32
reg [7:0] sel;
reg [63:0] dat, dati;
wire [63:0] datis = dati >> {ealow[1:0],3'b0};
`endif

// Build an insert mask for data cache store operations.
wire [511:0] stmask;

rfPhoenix_stmask ustmsk (sel_o, adr_o[5:4], stmask);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// PMA Checker
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

REGION region;
wire [2:0] region_num;
reg rgn_wr;
reg rgn_en;
reg [5:0] rgn_adr;
Value rgn_dat;
Value rgn_dat_o;

rfPhoenix_active_region uargn
(
	.clk(clk),
	.wr(rgn_wr),
	.rwa(rgn_adr),
	.i(rgn_dat),
	.o(rgn_dat_o),
	.adr(next_adr_o),
	.region_num(),
	.region(region),
	.err()
);

PMTE arti = dat_i;
Address pmt_adr = adr_o[AWID-1:0] - region.start[AWID-1:0];

wire [3:0] ififo_cnt, ofifo_cnt;

wire [16:0] lfsr_o;

lfsr ulfsr1
(
	.rst(rst),
	.clk(clk),
	.ce(1'b1),
	.cyc(1'b0),
	.o(lfsr_o)
);

wire fifoToCtrl_empty;
wire fifoToCtrl_v;

wire pev;
edge_det ued1 (.rst(rst), .clk(clk), .ce(1'b1), .i(fifoToCtrl_v), .pe(pev), .ne(), .ee());

rfPhoenix_mem_req_queue umreqq
(
	.rst(rst),
	.clk(clk),
	.wr0(fifoToCtrl_i.wr),
	.wr_ack0(fifoToCtrl_wack),
	.i0(fifoToCtrl_i),
	.wr1(1'b0),
	.wr_ack1(),
	.i1('d0),
	.rd(memreq_rd & ~pev),
	.o(imemreq),
	.valid(fifoToCtrl_v),
	.empty(fifoToCtrl_empty),
	.ldo0(),
	.found0(),
	.ldo1(),
	.found1(),
  .full(fifoToCtrl_full_o)
);

wire memresp_full;
MemoryResponseFifo uofifo1
(
  .clk(clk),      // input wire clk
  .srst(rst),    // input wire srst
  .din(memresp),      // input wire [197 : 0] din
  .wr_en(memresp.wr),  // input wire wr_en
  .rd_en(fifoFromCtrl_rd),  // input wire rd_en
  .dout(fifoFromCtrl_o),    // output wire [197 : 0] dout
  .full(memresp_full),    // output wire full
  .empty(fifoFromCtrl_empty),  // output wire empty
  .valid(fifoFromCtrl_v)  // output wire valid
);


reg rd_memq;
sMemoryResponse memq_o, memr;
MemoryResponseFifo uofifo2
(
  .clk(clk),      // input wire clk
  .srst(rst),    // input wire srst
  .din(mem_resp[7]),      // input wire [197 : 0] din
  .wr_en(mem_resp[7].wr),  // input wire wr_en
  .rd_en(rd_memq),  // input wire rd_en
  .dout(memq_o),    // output wire [197 : 0] dout
  .full(),    // output wire full
  .empty(),  // output wire empty
  .valid(memq_v)  // output wire valid
);
/*
bc_fifo16X #(.WID($bits(MemoryResponse))) uififo2
(
	.clk(clk),
	.reset(rst),
	.wr(memresp.fifo_wr),
	.rd(fifoFromCtrl_rd),
	.di(memresp),
	.dout(fifoFromCtrl_o),
	.ctr(ofifo_cnt)
);

assign fifoFromCtrl_empty = ofifo_cnt==4'd0;
*/

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Instruction cache
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

reg [1:0] ic_rway,ic_wway;
reg icache_wr;
always_comb icache_wr = state==IFETCH3;
reg ic_invline,ic_invall;
Address ipo,ip2,ip3,ip4;
wire [AWID-1:6] ictag [0:3];
wire [512/4-1:0] icvalid [0:3];

reg [639:0] ici;		// Must be a multiple of 128 bits wide for shifting.
wire [AWID-7:0] ic_tag;
reg [2:0] ivcnt;
reg [2:0] vcn;
reg [pL1ICacheLineSize-1:0] ivcache [0:4];
reg [AWID-1:6] ivtag [0:4];
reg [4:0] ivvalid;
wire ic_valid;
wire ihit2;
reg ihit3;

always_ff @(posedge clk)
	ip2 <= ip;
always_ff @(posedge clk)
	ip3 <= ip2;
always_ff @(posedge clk)
	ip4 <= ip3;
// line up ihit output with cache line output.
always_ff @(posedge clk)
	ihit3 <= ihit2;
always_ff @(posedge clk)
	ihit <= ihit3;

// 640 wide x 512 deep, uses output register so there is a 2 cycle read latency.
icache_blkmem uicm (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(icache_wr),      // input wire [0 : 0] wea
  .addra({waycnt,ipo[12:6]}),  // input wire [8 : 0] addra
  .dina(ici[pL1ICacheLineSize-1:0]),    // input wire [511 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),//!ifStall),      // input wire enb
  .addrb({ic_rway,ip2[12:6]}),  // input wire [8 : 0] addrb
  .doutb(ic_line)  // output wire [511 : 0] doutb
);

rfPhoenix_ictag 
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
uictag1
(
	.clk(clk),
	.wr(icache_wr),
	.ipo(ipo),
	.way(waycnt),
	.rclk(clk),
	.ip(ip),
	.tag(ictag)
);

rfPhoenix_ichit
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
uichit1
(
	.clk(clk),
	.ip(ip),
	.tag(ictag),
	.valid(icvalid),
	.ihit(ihit2),
	.rway(ic_rway),
	.vtag(ic_tag),
	.icv(ic_valid)
);

rfPhoenix_icvalid 
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
uicval1
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4),
	.ip(ipo),
	.adr(adr_o),
	.wr(icache_wr),
	.way(waycnt),
	.invline(ic_invline),
	.invall(ic_invall),
	.valid(icvalid)
);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Key Cache
// - the key cache is direct mapped, 64 lines of 512 bits.
// - keys are stored in the low order 20 bits of a 32-bit memory cell
// - 16 keys per 512 bit cache line
// - one cache line is enough to cover 256kB of memory
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

`ifdef SUPPORT_KEYCHK
reg [19:0] io_keys [0:511];
initial begin
	for (n = 0; n < 512; n = n + 1)
		io_keys[n] = 20'h0;
reg [511:0] kyline [0:63];
reg [AWID-19:0] kytag;
reg [63:0] kyv;
reg kyhit;
reg io_adr;
always_comb
	io_adr <= adr_o[31:23]==9'b1111_1111_1;
always_comb
	kyhit <= kytag[adr_o[23:18]]==adr_o[AWID-1:18] && kyv[adr_o[23:18]] || io_adr;
initial begin
	kyv = 64'd0;
	for (n = 0; n < 64; n = n + 1) begin
		kyline[n] = 512'd0;
		kytag[n] = 32'd1;
	end
end
reg [19:0] kyut;
always_comb
	kyut <= io_adr ? io_keys[adr_o[31:23]] : kyline[adr_o[23:18]] >> {adr_o[17:14],5'd0};
`endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Data Cache
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
wire [3:0] tlbacr;

reg [2:0] dwait;		// wait state counter for dcache
Address dadr;
reg [511:0] dci;		// 512 + 120 bit overflow area
wire [511:0] dc_eline, dc_oline;
reg [1023:0] dc_line;
reg [1023:0] datil;
reg dcachable;
reg [1:0] dc_erway,prev_dc_erway;
reg [1:0] dc_orway,prev_dc_orway;
wire [1:0] dc_ewway;
wire [1:0] dc_owway;
reg dcache_ewr, dcache_owr;
reg dc_invline,dc_invall;

dcache_blkmem udcb1e (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(dcache_ewr),      // input wire [0 : 0] wea
  .addra({dc_ewway,dadr[13:7]+adr_o[6]}),  // input wire [8 : 0] addra
  .dina(dci),    // input wire [511 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb({dc_erway,adr_o[13:7]+adr_o[6]}),  // input wire [8 : 0] addrb
  .doutb(dc_eline)  // output wire [511 : 0] doutb
);

dcache_blkmem udcb1o (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(dcache_owr),      // input wire [0 : 0] wea
  .addra({dc_owway,dadr[13:7]}),  // input wire [8 : 0] addra
  .dina(dci),    // input wire [511 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb({dc_orway,adr_o[13:7]}),  // input wire [8 : 0] addrb
  .doutb(dc_oline)  // output wire [511 : 0] doutb
);

always_comb
	case(adr_o[6])
	1'b0:	dc_line = {dc_oline,dc_eline};
	1'b1:	dc_line = {dc_eline,dc_oline};
	endcase

wire [AWID-7:0] dc_etag [3:0];
wire [127:0] dc_evalid [0:3];
wire [3:0] dhit1e;
wire [AWID-7:0] dc_otag [3:0];
wire [127:0] dc_ovalid [0:3];
wire [3:0] dhit1o;
wire dhito,dhite;

rfPhoenix_dchit udchite
(
	.rst(rst),
	.clk(clk),
	.tags(dc_etag),
	.ndx(adr_o[13:7]+adr_o[6]),
	.adr(adr_o),
	.valid(dc_evalid),
	.hits(dhit1e),
	.hit(dhite),
	.rway(dc_erway)
);

rfPhoenix_dchit udchito
(
	.rst(rst),
	.clk(clk),
	.tags(dc_otag),
	.ndx(adr_o[13:7]),
	.adr(adr_o),
	.valid(dc_ovalid),
	.hits(dhit1o),
	.hit(dhito),
	.rway(dc_orway)
);

reg dhit;
always_comb
	dhit = (dhite & dhito) || (adr_o[6] ? (dhito && adr_o[5:4] != 2'b11) : (dhite && adr_o[5:4] != 2'b11));

rfPhoenix_dctag
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
udcotag
(
	.clk(clk),
	.wr(state==DFETCH7 && dadr[6]),
	.adr(dadr),
	.way(lfsr_o[1:0]),
	.rclk(tlbclk),
	.ndx(adr_o[13:7]),
	.tag(dc_otag)
);

rfPhoenix_dctag
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
udcetag
(
	.clk(clk),
	.wr(state==DFETCH7 && ~dadr[6]),
	.adr(dadr),
	.way(lfsr_o[1:0]),
	.rclk(tlbclk),
	.ndx(adr_o[13:7]+adr_o[6]),
	.tag(dc_etag)
);

rfPhoenix_dcvalid
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
udcovalid
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4 && adr_o[6]),
	.dadr(dadr),
	.adr(adr_o),
	.wr(state==DFETCH7 && dadr[6]),
	.way(lfsr_o[1:0]),
	.invline(dc_invline),
	.invall(dc_invall),
	.valid(dc_ovalid)
);

rfPhoenix_dcvalid
#(
	.LINES(128),
	.WAYS(4),
	.AWID(AWID)
)
udcevalid
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4 && ~adr_o[6]),
	.dadr(dadr),
	.adr(adr_o),
	.wr(state==DFETCH7 && ~dadr[6]),
	.way(lfsr_o[1:0]),
	.invline(dc_invline),
	.invall(dc_invall),
	.valid(dc_evalid)
);

rfPhoenix_dcache_wr udcwre
(
	.clk(clk),
	.state(state),
	.ack(ack_i),
	.func(memreq.func),
	.dce(dce),
	.hit(dhit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(tlbacr),
	.eaeo(~ealow[6]),
	.daeo(~dadr[6]),
	.wr(dcache_ewr)
);

rfPhoenix_dcache_wr udcwro
(
	.clk(clk),
	.state(state),
	.ack(ack_i),
	.func(memreq.func),
	.dce(dce),
	.hit(dhit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(tlbacr),
	.eaeo(ealow[6]),
	.daeo(dadr[6]),
	.wr(dcache_owr)
);

rfPhoenix_dcache_way udcwaye
(
	.rst(rst),
	.clk(clk),
	.state(state),
	.ack(ack_i),
	.func(memreq.func),
	.dce(dce),
	.hit(dhit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(tlbacr),
	.eaeo(~ealow[6]),
	.daeo(~dadr[6]),
	.lfsr(lfsr_o[1:0]),
	.rway(dc_erway),
	.wway(dc_ewway)
);

rfPhoenix_dcache_way udcwayo
(
	.rst(rst),
	.clk(clk),
	.state(state),
	.ack(ack_i),
	.func(memreq.func),
	.dce(dce),
	.hit(dhit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(tlbacr),
	.eaeo(ealow[6]),
	.daeo(dadr[6]),
	.lfsr(lfsr_o[1:0]),
	.rway(dc_orway),
	.wway(dc_owway)
);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// TLB
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

reg tlb_access = 1'b0;
TLBE tmptlbe;
reg [5:0] ipt_miss_count;
reg tlben, tlbwr;
wire tlbmiss;
wire tlbrdy;
TLBE tlbdato;
reg [31:0] tlb_ia;
TLBE tlb_ib;
wire tlb_cyc;
wire [127:0] tlb_dat;
Address tlb_adr;
reg tlb_ack;
reg inext;
VirtualAddress tlbmiss_adr;
VirtualAddress miss_adr;
reg wr_ptg;
always_comb
begin
	tlb_ib[ 63:  0] <= tlb_bucket[0];
	tlb_ib[127: 64] <= tlb_bucket[1];
	tlb_ib[191:128] <= tlb_bucket[2];
	tlb_ib[255:128] <= tlb_bucket[3];
	tlb_ib.adr 			<= tlb_bucket[4];
	tlb_ib.pmtadr 	<= tlb_bucket[5];
	tlb_ia <= tlb_bucket[6][31:0];
end

`ifndef SUPPORT_HASHPT
PMTE pmtram_dinb;
PMTE pmtram_doutb;
wire pmtram_web;
wire [13:0] pmtram_adrb;

rfPhoenix_tlb utlb (
  .rst_i(rst),
  .clk_i(tlbclk),
  .al_i(ptbr[7:6]),
  .clock(clock),
  .rdy_o(tlbrdy),
  .asid_i(ASID),
  .sys_mode_i(vpa_o ? ~UserMode : ~MUserMode),
  .xlaten_i(xlaten),
  .we_i(we_o),
  .dadr_i(dadr),
  .next_i(inext),
  .iacc_i(iaccess),
  .dacc_i(daccess),
  .iadr_i(iadr),
  .padr_o(adr_o),
  .acr_o(tlbacr),
  .tlben_i(tlben),
  .wrtlb_i(tlbwr),
  .tlbadr_i(tlb_ia[15:0]),
  .tlbdat_i(tlb_ib),
  .tlbdat_o(tlbdato),
  .tlbmiss_o(tlbmiss),
  .tlbmiss_adr_o(tlbmiss_adr),
  .m_cyc_o(tlb_cyc),
  .m_ack_i(tlb_ack),
  .m_adr_o(tlb_adr),
  .m_dat_o(tlb_dat),
  .pmt_we(pmtram_web),
  .pmt_adr(pmtram_adrb),
  .pmt_din(pmtram_doutb),
  .pmt_dout(pmtram_dinb)
);
`endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

reg pmtram_ena;
reg pmtram_wea;
reg [13:0] pmtram_adra;
reg [159:0] pmtram_dina;
wire [159:0] pmtram_douta;
`ifdef SUPPORT_HASHPT
reg pmtram_web;
reg [13:0] pmtram_adrb;
PMTE pmtram_dinb;
PMTE pmtram_doutb;
`endif

PMT_RAM pmtram1 (
  .clka(clk),    // input wire clka
  .ena(pmtram_ena),      // input wire ena
  .wea(pmtram_wea),      // input wire [0 : 0] wea
  .addra(pmtram_adra),  // input wire [13 : 0] addra
  .dina(pmtram_dina),    // input wire [159 : 0] dina
  .douta(pmtram_douta),  // output wire [159 : 0] douta
  .clkb(tlbclk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .web(pmtram_web),      // input wire [0 : 0] web
  .addrb(pmtram_adrb),  // input wire [13 : 0] addrb
  .dinb(pmtram_dinb),    // input wire [159 : 0] dinb
  .doutb(pmtram_doutb)  // output wire [159 : 0] doutb
);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// IPT
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

reg [6:0] ptg_state = IPT_IDLE;
reg pmt_store;
reg [7:0] fault_code;
reg ptg_fault;
reg clr_ptg_fault;
wire ptg_en = ptbr[2];
PTG ptg;
PTE tmptlbe2;
PTGCE [PTGC_DEP-1:0] ptgc;
reg pte_found;
wire [2:0] entry_num;
reg [3:0] span_lo, span_hi;
wire [15:0] hash;
reg [127:0] ndat;		// next data output
reg ptgram_wr;
reg ptgram_en;
reg [14:0] ptgram_adr;
reg [127:0] ptgram_dati;
wire [127:0] ptgram_dato;
reg ptgram_web = 1'b0;
reg [11:0] ptgram_adrb = 'd0;
PTG ptgram_datib;
Address ptg_lookup_address;
reg [3:0] ptgacr = 4'd15;
wire pe_clock;
reg clock_r = 1'b0;
reg [11:0] clock_count = 'd0;

// SIM debugging
reg [5:0] ptg_lac = 'd0;
Address [63:0] ptg_last_adr;

`ifdef SUPPORT_HASHPT

always_ff @(posedge clk)
begin
	if (ptgram_wr) begin
		ptg_last_adr[ptg_lac] <= ptgram_adr;
		ptg_lac <= ptg_lac + 1'd1;
	end
end

PTG_RAM uptgram (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(ptgram_wr),      // input wire [0 : 0] wea
  .addra(ptgram_adr),  // input wire [13 : 0] addra
  .dina(ptgram_dati),    // input wire [159 : 0] dina
  .douta(ptgram_dato),  // output wire [159 : 0] douta
  .clkb(tlbclk),  // input wire clkb
  .enb(1'b1),      // input wire enb
  .web(ptgram_web & ~ptgram_wr),      // input wire [0 : 0] web
  .addrb(ptgram_adrb),  // input wire [10 : 0] addrb
  .dinb(ptgram_datib),    // input wire [1279 : 0] dinb
  .doutb(ptg)  // output wire [1279 : 0] doutb
);
`endif

`ifdef SUPPORT_HASHPT2
rfPhoenix_ipt_hash uhash
(
	.clk(clk),
	.asid(ASID),
	.adr(miss_adr),
	.mask(ptbr[127:96]),
	.hash(hash)
);

rfPhoenix_ptg_search uptgs
(
	.ptg(ptg),
	.asid(ASID),
	.miss_adr(miss_adr),
	.pte(tmptlbe2),
	.found(pte_found),
	.entry_num(entry_num)
);

`endif

// Hold onto the previous idadr if none is selected, to allow the update of
// the PTG RAM to complete without changes. A PTG write cycle will bounce
// back to the memory IDLE state almost immediately, this leaves the address
// to be maintained.
Address idadr, prev_idadr;
always_comb
	case(1'b1)
	daccess: idadr <= dadr;
	iaccess: idadr <= iadr;
	default:	idadr <= 32'hFF7FFFFF;
	endcase
always_ff @(posedge clk)
	prev_idadr <= idadr;

`ifdef SUPPORT_HASHPT
rfPhoenix_ipt_hash uhash
(
	.clk(clk),
	.asid(ASID),
	.adr(idadr),
	.mask(ptbr[127:96]),
	.hash(hash)
);

rfPhoenix_ptg_search uptgs
(
	.ptg(ptg),
	.asid(ASID),
	.miss_adr(idadr),
	.pte(tmptlbe2),
	.found(pte_found),
	.entry_num(entry_num)
);

always_comb
begin
	next_adr_o <= adr_o;
	if (ptg_en) begin
		if (pte_found)
			next_adr_o <= {tmptlbe2.ppn,idadr[15:12]+tmptlbe2.mb,idadr[11:0]};
	end
	else
		next_adr_o <= idadr;
end

always @(posedge tlbclk)
begin
	adr_o <= next_adr_o;
	if (ptg_en) begin
		if (pte_found) begin
			if (idadr[15:12] + tmptlbe2.mb <= tmptlbe2.me)
				ptgacr <= tmptlbe2.rwx;
			else
				ptgacr <= 4'd0;
		end
	end
	else
		ptgacr <= 4'd15;
end

assign tlbacr = ptgacr;
assign tlbrdy = 1'b1;
assign tlb_cyc = 1'b0;
`else
always_comb
begin
	next_adr_o <= adr_o;
	/*
	if (ptg_en) begin
		if (pte_found)
			next_adr_o <= {tmptlbe2.ppn,idadr[15:12]+tmptlbe2.mb,idadr[11:0]};
	end
	else
	*/
		next_adr_o <= idadr;
end
`endif

// 0   159  319 479  639  799   959  1119  1279
// 0 128 255 383 511 639 767 895 1023 1151 1279
always_ff @(posedge clk)
	case(entry_num)
	3'd0:	begin span_lo <= 4'd0; span_hi <= 4'd1; end
	3'd1: begin span_lo <= 4'd1; span_hi <= 4'd2; end
	3'd2: begin span_lo <= 4'd2; span_hi <= 4'd3; end
	3'd3: begin span_lo <= 4'd3; span_hi <= 4'd4; end
	3'd4: begin span_lo <= 4'd5; span_hi <= 4'd6; end
	3'd5: begin span_lo <= 4'd6; span_hi <= 4'd7; end
	3'd6: begin span_lo <= 4'd7; span_hi <= 4'd8; end
	3'd7: begin span_lo <= 4'd8; span_hi <= 4'd9; end
	endcase


integer j;
reg [11:0] square_table [0:63];
initial begin
	for (j = 0; j < 64; j = j + 1)
		square_table[j] = j * j;
end

wire cd_idadr;
reg cd_idadr_r;
edge_det uclked1 (.rst(rst), .clk(tlbclk), .ce(1'b1), .i(clock), .pe(pe_clock), .ne(), .ee());
change_det uchgdt1 (.rst(rst), .clk(tlbclk), .ce(1'b1), .i(idadr), .cd(cd_idadr));

reg special_ram;
always_comb
	special_ram = ptgram_en || pmtram_ena || rgn_en || tlb_access;

reg [15:0] hash_r;
`ifdef SUPPORT_HASHPT
integer n6;
always_ff @(posedge tlbclk)
if (rst)
	pmtram_dinb <= 'd0;
else begin
	pmtram_web <= 1'b0;
	if (clr_ptg_fault|clr_ipage_fault) begin
		ipt_miss_count <= 'd0;
		ptg_fault <= 1'b0;
	end
	if (pe_clock)
		clock_r <= 1'b1;
	if (cd_idadr)
		cd_idadr_r <= TRUE;

	case (ptg_state)
	IPT_IDLE:
		begin
			ipt_miss_count <= 'd0;
			if ((!pte_found || cd_idadr_r) && ptg_en && (iaccess||daccess) && !special_ram) begin
				cd_idadr_r <= FALSE;
				ptg_state <= IPT_RW_PTG2;
				ptgram_adrb <= hash & 16'hFFFF;
				hash_r <= hash;
			end
			else if (clock_r) begin
				clock_r <= 1'b0;
				ptg_state <= IPT_CLOCK1;
				pmtram_adrb <= clock_count;
				clock_count <= clock_count + 2'd1;
			end
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware routine to find an address translation.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// 
	IPT_FETCH1:
		begin
			// Open addressing with quadratic probing
			ptgram_adrb <= ((hash_r + square_table[ipt_miss_count]) & 16'hFFFF);
	    if (ipt_miss_count==6'd12)
	    	ptg_fault <= 1'b1;
	    else
	    	ptg_state <= IPT_RW_PTG2;
		end
	IPT_RW_PTG2:
		begin
			ipt_miss_count <= ipt_miss_count + 2'd1;
 			ptg_state <= IPT_RW_PTG3;
		end
	// Region is not valid until after next_adr_o is set
	IPT_RW_PTG3:
		begin
			pmtram_adrb <= {region.pmt[31:4],4'h0} + tmptlbe2.ppn[13:0];
			ptg_state <= IPT_RW_PTG4;
		end
	IPT_RW_PTG4:
		begin
			ptg_state <= IPT_RW_PTG5;
		end
	IPT_RW_PTG5:
		ptg_state <= IPT_RW_PTG6;
	IPT_RW_PTG6:
		begin
	    pmtram_web <= 1'b1;
  		pmtram_dinb <= pmtram_doutb;
			pmtram_dinb.access_count <= pmtram_doutb.access_count + 2'd1;
			if (pmt_store)
				pmtram_dinb.m <= 1'b1;
  		ptg_state <= pte_found ? IPT_IDLE : IPT_FETCH1;
		end	

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Age access counts
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	IPT_CLOCK1:
		ptg_state <= IPT_CLOCK2;
	IPT_CLOCK2:
		ptg_state <= IPT_CLOCK3;
	IPT_CLOCK3:
		begin
	    pmtram_web <= 1'b1;
  		pmtram_dinb <= pmtram_doutb;
 			pmtram_dinb.access_count <= {1'b0,pmtram_dinb.access_count[31:1]};
  		ptg_state <= IPT_IDLE;
		end
	
	default:
		ptg_state <= IPT_IDLE;

	endcase
end
`endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// PT
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Page table vars
reg [2:0] dep;
reg [12:0] adr_slice;
PTE pte;
PDE pde;
reg wr_pte;
PDCE [11:0] ptc;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// State Machine
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

reg dfetch2;
always_ff @(posedge clk)
if (rst) begin
	dce <= TRUE;
	zero_data <= FALSE;
	dcachable <= TRUE;
	ivvalid <= 5'h00;
	ivcnt <= 3'd0;
	vcn <= 3'd0;
	for (n = 0; n < 5; n = n + 1) begin
		ivtag[n] <= 32'd1;
		ivcache[n] <= {16{NOP_INSN}};
	end
	shr_ma <= 6'd0;
	tlben <= TRUE;
	iadr <= RSTIP;
	dadr <= RSTIP;	// prevents MR_TLB miss at startup
	tDeactivateBus();
	dat <= 256'd0;
	sr_o <= LOW;
	cr_o <= LOW;
	waycnt <= 2'd0;
	ic_wway <= 2'b00;
	dwait <= 3'd0;
	iaccess <= FALSE;
	daccess <= FALSE;
	ici <= 512'd0;
	dci <= 512'd0;
	memreq_rd <= FALSE;
	memresp <= 620'd0;
  xlaten <= FALSE;
  tmptlbe <= 'd0;
  wr_pte <= 1'b0;
  wr_ptg <= 1'b0;
  tlb_ack <= 1'b0;
  ptgram_wr <= FALSE;
  ptg_fault <= 1'b0;
	clr_ptg_fault <= 1'b0;
	ipage_fault <= 1'b0;
	itlbmiss <= 1'b0;
	ptgram_en <= 1'b0;
	rgn_en <= 1'b0;
	tlb_access <= 1'b0;
	pmtram_ena <= 1'b0;
	pmtram_wea <= 1'b0;
	pmt_store <= 1'b0;
	sel <= 'd0;
	dfetch2 <= 1'b0;
	rd_memq <= 'd0;
	goto (MEMORY_INIT);
end
else begin
	dcachable <= TRUE;
	inext <= FALSE;
//	memreq_rd <= FALSE;
	memresp.wr <= FALSE;
	tlbwr <= FALSE;
	tlb_ack <= FALSE;
	ptgram_wr <= FALSE;
	pmtram_wea <= FALSE;
	clr_ptg_fault <= 1'b0;
	if (clr_ipage_fault)
		ipage_fault <= 1'b0;
	if (clr_itlbmiss)
		itlbmiss <= 1'b0;

	case(state)
	MEMORY_INIT:
		begin
			for (n5 = 0; n5 < 8; n5 = n5 + 1)
				ptc[n5] <= 'd0;
			rd_memq <= TRUE;
			goto (MEMORY1);
		end

	MEMORY1:
		if (memq_v) begin
			rd_memreq <= FALSE;
			memr <= memq_o;
			goto (MEMORY_DISPATCH);
		end

	MEMORY_DISPATCH:
		begin
//			memreq_rd <= FALSE;
			tMemoryDispatch();
		end

	// The following two states for MR_TLB translation lookup
	// Must check for two PTG states since that machine is clocked at twice
	// the rate.
	MEMORY3:
`ifdef SUPPORT_HASHPT
		if (ptg_state==IPT_RW_PTG5 || ptg_state==IPT_RW_PTG6 || !ptg_en || special_ram)
			goto (MEMORY4);
`else
		goto (MEMORY4);
`endif
`ifdef SUPPORT_KEYCHK
	MEMORY4:
		goto (MEMORY_KEYCHK1);
`else
	MEMORY4:
		goto (MEMORY5);
`endif
`ifdef SUPPORT_KEYCHK
	MEMORY_KEYCHK1:
		tKeyCheck(MEMORY5);
	KEYCHK_ERR:
		begin
			memresp.step <= memreq.step;
	    memresp.cause <= {4'h8,FLT_KEY};	// KEY fault
	    memresp.cmt <= TRUE;
			memresp.tid <= memreq.tid;
		  memresp.badAddr <= ea;
		  memresp.wr <= TRUE;
			memresp.res <= 128'd0;
		  ret();
		end
`endif
	MEMORY5: goto (MEMORY5a);
	MEMORY5a:		// Allow time for lookup
		goto (MEMORY_ACTIVATE_LO);

	MEMORY_ACTIVATE_LO:
		tMemoryActivateLo();

	MEMORY_ACKLO:
		tMemoryAckLo();

	MEMORY_NACKLO:
		tMemoryNackLo();

	MEMORY8:
	  begin
      if (memreq.func==MR_LOAD && memreq.func2==MR_LDOO) begin
       	if (strips != 2'd3) begin
      		strips <= strips + 2'd1;
	    		goto (MEMORY3);
	    	end
	    	else
	    		goto(DATA_ALIGN);
      end
      else
	    	goto (MEMORY9);
	    xlaten <= TRUE;
	    dadr <= {dadr[AWID-1:4] + 2'd1,4'd0};
	    tEA({ea[AWID-1:4] + 2'd1,4'd0});
	  end
  
	// Wait a couple of clocks for MR_TLB lookup
	MEMORY9:
		begin
`ifdef SUPPORT_HASHPT
			if (ptg_state==IPT_RW_PTG5 || ptg_state==IPT_RW_PTG6 || !ptg_en)
`endif			
	  	goto (MEMORY10);
		end
`ifdef SUPPORT_KEYCHK
	MEMORY10:
		begin
		  goto (MEMORY_KEYCHK2);
		end
 
	MEMORY_KEYCHK2:
		tKeyCheck(MEMORY11);
`else
	MEMORY10:
	  goto (MEMORY11);
`endif

	MEMORY11:		// Allow time for lookup
		goto (MEMORY_ACTIVATE_HI);

	MEMORY_ACTIVATE_HI:
		tMemoryActivateHi();

	MEMORY_ACKHI:
		tMemoryAckHi();

	MEMORY13:
		tMemoryNackHi();

	DATA_ALIGN:
		tDataAlign();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Complete TLB access cycle
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	TLB1:
		goto (TLB2);	// Give time for MR_TLB to process
	TLB2:
		goto (TLB3);	// Give time for MR_TLB to process
	TLB3:
		begin
			memresp.cause <= 12'h0;
			memresp.step <= memreq.step;
	    memresp.res <= {432'd0,tlbdato};
	    memresp.cmt <= TRUE;
			memresp.tid <= memreq.tid;
			memresp.wr <= TRUE;
	   	ret();
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to load an instruction cache line.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Use ipo to hold onto the original ip value. The ip value might
	// change during a cache load due to a branch. We also want the start
	// of the cache line identified as the access will span into the next
	// cache line.
	IFETCH0:
		begin
			ipo <= {ip[$bits(Address)-1:6],6'b0};
			iadr <= {ip[$bits(Address)-1:6],6'b0};
			goto (IFETCH1);
			for (n = 0; n < 5; n = n + 1) begin
				if (ivtag[n]==ip[AWID-1:6] && ivvalid[n]) begin
					vcn <= n;
		    	goto (IFETCH4);
	    	end
			end
		end
	// Hardware subroutine to fetch instruction cache line
	IFETCH1:
		begin
`ifdef SUPPORT_HASHPT			
			if (ptg_fault) begin
				ipage_fault <= 1'b1;
				ici <= {40{8'b0,NOP}};
				goto (IFETCH3);
			end
			if (pte_found || !ptg_en)
`endif
`ifndef SUPPORT_HWWALK
			if (tlbmiss) begin
				itlbmiss <= 1'b1;
				ici <= {40{8'b0,NOP}};
				goto (IFETCH3);
			end
`endif
			begin
			  if (!ack_i) begin
			  	// Cache miss, select an entry in the victim cache to
			  	// update.
			  	if (ic_valid) begin
						ivcnt <= ivcnt + 2'd1;
						if (ivcnt>=3'd4)
							ivcnt <= 3'd0;
						ivcache[ivcnt] <= ic_line;
						ivtag[ivcnt] <= ic_tag;
						ivvalid[ivcnt] <= TRUE;
						if (ic_line=='d0)
							$stop;
					end
			  	vpa_o <= HIGH;
			  	bte_o <= 2'b00;
			  	cti_o <= 3'b001;	// constant address burst cycle
			    cyc_o <= HIGH;
					stb_o <= HIGH;
			    sel_o <= 16'hFFFF;
		  		goto (IFETCH2);
			  end
			end
		end
	IFETCH2:
	  begin
	  	stb_o <= HIGH;
	  	if (tlbmiss)
	  		tTlbMiss(tlbmiss_adr, ptbr[0] ? PT_FETCH1 : IPT_FETCH1, FLT_CPF);
	    else if (ack_i) begin
	      ici <= {dat_i,ici[639:128]};	// shift in the data
	      icnt <= icnt + 4'd4;					// increment word count
	      if (icnt[4:2]==3'd4) begin		// Are we done?
	      	tDeactivateBus();
	      	iaccess <= FALSE;
	      	goto (IFETCH3);
	    	end
	    	else if (!bok_i) begin				// burst mode supported?
	    		cti_o <= 3'b000;						// no, use normal cycles
	    		goto (IFETCH6);
	    	end
	    end
	    /*
		  // PMA Check
		  // Abort cycle that has already started.
		  for (n = 0; n < 8; n = n + 1)
		    if (adr_o[31:4] >= PMA_LB[n] && adr_o[31:4] <= PMA_UB[n]) begin
		      if (!PMA_AT[n][0]) begin
		        //memresp.cause <= 16'h803D;
		        tDeactivateBus();
		    	end
		    end
			*/
		end
	IFETCH3:
		begin
		  ic_wway <= waycnt;
		  xlaten <= FALSE;
		  goto (IFETCH3a);
	  	//ret();
		end
	IFETCH3a:
		begin
			ret();
		end
	
	IFETCH4:
		goto (IFETCH5);		// delay for block ram read
	IFETCH5:
		begin
			ici <= {96'd0,ivcache[vcn]};
			if (ic_valid) begin
				ivcache[vcn] <= ic_line;
				ivtag[vcn] <= ic_tag;
				ivvalid[vcn] <= 1'b1;
				if (ic_line=='d0)
					$stop;
			end
			goto (IFETCH3);
		end

	IFETCH6:
		begin
			stb_o <= LOW;
			if (!ack_i)	begin							// wait till consumer ready
				inext <= TRUE;
				goto (IFETCH2);
			end
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	DFETCH2:
	  begin
	    goto(DFETCH3);
	  end
	DFETCH3:
	  begin
	 		xlaten <= FALSE;
		  begin
	  		goto (DFETCH4);
		  	if (tlbmiss)
		  		tTlbMiss(tlbmiss_adr, ptbr[0] ? PT_FETCH1 : IPT_FETCH1, FLT_DPF);
			  // First time in, set to miss address, after that increment
			  if (!dfetch2) begin
      		dadr <= {adr_o[AWID-1:6],6'h0};
      	end
		  end
	  end

	// Initiate burst access
	DFETCH4:
	  if (!ack_i) begin
	  	vda_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
	    sel_o <= 16'hFFFF;
	    goto (DFETCH5);
	  end

	// Sustain burst access
	DFETCH5:
	  begin
	  	daccess <= FALSE;
	  	stb_o <= HIGH;
	    if (ack_i) begin
	    	dcnt <= dcnt + 4'd4;
	      dci <= {dat_i,dci[511:128]};
	      if (dcnt[4:2]==3'd3) begin		// Are we done?
	      	tDeactivateBus();
	      	goto (DFETCH7);
	    	end
	    	if (!bok_i) begin							// burst mode supported?
	    		cti_o <= 3'b000;						// no, use normal cycles
	    		goto (DFETCH6);
	    	end
	    end
	  end
  
  // Increment address and bounce back for another read.
  DFETCH6:
		begin
			stb_o <= LOW;
			if (!ack_i)	begin							// wait till consumer ready
				inext <= TRUE;
				goto (DFETCH5);
			end
		end

	// Trgger a data cache update. The data cache line is in dci. The only thing
	// left to do is update the tag and valid status.
	DFETCH7:
	  begin
			xlaten <= xlaten_stk;
    	goto (DFETCH8);
	  end
	DFETCH8:
		goto (DFETCH9);
	DFETCH9:
		begin
			dfetch2 <= 1'b1;
			goto (DFETCH2);
			if (dhit) begin
				dfetch2 <= 1'b0;
				//tPopBus();????
				tDeactivateBus();
				ret();
			end
			// If got a hit on the even address, the odd one must be missing
			else if (dhite)
				dadr <= {ea[AWID-1:7],1'b1,6'h0};
			// Otherwise the even one must be missing
			else
				dadr <= {ea[AWID-1:6]+ea[6],6'h0};
		end

`ifdef SUPPORT_HWWALK
`ifdef SUPPORT_HASHPT2
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to find an address translation and update the TLB.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// 
	IPT_FETCH1:
		begin
			// Open addressing with quadratic probing
//			dadr <= ptbr + {ptg.link,7'h0};
			dadr <= ptbr + ({(hash + square_table[ipt_miss_count]) & 16'hFFFF,6'h0});//ptbr + {ptg.link,7'h0};
	 		xlaten <= FALSE;
	 		wr_ptg <= 1'b0;
	    if (ipt_miss_count==6'd12)
	    	tPageFault(fault_code,miss_adr);
	    else
	    	gosub (IPT_RW_PTG2);
	    if (pte_found) begin
	    	tmptlbe <= tmptlbe2;
	    	goto (IPT_FETCH2);
	    end
		end
	IPT_FETCH2:
		begin
			tlbwr <= 1'b1;
			tlb_ia <= 'd0;
			tlb_ia[31:20] <= 2'b10;	// write a random way
			tlb_ia[19:15] <= 5'h0;
			tlb_ia[14:0] <= {miss_adr[25:16],5'h0};
			tlb_ib <= tmptlbe;
			tlb_ib.a <= 1'b1;
			tlb_ib.adr <= dadr;
//			wr_ptg <= 1'b1;
//			ptg[entry_num * $bits(PTE) + 132] <= 1'b1;	// The 'a' bit in the pte
//			if (tmptlbe.av)
//				call (IPT_RW_PTG2,IPT_FETCH3);
//			else
			if (tmptlbe.av)
				goto (IPT_FETCH3);
			else
				gosub(PMT_FETCH1);
		end
	// Delay a couple of cycles to allow TLB update
	IPT_FETCH3:
		begin
			tlbwr <= 1'b0;
			wr_ptg <= 1'b0;
			if (fault_code==FLT_DPF) begin
				xlaten <= xlaten_stk;
				dadr <= dadr_stk;
				goto (IPT_FETCH4);
			end
			else begin
				xlaten <= xlaten_stk;
				iadr <= iadr_stk;
			  if (!ack_i)
		  		goto (IPT_FETCH4);
			end	
		end
	IPT_FETCH4:
		goto (IPT_FETCH5);
	IPT_FETCH5:
		begin
			// Restore the bus state, it should not miss now.
			tPopBus();
			ret();
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to read / write a page table group.
	//
	// Writes only as much as it needs to. For writes just the PTE needs
	// to be updated.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
`ifdef SOMETHING
	IPT_RW_PTG2:
		begin
			ipt_miss_count <= ipt_miss_count + 2'd1;
	 		xlaten <= FALSE;
			daccess <= TRUE;
			iaccess <= FALSE;
			dcnt <= 'd0;
	  	vpa_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
`ifdef SUPPORT_SHPTE
			sel_o <= dadr[3] ? 16'hFF00 : 16'h00FF;
`else
	    sel_o <= 16'hFFFF;
`endif	    
	    we_o <= wr_ptg;
	    // We need only to write the access bit which is in the upper half of
	    // the pte.
  		case(span_lo)
`ifdef SUPPPORT_SHPTE
  		4'd0:	dat_o <= {2{ptg[63:0]}};
  		4'd1: dat_o <= {2{ptg[127:64]}};
  		4'd2:	dat_o <= {2{ptg[191:128]};
  		4'd3:	dat_o <= {2{ptg[255:192]};
  		4'd3:	dat_o <= {2{ptg[319:256]};
  		4'd3:	dat_o <= {2{ptg[383:320]};
  		4'd3:	dat_o <= {2{ptg[447:384]};
  		4'd3:	dat_o <= {2{ptg[511:448]};
`else
  		4'd0:	dat_o <= ptg[255:128];
  		4'd1: dat_o <= ptg[383:256];
  		4'd2:	dat_o <= ptg[511:384];
  		4'd3:	dat_o <= ptg[639:512];
  		4'd4:	dat_o <= ptg[767:640];
  		4'd5: dat_o <= ptg[895:768];
  		4'd6: dat_o <= ptg[1023:895];
  		4'd7: dat_o <= ptg[1151:1024];
  		4'd8:	dat_o <= ptg[1279:1152];
//  		4'd9:	dat_o <= ptg[1407:1280];
//  		4'd10:	dat_o <= ptg[1535:1408];
`endif
  		default:	;
  		endcase
  		if (dce & dhit & ~wr_ptg) begin
  			tDeactivateBus();
  		end
  		goto (IPT_RW_PTG4);
`ifdef SUPPORT_MMU_CACHE  		
			if (!wr_ptg) begin
				for (n4 = 0; n4 < PTGC_DEP; n4 = n4 + 1) begin
					if (ptgc[n4].dadr == dadr && ptgc[n4].v) begin
						tDeactivateBus();
						ptg <= ptgc[n4];
						ret();
					end
				end
			end
`endif			
		end
	IPT_RW_PTG4:
		begin
			if (dce & dhit & ~wr_ptg) begin
				ptg <= dc_line;
  			tDeactivateBus();
      	daccess <= FALSE;
`ifdef SUPPORT_MMU_CACHE		      	
      	for (n4 = 1; n4 < PTGC_DEP; n4 = n4 + 1)
      		ptgc[n4] <= ptgc[n4-1];
      	ptgc[0].dadr <= dadr;
`ifdef SUPPORT_SHPTE
    		ptgc[0].ptg <= {dat_i,ptg[383:0]};
`else		      	
    		ptgc[0].ptg <= {dat_i,ptg[1151:0]};
`endif	      		
    		ptgc[0].v <= 1'b1;
`endif	      		
      	ret();
			end
			else begin
				if (dce & dhit)
					dci <= dc_line;
				if (wr_ptg) begin
					memreq.func <= MR_STORE;
					case({dadr[4:3],sel_o})
					18'h000FF:	dci[63:0] <= ptg[63:0];
					18'h0FF00:	dci[127:64] <= ptg[127:64];
					18'h100FF:	dci[191:128] <= ptg[191:128];
					18'h1FF00:	dci[255:192] <= ptg[255:192];
					18'h200FF:	dci[319:256] <= ptg[319:256];
					18'h2FF00:	dci[383:320] <= ptg[383:320];
					18'h300FF:	dci[447:384] <= ptg[447:384];
					18'h3FF00:	dci[511:448] <= ptg[511:448];
					default:		dci <= dc_line;
					endcase
				end
	  		stb_o <= HIGH;
		    if (ack_i) begin
		    	if (wr_ptg) begin
		      	tDeactivateBus();
		      	daccess <= FALSE;
		      	goto(IPT_RW_PTG6);
		    	end
		    	else begin
			    	case(dcnt[3:0])
			    	4'd0:	ptg[127:  0] <= dat_i;
			    	4'd1: ptg[255:128] <= dat_i;
			    	4'd2:	ptg[383:256] <= dat_i;
			    	4'd3: ptg[511:384] <= dat_i;
`ifndef SUPPORT_SHPTE		    	
			    	4'd4:	ptg[639:512] <= dat_i;
			    	4'd5: ptg[767:640] <= dat_i;
			    	4'd6: ptg[895:768] <= dat_i;
			    	4'd7: ptg[1023:896] <= dat_i;
`endif		    	
	//		    	4'd8: ptg[1151:1024] <= dat_i;
	//		    	4'd9: ptg[1279:1152] <= dat_i;
	//		    	4'd10: 	ptg[1407:1280] <= dat_i;
	//		    	4'd11: 	ptg[1535:1408] <= dat_i;
			    	default:	;
			    	endcase
`ifdef SUPPORT_SHPTE
			      if (dcnt[3:0]==4'd3) begin		// Are we done?
`else		    	
				    if (dcnt[3:0]==rfPhoenix_mmupkg::PtgSize/128-1) begin		// Are we done?
`endif		      	
`ifdef SUPPORT_MMU_CACHE		      	
			      	for (n4 = 1; n4 < PTGC_DEP; n4 = n4 + 1)
			      		ptgc[n4] <= ptgc[n4-1];
			      	ptgc[0].dadr <= dadr;
`ifdef SUPPORT_SHPTE
		      		ptgc[0].ptg <= {dat_i,ptg[383:0]};
`else		      	
		      		ptgc[0].ptg <= {dat_i,ptg[1151:0]};
`endif	      		
		      		ptgc[0].v <= 1'b1;
`endif	      		
			      	tDeactivateBus();
			      	daccess <= FALSE;
			      	ret();
			    	end
			    	else if (!bok_i) begin				// burst mode supported?
			    		cti_o <= 3'b000;						// no, use normal cycles
			    		goto (IPT_RW_PTG5);
			    	end
				  end
		      dcnt <= dcnt + 2'd1;					// increment word count
		    end
	  	end
  	end
  // Increment address and bounce back for another read.
  IPT_RW_PTG5:
		begin
			stb_o <= LOW;
			if (!ack_i)	begin							// wait till consumer ready
				inext <= TRUE;
				goto (IPT_RW_PTG4);
			end
		end
	IPT_RW_PTG6:
		ret();

	IPT_WRITE_PTE:
		begin
			ptg <= 'd0;
`ifdef SUPPORT_SHPTE
			ptg <= tlb_dat[63:0] << (tlb_dat.en * $bits(SHPTE));	// will cause entry_num to be zero.
`else
			ptg <= tlb_dat[159:0] << (tlb_dat.en * $bits(PTE));	// will cause entry_num to be zero.
`endif
			case(tlb_dat.en)
`ifdef SUPPORT_SHPTE
			3'd0:	dadr <= tlb_dat.adr;
			3'd1:	dadr <= tlb_dat.adr + 12'd8;
			3'd2:	dadr <= tlb_dat.adr + 12'd16;
			3'd3:	dadr <= tlb_dat.adr + 12'd24;
			3'd4:	dadr <= tlb_dat.adr + 12'd32;
			3'd5:	dadr <= tlb_dat.adr + 12'd40;
			3'd6:	dadr <= tlb_dat.adr + 12'd48;
			3'd7:	dadr <= tlb_dat.adr + 12'd56;
`else				
			3'd0:	dadr <= tlb_dat.adr;
			3'd1:	dadr <= tlb_dat.adr + 12'd16;
			3'd2:	dadr <= tlb_dat.adr + 12'd48;
			3'd3:	dadr <= tlb_dat.adr + 12'd64;
			3'd4:	dadr <= tlb_dat.adr + 12'd96;
			3'd5:	dadr <= tlb_dat.adr + 12'd112;
			3'd6:	dadr <= tlb_dat.adr + 12'd144;
			3'd7:	dadr <= tlb_dat.adr + 12'd160;
`endif			
			endcase
			tInvalidatePtgc(tlb_dat.adr,tlb_dat.adr + 12'd160);
			miss_adr <= {tlb_dat.vpn,16'd0};
			wr_ptg <= 1'b1;
			goto (IPT_RW_PTG2);
		end

`endif
`endif	// SOMETHING

`ifdef SUPPORT_HIERPT
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to find an address translation and update the TLB.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	PT_FETCH1:
		begin
			dep <= ptbr[10:8];
			wr_pte <= 1'b0;
	  	case(ptbr[10:8])
	  	3'd1:
	  		begin
	  			pte <= ptbr[31:16];
	  			pte.lvl <= 3'd1;
	  			pte.m <= 1'b0;
	  			pte.a <= 1'b1;
	  			pte.v <= 1'b1;
	  			adr_slice <= {miss_adr[27:16],1'b0};
	  			if (miss_adr[AWID-1:28] != 'd0 && miss_adr[AWID-1:28] != {AWID-28{1'b1}})
	  				tPageFault(0,miss_adr);
	  			else
	  				call (PT_RW_PTE1, PT_FETCH3);
	  		end
	  	3'd2:
	  		begin
	  			pde <= ptbr[31:16];
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd2;
	  			adr_slice <= miss_adr[31:28];	// [40:28]
	  			call (PT_RW_PDE1, PT_FETCH2);
	  		end // 8 bits
	  	/*
	  	3'd3:	
	  		begin
	  			pde <= ptbr[31:12];
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd3;
	  			adr_slice <= miss_adr[53:41];
	  			call (PT_RW_PDE1, PT_FETCH2);
	  		end // 13 bits
	  	3'd4:
	  		begin
	  			pde <= ptbr[31:12];	
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd4;
	  			adr_slice <= miss_adr[66:54];
	  			call (PT_READ_PDE1, PT_FETCH2);
	  		end // 13 bits
	  	3'd5:
	  		begin
	  			pde <= ptbr[31:12];
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd5;
	  			adr_slice <= miss_adr[79:67];
	  			call (PT_READ_PDE1, PT_FETCH2);
	  		end // 13 bits
	  	3'd6:
	  		begin
	  			pde <= ptbr[31:12];
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd6;
	  			adr_slice <= miss_adr[92:80];
	  			call (PT_READ_PDE1, PT_FETCH2);
	  		end // 13 bits
	  	3'd7:
	  		begin
	  			pde <= ptbr[31:12];
	  			pde.v <= 1'b1;
	  			pde.d <= 1'b0;
	  			pde.a <= 1'b1;
	  			pde.lvl <= 3'd7;
	  			adr_slice <= miss_adr[105:93];
	  			call (PT_READ_PDE1, PT_FETCH2);
	  		end // 13 bits
	  	*/
	  	default:	ret();
	  	endcase
		end
	PT_FETCH2:
	  begin
	  	if (pde.lvl >= dep)
	  		tPageFault(FLT_LVL,adr_o); 
	  	else
		  	case(dep)
		  	3'd1:
		  		begin
		  			pte[15:0] <= pde[15:0];
		  			adr_slice <= {miss_adr[27:16],1'b0};
		  			call (PT_RW_PTE1, PT_FETCH3);
		  		end
/*		  	
		  	3'd2:
		  		begin
		  			adr_slice <= miss_adr[31:28];	// [40:28];
	  				gosub (PT_RW_PDE1);
	  				dep <= pde.lvl;
		  		end // 13 bits
			  3'd3:
			  	begin
			  		adr_slice <= miss_adr[53:41];
			  		gosub (PT_RW_PTE1);
			  		dep <= pde.lvl;
			  	end // 13 bits
		  	3'd4:
		  		begin
		  			adr_slice <= miss_adr[66:54];
		  			gosub (PT_READ_PDE1);
		  			dep <= pde.lvl;
		  		end // 13 bits
		  	3'd5:
		  		begin
		  			adr_slice <= miss_adr[79:67];
		  			gosub (PT_READ_PDE1);
		  			dep <= pde.lvl;
		  		end // 13 bits
		  	3'd6:
		  		begin
		  			adr_slice <= miss_adr[92:80];
		  			gosub (PT_READ_PDE1);
		  			dep <= pde.lvl;
		  		end // 13 bits
		  	3'd7:
		  		begin
		  			adr_slice <= miss_adr[105:93];
		  			gosub (PT_READ_PDE1);
		  			dep <= pde.lvl;
		  		end // 13 bits
*/		  		
		  	default:	ret();
		  	endcase
	  end
	PT_FETCH3:
		begin
			tlbwr <= 1'b1;
			tlb_ia <= 'd0;
			tlb_ib <= 'd0;
			tlb_ia[31] <= 1'b1;	// write to tlb
			tlb_ia[15:14] <= 2'b10;	// write a random way
			tlb_ia[13:10] <= 4'h0;
			tlb_ia[9:0] <= miss_adr[25:16];
			tlb_ib.ppn <= pte.ppn;
			tlb_ib.d <= pte.d;
			tlb_ib.u <= pte.u;
			tlb_ib.s <= pte.s;
			tlb_ib.a <= pte.a;
			tlb_ib.c <= pte.c;
			tlb_ib.r <= pte.r;
			tlb_ib.w <= pte.w;
			tlb_ib.x <= pte.x;
			tlb_ib.sc <= pte.sc;
			tlb_ib.sr <= pte.sr;
			tlb_ib.sw <= pte.sw;
			tlb_ib.sx <= pte.sx;
			tlb_ib.v <= pte.v;
			tlb_ib.g <= pte.g;
			tlb_ib.bc <= pte.lvl;
			tlb_ib.n <= pte.n;
			tlb_ib.av <= pte.av;
			tlb_ib.mb <= pte.mb;
			tlb_ib.me <= pte.me;
			tlb_ib.adr <= dadr;
			pte.a <= 1'b1;
//			tlb_ib <= tmptlbe;
			tlb_ib.a <= 1'b1;
			wr_pte <= 1'b1;
			if (pte.av)
				goto (PT_FETCH4);
			else
				gosub(PMT_FETCH1);
		end
	PT_FETCH4:
		begin
			tlbwr <= 1'b0;
			wr_pte <= 1'b0;
			xlaten <= xlaten_stk;
			if (fault_code==FLT_DPF) begin
				dadr <= dadr_stk;
				goto (PT_FETCH5);
			end
			else begin
				iadr <= iadr_stk;
			  if (!ack_i)
		  		goto (PT_FETCH5);
			end	
		end
	// Delay a couple of cycles to allow TLB update
	PT_FETCH5:
		begin
			goto (PT_FETCH6);
		end
	PT_FETCH6:
		begin
			// Restore the bus state, it should not miss now.
			tPopBus();
			ret();
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to read or write a PTE.
	// If the PTE is not valid then a page fault occurs.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	PT_RW_PTE1:
		begin
	 		xlaten <= FALSE;
			daccess <= TRUE;
			iaccess <= FALSE;
			dadr <= {pte[15:0],adr_slice[12:1],4'h0};
			goto (PT_RW_PTE3);
		end
`endif
	PT_RW_PTE2:
		goto (PT_RW_PTE3);
	PT_RW_PTE3:
		begin
			if (!ack_i) begin
				vda_o <= HIGH;
		  	bte_o <= 2'b00;
		  	cti_o <= 3'b001;	// constant address burst cycle
		    cyc_o <= HIGH;    
				stb_o <= HIGH;
				we_o <= wr_pte;
		    sel_o <= 16'hFFFF;
		    dat_o <= pte;
		    goto (PT_RW_PTE4);
			end
		end
	PT_RW_PTE4:
		if (ack_i) begin
			tDeactivateBus();
			if (!wr_pte)
				pte <= dat_i;
			goto (PT_RW_PTE5);
		end
	PT_RW_PTE5:
		begin
			if (pte.v)
				ret();
			else
				tPageFault(fault_code,miss_adr);
		end
	
	PT_WRITE_PTE:
		begin
	 		xlaten <= FALSE;
			daccess <= TRUE;
			iaccess <= FALSE;
			wr_pte <= TRUE;
			pte <= tlb_dat;
			dadr <= {tlb_adr[AWID-1:5],5'h0} + 5'd16;
			miss_adr <= {tlb_adr[AWID-1:5],5'h0} + 5'd16;
			goto (PT_RW_PTE2);
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Hardware subroutine to read or write a PDE.
	// If the PDE is not valid then a page fault occurs.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	PT_RW_PDE1:
		begin
			goto (PT_RW_PDE3);
	 		xlaten <= FALSE;
			daccess <= TRUE;
			iaccess <= FALSE;
			dadr <= {pde[15:0],adr_slice[12:1],4'h0};
`ifdef SUPPORT_MMU_CACHE			
			if (!wr_pte)
				for (n4 = 0; n4 < 12; n4 = n4 + 1)
					if (ptc[n4].adr=={pde[15:0],adr_slice[12:0],3'h0} && ptc[n4].v) begin
						pde <= ptc[n4].pde;
						ret();
					end
`endif					
		end
	PT_RW_PDE3:
		if (!ack_i) begin
			vda_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;    
			stb_o <= HIGH;
			we_o <= wr_pte;
	    sel_o <= 16'hFFFF;
	    dat_o <= pde;
	    goto (PT_RW_PDE4);
		end
	PT_RW_PDE4:
		if (ack_i) begin
			tDeactivateBus();
			if (!wr_pte)
				pde <= adr_slice[0] ? dat_i[127:64] : dat_i[63:0];
			pde.padr <= adr_o;
			goto (PT_RW_PDE5);
		end
	PT_RW_PDE5:
		begin
			if (pde.v) begin
`ifdef SUPPORT_MMU_CACHE				
				for (n4 = 0; n4 < 11; n4 = n4 + 1)
					ptc[n4+1] <= ptc[n4];
				ptc[0].v <= 1'b1;
				ptc[0].adr <= dadr|{adr_slice[0],3'b0};
				ptc[0].pde <= pde;
`endif				
				ret();
			end
			else
				tPageFault(fault_code,miss_adr);
		end
`endif	// SUPPORT_HWWALK

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Subroutine to fetch access rights.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	PMT_FETCH1:
		begin
	 		xlaten <= FALSE;
			daccess <= TRUE;
			iaccess <= FALSE;
			dadr <= {region.pmt[AWID-1:4],4'h0} + {pmt_adr[AWID-1:12] << region.pmt[1:0],4'h0};
			goto (PMT_FETCH2);
		end
	PMT_FETCH2:
		goto (PMT_FETCH3);
	PMT_FETCH3:
		if (!ack_i) begin
			vda_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
			we_o <= wr_pte;
	    sel_o <= 16'hFFFF;
	    goto (PMT_FETCH4);
		end
	PMT_FETCH4:
		if (ack_i) begin
			tDeactivateBus();
			tmptlbe.pl <= arti.pl;
			tmptlbe.key <= arti.key;
			tmptlbe.access_count <= arti.access_count;
			goto (PMT_FETCH5);
		end
	PMT_FETCH5:
		begin
			ret();
		end

	default:
		goto (MEMORY_IDLE);
	endcase
end

task tInvalidatePtgc;
input Address adrlo;
input Address adrhi;
integer n5;
begin
`ifdef SUPPORT_MMU_CACHE
	for (n5 = 0; n5 < PTGC_DEP; n5 = n5 + 1)
		if (ptgc[n5].dadr >= adrlo && ptgc[n5].dadr <= adrhi)
			ptgc[n5].v <= 1'b0;
`endif			
end
endtask

always_ff @(posedge clk)
if (rst) begin
	memreq_rd <= FALSE;
	mem_resp[0] <= 'd0;
	mem_resp[1] <= 'd0;
	mem_resp[2] <= 'd0;
	mem_resp[3] <= 'd0;
	mem_resp[4] <= 'd0;
	mem_resp[5] <= 'd0;
	mem_resp[6] <= 'd0;
end
else begin
	tStage0();
	tStage1();
	tStage2();
	tStage3();
	tStage4();
	tStage5();
	tStage6();
	tDataAlignStage();
end

task tStage0;
begin
	memreq_rd <= TRUE;
	mem_resp[0] <= 'd0;
	if (tlbrdy) begin
		if (tlb_cyc) begin
			mem_resp[0].func = MR_TLB;
			mem_resp[0].badAddr <= {tlb_adr[AWID-1:5],5'h0} + 5'd16;
		end
		else if (!ihit && fifoToCtrl_empty) begin
			waycnt <= waycnt + 2'd1;
			mem_resp.func <= MR_ICACHE_LOAD;
			mem_resp[0].badAddr <= ip4;
		end
		else if (fifoToCtrl_v) begin
			mem_resp[0].ip <= memreq.ip;
			mem_resp[0].tid <= memreq.tid;
			mem_resp[0].rid <= memreq.rid;
			mem_resp[0].func <= memreq.func;
			mem_resp[0].func2 <= memreq.func2;
			mem_resp[0].count <= memreq.count;
			mem_resp[0].step <= memreq.step;
			mem_resp[0].badAddr <= memreq.adr;
			mem_resp[0].res <= memreq.dat;
			case(memreq.sz)
			byt:	mem_resp[0].sel <= 32'h00000001;
			wyde:	mem_resp[0].sel <= 32'h00000003;
			tetra:mem_resp[0].sel <= 32'h0000000F;
			default:	mem_resp[0].sel <= 32'h0000000F;
			endcase
		end
	end
end
endtask

task tStage1;
begin
	tlb_access <= 1'b0;
	rgn_en <= 1'b0;
	ptgram_en <= 1'b0;
	pmtram_ena <= 1'b0;
	pmt_store <= 1'b0;
	mem_resp[1] <= mem_resp[0];
	mem_resp[1].cause <= 'd0;
	daccess <= FALSE;
	xlaten <= FALSE;
	if (memreq.func==MR_LOAD || memreq.func==MR_LOADZ || memreq.func==MR_STORE) begin
		vaccess <= TRUE;
		xlaten <= TRUE;
		vadr <= mem_resp[0].badAddr;
		ptgram_adr <= mem_resp[0].badAddr[18:4];
		pmtram_adra <= mem_resp[0].badAddr[18:5];
		rgn_adr <= mem_resp[0].badAddr[9:4];
    mem_resp[1].sel <= {32'h0,mem_resp[0].sel} << mem_resp[0].badAddr[3:0];
		casez(memreq.adr)
		32'hFF9F????:
			begin
				rgn_en <= 1'b1;
				rgn_wr <= mem_resp[0].func==MR_STORE;
			end
		32'hFFA?????:
			begin
				ptgram_en <= 1'b1;
				ptgram_wr <= mem_resp[0].func==MR_STORE;
			end
		32'hFFD?????:
			begin
				pmtram_ena <= 1'b1;
				pmtram_wea <= mem_resp[0].func==MR_STORE;
			end
		32'hFFE?????:
			begin
				tlbwr <= mem_resp[0].func==MR_STORE;
				tlb_access <= 1'b1;
				tlb_bucket[0] <= tlbdato[ 63:  0];
				tlb_bucket[1] <= tlbdato[127: 64];
				tlb_bucket[2] <= tlbdato[191:128];
				tlb_bucket[3] <= tlbdato[255:192];
				tlb_bucket[4] <= tlbdato[287:256];
				tlb_bucket[5] <= tlbdato[319:288];
			end
		default:	;
		endcase
	end
end
endtask

task tStage2;	// data tag lookup
begin
	mem_resp[2] <= mem_resp[1];
end
endtask

task tStage3;	// data cache fetch
begin
	mem_resp[3] <= mem_resp[2];
end
endtask

task tStage4;	// data cache fetch
begin
	mem_resp[4] <= mem_resp[3];
end
endtask

task tStage5;
begin
	mem_resp[5] <= mem_resp[4];
	if (tlbmiss) begin
		mem_resp[5].cause <= {4'h8,FLT_TLB};
 		//tTlbMiss(mem_resp[4].badAddr, ptbr[0] ? PT_FETCH1 : IPT_FETCH1, FLT_DPF);
 	end
 	if (!tlbacr[2] && (mem_resp[4].func==MR_LOAD || mem_resp[4].func==MR_LOADZ)) begin
 		mem_resp[5].cause <= {4'h8,FLT_DRF};
 		//tReadViolation(mem_resp[4].badAddr);
 	end
end
endtask

task tStage6;
begin
	mem_resp[6] <= mem_resp[5];
	case(mem_resp[5].func)
	MR_LOAD,MR_LOADZ:
		case(1'b1)
		tlb_access:	begin mem_resp[6].res <= tlbdato; mem_resp[6].v <= 1'b1; end
		ptgram_en:	begin mem_resp[6].res <= ptgram_dato; mem_resp[6].v <= 1'b1; end
		rgn_en:			begin mem_resp[6].res <= rgn_dat_o; mem_resp[6].v <= 1'b1; end
		pmtram_ena:	begin mem_resp[6].res <= pmtram_douta; mem_resp[6].v <= 1'b1; end
		default:		
			begin
				mem_resp[6].res <= dc_line;
				mem_resp[6].v <= dce & dhit & tlbacr[3];
				if (!(dce & dhit & tlbacr[3]))
					mem_resp[6].cause <= {4'h8,FLT_DCM};
			end
	  endcase
	MR_STORE:	;
	default:	;
	endcase
end
endtask

// Align the data and send it back
task tDataAlignStage;
begin
	memresp.wr <= TRUE;			// always send something back
	memresp <= mem_resp[6];
	mem_resp[7] <= mem_resp[6];
	if (!mem_resp[6].v) begin
		mem_resp[7].wr <= TRUE;
		tMemque(mem_resp[6]);
	end
  case(mem_resp[6].func)
  MR_LOAD,MR_MOVLD:
		if (mem_resp[6].func2!=MR_LDV)
    	case(memreq.sz)
    	nul:	memresp.res[mem_resp[6].count] <= 'h0;
    	byt:	memresp.res[mem_resp[6].count] <= {{56{datis[7]}},datis[7:0]};
    	wyde:	memresp.res[mem_resp[6].count] <= {{48{datis[15]}},datis[15:0]};
    	tetra:	memresp.res[mem_resp[6].count] <= {{32{datis[31]}},datis[31:0]};
//    	octa:	begin memresp.res[mem_resp[5].count] <= {{64{datis[63]}},datis[63:0]}; end
//    	hexi:	begin memresp.res <= datis[127:0]; end
//    	hexipair:	memresp.res <= dati;
//    	hexiquad:	begin memresp.res <= dati512; end
    	default:	memresp.res[mem_resp[6].count] <= mem_resp[5].res;
    	endcase
  MR_LOADZ:
		if (mem_resp[6].func2!=MR_LDV)
    	case(mem_resp[6].sz)
    	nul:	memresp.res[mem_resp[6].count] <= 'h0;
    	byt:	begin memresp.res[mem_resp[6].count] <= {56'd0,datis[7:0]}; end
    	wyde:	begin memresp.res[mem_resp[6].count] <= {48'd0,datis[15:0]}; end
    	tetra:	begin memresp.res[mem_resp[6].count] <= {32'd0,datis[31:0]}; end
//    	octa:	begin memresp.res[mem_resp[5].count] <= {64'd0,datis[63:0]}; end
//    	hexi:	begin memresp.res <= datis[127:0]; end
//    	hexipair:	memresp.res <= dati;
//    	hexiquad:	begin memresp.res <= dati512; end
    	default:	memresp.res[mem_resp[6].count] <= mem_resp[6].res;
    	endcase
  default:  ;
  endcase
end
endtask

task tMemoryDispatch;
begin
	dfetch2 <= 1'b0;
	strips <= 2'd0;
	// Detect cache controller commands
	case(memr.func)
	MR_LOAD,MR_LOADZ,MR_MOVLD:
		case(memr.func2)
		default:
    	begin
	    	daccess <= TRUE;
    		xlaten <= TRUE;
		  	dadr <= memr.badAddr;
	  		goto (MEMORY3);
			end
		endcase
	/*
	MR_CACHE:
		begin
			ic_invline <= memr.res[1:0]==3'd1;
			ic_invall	<= memr.res[1:0]==3'd2;
			dc_invline <= memr.res[4:2]==3'd3;
			dc_invall	<= memr.res[4:2]==3'd4;
			memr.cause <= 12'h0;
			if (memr.res[4:2]==3'd1)
				dce <= TRUE;
			if (memr.res[4:2]==3'd2)
				dce <= FALSE;
	    memresp.cmt <= TRUE;
			memresp.tid <= memreq.tid;
			memresp.wr <= TRUE;
			memresp.res <= 128'd0;
			ret();
		end
	*/
	MR_STORE:
		begin
	    begin
	    	daccess <= TRUE;
		  	dadr <= memr.badAddr;
    		xlaten <= TRUE;
    		// Setup proper select lines
	      sel <= zero_data ? 32'h0001 << ea[3:0] : {32'h0,memreq_sel} << ea[3:0];
	      // Shift output data into position
  		  dat <= zero_data ? 256'd0 : {128'd0,memreq.dat} << {ea[3:0],3'b0};
	  		goto (MEMORY3);
  		end
		end
	MR_MOVST:
		begin
	    begin
	    	daccess <= TRUE;
  		  tEA(ea);
    		xlaten <= TRUE;
    		// Setup proper select lines
	      sel <= {16'h0,memreq_sel} << ea[3:0];
	      // Shift output data into position
  		  dat <= {128'd0,memresp.res} << {ea[3:0],3'b0};
	  		goto (MEMORY3);
  		end
		end
	default:	ret();	// unknown operation
	endcase
	casez(memreq.adr)
	32'hFF9F????:
		begin
			rgn_en <= 1'b1;
			rgn_wr <= memreq.func==MR_STORE;
		end
	32'hFFA?????:
		begin
			ptgram_en <= 1'b1;
			ptgram_wr <= memreq.func==MR_STORE;
		end
	32'hFFD?????:
		begin
			pmtram_ena <= 1'b1;
			pmtram_wea <= memreq.func==MR_STORE;
		end
	32'hFFE?????:
		begin
			tlbwr <= memreq.func==MR_STORE && memreq.adr[6:3]==4'd15;
			tlb_access <= 1'b1;
			if (memreq.func==MR_STORE) begin
				tlb_bucket[memreq.adr[6:3]] <= memreq.dat[63:0];
			end
			else begin
				tlb_bucket[0] <= tlbdato[ 63:  0];
				tlb_bucket[1] <= tlbdato[127: 64];
				tlb_bucket[2] <= tlbdato[191:128];
				tlb_bucket[3] <= tlbdato[255:192];
				tlb_bucket[4] <= tlbdato[287:256];
				tlb_bucket[5] <= tlbdato[319:288];
				//tlb_bucket[5] <= tlbdato[383:320];
				//tlb_bucket[6] <= tlbdato[447:384];
			end
		end
	default:	;
	endcase
	pmt_store <= memreq.func==MR_STORE;
	rgn_adr <= memreq.adr[9:4];
	rgn_dat <= memreq.dat;
	pmtram_adra <= memreq.adr[18:5];
	pmtram_dina <= memreq.dat;
`ifdef SUPPORT_HASHPT
	ptgram_adr <= memreq.adr[18:4];
	ptgram_dati <= memreq.dat;
`endif
	//tlb_ia <= memreq.adr[15:0];
	//tlb_ib <= memreq.dat;
end
endtask

task tMemoryActivateLo;
begin
  dwait <= 3'd0;
`ifndef SUPPORT_HASHPT
  goto (MEMORY_ACKLO);
	if (tlbmiss)
 		tTlbMiss(tlbmiss_adr, ptbr[0] ? PT_FETCH1 : IPT_FETCH1, FLT_DPF);
`endif
`ifdef SUPPORT_HASHPT
	if (ptg_fault) begin
		clr_ptg_fault <= 1'b1;
		tPageFault(FLT_DPF,dadr);
	end
  if (memreq.func != MR_CACHE && (pte_found || !ptg_en || special_ram)) begin
  	goto (MEMORY_ACKLO);
`else 		
  else if (memreq.func != MR_CACHE) begin
`endif
		if (!special_ram) begin
	  	vda_o <= HIGH;
	    cyc_o <= HIGH;
	    stb_o <= HIGH;
	    for (n = 0; n < 16; n = n + 1)
	    	sel_o[n] <= sel[n];
	    if (memreq.func==MR_LOAD && memreq.func2==MR_LDOO)
	    	sel_o <= 16'hFFFF;
	//	      sel_o <= sel[15:0];
	    dat_o <= dat[127:0];
	    we_o <= memreq.func==MR_STORE || memreq.func==MR_MOVST;
  	end
		//tPMAEA((memreq.func==MR_STORE || memreq.func==MR_MOVST),tlbacr[1]);
    case(memreq.func)
    MR_LOAD,MR_LOADZ,MR_MOVLD,M_JALI://,RTS2:
    	begin
   			sr_o <= memreq.func2==MR_LDOR;
   			if (tlbacr[2]) begin
	    		if (dhit & tlbacr[3]) begin
	    			tDeactivateBus();
    				sr_o <= LOW;
      		end
    		end
    		else
    			tReadViolation(adr_o);
    	end
    MR_STORE,MR_MOVST,M_CALL:
    	begin
`ifdef SUPPORT_HWWALK    		
    		// Invalidate PTCEs when a store occurs to the PDE
				for (n4 = 0; n4 < 12; n4 = n4 + 1)
					if (ptc[n4].pde.padr[AWID-1:4]==adr_o[AWID-1:4])
						ptc[n4].v <= 1'b0;
`endif						
  			case(1'b1)
  			rgn_en:
  				begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
			    	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
  				end
  			ptgram_en:
  				begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
			    	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
  				end
  			pmtram_ena:
  				begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
			    	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
					end
				tlb_access:
					begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
			    	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
					end
				default:
	  			cr_o <= memreq.func2==MR_STOC;
  			endcase
    	end
    default:
    	tDeactivateBus();
    endcase
  end
end
endtask

task tMemoryAckLo;
begin
	case(1'b1)
  ic_invline:	ret();
  ic_invall:	ret();
  dc_invline:	ret();
  dc_invall:	ret();
  dce & dhit & tlbacr[3]:
    begin
    	datil <= dc_line;
	  	case(1'b1)
	  	tlb_access:	begin dati <= tlbdato; goto (DATA_ALIGN); end
	  	ptgram_en:	begin dati <= ptgram_dato; goto (DATA_ALIGN); end
	  	rgn_en:			begin dati <= rgn_dat_o; goto (DATA_ALIGN); end
	  	pmtram_ena:	begin dati <= pmtram_douta; goto (DATA_ALIGN); end
	  	default:
	  		if (memreq.func==MR_STORE || memreq.func==MR_MOVST || memreq.func==M_CALL) begin
	  			if (ack_i || !stb_o) begin
	  				if (ealow[6])
			  			dci <= (dc_oline & ~stmask) | ((dat << {adr_o[5:4],7'b0}) & stmask);
	  				else
			  			dci <= (dc_eline & ~stmask) | ((dat << {adr_o[5:4],7'b0}) & stmask);
	  			  goto (MEMORY_NACKLO);
			      stb_o <= LOW;
			      if (sel[31:16]==1'h0)
			      	tDeactivateBus();
			    end
	  		end
	    	else begin
	    		dwait <= dwait + 2'd1;
	    		if (dwait==3'd2)
		      	goto (MEMORY_NACKLO);
		    end
		  endcase
		end
  default:
  	case(1'b1)
  	tlb_access:	begin dati <= tlbdato; goto (DATA_ALIGN); end
  	ptgram_en:	begin dati <= ptgram_dato; goto (DATA_ALIGN); end
  	rgn_en:			begin dati <= rgn_dat_o; goto (DATA_ALIGN); end
  	pmtram_ena:	begin dati <= pmtram_douta; goto (DATA_ALIGN); end
  	default:
	    if (ack_i || !stb_o) begin
	      goto (MEMORY_NACKLO);
	      stb_o <= LOW;
	      dati <= {128'd0,dat_i};
	      dati512 <= {dat_i,dati512[511:128]};
	      if (sel[31:16]==1'h0) begin
	      	tDeactivateBus();
	      end
	    end
	   endcase
	endcase
end
endtask

task tMemoryNackLo;
begin
  if (~ack_i) begin
    case(memreq.func)
    MR_LOAD,MR_LOADZ,MR_MOVLD,M_JALI://,RTS2:
    	begin
		    if (|sel[31:16] && !(dce && dhit && tlbacr[3]) && !ptgram_en)
	  	    goto (MEMORY8);
	  	  else begin
    			tDeactivateBus();
	        goto (DATA_ALIGN);
	      end
    		if (dce & dhit & tlbacr[3]) begin
    			dati <= datil >> {adr_o[6:4],7'b0};
    			dati512 <= datil[511:0];
    			tDeactivateBus();
	        goto (DATA_ALIGN);
	      end
    	end
    MR_STORE,MR_MOVST,M_CALL:
    	begin
		    if (|sel[31:16] & ~ptgram_en)
			    goto (MEMORY8);
			  else begin
	    		if (memreq.func2==MR_STPTR) begin	// STPTR
			    	if (~|ea[AWID-5:0] || shr_ma[5:3] >= region.at[18:16]) begin
	  					memresp.cause <= 12'h0;
			  			memresp.step <= memreq.step;
			    	 	memresp.cmt <= TRUE;
  						memresp.tid <= memreq.tid;
  						memresp.wr <= TRUE;
							memresp.res <= {127'd0,rb_i};
							if (!memresp_full)
								ret();
			    	end
			    	else begin
			    		if (shr_ma=='d0) begin
			    			cta <= region.cta;
			    			// Turn request address into an index into region
			    			memreq.adr <= memreq.adr - region.start;
			    		end
			    		shr_ma <= shr_ma + 4'd8;
			    		zero_data <= TRUE;
			    		goto (MEMORY_DISPATCH);
			    	end
	    		end
	    		else begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
			    	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
		      end
	    	end
    	end
    default:
	    if (|sel[31:16])
	      goto (MEMORY8);
	    else
      	goto (DATA_ALIGN);
    endcase
  end
end
endtask

task tMemoryActivateHi;
begin
  xlaten <= FALSE;
`ifndef SUPPORT_HASHPT
  dwait <= 3'd0;
//    dadr <= adr_o;
  goto (MEMORY_ACKHI);
	if (tlbmiss)
 		tTlbMiss(tlbmiss_adr, ptbr[0] ? PT_FETCH1 : IPT_FETCH1, FLT_DPF);
	else begin
`else 		
	if (ptg_fault) begin
		clr_ptg_fault <= 1'b1;
		tPageFault(FLT_DPF,dadr);
	end
	if (pte_found || !ptg_en) begin
	  dwait <= 3'd0;
  	goto (MEMORY_ACKHI);
`endif
		if (dhit && (memreq.func==MR_LOAD || memreq.func==MR_LOADZ || memreq.func==MR_MOVLD || memreq.func==M_JALI/*|| memreq.func==RTS2*/) && dce && tlbacr[3])
 			tDeactivateBus();
		else begin
    	vda_o <= HIGH;
			cyc_o <= HIGH;
    	stb_o <= HIGH;
      for (n = 0; n < 16; n = n + 1)
      	sel_o[n] <= sel[n+16];
//	      	sel_o <= sel[31:16];
    	dat_o <= dat[255:128];
   		// Invalidate PTCEs when a store occurs to the PDE
`ifdef SUPPORT_HWWALK
    	if (memreq.func==MR_STORE) begin
				tInvalidatePtgc(adr_o,adr_o + 12'd224);
				for (n4 = 0; n4 < 12; n4 = n4 + 1)
					if (ptc[n4].pde.padr[AWID-1:4]==adr_o[AWID-1:4])
						ptc[n4].v <= 1'b0;
			end
`endif
			//tPMAEA((memreq.func==MR_STORE || memreq.func==MR_MOVST),tlbacr[1]);
  	end
  end
end
endtask

task tMemoryAckHi;
begin
  if (dhit & dce & tlbacr[3]) begin
    tDeactivateBus();
  	datil <= dc_line;
		if (memreq.func==MR_STORE || memreq.func==MR_MOVST || memreq.func==M_CALL) begin
			if (ack_i) begin
				if (ealow[6])
  				dci <= (dc_eline & ~stmask) | ((dat << {adr_o[5:4],7'b0}) & stmask);
				else
  				dci <= (dc_oline & ~stmask) | ((dat << {adr_o[5:4],7'b0}) & stmask);
	      goto (MEMORY13);
	      stb_o <= LOW;
	    end
		end
  	else begin
    	dwait <= dwait + 2'd1;
    	if (dwait==3'd2)
      	goto (MEMORY13);
    end
	end
  else if (ack_i || !stb_o) begin
    goto (MEMORY13);
    dati[255:128] <= dat_i;
    tDeactivateBus();
  end
end
endtask

task tMemoryNackHi;
begin
  if (~ack_i) begin
    begin
      case(memreq.func)
      MR_LOAD,MR_LOADZ,MR_MOVLD,M_JALI://,RTS2:
      	begin
      		if (dhit & dce & tlbacr[3])
      			dati <= datil >> {adr_o[5:3],6'b0};
	        goto (DATA_ALIGN);
      	end
	    MR_STORE,MR_MOVST,M_CALL:
	    	begin
	    		if (memreq.func2==MR_STPTR) begin	// STPTR
			    	if (~|ea[AWID-5:0] || shr_ma[5:3] >= region.at[18:16]) begin
	  					memresp.cause <= 12'h0;
			  			memresp.step <= memreq.step;
			    	 	memresp.cmt <= TRUE;
			  			memresp.tid <= memreq.tid;
			  			memresp.wr <= TRUE;
							memresp.res <= {127'd0,rb_i};
							if (!memresp_full)
								ret();
			    	end
			    	else begin
			    		if (shr_ma=='d0) begin
			    			cta <= region.cta;
			    			memreq.adr <= memreq.adr - region.start;
			    		end
			    		shr_ma <= shr_ma + 4'd9;
			    		zero_data <= TRUE;
			    		goto (MEMORY_DISPATCH);
			    	end
	    		end
	    		else begin
  					memresp.cause <= 12'h0;
		  			memresp.step <= memreq.step;
		    	 	memresp.cmt <= TRUE;
		  			memresp.tid <= memreq.tid;
		  			memresp.wr <= TRUE;
						memresp.res <= {127'd0,rb_i};
						if (!memresp_full)
							ret();
		      end
	    	end
      default:
        goto (DATA_ALIGN);
      endcase
    end
  end
end
endtask

task tDataAlign;
begin
	tDeactivateBus();
	if ((memreq.func==MR_LOAD || memreq.func==MR_LOADZ || memreq.func==M_JALI || memreq.func==MR_MOVLD/*|| memreq.func==RTS2*/) & ~dhit & dcachable & tlbacr[3] & dce &
	 	~ptgram_en & ~rgn_en & ~tlb_access & ~pmtram_ena)
		goto (DFETCH2);
	else if (memreq.func==MR_MOVLD) begin
		memreq.func <= MR_MOVST;
		goto (MEMORY_DISPATCH);
	end
	else if (!memresp_full)
  	ret();
	memresp.cause <= 12'h0;
	if (memreq.func2==MR_LDG) begin
		if (memreq.step == NLANES-1) begin
			memresp.wr <= TRUE;
		end
		memresp.res[memreq.count] <= datis[63:0];
		memresp.step <= memreq.step;
		memresp.tid <= memreq.tid;
	end
	else begin
		memresp.step <= memreq.step;
	  memresp.cmt <= TRUE;
		memresp.tid <= memreq.tid;
		memresp.wr <= TRUE;
	end
	sr_o <= LOW;
  case(memreq.func)
  MR_LOAD,MR_MOVLD:
  	begin
  		if (memreq.func2==MR_LDV)
  			memresp.res <= dati512;
  		else
	    	case(memreq.sz)
	    	nul:	memresp.res[memreq.count] <= 'h0;
	    	byt:	begin memresp.res[memreq.count] <= {{56{datis[7]}},datis[7:0]}; end
	    	wyde:	begin memresp.res[memreq.count] <= {{48{datis[15]}},datis[15:0]}; end
	    	tetra:	begin memresp.res[memreq.count] <= {{32{datis[31]}},datis[31:0]}; end
	    	octa:	begin memresp.res[memreq.count] <= {{64{datis[63]}},datis[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[memreq.count] <= memreq.dat;
	    	endcase
  	end
  MR_LOADZ:
  	begin
  		if (memreq.func2==MR_LDV)
  			memresp.res <= dati512;
  		else
	    	case(memreq.sz)
	    	nul:	memresp.res[memreq.count] <= 'h0;
	    	byt:	begin memresp.res[memreq.count] <= {56'd0,datis[7:0]}; end
	    	wyde:	begin memresp.res[memreq.count] <= {48'd0,datis[15:0]}; end
	    	tetra:	begin memresp.res[memreq.count] <= {32'd0,datis[31:0]}; end
	    	octa:	begin memresp.res[memreq.count] <= {64'd0,datis[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[memreq.count] <= memreq.dat;
	    	endcase
  	end
//    	RTS2:	begin memresp.res <= datis[63:0]; memresp.ret <= TRUE; end
  default:  ;
  endcase
end
endtask


// TLB miss processing
//
// TLB misses may be handled by either software or hardware.
// Software handling terminates the current bus cycle then sends an exception
// response back to the mainline.
// Hardware handling pushes the current bus cycle on a stack then terminates
// the current bus cycle. Next a hardware subroutine is called to walk the 
// page tables and update the TLB with a translation.

task tTlbMiss;
input sMemoryRequest req;
input sMemoryResponse oresp;
output sMemoryResponse resp;
begin
	//tDeactivateBus();
	resp <= oresp;
	resp.step <= req.step;
	resp.cmt <= TRUE;
  resp.cause <= {4'h8,FLT_TLBMISS};
	resp.tid <= req.tid;
  resp.badAddr <= req.adr;
  resp.wr <= TRUE;
	resp.res <= 'd0;
end
endtask

// Page faults occur only during hardware page table walks when a translation
// cannot be found.

task tPageFault;
input [7:0] typ;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= {4'h8,typ};
	memresp.tid <= memreq.tid;
  memresp.badAddr <= ba;
  memresp.wr <= TRUE;
	memresp.res <= 128'd0;
	tDeactivateBus();
	if (!memresp_full)
		goto (MEMORY_IDLE);
end
endtask

task tWriteViolation;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= {4'h8,FLT_WRV};
	memresp.tid <= memreq.tid;
  memresp.badAddr <= ba;
  memresp.wr <= TRUE;
	memresp.res <= 128'd0;
	tDeactivateBus();
	if (!memresp_full)
		goto (MEMORY_IDLE);
end
endtask

task tReadViolation;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= {4'h8,FLT_RDV};
	memresp.tid <= memreq.tid;
  memresp.badAddr <= ba;
  memresp.wr <= TRUE;
	memresp.res <= 128'd0;
	tDeactivateBus();
	if (!memresp_full)
		goto (MEMORY_IDLE);
end
endtask

task tKeyViolation;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= {4'h8,FLT_KEY};
	memresp.tid <= memreq.tid;
  memresp.badAddr <= ba;
  memresp.wr <= TRUE;
	memresp.res <= 128'd0;
	tDeactivateBus();
	if (!memresp_full)
		goto (MEMORY_IDLE);
end
endtask

`ifdef SUPPORT_KEYCHK
task tKeyCheck;
input [6:0] nst;
begin
	if (!kyhit)
		gosub(KYLD);
	else begin
		goto (KEYCHK_ERR);
		for (n = 0; n < 8; n = n + 1)
			if (kyut == keys[n] || kyut==20'd0)
				goto(nst);
	end
	if (memreq.func==MR_CACHE)
  	tPMAEA();
  if (adr_o[31:16]==IO_KEY_ADR) begin
		memresp.cause <= 12'h0;
  	memresp.step <= memreq.step;
  	memresp.cmt <= TRUE;
  	memresp.res <= io_keys[adr_o[12:2]];
  	memresp.wr <= TRUE;
  	if (memreq.func==MR_STORE) begin
  		io_keys[adr_o[12:2]] <= memreq.dat[19:0];
  	end
		if (!memresp_full)
	  	ret();
	end
end
endtask
`endif

task tEA;
input Address iea;
begin
/*
  if ((memreq.func==MR_STORE || memreq.func==MR_MOVST || memreq.func==M_CALL) && !ea_acr.w)
  	tWriteViolation(iea);
  else if ((memreq.func==MR_LOAD || memreq.func==MR_LOADZ || memreq.func==MR_MOVLD || memreq.func==M_JALI || memreq.func==RTS2) && !ea_acr.r)
  	tReadViolation(iea);
//	if (iea[AWID-1:24]=={AWID-24{1'b1}})
//		dadr <= iea;
//	else
*/
		dadr <= iea;
		ptgram_adr <= iea[18:4];
		pmtram_adra <= iea[18:5];
		rgn_adr <= iea[9:4];
//	dcachable <= ea_acr.c;
end
endtask


task tPMAEA;
input wr;
input tlbwr;
begin
	we_o <= 1'b0;
  if (keyViolation && omode == 2'd0)
  	tKeyViolation(adr_o);
  // PMA Check
 	we_o <= wr & tlbwr & region.at[1];
  if (wr && !region.at[1])
  	tWriteViolation(dadr);
  else if (~wr && !region.at[2])
    tReadViolation(dadr);
//	memresp.cause <= {4'h8,FLT_PMA};
	dcachable <= dcachable & region.at[3];
end
endtask

task tPMAIP;
begin
  // PMA Check
  // Abort cycle that has already started.
  if (!region.at[0]) begin
    memresp.cause <= {4'h8,FLT_PMA};
    tDeactivateBus();
	end
end
endtask

task tDeactivateBus;
begin
	vpa_o <= LOW;			//
	vda_o <= LOW;
	cti_o <= 3'b000;	// Normal cycles again
	cyc_o <= LOW;
	stb_o <= LOW;
	we_o <= LOW;
	sel_o <= 16'h0000;
  xlaten <= FALSE;
end
endtask

task tPushBus;
begin
	xlaten_stk <= xlaten;
	vpa_stk <= vpa_o;
	vda_stk <= vda_o;
	bte_stk <= bte_o;
	cti_stk <= cti_o;
	cyc_stk <= cyc_o;
	stb_stk <= stb_o;
	we_stk <= we_o;
	sel_stk <= sel_o;
	dadr_stk <= dadr;
	iadr_stk <= iadr;
	dato_stk <= dat_o;
end
endtask

task tPopBus;
begin
	xlaten <= xlaten_stk;
	vpa_o <= vpa_stk;
	vda_o <= vda_stk;
	bte_o <= bte_stk;
	cti_o <= cti_stk;
	cyc_o <= cyc_stk;
	stb_o <= stb_stk;
	we_o <= we_stk;
	sel_o <= sel_stk;
//	dadr <= dadr_stk;
//	iadr <= iadr_stk;
	dat_o <= dato_stk;
end
endtask

task goto;
input [6:0] nst;
begin
	state <= nst;
end
endtask

task call;
input [6:0] nst;
input [6:0] rst;
begin
	goto(nst);
	stk_state1 <= rst;
	stk_state2 <= stk_state1;
	stk_state3 <= stk_state2;
	stk_state4 <= stk_state3;
	stk_state5 <= stk_state4;
end
endtask

task gosub;
input [6:0] nst;
begin
	stk_state1 <= state;
	stk_state2 <= stk_state1;
	stk_state3 <= stk_state2;
	stk_state4 <= stk_state3;
	stk_state5 <= stk_state4;
	state <= nst;
end
endtask

task ret;
begin
	state <= stk_state1;
	stk_state1 <= stk_state2;
	stk_state2 <= stk_state3;
	stk_state3 <= stk_state4;
	stk_state4 <= stk_state5;
end
endtask

endmodule

module biu_dati_align(dati, datis, amt);
input [255:0] dati;
output reg [127:0] datis;
input [7:0] amt;

reg [255:0] shift1;
reg [255:0] shift2;
reg [255:0] shift3;
reg [255:0] shift4;
always_comb
begin
	shift1 = dati >> {amt[7:6],6'd0};
	shift2 = shift1 >> {amt[5:4],4'd0};
	shift3 = shift2 >> {amt[3:2],2'd0};
	shift4 = shift3 >> amt[1:0];
	datis = shift4[127:0];
end

endmodule
