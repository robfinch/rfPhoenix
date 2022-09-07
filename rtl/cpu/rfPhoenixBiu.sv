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

module rfPhoenixBiu(rst,clk,tlbclk,clock,UserMode,MUserMode,omode,bounds_chk,pe,
	ip,ip_o,ihit,ifStall,ic_line,ic_valid,ic_tag, fifoToCtrl_wack,
	fifoToCtrl_i,fifoToCtrl_full_o,fifoFromCtrl_o,fifoFromCtrl_rd,fifoFromCtrl_empty,fifoFromCtrl_v,
	bok_i, bte_o, cti_o, vpa_o, vda_o, cyc_o, stb_o, ack_i, we_o, sel_o, adr_o,
	dat_i, dat_o, sr_o, cr_o, rb_i, dce, keys, arange, ptbr, ipage_fault, clr_ipage_fault,
	itlbmiss, clr_itlbmiss, rollback, rollback_thread, rollback_bitmap);
parameter AWID=32;
input rst;
input clk;
input tlbclk;
input clock;							// clock for clock algorithm
input UserMode;
input MUserMode;
input [1:0] omode;
input bounds_chk;
input pe;									// protected mode enable
input CodeAddress ip;
output CodeAddress ip_o;
output reg ihit;
input ifStall;
output ICacheLine ic_line;
output reg ic_valid;
output reg [AWID-7:0] ic_tag;
// Fifo controls
output fifoToCtrl_wack;
input MemoryRequest fifoToCtrl_i;
output fifoToCtrl_full_o;
output MemoryResponse fifoFromCtrl_o;
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
input rollback;
input Tid rollback_thread;
output reg [127:0] rollback_bitmap;

parameter TRUE = 1'b1;
parameter FALSE = 1'b0;
parameter HIGH = 1'b1;
parameter LOW = 1'b0;

parameter VLOOKUP1	= 4'd2;
parameter VLOOKUP2  = 4'd3;
parameter PADR_SET 	= 4'd4;
parameter DATA_ALN	= 4'd5;
parameter VLOOKUP3  = 4'd6;

parameter IO_KEY_ADR	= 16'hFF88;

integer m,n,k;
integer n4,n5,n7;
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
reg [7:0] last_tid;
reg [1:0] waycnt;
reg iaccess;
reg daccess;
reg [4:0] icnt;
reg [4:0] dcnt;
Address iadr;
reg keyViolation = 1'b0;
reg xlaten;
reg [31:0] memreq_sel;
CodeAddress last_cadr;
PDCE ptc;
PhysicalAddress padrd1,padrd2,padrd3;

MemoryRequest memreq,imemreq;
reg memreq_rd = 0;
MemoryResponse memresp;
MemoryResponse [6:0] mem_resp;	// memory pipeline
reg zero_data = 0;
Value movdat;
wire [127:0] rb_bitmap1;
reg [127:0] rb_bitmap2, rb_bitmap3, rb_bitmap4;
reg [127:0] rb_bitmaps2 [0:NTHREADS-1];
reg [1023:0] dc_linein;
wire [1023:0] stmask;

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
reg [255:0] dat, dati;
wire [127:0] datis,datis2;

biu_dati_align uda1
(
	.dati(mem_resp[PADR_SET].res),
	.datis(datis), 
	.amt({mem_resp[PADR_SET].badAddr[6:0],3'b0})
);

biu_dati_align uda2
(
	.dati(dati),
	.datis(datis2), 
	.amt({3'b0,adr_o[3:0],3'b0})
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

rfPhoenix_stmask ustmsk (memr.sel, memr.badAddr[5:0], stmask);
always_comb
	dc_linein = (dc_line & ~stmask) | ((memr.res << {memr.badAddr[5:0],3'b0}) & stmask);

always_comb
	rb_bitmap2 = rb_bitmaps2[rollback_thread];

always_comb
	rollback_bitmap = rb_bitmap1|rb_bitmap2|rb_bitmap3|rb_bitmap4;

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
wire [31:0] padr;

rfPhoenix_active_region uargn
(
	.clk(clk),
	.wr(rgn_wr),
	.rwa(rgn_adr),
	.i(rgn_dat),
	.o(rgn_dat_o),
	.adr(padr),
	.region_num(),
	.region(region),
	.err()
);

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
  .full(fifoToCtrl_full_o),
  .rollback(),
  .rollback_thread(rollback_thread),
  .rollback_bitmap(rb_bitmap1)
);

wire memresp_full;
wire [5:0] fifoFromCtrl_cnt;
assign fifoFromCtrl_empty = fifoFromCtrl_cnt=='d0;

// This fifo is at the output of the external bus to the mainline execution.

rfPhoenix_mem_resp_fifo uofifo1
(
	.rst(rst),
	.clk(clk),
	.wr(memresp.wr),
	.di(memresp),
	.rd(fifoFromCtrl_rd),
	.dout(fifoFromCtrl_o),
	.cnt(fifoFromCtrl_cnt),
	.full(memresp_full),
	.v(fifoFromCtrl_v),
	.rollback(rollback),
	.rollback_thread(rollback_thread),
	.rollback_bitmap(rb_bitmap3)
);

// This fifo sits between the output of the data cache lookup memory pipe and
// the external bus. 

reg rd_memq;
wire [5:0] memq_cnt;
MemoryResponse memq_o, memr;

rfPhoenix_mem_resp_fifo uofifo2
(
	.rst(rst),
	.clk(clk),
	.wr(mem_resp[DATA_ALN].wr),
	.di(mem_resp[DATA_ALN]),
	.rd(rd_memq),
	.dout(memq_o),
	.cnt(memq_cnt),
	.full(),
	.v(memq_v),
	.rollback(rollback),
	.rollback_thread(rollback_thread),
	.rollback_bitmap(rb_bitmap4)
);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Instruction cache
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ICacheLine [pL1ICacheWays-1:0] ic_lin;
reg [1:0] ic_rway,ic_wway;
reg [pL1ICacheWays-1:0] icache_w;
reg icache_wr;
always_comb icache_wr = state==IFETCH3;
reg ic_invline,ic_invall;
CodeAddress ipo,ip2,ip3,ip4,ip5;
wire [AWID-1:6] ictag [0:3];
wire [512/4-1:0] icvalid [0:3];

reg [639:0] ici;		// Must be a multiple of 128 bits wide for shifting.
reg [2:0] ivcnt;
reg [2:0] vcn;
ICacheLine [4:0] ivcache;
reg [AWID-1:6] ivtag [0:4];
reg [4:0] ivvalid;
wire ihit2;
reg ihit3;
wire ic_valid2;
reg ic_valid3;
wire [AWID-7:0] ic_tag2;
reg [AWID-7:0] ic_tag3;

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
always_ff @(posedge clk)
	ihit <= ihit2;
always_ff @(posedge clk)
	ic_valid3 <= ic_valid2;
always_ff @(posedge clk)
	ic_valid <= ic_valid2;
assign ip_o = ip3;
always_ff @(posedge clk)
	ic_tag3 <= ic_tag2;
always_ff @(posedge clk)
	ic_tag <= ic_tag3;
	
// 552 wide x 512 deep, uses output register so there is a 2 cycle read latency.
icache_blkmem uicm (
  .clka(clk),    // input wire clka
  .ena(icache_wr),      // input wire ena
  .wea(icache_wr),      // input wire [0 : 0] wea
  .addra({ic_wway,ipo[12:6]}),  // input wire [8 : 0] addra
  .dina(ici[pL1ICacheLineSize-1:0]),    // input wire [551 : 0] dina
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
	.way(ic_wway),
	.rclk(clk),
	.ip(padr),
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
	.vtag(ic_tag2),
	.icv(ic_valid2)
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
	.way(ic_wway),
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
reg [3:0] tlbacrd;
always_ff @(posedge clk)
	tlbacrd <= tlbacr;

reg [2:0] dwait;		// wait state counter for dcache
Address dadr;
DCacheLine dci;
DCacheLine [pL1DCacheWays-1:0] dc_eline, dc_oline;
DCacheLine [pL1DCacheWays-1:0] dc_elin, dc_olin;
reg [1023:0] dc_line;
reg [1023:0] datil;
reg dcachable;
reg [1:0] dc_erway,prev_dc_erway;
reg [1:0] dc_orway,prev_dc_orway;
wire [1:0] dc_ewway;
wire [1:0] dc_owway;
reg [pL1DCacheWays-1:0] dcache_ewr, dcache_owr;
wire dc_ewr, dc_owr;
reg dc_invline,dc_invall;

dcache_blkmem udcb1e (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(dc_ewr),      // input wire [0 : 0] wea
  .addra({dc_ewway,dadr[13:7]+dadr[6]}),  // input wire [8 : 0] addra
  .dina(dci),    // input wire [511 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb({dc_erway,padrd1[13:7]+padrd1[6]}),  // input wire [8 : 0] addrb
  .doutb(dc_eline)  // output wire [511 : 0] doutb
);

dcache_blkmem udcb1o (
  .clka(clk),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(dc_owr),      // input wire [0 : 0] wea
  .addra({dc_owway,dadr[13:7]}),  // input wire [8 : 0] addra
  .dina(dci),    // input wire [511 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb({dc_orway,padrd1[13:7]}),  // input wire [8 : 0] addrb
  .doutb(dc_oline)  // output wire [511 : 0] doutb
);

always_comb
	case(padrd1[6])
	1'b0:	dc_line = {dc_oline,dc_eline};
	1'b1:	dc_line = {dc_eline,dc_oline};
	endcase

wire [AWID-7:0] dc_etag [3:0];
wire [127:0] dc_evalid [0:3];
wire [3:0] dhit1e;	// debugging
wire [3:0] dhit1o;
wire [AWID-7:0] dc_otag [3:0];
wire [127:0] dc_ovalid [0:3];
wire dhito,dhite;
Address vadr2e;
always_comb
	vadr2e <= padr[13:7]+padr[6];
	
rfPhoenix_dchit udchite
(
	.rst(rst),
	.clk(clk),
	.tags(dc_etag),
	.ndx(vadr2e),
	.adr(padr),
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
	.ndx(padr[13:7]),
	.adr(padr),
	.valid(dc_ovalid),
	.hits(dhit1o),
	.hit(dhito),
	.rway(dc_orway)
);

reg dhit;
always_comb
	dhit = (dhite & dhito) || (adr_o[6] ? (dhito && padrd1[5:0] < 6'd61) : (dhite && padrd1[5:0] < 6'd61));

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
	.ndx(padrd1[13:7]),
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
	.ndx(padrd1[13:7]+padrd1[6]),
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
	.hit(|memr.sz),
	.hit2(memr.dchit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(memr.acr),
	.eaeo(~memr.badAddr[6]),
	.daeo(~adr_o[6]),
	.wr(dc_ewr)
);

rfPhoenix_dcache_wr udcwro
(
	.clk(clk),
	.state(state),
	.ack(ack_i),
	.func(memreq.func),
	.dce(dce),
	.hit(|memr.sz),
	.hit2(memr.dchit),
	.inv(ic_invline|ic_invall|dc_invline|dc_invall),
	.acr(memr.acr),
	.eaeo(memr.badAddr[6]),
	.daeo(adr_o[6]),
	.wr(dc_owr)
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
	.eaeo(~memr.badAddr[6]),
	.daeo(~adr_o[6]),
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
	.eaeo(memr.badAddr[6]),
	.daeo(adr_o[6]),
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
/*
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
*/
rfPhoenix_tlb utlb
(
  .rst_i(rst),
  .clk_i(tlbclk),
  .al_i(ptbr[7:6]),
  .clock(clock),
  .rdy_o(tlbrdy),
  .asid_i(mem_resp[0].asid),
  .sys_mode_i(vpa_o ? ~UserMode : ~MUserMode),
  .xlaten_i(xlaten),
  .we_i(we_o),
  .dadr_i(dadr),
  .next_i(inext),
  .iacc_i(mem_resp[0].v),//iaccess|daccess),
  .dacc_i(1'b0),
  .iadr_i(mem_resp[0].badAddr),
  .padr_o(padr),
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
  .m_dat_o(tlb_dat)
);

always_ff @(posedge clk)	// delay for data tag lookup
	padrd1 <= padr;
always_ff @(posedge clk)	// two cycle delay for data fetch
	padrd2 <= padrd1;
always_ff @(posedge clk)
	padrd3 <= padrd2;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// IPT
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

reg [6:0] ptg_state = IPT_IDLE;
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
	special_ram = ptgram_en || rgn_en || tlb_access;

reg [15:0] hash_r;
`ifdef SUPPORT_HASHPT
integer n6;
always_ff @(posedge tlbclk)
begin
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
	icnt <= 'd0;
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
	dat <= 'd0;
	sr_o <= LOW;
	cr_o <= LOW;
	waycnt <= 2'd0;
	ic_wway <= 2'b00;
	dwait <= 3'd0;
	iaccess <= FALSE;
	daccess <= FALSE;
	ici <= 'd0;
	dci <= 'd0;
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
	sel <= 'd0;
	dfetch2 <= 1'b0;
	rd_memq <= 'd0;
	memreq_rd <= FALSE;
	mem_resp[0] <= 'd0;
	mem_resp[1] <= 'd0;
	mem_resp[2] <= 'd0;
	mem_resp[3] <= 'd0;
	mem_resp[4] <= 'd0;
	mem_resp[5] <= 'd0;
	mem_resp[6] <= 'd0;
	last_tid <= 'd0;
	last_cadr <= 'd0;
	for (n = 0; n < 7; n = n + 1)
		rb_bitmaps2[n] <= 'd0;
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
	clr_ptg_fault <= 1'b0;
	if (clr_ipage_fault)
		ipage_fault <= 1'b0;
	if (clr_itlbmiss)
		itlbmiss <= 1'b0;

	mem_resp[DATA_ALN].wr <= FALSE;
	tlbwr <= FALSE;
	tlb_ack <= FALSE;
	ptgram_wr <= FALSE;
	tStage0();
	tStage1();
	tAddressXlat();
	tCacheAccess();
	tCacheDataAlign();

	if (rollback) begin
		for (n5 = 0; n5 < 7; n5 = n5 + 1)
			if (mem_resp[n5].thread==rollback_thread)
				mem_resp[n5].v <= 1'b0;
		rb_bitmaps2[rollback_thread] <= 'd0;
	end

	case(state)
	MEMORY_INIT:
		begin
			for (n5 = 0; n5 < 8; n5 = n5 + 1)
				ptc[n5] <= 'd0;
			rd_memq <= FALSE;
			goto (MEMORY1);
		end

	MEMORY1:
		begin
			rd_memq <= FALSE;
			if (memq_v)
				rd_memq <= TRUE;
			if (rd_memq) begin
				if (memq_o.tid != last_tid) begin
					rd_memq <= FALSE;
					last_tid <= memq_o.tid;
					memr <= memq_o;
					memreq <= memq_o;
					gosub (MEMORY_ACTIVATE_LO);
				end
			end
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
    	goto (MEMORY9);
	    dadr <= {dadr[AWID-1:4] + 2'd1,4'd0};
			ptgram_adr <= memr.badAddr[18:4];
			rgn_adr <= memr.badAddr[9:4];
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
			memresp.cause <= FLT_NONE;
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
			ipo <= {memr.badAddr[$bits(Address)-1:6],6'b0};
			goto (IFETCH1);
			for (n = 0; n < 5; n = n + 1) begin
				if (ivtag[n]==memr.badAddr[$bits(Address)-1:6] && ivvalid[n]) begin
					vcn <= n;
		    	goto (IFETCH4);
	    	end
			end
		end
	// Hardware subroutine to fetch instruction cache line
	IFETCH1:
	  if (!ack_i) begin
	  	// Cache miss, select an entry in the victim cache to
	  	// update.
	  	if (memr.sz!=nul) begin	// ic_valid flag
				ivcnt <= ivcnt + 2'd1;
				if (ivcnt>=3'd4)
					ivcnt <= 3'd0;
				ivcache[ivcnt] <= memr.res;
				ivtag[ivcnt] <= memr.vcadr[$bits(Address)-1:6];
				ivvalid[ivcnt] <= TRUE;
				if (ic_line=='d0)
					$stop;
			end
			icnt <= 'd0;
	  	vpa_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
	    sel_o <= 16'hFFFF;
	    adr_o <= ipo;
  		goto (IFETCH2);
		end
	IFETCH2:
	  begin
	  	stb_o <= HIGH;
	    if (ack_i) begin
	      ici <= {dat_i,ici[639:128]};	// shift in the data
	      icnt <= icnt + 4'd4;					// increment word count
	      if (icnt[4:2]==3'd4) begin		// Are we done?
	      	tDeactivateBus();
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
	  	ret();
		end
	
	IFETCH4:
		begin
			ici <= {96'd0,ivcache[vcn]};
			if (memr.sz!=nul) begin
				ivcache[vcn] <= memr.res;
				ivtag[vcn] <= memr.vcadr[$bits(Address)-1:6];
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
				goto (IFETCH2);
			end
		end

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// Initiate burst access
	DFETCH2:
	  if (!ack_i) begin
	  	vda_o <= HIGH;
	  	bte_o <= 2'b00;
	  	cti_o <= 3'b001;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
	    sel_o <= 16'hFFFF;
	    goto (DFETCH5);
	    case(memr.sz)
	    2'b00:		// need both even and odd cache lines (start with even)
					adr_o <= {memr.badAddr[AWID-1:7]+memr.badAddr[6],1'b0,6'h0};
	    2'b01:		// If got a hit on the even address, the odd one must be missing
					adr_o <= {memr.badAddr[AWID-1:7],1'b1,6'h0};
			2'b10:		// Otherwise the even one must be missing
					adr_o <= {memr.badAddr[AWID-1:7]+memr.badAddr[6],1'b0,6'h0};
			2'b11:		// noi missing lines, finished
				begin
					tDeactivateBus();
					ret();
				end
			endcase
	  end

	// Sustain burst access
	DFETCH5:
	  begin
	  	stb_o <= HIGH;
	    if (ack_i) begin
	    	dcnt <= dcnt + 4'd4;
	      dci <= {dat_i,dci[511:128]};
	      if (dcnt[4:2]==3'd3) begin		// Are we done?
	      	case(memr.hit)
	      	2'b00:	memr.hit <= 2'b01;
	      	2'b01:	memr.hit <= 2'b11;
	      	2'b10:	memr.hit <= 2'b11;
	      	2'b11:	memr.hit <= 2'b11;
	      	endcase
	      	// Fill in missing memory data.
	      	case(memr.hit)
	      	2'b00:	memr.res[ 511:  0] <= {dat_i,dci[511:12]};
	      	2'b01:	memr.res[1023:512] <= {dat_i,dci[511:12]};
	      	2'b10:	memr.res[ 511:  0] <= {dat_i,dci[511:12]};
	      	2'b11:	;
	      	endcase
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
				goto (DFETCH5);
			end
		end

	// Trgger a data cache update. The data cache line is in dci. The only thing
	// left to do is update the tag and valid status.
	DFETCH7:
		if (memr.sz==2'b11) begin
			tDeactivateBus();
			ret();
		end
		else
			goto(DFETCH2);

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
			goto (IPT_FETCH3);
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
			goto (PT_FETCH4);
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

// Add request to pipeline
// Compute data select signals
task tStage0;
begin
	memreq_rd <= FALSE;
	xlaten <= FALSE;
	if (!fifoToCtrl_empty && tlbrdy)
		memreq_rd <= TRUE;
	mem_resp[0] <= 'd0;
	if (memreq_rd) begin
		if (tlbrdy) begin
			if (tlb_cyc) begin
				mem_resp[0].func <= MR_TLB;
				mem_resp[0].badAddr <= {tlb_adr[AWID-1:5],5'h0} + 5'd16;
				rb_bitmaps2[imemreq.thread][imemreq.tgt] <= 1'b1;
			end
			else if (fifoToCtrl_v) begin
				waycnt <= waycnt + 2'd1;
				xlaten <= TRUE;
				mem_resp[0].v <= 1'b1;
				mem_resp[0].ip <= imemreq.ip;
				mem_resp[0].tid <= imemreq.tid;
				mem_resp[0].thread <= imemreq.thread;
				mem_resp[0].omode <= imemreq.omode;
				mem_resp[0].func <= imemreq.func;
				mem_resp[0].func2 <= imemreq.func2;
				mem_resp[0].step <= imemreq.step;
				mem_resp[0].asid <= imemreq.asid;
				mem_resp[0].badAddr <= imemreq.adr;
				mem_resp[0].vcadr <= imemreq.vcadr;
				mem_resp[0].res <= imemreq.dat;
				mem_resp[0].dat <= imemreq.dat;
				mem_resp[0].sz <= imemreq.sz;
				mem_resp[0].sel <= imemreq.sel;
				mem_resp[0].tgt <= imemreq.tgt;
				rb_bitmaps2[imemreq.thread][imemreq.tgt] <= 1'b1;
			end
		end
	end
end
endtask

task tStage1;
begin
	tlb_access <= 1'b0;
	rgn_en <= 1'b0;
	ptgram_en <= 1'b0;
	mem_resp[1] <= mem_resp[0];
	mem_resp[1].cause <= FLT_NONE;
	if (mem_resp[0].func==MR_CACHE) begin
		ic_invline <= mem_resp[0].dat[2:0]==3'd1;
		ic_invall	<= mem_resp[0].dat[2:0]==3'd2;
		dc_invline <= mem_resp[0].dat[5:3]==3'd3;
		dc_invall	<= mem_resp[0].dat[5:3]==3'd4;
		if (mem_resp[0].dat[5:3]==3'd1)
			dce <= TRUE;
		if (mem_resp[0].dat[4:2]==3'd2)
			dce <= FALSE;
    mem_resp[1].cmt <= TRUE;
		mem_resp[1].wr <= TRUE;
		mem_resp[1].res <= 'd0;
	end
	else if ((mem_resp[0].func==MR_LOAD || mem_resp[0].func==MR_LOADZ || mem_resp[0].func==MR_STORE || 
		mem_resp[0].func==MR_TLBRD || mem_resp[0].func==MR_TLBRW ||
		mem_resp[0].func==MR_ICACHE_LOAD) && mem_resp[0].v) begin
    mem_resp[1].sel <= {32'h0,mem_resp[0].sel} << mem_resp[0].badAddr[3:0];
		casez(mem_resp[0].badAddr)
		32'hFF9F????:
			begin
				mem_resp[1].rgn_en <= 1'b1;
				rgn_en <= 1'b1;
				rgn_wr <= mem_resp[0].func==MR_STORE;
			end
		32'hFFA?????:
			begin
				mem_resp[1].ptgram_en <= 1'b1;
				ptgram_en <= 1'b1;
				ptgram_wr <= mem_resp[0].func==MR_STORE;
			end
		32'hFFD?????:
			begin
			end
		32'hFFE?????:
			begin
				tlbwr <= mem_resp[0].func==MR_TLBRW;
				mem_resp[1].tlb_access <= 1'b1;
				tlb_access <= 1'b1;
				tlb_ia <= mem_resp[0].badAddr[15:0];
				tlb_ib <= mem_resp[0].res;
				/*
				tlb_bucket[0] <= tlbdato[ 63:  0];
				tlb_bucket[1] <= tlbdato[127: 64];
				tlb_bucket[2] <= tlbdato[191:128];
				tlb_bucket[3] <= tlbdato[255:192];
				tlb_bucket[4] <= tlbdato[287:256];
				tlb_bucket[5] <= tlbdato[319:288];
				*/
			end
		default:	;
		endcase
		rgn_adr <= mem_resp[0].badAddr[9:4];
		rgn_dat <= mem_resp[0].res;
`ifdef SUPPORT_HASHPT
		ptgram_adr <= mem_resp[0].badAddr[18:4];
		ptgram_dati <= mem_resp[0].res;
`endif
	end
end
endtask

// Perform virtual to physical translation.
// Perform PMA checks on physical address

task tAddressXlat;
begin
	// VLOOKUP1 is in line with the output of the TLB
	mem_resp[VLOOKUP1] <= mem_resp[1];				// tag lookup
	if (mem_resp[VLOOKUP1].func==MR_TLBRW || mem_resp[VLOOKUP1].func==MR_TLBRD)
		mem_resp[VLOOKUP1].res <= tlbdato;
	mem_resp[VLOOKUP1].acr <= tlbacr;
	mem_resp[VLOOKUP3] <= mem_resp[VLOOKUP1];	// data tag lookup
//	mem_resp[VLOOKUP3] <= mem_resp[VLOOKUP2];	// data fetch 1
	mem_resp[PADR_SET] <= mem_resp[VLOOKUP3];	// data fetch 2
	// No address translations for machine mode
	if (mem_resp[VLOOKUP3].omode!=2'd3) begin
		mem_resp[PADR_SET].badAddr <= padrd1;
	end
	else
		mem_resp[PADR_SET].acr <= region.at[3:0];
	if (mem_resp[VLOOKUP3].v) begin
	  if (!region.at[0] && mem_resp[VLOOKUP3].func==MR_ICACHE_LOAD)
	    mem_resp[PADR_SET].cause <= FLT_PMA;
	 	//we_o <= wr & tlbwr & region.at[1];
	  if (mem_resp[VLOOKUP3].func==MR_STORE && !region.at[1])
		  mem_resp[PADR_SET].cause <= FLT_WRV;
	  else if (mem_resp[VLOOKUP3].func!=MR_STORE && !region.at[2])
		  mem_resp[PADR_SET].cause <= FLT_RDV;
	   // TLB miss has higher precedence than PMA
	   // No TLB miss in machine mode
		if (tlbmiss && mem_resp[VLOOKUP3].omode!=2'd3)
			mem_resp[PADR_SET].cause <= FLT_TLBMISS;
	 	if (!tlbacr[2] && (mem_resp[VLOOKUP3].func==MR_LOAD || mem_resp[VLOOKUP3].func==MR_LOADZ)) begin
	 		mem_resp[PADR_SET].cause <= FLT_RDV;
	 		//tReadViolation(mem_resp[4].badAddr);
//		if (tlbacr[3])
//			mem_resp[PADR_SET].func2 <= MR_CACHE;
	 	end
	end
//	memresp.cause <= {4'h8,FLT_PMA};
	//dcachable <= dcachable & region.at[3];
end
endtask

// Access cached data
// Determine whether a memory access is required.

task tCacheAccess;
begin
	if (mem_resp[VLOOKUP3].v) begin
`ifdef SUPPORT_HASHPT
	if (ptg_fault) begin
		clr_ptg_fault <= 1'b1;
		if (mem_resp[VLOOKUP3].func==MR_ICACHE_LOAD)
			mem_resp[PADR_SET].cause <= {4'h8,FLT_CPF};
		else
			mem_resp[PADR_SET].cause <= {4'h8,FLT_DPF};
	end
`endif
	mem_resp[PADR_SET].dchit <= dhit & mem_resp[VLOOKUP3].acr[3];	// hit and cachable data
	case(mem_resp[VLOOKUP3].func)
	MR_TLBRW,MR_TLBRD:
		mem_resp[PADR_SET].wr <= TRUE;
	MR_ICACHE_LOAD:
		mem_resp[PADR_SET].wr <= TRUE;
	MR_LOAD,MR_LOADZ:
		case(1'b1)
		mem_resp[VLOOKUP3].tlb_access:	begin mem_resp[PADR_SET].res <= tlbdato; end
		mem_resp[VLOOKUP3].ptgram_en:		begin mem_resp[PADR_SET].res <= ptgram_dato; end
		mem_resp[VLOOKUP3].rgn_en:			begin mem_resp[PADR_SET].res <= rgn_dat_o; end
		default:		
			begin
				mem_resp[PADR_SET].res <= dc_line;
				mem_resp[PADR_SET].wr <= !dce & dhit & mem_resp[VLOOKUP3].acr[3];
				mem_resp[PADR_SET].hit <= {dhito,dhite};
				if (!(dce & dhit & mem_resp[VLOOKUP3].acr[3]))
					mem_resp[PADR_SET].cause <= FLT_DCM;
			end
	  endcase
	MR_STORE:	
		begin
			mem_resp[PADR_SET].wr <= TRUE;
			case(mem_resp[VLOOKUP3].sz)
			byt:	;	// Cant be unaligned
			wyde:	if (mem_resp[VLOOKUP3].badAddr[5:0] > 6'd62)	mem_resp[PADR_SET].cause <= FLT_ALN;
			tetra:if (mem_resp[VLOOKUP3].badAddr[5:0] > 6'd60)	mem_resp[PADR_SET].cause <= FLT_ALN;
			default:	if (mem_resp[VLOOKUP3].badAddr[5:0] > 6'd60)	mem_resp[PADR_SET].cause <= FLT_ALN;
			endcase
 			mem_resp[PADR_SET].res <= (dc_line & ~stmask) | ((mem_resp[VLOOKUP3].res << {mem_resp[VLOOKUP3].badAddr[6:0],3'b0}) & stmask);
		end
	default:	;
	endcase
	end
end
endtask

// Align the data and send it back
task tCacheDataAlign;
begin
	rb_bitmaps2[mem_resp[DATA_ALN].thread][mem_resp[DATA_ALN].tgt] <= 1'b0;
	mem_resp[DATA_ALN] <= mem_resp[PADR_SET];
	if (mem_resp[PADR_SET].func!=MR_ICACHE_LOAD || last_cadr != mem_resp[PADR_SET].badAddr) begin
		if (mem_resp[PADR_SET].func==MR_ICACHE_LOAD)
			last_cadr <= mem_resp[PADR_SET].badAddr;
		mem_resp[DATA_ALN].wr <= mem_resp[PADR_SET].wr & mem_resp[PADR_SET].v;
	end
	else
		mem_resp[DATA_ALN].wr <= 1'b0;
	if (mem_resp[PADR_SET].v) begin
		memresp.wr <= TRUE;			// always send something back
		memresp <= mem_resp[PADR_SET];
		case(1'b1)
		mem_resp[PADR_SET].tlb_access:	;
		mem_resp[PADR_SET].ptgram_en:		;
		mem_resp[PADR_SET].rgn_en:			;
		default:		
		  case(mem_resp[PADR_SET].func)
		  MR_LOAD,MR_MOVLD:
	    	case(memreq.sz)
	    	nul:	memresp.res[mem_resp[PADR_SET].step] <= 'h0;
	    	byt:	memresp.res[mem_resp[PADR_SET].step] <= {{56{datis[7]}},datis[7:0]};
	    	wyde:	memresp.res[mem_resp[PADR_SET].step] <= {{48{datis[15]}},datis[15:0]};
	    	tetra:	memresp.res[mem_resp[PADR_SET].step] <= {{32{datis[31]}},datis[31:0]};
	//    	octa:	begin memresp.res[mem_resp[5].step] <= {{64{datis[63]}},datis[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[mem_resp[PADR_SET].step] <= mem_resp[PADR_SET].res;
	    	endcase
		  MR_LOADZ:
	    	case(mem_resp[PADR_SET].sz)
	    	nul:	memresp.res[mem_resp[PADR_SET].step] <= 'h0;
	    	byt:	begin memresp.res[mem_resp[PADR_SET].step] <= {56'd0,datis[7:0]}; end
	    	wyde:	begin memresp.res[mem_resp[PADR_SET].step] <= {48'd0,datis[15:0]}; end
	    	tetra:	begin memresp.res[mem_resp[PADR_SET].step] <= {32'd0,datis[31:0]}; end
	//    	octa:	begin memresp.res[mem_resp[5].step] <= {64'd0,datis[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[mem_resp[PADR_SET].step] <= mem_resp[PADR_SET].res;
	    	endcase
		  default:  ;
		  endcase
		endcase
	end
end
endtask

task tMemoryActivateLo;
begin
	dfetch2 <= 1'b0;
	strips <= 2'd0;
	// Detect cache controller commands
	case(memr.func)
	MR_STORE,MR_MOVST:
		begin
`ifdef SUPPORT_HWWALK    		
			// Invalidate PTCEs when a store occurs to the PDE
			for (n4 = 0; n4 < 12; n4 = n4 + 1)
				if (ptc[n4].pde.padr[AWID-1:4]==memr.badAddr[AWID-1:4])
					ptc[n4].v <= 1'b0;
`endif						
/*
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
*/
			vpa_o <= LOW;
			vda_o <= HIGH;
			bte_o <= 2'b00;
			cti_o <= 3'b000;
			cyc_o <= HIGH;
			stb_o <= HIGH;
			we_o <= HIGH;
			sel_o <= memr.sel[15:0];
	  	adr_o <= memr.badAddr;
	  	dat_o <= memr.res;
			cr_o <= memr.func2==MR_STC;
  		goto (MEMORY_ACKLO);
		end
	MR_ICACHE_LOAD:
		goto (IFETCH0);
	MR_LOAD,MR_LOADZ:
		begin
			if (memr.func2==MR_LDV)
				call (DFETCH2,DATA_ALIGN);
			if (memr.acr[3])
				goto (DFETCH2);
			else begin
  			vpa_o <= LOW;
  			vda_o <= HIGH;
  			bte_o <= 2'b00;
  			cti_o <= 3'b000;
  			cyc_o <= HIGH;
  			stb_o <= HIGH;
  			we_o <= LOW;
  			sel_o <= memr.sel[15:0];
  			adr_o <= memr.badAddr;
   			sr_o <= memr.func2==MR_LDR;
	  		goto (MEMORY_ACKLO);
			end
		end
	default:	ret();	// unknown operation
	endcase
end
endtask

task tMemoryAckLo;
begin
	case(memr.func)
	MR_STORE,MR_MOVST:
		if (ack_i || !stb_o) begin
			if (memr.badAddr[5])
 				dci <= memr.res[511:256];
			else
 				dci <= memr.res[255:0];
		  goto (MEMORY_NACKLO);
      stb_o <= LOW;
      if (memr.sel[31:16]=='d0)
      	tDeactivateBus();
    end
  MR_LOAD,MR_LOADZ:
    if (ack_i || !stb_o) begin
      goto (MEMORY_NACKLO);
      stb_o <= LOW;
      dati <= {128'd0,dat_i};
      if (sel[31:16]==1'h0) begin
      	tDeactivateBus();
      end
    end
  default:	;
	endcase
end
endtask

task tMemoryNackLo;
begin
  if (~ack_i) begin
    case(memr.func)
    MR_LOAD,MR_LOADZ,MR_MOVLD:
    	begin
		    if (|memr.sel[31:16])
	  	    goto (MEMORY_ACTIVATE_HI);
	  	  else begin
    			tDeactivateBus();
	        goto (DATA_ALIGN);
	      end
    	end
    MR_STORE,MR_MOVST:
    	begin
		    if (|sel[31:16] & ~ptgram_en)
			    goto (MEMORY_ACTIVATE_HI);
			  else begin
	    		if (memreq.func2==MR_STPTR) begin	// STPTR
			    	if (~|ea[AWID-5:0] || shr_ma[5:3] >= region.at[18:16]) begin
	  					memresp.cause <= FLT_NONE;
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
  					memresp.cause <= FLT_NONE;
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
`ifndef SUPPORT_HASHPT
  dwait <= 3'd0;
  memr.badAddr[6] <= ~memr.badAddr[6];
//    dadr <= adr_o;
  goto (MEMORY_ACKHI);
  begin
`else 		
	if (ptg_fault) begin
		clr_ptg_fault <= 1'b1;
		tPageFault(FLT_DPF,dadr);
	end
	if (pte_found || !ptg_en) begin
	  dwait <= 3'd0;
  	goto (MEMORY_ACKHI);
`endif
		if (dhit && (memreq.func==MR_LOAD ||
			memreq.func==MR_LOADZ || memreq.func==MR_MOVLD /*|| memreq.func==RTS2*/) && dce && tlbacr[3])
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
	case(memr.func)
	MR_STORE,MR_MOVST:
	  if (ack_i || !stb_o) begin
			if (memr.badAddr[6])
				dci <= dc_linein[1023:512];
			else
				dci <= dc_linein[511:0];
		end
	default:	;
	endcase
  goto (MEMORY13);
  dati[255:128] <= dat_i;
  tDeactivateBus();
end
endtask

task tMemoryNackHi;
begin
	if (memr.badAddr[6])
		dci <= dc_linein[511:0];
	else
		dci <= dc_linein[1023:512];
  if (~ack_i) begin
    case(memreq.func)
    MR_LOAD,MR_LOADZ,MR_MOVLD:
      goto (DATA_ALIGN);
    MR_STORE,MR_MOVST:
    	begin
    		if (memreq.func2==MR_STPTR) begin	// STPTR
		    	if (~|ea[AWID-5:0] || shr_ma[5:3] >= region.at[18:16]) begin
  					memresp.cause <= FLT_NONE;
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
					memresp.cause <= FLT_NONE;
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
endtask


task tDataAlign;
begin
	memresp.cause <= FLT_NONE;
	tDeactivateBus();
	if ((memr.func==MR_LOAD || memr.func==MR_LOADZ || memr.func==MR_MOVLD) & memr.sz!=2'b11 & dcachable & memr.acr[3] & dce &
	 	~memr.ptgram_en & ~memr.rgn_en & ~memr.tlb_access) begin
		memresp.cause <= FLT_DCM;
	end
	else if (memreq.func==MR_MOVLD) begin
		ret();
	end
	else if (!memresp_full)
  	ret();
	if (memreq.func2==MR_LDG) begin
		if (memreq.step == NLANES-1) begin
			memresp.wr <= TRUE;
		end
		memresp.res[memreq.count] <= datis2[63:0];
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
  case(memr.func)
  MR_LOAD,MR_MOVLD:
  	begin
  		if (memr.func2==MR_LDV)
  			memresp.res <= memresp.res >> {memresp.badAddr[6:0],3'b0};
  		else
	    	case(memr.sz)
	    	nul:	memresp.res[memr.step] <= 'h0;
	    	byt:	begin memresp.res[memr.step] <= {{56{datis2[7]}},datis2[7:0]}; end
	    	wyde:	begin memresp.res[memr.step] <= {{48{datis2[15]}},datis2[15:0]}; end
	    	tetra:	begin memresp.res[memr.step] <= {{32{datis2[31]}},datis2[31:0]}; end
	    	octa:	begin memresp.res[memr.step] <= {{64{datis2[63]}},datis2[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[memr.step] <= memr.dat;
	    	endcase
  	end
  MR_LOADZ:
  	begin
  		if (memr.func2==MR_LDV)
  			memresp.res <= memresp.res >> {memresp.badAddr[6:0],3'b0};
  		else
	    	case(memr.sz)
	    	nul:	memresp.res[memr.step] <= 'h0;
	    	byt:	begin memresp.res[memr.step] <= {56'd0,datis2[7:0]}; end
	    	wyde:	begin memresp.res[memr.step] <= {48'd0,datis2[15:0]}; end
	    	tetra:	begin memresp.res[memr.step] <= {32'd0,datis2[31:0]}; end
	    	octa:	begin memresp.res[memr.step] <= {64'd0,datis2[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp.res[memr.step] <= memr.dat;
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
input MemoryRequest req;
input MemoryResponse oresp;
output MemoryResponse resp;
begin
	//tDeactivateBus();
	resp <= oresp;
	resp.step <= req.step;
	resp.cmt <= TRUE;
  resp.cause <= FLT_TLBMISS;
	resp.tid <= req.tid;
  resp.badAddr <= req.adr;
  resp.wr <= TRUE;
	resp.res <= 'd0;
end
endtask

// Page faults occur only during hardware page table walks when a translation
// cannot be found.

task tPageFault;
input CauseCode typ;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= typ;
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
  memresp.cause <= FLT_WRV;
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
  memresp.cause <= FLT_RDV;
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
  memresp.cause <= FLT_KEY;
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
		memresp.cause <= FLT_NONE;
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
input [1023:0] dati;
output reg [127:0] datis;
input [9:0] amt;

reg [1023:0] shift0;
reg [1023:0] shift1;
reg [1023:0] shift2;
reg [1023:0] shift3;
reg [1023:0] shift4;
always_comb
begin
	shift0 = dati >> {amt[9:8],8'd0};
	shift1 = shift0 >> {amt[7:6],6'd0};
	shift2 = shift1 >> {amt[5:4],4'd0};
	shift3 = shift2 >> {amt[3:2],2'd0};
	shift4 = shift3 >> amt[1:0];
	datis = shift4[127:0];
end

endmodule
