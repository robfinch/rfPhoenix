// ============================================================================
//        __
//   \\__/ o\    (C) 2021-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenix_biu.sv
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

import wishbone_pkg::*;
import rfPhoenixPkg::*;
import rfPhoenixMmupkg::*;

module rfPhoenix_biu(rst,clk,tlbclk,clock,UserMode,MUserMode,omode,bounds_chk,pe,
	ip,ip_o,ihit,ihite,ihito,ifStall,ic_line,ic_valid,ic_tage, ic_tago, fifoToCtrl_wack,
	fifoToCtrl_i,fifoToCtrl_full_o,fifoFromCtrl_o,fifoFromCtrl_rd,fifoFromCtrl_empty,fifoFromCtrl_v,
	bte_o, tid_o, cti_o, seg_o, cyc_o, stb_o, we_o, sel_o, adr_o, dat_o, csr_o,
	stall_i, next_i, rty_i, ack_i, err_i, tid_i, dat_i, rb_i,
	dce, keys, arange, ptbr, ipage_fault, clr_ipage_fault,
	itlbmiss, clr_itlbmiss, rollback, rollback_bitmaps);
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
input code_address_t ip;
output code_address_t ip_o;
output reg ihit;
output reg ihite;
output reg ihito;
input ifStall;
output reg [$bits(ICacheLine)*2-1:0] ic_line;
output reg ic_valid;
output reg [$bits(Address)-1:7] ic_tage;
output reg [$bits(Address)-1:7] ic_tago;
// Fifo controls
output fifoToCtrl_wack;
input MemoryArg_t fifoToCtrl_i;
output fifoToCtrl_full_o;
output MemoryArg_t fifoFromCtrl_o;
input fifoFromCtrl_rd;
output fifoFromCtrl_empty;
output fifoFromCtrl_v;
// Bus controls
//output wb_write_request128_t wbm_req;
//input wb_read_response128_t wbm_resp;
output wb_burst_type_t bte_o;
output wb_tranid_t tid_o;
output wb_cycle_type_t cti_o;
output wb_segment_t seg_o;
output reg cyc_o;
output reg stb_o;
input stall_i;
input next_i;
input ack_i;
input rty_i;
input err_i;
input wb_tranid_t tid_i;
output reg we_o;
output reg [15:0] sel_o;
output Address adr_o;
input [127:0] dat_i;
output reg [127:0] dat_o;
output reg csr_o;
input rb_i;

output reg dce;							// data cache enable
input [23:0] keys [0:7];
input [2:0] arange;
input [127:0] ptbr;
output reg ipage_fault;
input clr_ipage_fault;
output reg itlbmiss;
input clr_itlbmiss;
input [NTHREADS-1:0] rollback;
output reg [127:0] rollback_bitmaps [0:NTHREADS-1];

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

/*
always_comb
begin
	wbm_req.bte = bte_o;
	wbm_req.cti = cti_o;
	wbm_req.bndx = bndx_o;
	wbm_req.seg = seg_o;
	wbm_req.cyc = cyc_o;
	wbm_req.stb = stb_o;
	
end
*/

reg [5:0] shr_ma;

reg [6:0] state;
// States for hardware routine stack, five deep.
// States go at least 3 deep.
// Memory1
// PT_FETCH <on a tlbmiss>
// READ_PDE/PTE
// 
Address next_adr_o;
reg [6:0] stk_state [0:15];
reg [3:0] stk_dep;

wb_segment_t last_seg;
reg xlaten_stk;
wb_segment_t seg_stk;
wb_burst_type_t bte_stk;
wb_cycle_type_t cti_stk;
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
wire memq_v;
reg [31:0] memreq_sel;
code_address_t last_cadr;
PDCE ptc;
PhysicalAddress padrd1,padrd2,padrd3;

MemoryArg_t memreq,imemreq;
reg memreq_rd = 0;
MemoryArg_t memresp, memresp2;
MemoryArg_t [6:0] mem_resp;	// memory pipeline
reg zero_data = 0;
wb_tranid_t tid_cnt = 'd0;
value_t movdat;
reg [127:0] rb_bitmaps1 [0:NTHREADS-1];
reg [127:0] rb_bitmaps2 [0:NTHREADS-1];
reg [127:0] rb_bitmaps3 [0:NTHREADS-1];
reg [127:0] rb_bitmaps4 [0:NTHREADS-1];
reg [1023:0] dc_line;
reg [1023:0] dc_linein;
reg [1:0] dc_line_mod;
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
	afilt = (memreq.func==MR_MOVST) ? memreq.res : memreq.adr;

always_comb
	ea = cta + (afilt >> shr_ma);

reg [7:0] ealow;

reg [1:0] strips;
reg [127:0] sel;
reg [127:0] nsel;
reg [1023:0] dat, dati;
wire [127:0] datis,datis2;

biu_dati_align uda1
(
	.dati(mem_resp[PADR_SET].res),
	.datis(datis), 
	.amt({mem_resp[PADR_SET].adr[6:0],3'b0})
);

biu_dati_align uda2
(
	.dati(dati),
	.datis(datis2), 
	.amt({adr_o[6:0],3'b0})
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

rfPhoenix_stmask ustmsk1 (mem_resp[VLOOKUP3].sel, mem_resp[VLOOKUP3].adr[5:0], stmask);
always_comb
	dc_linein = (dc_line & ~stmask) | ((mem_resp[VLOOKUP3].res << {mem_resp[VLOOKUP3].adr[5:0],3'b0}) & stmask);

integer n10;
always_comb
	for (n10 = 0; n10 < NTHREADS; n10 = n10 + 1)
		rollback_bitmaps[n10] = rb_bitmaps1[n10]|rb_bitmaps2[n10]|rb_bitmaps3[n10]|rb_bitmaps4[n10];

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

lfsr17 #(.WID(17)) ulfsr1
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
  .rollback(rollback),
  .rollback_bitmaps(rb_bitmaps1)
);

wire memresp_full;
wire [3:0] fifoFromCtrl_cnt;
assign fifoFromCtrl_empty = fifoFromCtrl_cnt=='d0;

// This fifo is at the output of the external bus to the mainline execution.
// There are two places this fifo is loaded from.
// 1) at the end of an external bus access when required
// 2) at the end of the memory access pipeline if the cache was hit
// Responses from the memory access pipeline take precedence.

rfPhoenix_mem_resp_fifo uofifo1
(
	.rst(rst),
	.clk(clk),
	.wr(memresp.wr|memresp2.wr),
	.di(memresp.wr ? memresp : memresp2),
	.rd(fifoFromCtrl_rd),
	.dout(fifoFromCtrl_o),
	.cnt(fifoFromCtrl_cnt),
	.full(memresp_full),
	.v(fifoFromCtrl_v),
	.rollback(rollback),
	.rollback_bitmaps(rb_bitmaps3)
);

// This fifo sits between the output of the data cache lookup memory pipe and
// the external bus sequencer. 

reg rd_memq;
wire memq_empty;
wire [3:0] memq_cnt;
MemoryArg_t memq_o, memr;

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
	.empty(memq_empty),
	.v(memq_v),
	.rollback(rollback),
	.rollback_bitmaps(rb_bitmaps4)
);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Instruction cache
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ICacheLine ic_eline, ic_oline;
reg wr_ic1, wr_ic2;
reg [1:0] ic_rwaye,ic_rwayo,ic_wway;
reg icache_wre, icache_wro;
always_comb icache_wre = wr_ic2 && !upd_adr[5];
always_comb icache_wro = wr_ic2 &&  upd_adr[5];
reg ic_invline,ic_invall;
code_address_t ipo,ip2,ip3,ip4,ip5;
wire [$bits(code_address_t)-1:14] ictage [0:3];
wire [$bits(code_address_t)-1:14] ictago [0:3];
wire [1024/4-1:0] icvalide [0:3];
wire [1024/4-1:0] icvalido [0:3];
wb_address_t upd_adr = 'd0;

ICacheLine ici;		// Must be a multiple of 128 bits wide for shifting.
reg [2:0] ivcnt;
reg [2:0] vcn;
ICacheLine [4:0] ivcache;
reg [$bits(code_address_t)-1:14] ivtag [0:4];
reg [4:0] ivvalid;
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
	if (ip2[4:0] < 5'd22)
		ic_valid <= ip2[5] ? ic_valid2o : ic_valid2e;
	else
		ic_valid <= ic_valid2o & ic_valid2e;

// 256 wide x 1024 deep, 1 cycle read latency.
sram_256x1024_1r1w uicme
(
	.rst(rst),
	.clk(clk),
	.wr(icache_wre),
	.wadr({ic_wway,upd_adr[13:6]+upd_adr[5]}),
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
DCacheLine dci [0:1];
DCacheLine dc_eline, dc_oline;
DCacheLine dc_elin, dc_olin;
reg [1023:0] datil;
reg dcachable;
reg [1:0] dc_erway,prev_dc_erway;
reg [1:0] dc_orway,prev_dc_orway;
wire [1:0] dc_ewway;
wire [1:0] dc_owway;
reg [pL1DCacheWays-1:0] dcache_ewr, dcache_owr;
wire dc_ewr, dc_owr;
reg dc_invline,dc_invall;

sram_257x1024_1r1w udcme
(
	.rst(rst),
	.clk(clk),
	.wr(dc_ewr),
	.wadr({dc_ewway,dadr[13:6]+dadr[5]}),
	.radr({dc_erway,padrd1[13:6]+padrd1[5]}),
	.i(dci[0]),
	.o(dc_eline)
);

sram_257x1024_1r1w udcmo
(
	.rst(rst),
	.clk(clk),
	.wr(dc_owr),
	.wadr({dc_owway,dadr[13:6]}),
	.radr({dc_orway,padrd1[13:6]}),
	.i(dci[0]),
	.o(dc_oline)
);

always_ff @(posedge clk)
	case(padrd1[5])
	1'b0:	dc_line = {dc_oline.data,dc_eline.data};
	1'b1:	dc_line = {dc_eline.data,dc_oline.data};
	endcase
always_ff @(posedge clk)
	dc_line_mod = {dc_oline.m,dc_eline.m};

wire [$bits(Address)-1:14] dc_etag [3:0];
wire [255:0] dc_evalid [0:3];
wire [3:0] dhit1e;	// debugging
wire [3:0] dhit1o;
wire [$bits(Address)-1:14] dc_otag [3:0];
wire [255:0] dc_ovalid [0:3];
wire dhito,dhite;
reg [7:0] vadr2e;
always_comb
	vadr2e <= padr[13:6]+padr[5];
	
rfPhoenix_dchit #(.LINES(256)) udchite
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

rfPhoenix_dchit #(.LINES(256)) udchito
(
	.rst(rst),
	.clk(clk),
	.tags(dc_otag),
	.ndx(padr[13:6]),
	.adr(padr),
	.valid(dc_ovalid),
	.hits(dhit1o),
	.hit(dhito),
	.rway(dc_orway)
);

reg dhit;
always_ff @(posedge clk)
	dhit = (dhite & dhito) || (adr_o[5] ? (dhito && padrd1[4:0] < 5'd23) : (dhite && padrd1[4:0] < 5'd23));
reg dhite_d1, dhito_d1;
always_ff @(posedge clk)
	dhite_d1 <= dhite;
always_ff @(posedge clk)
	dhito_d1 <= dhito;

rfPhoenix_dctag
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
udcotag
(
	.clk(clk),
	.wr(state==DFETCH7 && adr_o[5]),
	.adr(adr_o),
	.way(lfsr_o[1:0]),
	.rclk(tlbclk),
	.ndx(padrd1[13:6]),
	.tag(dc_otag)
);

rfPhoenix_dctag
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
udcetag
(
	.clk(clk),
	.wr(state==DFETCH7 && ~adr_o[5]),
	.adr(adr_o),
	.way(lfsr_o[1:0]),
	.rclk(tlbclk),
	.ndx(padrd1[13:6]+padrd1[5]),
	.tag(dc_etag)
);

rfPhoenix_dcvalid
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
udcovalid
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4 && adr_o[5]),
	.dadr(adr_o),
	.adr(adr_o),
	.wr(state==DFETCH7 && adr_o[5]),
	.way(lfsr_o[1:0]),
	.invline(dc_invline),
	.invall(dc_invall),
	.valid(dc_ovalid)
);

rfPhoenix_dcvalid
#(
	.LINES(256),
	.WAYS(4),
	.LOBIT(6)
)
udcevalid
(
	.rst(rst),
	.clk(clk),
	.invce(state==MEMORY4 && ~adr_o[5]),
	.dadr(adr_o),
	.adr(adr_o),
	.wr(state==DFETCH7 && ~adr_o[5]),
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
	.eaeo(~memr.adr[5]),
	.daeo(~adr_o[5]),
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
	.eaeo(memr.adr[5]),
	.daeo(adr_o[5]),
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
	.eaeo(~memr.adr[5]),
	.daeo(~adr_o[5]),
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
	.eaeo(memr.adr[5]),
	.daeo(adr_o[5]),
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
  .sys_mode_i(seg_o==wishbone_pkg::CODE ? ~UserMode : ~MUserMode),
  .xlaten_i(xlaten),
  .we_i(we_o),
  .dadr_i(dadr),
  .next_i(inext),
  .iacc_i(mem_resp[0].v),//iaccess|daccess),
  .dacc_i(1'b0),
  .iadr_i(mem_resp[0].adr),
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

reg [4:0] mp_delay;
wire [3:0] region_at;
vtdl #(.WID($bits(PhysicalAddress)), .DEP(32)) umpd1 (.clk(clk), .ce(1'b1), .a(mp_delay), .d(padr), .q(padrd1));
vtdl #(.WID(4), .DEP(32)) umpd2 (.clk(clk), .ce(1'b1), .a(mp_delay), .d(region.at[3:0]), .q(region_at));

//always_ff @(posedge clk)	// delay for data tag lookup
//	padrd1 <= padr;
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
// Capture data and address
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed
{
	wb_address_t adr;
	wb_segment_t seg;
} req_table_t;

reg wr_reqtbl;
req_table_t [255:0] reqtbl;
req_table_t req;
always_ff @(posedge clk)
	if (wr_reqtbl)
		reqtbl[tid_o] <= {adr_o,seg_o};
assign req = reqtbl[tid_i];

always_ff @(posedge clk)
if (rst)
	ici.data <= 'd0;
else begin
	if (state==IFETCH4)
		ici.data <= ivcache[vcn];

	else if (ack_i && req.seg==wishbone_pkg::CODE) begin
		case(req.adr[4])
		1'd0:	ici.data[127:  0] <= dat_i;
		1'd1: ici.data[255:128] <= dat_i;
		default:	;
		endcase
		if (req.adr[4]=='d0)
			upd_adr <= {req.adr[$bits(wb_address_t)-1:5],5'h0};
	end
end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// State Machine
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

reg dfetch2,dstore1;
task tReset;
begin
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
	adr_o <= 'd0;
	dat <= 'd0;
	csr_o <= LOW;
	waycnt <= 2'd0;
	ic_wway <= 2'b00;
	dwait <= 3'd0;
	iaccess <= FALSE;
	daccess <= FALSE;
	dci[0] <= 'd0;
	dci[1] <= 'd0;
	memreq_rd <= FALSE;
	memresp <= 'd0;
	memresp2 <= 'd0;
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
	for (n = 0; n < NTHREADS; n = n + 1)
		rb_bitmaps2[n] <= 'd0;
	goto (MEMORY_INIT);
	dep <= 'd0;
	stk_dep <= 'd0;
	dcnt <= 'd0;
	mp_delay <= 'd0;
	wr_reqtbl <= 'd0;
	tid_cnt <= 'd0;
end
endtask

always_ff @(posedge clk)
if (rst) begin
	tReset();
end
else begin
	for (n = 0; n < NTHREADS; n = n + 1)
		if (rollback[n])
			rb_bitmaps2[n] <= 'd0;
	dcachable <= TRUE;
	inext <= FALSE;
//	memreq_rd <= FALSE;
	memresp.wr <= FALSE;
	memresp2.wr <= FALSE;
	tlbwr <= FALSE;
	tlb_ack <= FALSE;
	ptgram_wr <= FALSE;
	clr_ptg_fault <= 1'b0;
	if (clr_ipage_fault)
		ipage_fault <= 1'b0;
	if (clr_itlbmiss)
		itlbmiss <= 1'b0;
	wr_ic1 <= FALSE;
	wr_ic2 <= wr_ic1;
	if (ack_i && req.adr[4]==1'b1 && req.seg==wishbone_pkg::CODE)
		wr_ic1 <= TRUE;
	wr_reqtbl <= 'd0;

	mem_resp[DATA_ALN].wr <= FALSE;
	tlbwr <= FALSE;
	tlb_ack <= FALSE;
	ptgram_wr <= FALSE;
	tStage0();
	tStage1();
	tAddressXlat();
	tCacheAccess();
	tCacheDataAlign();

		for (n5 = 0; n5 < 7; n5 = n5 + 1)
			if (rollback[mem_resp[n5].thread]) begin
				mem_resp[n5].v <= 1'b0;
				rb_bitmaps2[mem_resp[n5].thread][mem_resp[n5].tgt] <= 1'b1;
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
			if (!memq_empty)
				rd_memq <= TRUE;
			if (rd_memq) begin
				if (memq_o.tid != last_tid) begin
					rd_memq <= FALSE;
					last_tid <= memq_o.tid;
					memr <= memq_o;
					memreq <= memq_o;
					dci[0].data <= memq_o.res[255:0];
					dci[1].data <= memq_o.res[511:256];
					dci[0].m <= 1'b0;
					dci[1].m <= 1'b0;
					gosub (MEMORY_ACTIVATE);
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
		  memresp.adr <= ea;
		  memresp.wr <= TRUE;
			memresp.res <= 128'd0;
		  ret();
		end
`endif
	MEMORY5: goto (MEMORY5a);
	MEMORY5a:		// Allow time for lookup
		goto (MEMORY_ACTIVATE);

	MEMORY_ACTIVATE:
		tMemoryActivate();

	MEMORY_ACK:
		tMemoryAck();

	MEMORY_NACK:
		tMemoryNack();
		
	MEMORY_UPD1:
		begin
			dci[0] <= dci[1].data;
			dci[1] <= dci[0].data;
			dci[0].m <= 1'b1;
			dci[1].m <= 1'b1;
			if (memr.hit==2'b11)
				goto (MEMORY_UPD2);
			else
				ret();
		end
	MEMORY_UPD2:
		ret();

	MEMORY8:
	  begin
    	goto (MEMORY9);
	    dadr <= {dadr[AWID-1:4] + 2'd1,4'd0};
			ptgram_adr <= memr.adr[18:4];
			rgn_adr <= memr.adr[9:4];
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
		  ic_wway <= waycnt;
			waycnt <= waycnt + 2'd1;
			ipo <= {memr.adr[$bits(Address)-1:5],5'b0};
			goto (IFETCH1);
			for (n = 0; n < 5; n = n + 1) begin
				if (ivtag[n]==memr.adr[$bits(Address)-1:5] && ivvalid[n]) begin
					vcn <= n;
		    	goto (IFETCH4);
	    	end
			end
		end
	// Hardware subroutine to fetch instruction cache line
	IFETCH1:
	  if (!next_i) begin
	  	// Cache miss, select an entry in the victim cache to
	  	// update.
	  	if (memr.sz!=nul) begin	// ic_valid flag
				ivcnt <= ivcnt + 2'd1;
				if (ivcnt>=3'd4)
					ivcnt <= 3'd0;
				ivcache[ivcnt] <= memr.res;
				ivtag[ivcnt] <= memr.vcadr[$bits(Address)-1:5];
				ivvalid[ivcnt] <= TRUE;
//				if (ic_line=='d0)
//					$stop;
			end
			icnt <= 'd0;
			seg_o <= wishbone_pkg::CODE;
			last_seg <= wishbone_pkg::CODE;
	  	bte_o <= wishbone_pkg::LINEAR;
	  	cti_o <= wishbone_pkg::INCR;
	    cyc_o <= HIGH;
			stb_o <= HIGH;
	    sel_o <= 16'hFFFF;
	    tid_o <= tid_cnt;
    	tid_cnt <= tid_cnt + 2'd1;
    	wr_reqtbl <= 1'b1;
	    case(memr.hit)
	    2'b00:		// need both even and odd cache lines (start with even)
	    	begin
					adr_o <= {memr.adr[$bits(Address)-1:6]+memr.adr[5],1'b0,5'h0};
					ipo <= {memr.adr[$bits(Address)-1:6]+memr.adr[5],1'b0,5'h0};
				end
	    2'b01:		// If got a hit on the even address, the odd one must be missing
	    	begin
					adr_o <= {memr.adr[$bits(Address)-1:6],1'b1,5'h0};
					ipo <= {memr.adr[$bits(Address)-1:6],1'b1,5'h0};
				end
			2'b10:		// Otherwise the even one must be missing
				begin
					adr_o <= {memr.adr[$bits(Address)-1:6]+memr.adr[5],1'b0,5'h0};
					ipo <= {memr.adr[$bits(Address)-1:6]+memr.adr[5],1'b0,5'h0};
				end
			2'b11:		// not missing lines, finished
				begin
					tDeactivateBus();
					ret();
				end
			endcase
  		goto (IFETCH2);
		end
	IFETCH2:
	  begin
	  	stb_o <= HIGH;
	    if (next_i) begin
	    	wr_reqtbl <= 1'b1;
	    	adr_o <= adr_o + 5'd16;
				seg_o <= wishbone_pkg::CODE;
	    	tid_o <= tid_cnt;
	    	tid_cnt <= tid_cnt + 2'd1;
	      icnt <= icnt + 4'd4;					// increment word count
	      if (icnt[4:2]==3'd0)
	      	cti_o <= wishbone_pkg::EOB;
	      if (icnt[4:2]==3'd1) begin		// Are we done?
	      	case(memr.hit)
	      	2'b00:	memr.hit <= 2'b01;
	      	2'b01:	memr.hit <= 2'b11;
	      	2'b10:	memr.hit <= 2'b11;
	      	2'b11:	memr.hit <= 2'b11;
	      	endcase
	      	tDeactivateBus();
	      	goto (IFETCH3);
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
		  if (memr.hit==2'b11)
		  	ret();
	  	else
	  		goto (IFETCH1);
		end
	
	IFETCH4:
		begin
			if (memr.sz!=nul) begin
				ivcache[vcn] <= memr.res;
				ivtag[vcn] <= memr.vcadr[$bits(Address)-1:5];
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
	  	seg_o <= wishbone_pkg::DATA;
			last_seg <= wishbone_pkg::DATA;
	  	bte_o <= wishbone_pkg::LINEAR;
	  	cti_o <= wishbone_pkg::FIXED;	// constant address burst cycle
	    cyc_o <= HIGH;
			stb_o <= HIGH;
	    sel_o <= 16'hFFFF;
	    goto (DFETCH5);
	    case(memr.hit)
	    2'b00:		// need both even and odd cache lines (start with even)
					adr_o <= {memr.adr[AWID-1:6]+memr.adr[5],1'b0,5'h0};
	    2'b01:		// If got a hit on the even address, the odd one must be missing
					adr_o <= {memr.adr[AWID-1:6],1'b1,5'h0};
			2'b10:		// Otherwise the even one must be missing
					adr_o <= {memr.adr[AWID-1:6]+memr.adr[5],1'b0,5'h0};
			2'b11:		// not missing lines, finished
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
	      dci[0].data <= {dat_i,dci[0].data[255:128]};
	      dci[0].m <= 1'b0;
	      if (dcnt[4:2]==3'd1) begin		// Are we done?
	      	case(memr.hit)
	      	2'b00:	memr.hit <= 2'b01;
	      	2'b01:	memr.hit <= 2'b11;
	      	2'b10:	memr.hit <= 2'b11;
	      	2'b11:	memr.hit <= 2'b11;
	      	endcase
	      	// Fill in missing memory data.
	      	case(memr.hit)
	      	2'b00:	memr.res[ 255:  0] <= {dat_i,dci[0].data[255:128]};
	      	2'b01:	memr.res[ 511:256] <= {dat_i,dci[0].data[255:128]};
	      	2'b10:	memr.res[ 255:  0] <= {dat_i,dci[0].data[255:128]};
	      	2'b11:	;
	      	endcase
	      	tDeactivateBus();
	      	goto (DFETCH7);
	    	end
	    	/*
	    	if (!bok_i) begin							// burst mode supported?
	    		cti_o <= wishbone_pkg::CLASSIC;						// no, use normal cycles
	    		goto (DFETCH6);
	    	end
	    	*/
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

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// This subroutine stores a data cache line for writeback cache.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	DSTORE1:
	  if (!ack_i) begin
			if (dstore1) begin
	  		dstore1 <= 1'b0;
				if (memr.adr[6])
					adr_o <= {memr.adr[AWID-1:6],1'b1,5'h0};
				else
					adr_o <= {memr.adr[AWID-1:6],1'b0,5'h0};
			end
	  	seg_o <= wishbone_pkg::DATA;
	  	bte_o <= wishbone_pkg::LINEAR;
	  	cti_o <= wishbone_pkg::CLASSIC;
	    cyc_o <= HIGH;
			stb_o <= HIGH;
  		sel_o <= 16'hFFFF;
			dat_o <= memr.res[127:0];
	    goto (DSTORE2);
	  end

	DSTORE2:
    if (ack_i)
  		goto (DSTORE3);
  
  // Increment address and bounce back for another write.
  DSTORE3:
		begin
			stb_o <= LOW;
			if (!ack_i)	begin							// wait till consumer ready
	    	dcnt <= dcnt + 4'd4;
				if (dcnt[4:2]==3'd3) begin
					memr.mod <= 2'b00;
					tDeactivateBus();
					ret();
				end
				else
					goto (DSTORE1);
				memr.res <= memr.res >> {5'd16,3'b0};
				adr_o <= adr_o + 5'd16;
			end
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
	  	seg_o <= wishbone_pkg::CODE;
	  	bte_o <= wishbone_pkg::LINEAR;
	  	cti_o <= wishbone_pkg::FIXED;	// constant address burst cycle
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
					/*
					case({dadr[4:3],sel_o})
					18'h000FF:	dci[].data[63:0] <= ptg[63:0];
					18'h0FF00:	dci[127:64] <= ptg[127:64];
					18'h100FF:	dci[191:128] <= ptg[191:128];
					18'h1FF00:	dci[255:192] <= ptg[255:192];
					18'h200FF:	dci[319:256] <= ptg[319:256];
					18'h2FF00:	dci[383:320] <= ptg[383:320];
					18'h300FF:	dci[447:384] <= ptg[447:384];
					18'h3FF00:	dci[511:448] <= ptg[511:448];
					default:		dci <= dc_line;
					endcase
					*/
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
			    		cti_o <= wishbone_pkg::CLASSIC;						// no, use normal cycles
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
				seg_o <= wishbone_pkg::DATA;
		  	bte_o <= wishbone_pkg::LINEAR;
		  	cti_o <= wishbone_pkg::FIXED;	// constant address burst cycle
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
			seg_o <= wishbone_pkg::DATA;
	  	bte_o <= wishbone_pkg::LINEAR;
	  	cti_o <= wishbone_plg::FIXED;	// constant address burst cycle
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

wire mem_pipe_adv = !memresp_full;

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
		if (mem_pipe_adv) begin
			if (tlbrdy) begin
				if (tlb_cyc) begin
					mem_resp[0].func <= MR_TLB;
					mem_resp[0].adr <= {tlb_adr[AWID-1:5],5'h0} + 5'd16;
					rb_bitmaps2[imemreq.thread][imemreq.tgt] <= 1'b1;
				end
				else if (fifoToCtrl_v) begin
					xlaten <= TRUE;
					mem_resp[0] <= imemreq;
					mem_resp[0].v <= 1'b1;
					rb_bitmaps2[imemreq.thread][imemreq.tgt] <= 1'b1;
				end
			end
		end
		// Hold onto the request if pipe could not advance.
		else
			mem_resp[0] <= mem_resp[0];
	end
	if (mem_pipe_adv)
		mp_delay <= 4'd0;
	else
		mp_delay <= mp_delay + 2'd1;
end
endtask

task tStage1;
begin
	if (mem_pipe_adv) begin
	tlb_access <= 1'b0;
	rgn_en <= 1'b0;
	ptgram_en <= 1'b0;
	mem_resp[1] <= mem_resp[0];
	mem_resp[1].cause <= FLT_NONE;
	if (mem_resp[0].func==MR_CACHE) begin
		ic_invline <= mem_resp[0].res[2:0]==3'd1;
		ic_invall	<= mem_resp[0].res[2:0]==3'd2;
		dc_invline <= mem_resp[0].res[5:3]==3'd3;
		dc_invall	<= mem_resp[0].res[5:3]==3'd4;
		if (mem_resp[0].res[5:3]==3'd1)
			dce <= TRUE;
		if (mem_resp[0].res[4:2]==3'd2)
			dce <= FALSE;
    mem_resp[1].cmt <= TRUE;
		mem_resp[1].wr <= TRUE;
		mem_resp[1].res <= 'd0;
	end
	else if ((mem_resp[0].func==MR_LOAD || mem_resp[0].func==MR_LOADZ || mem_resp[0].func==MR_STORE || 
		mem_resp[0].func==MR_TLBRD || mem_resp[0].func==MR_TLBRW ||
		mem_resp[0].func==MR_ICACHE_LOAD) && mem_resp[0].v) begin
    mem_resp[1].sel <= {32'h0,mem_resp[0].sel} << mem_resp[0].adr[3:0];
		casez(mem_resp[0].adr)
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
				tlb_ia <= mem_resp[0].adr[15:0];
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
		rgn_adr <= mem_resp[0].adr[9:4];
		rgn_dat <= mem_resp[0].res;
`ifdef SUPPORT_HASHPT
		ptgram_adr <= mem_resp[0].adr[18:4];
		ptgram_dati <= mem_resp[0].res;
`endif
	end
	end
end
endtask

// Perform virtual to physical translation.
// Perform PMA checks on physical address

task tAddressXlat;
begin
	if (mem_pipe_adv) begin
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
			mem_resp[PADR_SET].adr <= padrd1;
		end
		else
			mem_resp[PADR_SET].acr <= region_at[3:0];
		if (mem_resp[VLOOKUP3].v) begin
		  if (!region_at[0] && mem_resp[VLOOKUP3].func==MR_ICACHE_LOAD)
		    mem_resp[PADR_SET].cause <= FLT_PMA;
		 	//we_o <= wr & tlbwr & region.at[1];
		  if (mem_resp[VLOOKUP3].func==MR_STORE && !region_at[1])
			  mem_resp[PADR_SET].cause <= FLT_WRV;
		  else if (mem_resp[VLOOKUP3].func!=MR_STORE && !region_at[2])
			  mem_resp[PADR_SET].cause <= FLT_RDV;
		   // TLB miss has higher precedence than PMA
		   // No TLB miss in machine mode
			if (tlbmiss && mem_resp[VLOOKUP3].omode!=2'd3)
				mem_resp[PADR_SET].cause <= FLT_TLBMISS;
		 	if (!tlbacr[2] && (mem_resp[VLOOKUP3].func==MR_LOAD || mem_resp[VLOOKUP3].func==MR_LOADZ)) begin
		 		mem_resp[PADR_SET].cause <= FLT_RDV;
		 		//tReadViolation(mem_resp[4].adr);
	//		if (tlbacr[3])
	//			mem_resp[PADR_SET].func2 <= MR_CACHE;
		 	end
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
	if (mem_pipe_adv) begin
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
			mem_resp[VLOOKUP3].tlb_access:	begin mem_resp[PADR_SET].res <= tlbdato; mem_resp[PADR_SET].wr <= TRUE; end
			mem_resp[VLOOKUP3].ptgram_en:		begin mem_resp[PADR_SET].res <= ptgram_dato; mem_resp[PADR_SET].wr <= TRUE; end
			mem_resp[VLOOKUP3].rgn_en:			begin mem_resp[PADR_SET].res <= rgn_dat_o; mem_resp[PADR_SET].wr <= TRUE; end
			default:		
				begin
					mem_resp[PADR_SET].res <= dc_line;
					mem_resp[PADR_SET].wr <= !(dce & dhit & mem_resp[VLOOKUP3].acr[3]);
					// Allow cache hit to set hit to one, but not zero. hit may have been
					// one coming in if the cache line is not needed.
					if (dhite_d1)
						mem_resp[PADR_SET].hit[0] <= 1'b1;
					if (dhito_d1)
						mem_resp[PADR_SET].hit[1] <= 1'b1;
					mem_resp[PADR_SET].mod <= dc_line_mod;
					//if (!(dce & dhit & mem_resp[VLOOKUP3].acr[3]))
					//	mem_resp[PADR_SET].cause <= FLT_DCM;
				end
		  endcase
		MR_STORE:	
			begin
				mem_resp[PADR_SET].wr <= TRUE;
				/* Might want this check at some point.
				case(mem_resp[VLOOKUP3].sz)
				byt:	;	// Cant be unaligned
				wyde:	if (mem_resp[VLOOKUP3].adr[5:0] > 6'd62)	mem_resp[PADR_SET].cause <= FLT_ALN;
				tetra:if (mem_resp[VLOOKUP3].adr[5:0] > 6'd60)	mem_resp[PADR_SET].cause <= FLT_ALN;
				default:	if (mem_resp[VLOOKUP3].adr[5:0] > 6'd60)	mem_resp[PADR_SET].cause <= FLT_ALN;
				endcase
				*/
	 			mem_resp[PADR_SET].res <= dc_linein;	// Calculated above
	 			mem_resp[PADR_SET].mod <= {dhito_d1,dhite_d1};	// Only the lines that were hit are being modified.
			end
		default:	;
		endcase
		end
	end
end
endtask

// Align the data and send it back
task tCacheDataAlign;
begin
	if (mem_pipe_adv) begin
		rb_bitmaps2[mem_resp[DATA_ALN].thread][mem_resp[DATA_ALN].tgt] <= 1'b0;
		mem_resp[DATA_ALN] <= mem_resp[PADR_SET];
		if (mem_resp[PADR_SET].func!=MR_ICACHE_LOAD || last_cadr != mem_resp[PADR_SET].adr) begin
			if (mem_resp[PADR_SET].func==MR_ICACHE_LOAD)
				last_cadr <= mem_resp[PADR_SET].adr;
			mem_resp[DATA_ALN].wr <= mem_resp[PADR_SET].wr & mem_resp[PADR_SET].v;
		end
		else
			mem_resp[DATA_ALN].wr <= 1'b0;
		if (mem_resp[PADR_SET].v) begin
			// A response will be sent back here only on a load when there is a cache hit.
			// Otherwise the memory sequencer is needed.
			case(mem_resp[PADR_SET].func)
			MR_STORE:					memresp.wr <= FALSE;
			MR_LOAD,MR_LOADZ:	memresp.wr <= ~mem_resp[PADR_SET].wr;
			MR_TLBRW,MR_TLBRD:	memresp.wr <= TRUE;
			MR_ICACHE_LOAD:		memresp.wr <= TRUE;
			default:	memresp.wr <= FALSE;
			endcase
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
end
endtask

task tMemoryActivate;
begin
	dfetch2 <= 1'b0;
	dstore1 <= 1'b0;
	strips <= 2'd0;
	dcnt <= 'd0;
	// Detect cache controller commands
	case(memr.func)
	MR_STORE,MR_MOVST:
		begin
`ifdef SUPPORT_HWWALK    		
			// Invalidate PTCEs when a store occurs to the PDE
			for (n4 = 0; n4 < 12; n4 = n4 + 1)
				if (ptc[n4].pde.padr[AWID-1:4]==memr.adr[AWID-1:4])
					ptc[n4].v <= 1'b0;
`endif						
			seg_o <= wishbone_pkg::DATA;
	  	bte_o <= wishbone_pkg::LINEAR;
			cti_o <= wishbone_pkg::CLASSIC;
			cyc_o <= HIGH;
			stb_o <= HIGH;
			we_o <= HIGH;
			sel_o <= memr.sel[15:0];
			if (stk_dep=='d1)
	  		adr_o <= {memr.adr[31:4],4'd0};
	  	else
	  		adr_o <= adr_o + 5'd16;
	  	dat_o <= memr.res;
	  	dat <= memr.res;
			csr_o <= memr.func2==MR_STC;
  		goto (MEMORY_ACK);
		end
	MR_ICACHE_LOAD:
		goto (IFETCH0);
	MR_LOAD,MR_LOADZ:
		begin
			// It was cachable data and a miss occurred. Fetch the data and return
			// a miss status to the execute unit so it will try again.
			// If the line was modified, write it out first.
			if (memr.acr[3]) begin
				if (|memr.mod)
					gosub(DSTORE1);
				else
					goto (DFETCH2);
			end
			else begin
				seg_o <= wishbone_pkg::DATA;
		  	bte_o <= wishbone_pkg::LINEAR;
  			cti_o <= wishbone_pkg::CLASSIC;
  			cyc_o <= HIGH;
  			stb_o <= HIGH;
  			we_o <= LOW;
  			sel_o <= memr.sel[15:0];
				if (stk_dep=='d1)
		  		adr_o <= {memr.adr[31:4],4'd0};
		  	else
		  		adr_o <= adr_o + 5'd16;
   			csr_o <= memr.func2==MR_LDR;
	  		goto (MEMORY_ACK);
			end
		end
	default:	ret();	// unknown operation
	endcase
end
endtask

task tMemoryAck;
begin
	case(memr.func)
	MR_STORE,MR_MOVST:
		if (ack_i || !stb_o) begin
		  goto (MEMORY_NACK);
      stb_o <= LOW;
    end
  MR_LOAD,MR_LOADZ:
    if (ack_i || !stb_o) begin
      goto (MEMORY_NACK);
      stb_o <= LOW;
    end
  default:	ret();
	endcase
end
endtask

task tMemoryNack;
begin
  if (~ack_i) begin
   	memr.sel <= memr.sel >> 16;
    case(memr.func)
    MR_LOAD,MR_LOADZ,MR_MOVLD:
    	begin
	    	case(adr_o[6:4])
	    	3'd0:	dati[127:  0] <= dat_i;
	    	3'd1:	dati[255:128] <= dat_i;
	    	3'd2:	dati[383:256] <= dat_i;
	    	3'd3:	dati[511:384] <= dat_i;
	    	3'd4:	dati[639:512] <= dat_i;
	    	3'd5:	dati[767:640] <= dat_i;
	    	3'd6: dati[895:768] <= dat_i;
	    	3'd7: dati[1023:896] <= dat_i;
	    	default:	;
	    	endcase
		    if (|memr.sel[31:16]) begin
	  	    gosub (MEMORY_ACTIVATE);
	  	  end
	  	  else begin
	  	  	if (memr.sel[127:16]=='d0) begin
    				tDeactivateBus();
	        	goto (DATA_ALIGN);
	        end
	      end
    	end
    MR_STORE,MR_MOVST:
    	begin
		    if (|sel[31:16]) begin
      		memr.res <= memr.res >> 128;
		    	gosub (MEMORY_ACTIVATE);
			  end
			  else begin
			  	if (memr.sel[127:16]=='d0) begin
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
		    			tDeactivateBus();
	  					memresp.cause <= FLT_NONE;
			  			memresp.step <= memreq.step;
				    	memresp.cmt <= TRUE;
			  			memresp.tid <= memreq.tid;
			  			memresp.wr <= TRUE;
							memresp.res <= {127'd0,rb_i};
							if (!memresp_full) begin
								if (|memr.hit[1:0]) begin
									if (memr.adr[6])
										dci <= {dci[0],dci[1]};
									goto (MEMORY_UPD1);
								end
								else
									ret();
							end
			      end
		    	end
	    	end
    	end
    default:
    	begin
    		ret();
    	end
    endcase
  end
end
endtask

task tMemoryActivateHi;
begin
`ifndef SUPPORT_HASHPT
  dwait <= 3'd0;
  memr.adr[6] <= ~memr.adr[6];
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
			seg_o <= wishbone_pkg::DATA;
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

task tDataAlign;
begin
	memresp2.cause <= FLT_NONE;
	tDeactivateBus();
	if ((memr.func==MR_LOAD || memr.func==MR_LOADZ || memr.func==MR_MOVLD) & memr.sz!=2'b11 & dcachable & memr.acr[3] & dce &
	 	~memr.ptgram_en & ~memr.rgn_en & ~memr.tlb_access) begin
		memresp2.cause <= FLT_DCM;
	end
	else if (memreq.func==MR_MOVLD) begin
		ret();
	end
	else if (!memresp_full) begin
		// Find an open spot
		case(mem_resp[PADR_SET].func)
		MR_STORE:					ret();
		MR_LOAD,MR_LOADZ:	if (!mem_resp[PADR_SET].v) ret();
		MR_TLBRW,MR_TLBRD:	ret();
		MR_ICACHE_LOAD:		ret();
		default:	if (!mem_resp[PADR_SET].v) ret();
		endcase
	end
	if (memreq.func2==MR_LDG) begin
		if (memreq.step == NLANES-1) begin
			memresp2.wr <= TRUE;
		end
		memresp2.res[memreq.count] <= datis2[63:0];
		memresp2.step <= memreq.step;
		memresp2.tid <= memreq.tid;
	end
	else begin
		memresp2.step <= memreq.step;
	  memresp2.cmt <= TRUE;
		memresp2.tid <= memreq.tid;
		memresp2.wr <= TRUE;
	end
	csr_o <= LOW;
  case(memr.func)
  MR_LOAD,MR_MOVLD:
  	begin
  		if (memr.func2==MR_LDV)
  			memresp2.res <= dati >> {memresp.adr[6:0],3'b0};
  		else
	    	case(memr.sz)
	    	nul:	memresp2.res[memr.step] <= 'h0;
	    	byt:	begin memresp2.res[memr.step] <= {{56{datis2[7]}},datis2[7:0]}; end
	    	wyde:	begin memresp2.res[memr.step] <= {{48{datis2[15]}},datis2[15:0]}; end
	    	tetra:	begin memresp2.res[memr.step] <= {{32{datis2[31]}},datis2[31:0]}; end
	    	octa:	begin memresp2.res[memr.step] <= {{64{datis2[63]}},datis2[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp2.res[memr.step] <= memr.res;
	    	endcase
  	end
  MR_LOADZ:
  	begin
  		if (memr.func2==MR_LDV)
  			memresp2.res <= dati >> {memresp.adr[6:0],3'b0};
  		else
	    	case(memr.sz)
	    	nul:	memresp2.res[memr.step] <= 'h0;
	    	byt:	begin memresp2.res[memr.step] <= {56'd0,datis2[7:0]}; end
	    	wyde:	begin memresp2.res[memr.step] <= {48'd0,datis2[15:0]}; end
	    	tetra:	begin memresp2.res[memr.step] <= {32'd0,datis2[31:0]}; end
	    	octa:	begin memresp2.res[memr.step] <= {64'd0,datis2[63:0]}; end
	//    	hexi:	begin memresp.res <= datis[127:0]; end
	//    	hexipair:	memresp.res <= dati;
	//    	hexiquad:	begin memresp.res <= dati512; end
	    	default:	memresp2.res[memr.step] <= memr.res;
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
input MemoryArg_t req;
input MemoryArg_t oresp;
output MemoryArg_t resp;
begin
	//tDeactivateBus();
	resp <= oresp;
	resp.step <= req.step;
	resp.cmt <= TRUE;
  resp.cause <= FLT_TLBMISS;
	resp.tid <= req.tid;
  resp.adr <= req.adr;
  resp.wr <= TRUE;
	resp.res <= 'd0;
end
endtask

// Page faults occur only during hardware page table walks when a translation
// cannot be found.

task tPageFault;
input cause_code_t typ;
input Address ba;
begin
	memresp.step <= memreq.step;
	memresp.cmt <= TRUE;
  memresp.cause <= typ;
	memresp.tid <= memreq.tid;
  memresp.adr <= ba;
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
  memresp.adr <= ba;
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
  memresp.adr <= ba;
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
  memresp.adr <= ba;
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
  		io_keys[adr_o[12:2]] <= memreq.res[19:0];
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
	seg_o <= wishbone_pkg::DATA;
	cti_o <= wishbone_pkg::CLASSIC;	// Normal cycles again
	cyc_o <= LOW;
	stb_o <= LOW;
	we_o <= LOW;
	sel_o <= 16'h0000;
	csr_o <= LOW;
  xlaten <= FALSE;
end
endtask

task tPushBus;
begin
	xlaten_stk <= xlaten;
	seg_stk <= seg_o;
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
	seg_o <= seg_stk;
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
	stk_state[stk_dep] <= rst;
	stk_dep <= stk_dep+2'd1;
	state <= nst;
end
endtask

task gosub;
input [6:0] nst;
begin
	stk_state[stk_dep] <= state;
	stk_dep <= stk_dep+2'd1;
	state <= nst;
end
endtask

task ret;
integer n;
begin
	state <= stk_state[stk_dep-2'd1];
	stk_dep <= stk_dep - 2'd1;
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
