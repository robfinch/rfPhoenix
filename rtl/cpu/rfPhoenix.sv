`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
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

module rfPhoenix(hartid_i, rst_i, clk_i, clk2x_i, clk2d_i, wc_clk_i, clock,
		nmi_i, irq_i, icause_i,
		vpa_o, vda_o, bte_o, cti_o, bok_i, cyc_o, stb_o, lock_o, ack_i,
    err_i, we_o, sel_o, adr_o, dat_i, dat_o, cr_o, sr_o, rb_i, state_o, trigger_o,
    wcause);
input [31:0] hartid_i;
input rst_i;
input clk_i;
input clk2x_i;
input clk2d_i;
input wc_clk_i;
input clock;					// MMU clock algorithm
input nmi_i;
(* MARK_DEBUG="TRUE" *)
input [2:0] irq_i;
(* MARK_DEBUG="TRUE" *)
input [8:0] icause_i;
output vpa_o;
output vda_o;
output [1:0] bte_o;
output [2:0] cti_o;
input bok_i;
output cyc_o;
output stb_o;
output reg lock_o;
input ack_i;
input err_i;
output we_o;
output [15:0] sel_o;
output [31:0] adr_o;
input [127:0] dat_i;
output [127:0] dat_o;
output cr_o;
output sr_o;
input rb_i;
output [5:0] state_o;
output reg trigger_o;
output CauseCode wcause;

parameter IC_LATENCY = 2;

integer n10;

wire clk_g = clk_i;
ExecuteBuffer [NTHREADS-1:0] dcb, rfb1, rfb2, exb, agb, oub, wbb;
ExecuteBuffer ebq [0:NTHREADS-1];

reg [NTHREADS-1:0] gie;
reg [7:0] vl = 8'd8;
Tid xrid,mc_rid,mc_rid1,mc_rid2,mc_rido;
wire dcndx_v,exndx_v,wbndx_v;
wire agndx_v,oundx_v;
reg itndx_v;
reg rfndx1_v, rfndx2_v;
Tid dcndx,itndx,exndx,oundx,wbndx,rfndx1,rfndx2,agndx;
reg xrid_v,mcrid_v;
Tid mcv_ridi, mcv_rido;
Tid ithread, dthread, xthread, commit_thread;
reg dcndx_v, dthread_v;
reg [3:0] commit_wr;
reg commit_wrv;
reg [63:0] commit_mask;
Regspec commit_tgt;
VecValue commit_bus;
Tid ip_thread, ip_thread1, ip_thread2, ip_thread3, ip_thread4, ip_thread5;
reg ip_thread_v,ip_thread1_v, ip_thread2_v, ip_thread3_v;
ThreadInfo_t [NTHREADS-1:0] thread;
ThreadInfo_t [NTHREADS-1:0] thread_hist [0:3];
CodeAddress [NTHREADS-1:0] thread_ip;
reg [31:0] ip, ip2, ip3, ip4;
reg [NTHREADS-1:0] thread_busy;
CodeAddress iip, dip, ip_icline, ip_insn, ip1;
Instruction ir,dir,xir,mir,insn,mir1,mir2;
Instruction rf_insn;
Postfix pfx,irpfx,rf_pfx;
DecodeBus deco;
DecodeBus [NTHREADS-1:0] dco;
Regspec ra0,ra1,ra2,ra3,ra4;
Value rfo0, rfo1, rfo2, rfo3, rfo4;
Value ximm,mcimm;
ASID xasid;
VecValue vrfo0, vrfo1, vrfo2, vrfo3, vrfo4;
VecValue xa,xb,xc,xt,xm;
reg xta,xtb,xtt;
VecValue mca,mcb,mcc,mct,mcm;
VecValue mca1,mcb1,mcc1,mct1,mcm1;
VecValue mca2,mcb2,mcc2,mct2,mcm2;
reg [2:0] mca_busy;
Value res,mc_res,mc_res1,mc_res2;
VecValue vres,mc_vres,mc_vres1,mc_vres2;
wire mc_done, mcv_done;
wire mc_done1, mcv_done1;
wire mc_done2, mcv_done2;
wire ihit,ihite,ihito;
reg ihit2,ihite2,ihito2;
MemoryArg_t memreq;
MemoryArg_t memresp;
wire memreq_full;
reg memresp_fifo_rd;
wire memresp_fifo_empty;
wire memresp_fifo_v;
wire [pL1ICacheLineSize:0] ic_line;
reg [pL1ICacheLineSize:0] ic_line2;
wire ic_valid;
//reg [31:0] ptbr;
wire ipage_fault;
reg clr_ipage_fault;
wire itlbmiss;
reg clr_itlbmiss;
wire dce;
//reg [9:0] asid;
reg [1:0] omode [0:NTHREADS-1];
reg [NTHREADS-1:0] Usermode;
reg [NTHREADS-1:0] MUsermode;
wire takb;
wire [NTHREADS-1:0] ififo_almost_full;
reg [2:0] sp_sel [0:NTHREADS-1];
reg [2:0] sp_sel2;
reg [2:0] ic_sp_sel [0:NTHREADS-1];
reg [2:0] istk_depth [0:NTHREADS-1];
Instruction [NTHREADS-1:0] exc_bucket;
CodeAddress [NTHREADS-1:0] exc_ip;
CauseCode icause,dcause;
reg [7:0] tid;
CodeAddress last_adr;
reg [15:0] exv;
wire [25:0] ic_tag;
reg [25:0] ic_tag2;
ExecuteBuffer [NTHREADS-1:0] dcbuf;
ExecuteBuffer [NTHREADS-1:0] mceb;
reg [NTHREADS-1:0] mem_rollback, ou_rollback, rollback, rolledback1, rolledback2;
regs_bitmap_t mem_rollback_bitmaps [0:NTHREADS-1];
regs_bitmap_t ou_rollback_bitmaps [0:NTHREADS-1];
regs_bitmap_t rollback_bitmaps [0:NTHREADS-1];
CodeAddress [NTHREADS-1:0] rollback_ip;
reg [NTHREADS-1:0] rollback_ipv;
reg [NTHREADS-1:0] sb_will_issue, sb_issue;
wire [NTHREADS-1:0] sb_can_issue;
reg [NTHREADS-1:0] clr_ififo;
wire [NTHREADS-1:0] ififo_empty;
InstructionFetchbuf ic_ifb;
InstructionFetchbuf [NTHREADS-1:0] dc_ifb, dc1_ifb;
reg [5:0] imiss_count;
reg [NTHREADS-1:0] ou_stall;

// CSRs
reg [31:0] cr0;
reg [31:0] ptbr [0:NTHREADS-1];
reg [9:0] asid [0:NTHREADS-1];
reg [31:0] hmask,xhmask;
Address badaddr [0:NTHREADS-1][0:3];
CauseCode cause [0:NTHREADS-1][0:3];
CodeAddress tvec [0:3];
reg [63:0] plStack [0:NTHREADS-1];
reg [255:0] ipStack [0:NTHREADS-1];
status_reg_t [7:0] status [0:NTHREADS-1];
reg [2:0] ipl [0:NTHREADS-1];
reg [NTHREADS-1:0] mprv;
reg [NTHREADS-1:0] uie;
reg [NTHREADS-1:0] sie;
reg [NTHREADS-1:0] hie;
reg [NTHREADS-1:0] mie;
reg [NTHREADS-1:0] die;
reg [NTHREADS-1:0] trace_en;
integer n11;
always_comb
	for (n11 = 0; n11 < NTHREADS; n11 = n11 + 1) begin
		mprv[n11] = status[n11][0].mprv;
		uie[n11] = status[n11][0].uie;
		sie[n11] = status[n11][0].sie;
		hie[n11] = status[n11][0].hie;
		mie[n11] = status[n11][0].mie;
		die[n11] = status[n11][0].die;
		omode[n11] = status[n11][0].om;
		trace_en[n11] = status[n11][0].trace_en;
		ipl[n11] = status[n11][0].ipl;
	end
reg [31:0] tick;
reg [31:0] retired;
reg [63:0] wc_time;
reg [31:0] wc_time_dat;
reg ld_time, clr_wc_time_irq;
reg [31:0] dbg_cr;
reg [31:0] dbg_sr;
Address [3:0] dbg_adr;
Address [3:0] dbg_am;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Trace
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
reg wr_trace, rd_trace;
wire [10:0] trace_count;
wire trace_full;
wire trace_empty;
wire trace_valid;
reg tron;
wire [3:0] trace_match;
assign trace_match[0] = ((dbg_adr[0]|dbg_am[0])==(ic_ifb.ip|dbg_am[0]) && dbg_cr[17:16]==4'b00 && dbg_cr[24] && status[ic_ifb.thread][0].trace_en);
assign trace_match[1] = ((dbg_adr[1]|dbg_am[1])==(ic_ifb.ip|dbg_am[1]) && dbg_cr[19:18]==4'b00 && dbg_cr[25] && status[ic_ifb.thread][0].trace_en);
assign trace_match[2] = ((dbg_adr[2]|dbg_am[2])==(ic_ifb.ip|dbg_am[2]) && dbg_cr[21:20]==4'b00 && dbg_cr[26] && status[ic_ifb.thread][0].trace_en);
assign trace_match[3] = ((dbg_adr[3]|dbg_am[3])==(ic_ifb.ip|dbg_am[3]) && dbg_cr[23:22]==4'b00 && dbg_cr[27] && status[ic_ifb.thread][0].trace_en);
wire trace_on = 
  trace_match[0] ||
  trace_match[1] ||
  trace_match[2] ||
  trace_match[3]
  ;
wire trace_off = trace_full;
//wire trace_compress = dbcr[36];

always @(posedge clk_g)
if (rst_i) begin
  wr_trace <= 1'b0;
  tron <= FALSE;
end
else begin
  if (trace_off)
    tron <= FALSE;
  else if (trace_on) begin
    tron <= TRUE;
    wr_trace <= 1'b1;
  end
  wr_trace <= 1'b0;
  if (tron)
    wr_trace <= 1'b1;
end

TraceFifo utf1 (
  .clk(clk_g),                // input wire clk
  .srst(rst_i),              // input wire srst
  .din(ic_ifb.ip),                // input wire [31 : 0] din
  .wr_en(wr_trace & ic_ifb.v), // input wire wr_en
  .rd_en(rd_trace),            // input wire rd_en
  .dout(trace_dout),              // output wire [31 : 0] dout
  .full(trace_full),              // output wire full
  .empty(trace_empty),            // output wire empty
  .valid(trace_valid),            // output wire valid
  .data_count(trace_count)  // output wire [10 : 0] data_count
);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

genvar g;

function [5:0] fnSpSel;
input [3:0] thread;
input [5:0] i;
begin
	if (i==6'd31)
		case(sp_sel[thread])
		3'd1:	fnSpSel = 6'd52;
		3'd2:	fnSpSel = 6'd53;
		3'd3:	fnSpSel = 6'd54;
		3'd4:	fnSpSel = 6'd55;
		default:	fnSpSel = 6'd31;
		endcase
	else
		fnSpSel = i;
end
endfunction


wire [16:0] lfsr_o;
lfsr ulfs1
(	
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.cyc(1'b0),
	.o(lfsr_o)
);

integer n5;
always_comb
	for (n5 = 0; n5 < NTHREADS; n5 = n5 + 1) begin
		Usermode[n5] = omode[n5]==2'b00;
		MUsermode[n5] = omode[n5]==2'b00;
	end

integer n;
initial begin
	tid = 8'd1;
	gie = 'd0;
	ip = RSTIP;
	iip = RSTIP;
	ir = NOP;//_INSN;
	xir = NOP;//_INSN;
	mir = NOP;//_INSN;
	for (n = 0; n < NLANES; n = n + 1) begin
		xa[n] = 'd0;
		xb[n] = 'd0;
		xc[n] = 'd0;
		mca[n] = 'd0;
		mcb[n] = 'd0;
		mcc[n] = 'd0;
	end
	ximm = 'd0;
	mcimm = 'd0;
	ithread = 'd0;
	ip_thread = 'd0;
	mca_busy = 'd0;
	thread_busy = 'd0;
	for (n = 0; n < NTHREADS; n = n + 1)
		exb[n] = 'd0;
	dthread_v = 'd0;
	memreq = 'd0;
end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

wire memreq_wack;

rfPhoenix_biu ubiu
(
	.rst(rst_i),
	.clk(clk_g),
	.tlbclk(clk2x_i),
	.clock(clock),
	.UserMode(Usermode[0]),	// fix these
	.MUserMode(MUsermode[0]),
	.omode(omode[0]),
	.bounds_chk(),
	.pe(pe),
	.ip(ip),
	.ip_o(ip_icline),
	.ihit(ihit),
	.ihite(ihite),
	.ihito(ihito),
	.ifStall(1'b0),
	.ic_line(ic_line),
	.ic_valid(ic_valid),
	.ic_tag(ic_tag),
	.fifoToCtrl_i(memreq),
	.fifoToCtrl_full_o(memreq_full),
	.fifoToCtrl_wack(memreq_wack),
	.fifoFromCtrl_o(memresp),
	.fifoFromCtrl_rd(memresp_fifo_rd),
	.fifoFromCtrl_empty(memresp_fifo_empty),
	.fifoFromCtrl_v(memresp_fifo_v),
	.bok_i(bok_i),
	.bte_o(bte_o),
	.cti_o(cti_o),
	.vpa_o(vpa_o),
	.vda_o(vda_o),
	.cyc_o(cyc_o),
	.stb_o(stb_o),
	.ack_i(ack_i),
	.we_o(we_o),
	.sel_o(sel_o),
	.adr_o(adr_o),
	.dat_i(dat_i),
	.dat_o(dat_o),
	.sr_o(sr_o),
	.cr_o(cr_o),
	.rb_i(rb_i),
	.dce(dce),
	.keys(),//keys),
	.arange(),
	.ptbr(ptbr[0]),
	.ipage_fault(ipage_fault),
	.clr_ipage_fault(clr_ipage_fault),
	.itlbmiss(itlbmiss),
	.clr_itlbmiss(clr_itlbmiss),
	.rollback(mem_rollback),
	.rollback_bitmaps(mem_rollback_bitmaps)
);


rfPhoenix_decoder udec1
(
	.ifb(ic_ifb),
	.sp_sel(ic_ifb.sp_sel),
	.deco(deco)
);


wire [3:0] mrt, ort;
ffo12 ufo9 (.i({11'd0,mem_rollback}), .o(mrt));
ffo12 ufo10 (.i({11'd0,ou_rollback}), .o(ort));
assign mem_rollback_thread = mrt;
assign ou_rollback_thread = ort;

generate begin : gScoreboard
for (g = 0; g < NTHREADS; g = g + 1) begin
	always_comb
	begin
		rollback[g] <= 'd0;
		rollback_bitmaps[g] <= 'd0;
		if (mem_rollback[g]) begin
			rollback[g] <= 1'b1;
			rollback_bitmaps[g] <= mem_rollback_bitmaps[g];
		end
		else if (ou_rollback[g]) begin
			rollback[g] <= 1'b1;
			rollback_bitmaps[g] <= ou_rollback_bitmaps[g];
		end
	end

	rfPhoenix_scoreboard uscb1
	(
		.rst(rst_i),
		.clk(clk_g),
		.db(dco[g]),
//		.wb_v((exb[g].dec.rfwr|exb[g].dec.vrfwr) && wbndx==g && wbndx_v),
//		.wb_Rt(exb[g].dec.Rt),
		.wb_v(((|commit_wr & ~commit_tgt.vec) | (commit_wrv & commit_tgt.vec)) && commit_thread==g),
		.wb_Rt(commit_tgt),
		.will_issue(sb_will_issue[g]),
		.can_issue(sb_can_issue[g]),
		.rollback(rollback[g]),
		.rollback_bitmap(rollback_bitmaps[g])
	);

end
end
endgenerate
always_ff @(posedge clk_g)
	rolledback1 <= rollback;
always_ff @(posedge clk_g)
	rolledback2 <= rolledback1;
always_ff @(posedge clk_g)
if (rst_i)
	tick <= 'd0;
else
	tick <= tick + 2'd1;

rfPhoenix_branch_eval ube1
(
	.ir(xir),
	.a(xa[0]),
	.b(xc[0]),
	.o(takb)
);

rfPhoenix_gp_regfile ugprs1
(
	.rst(rst_i),
	.clk(clk_g),
	.wr(commit_wr),
	.wthread(commit_thread),
	.wa(commit_tgt),
	.i(commit_bus[0]),
	.rthread(rfndx1),
	.ra0(ra0),
	.ra1(ra1),
	.ra2(ra2),
	.ra3(ra3),
	.ra4(6'd0),
	.o0(rfo0),
	.o1(rfo1),
	.o2(rfo2),
	.o3(rfo3),
	.o4()
);

rfPhoenix_vec_regfile ugprs2
(
	.rst(rst_i),
	.clk(clk_g),
	.wr(commit_wrv),
	.wthread(commit_thread),
	.wmask(commit_mask),
	.wa(commit_tgt),
	.i(commit_bus),
	.rthread(dcndx),
	.ra0(ra0),
	.ra1(ra1),
	.ra2(ra2),
	.ra3(ra3),
	.ra4(6'd0),
	.o0(vrfo0),
	.o1(vrfo1),
	.o2(vrfo2),
	.o3(),				// mask register port not needed here
	.o4()
);

rfPhoenixVecAlu uvalu1 (
	.ir(xir),
	.a(xa),
	.b(xb),
	.c(xc),
	.t(xt),
	.imm(ximm),
	.Ta(xta),
	.Tb(xtb),
	.Tt(xtt),
	.asid(xasid),
	.hmask(xhmask),
	.trace_dout(trace_dout),
	.trace_empty(trace_empty),
	.trace_valid(trace_valid),
	.trace_count(trace_count),
	.o(vres)
);

rfPhoenixMcVecAlu uvalu2 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir),
	.a(mca),
	.b(mcb),
	.c(mcc),
	.imm(mcimm),
	.o(mc_vres),
	.done(mcv_done),
	.ridi(mcv_ridi),
	.rido(mcv_rido)
);

always_ff @(posedge clk_g)
	ic_line2 <= ic_line;
always_ff @(posedge clk_g)
	ip_insn <= ip_icline;
always_ff @(posedge clk_g)
	ihit2 <= ihit;
always_ff @(posedge clk_g)
	ihite2 <= ihite;
always_ff @(posedge clk_g)
	ihito2 <= ihito;
always_ff @(posedge clk_g)
	ic_tag2 <= ic_tag;
reg [NTHREADS-1:0] wr_ififo;
generate begin
for (g = 0; g < NTHREADS; g = g + 1) begin
	always_comb
		clr_ififo[g] <= rollback[g];
	always_ff @(posedge clk_g)
		sb_issue[g] <= sb_will_issue[g];
	always_comb
		wr_ififo[g] <= ic_ifb.thread==g && ic_ifb.v && ic_ifb.insn.any.opcode!=PFX;

	rfPhoenix_insn_fifo #(.DEP(14)) ufifo1
	(
		.rst(rst_i|rollback[g]),
		.clk(clk_g),
		.wr(wr_ififo[g]),
		.decin(rollback[g] ? 'd0 : deco),
		.ifbin(rollback[g] ? 'd0 : ic_ifb),
		.rd(dcndx==g && dcndx_v),//sb_will_issue[g]),
		.decout(dco[g]),
		.ifbout(dc_ifb[g]),
		.cnt(),
		.full(),
		.almost_full(ififo_almost_full[g]),
		.empty(ififo_empty[g]),
		.v()
	);
	
end
end
endgenerate

wire [3:0] issue_num;
ffo12 uffo1 (.i({12'd0,sb_will_issue}), .o(issue_num));

always_ff @(posedge clk_g)
if (rst_i) begin
	tReset();
end
else begin
`ifdef IS_SIM
	$display("=======================================");
	$display("=======================================");
	$display("Time %d", $time);
	$display("=======================================");
	$display("=======================================");
	$display("  exndx=%d exv=%h", exndx, exv);
`endif	
	tDisplayRegs();
	tDisplayPipe();
	tOnce();
	tInsnFetch();
	tDecode();
	tRegfetch();
	tExecute();
	tAgen();
	tOut();
	tMemory();
	tWriteback();
	tRollback();
end

task tReset;
integer n;
begin
	cr0 <= 32'h01;	// enable threads 0 to 3
	vl <= NLANES;		// number of vector elements
	tid <= 8'd1;
	gie <= 'd0;
	ip <= RSTIP;
	ip1 <= RSTIP;
	ip2 <= RSTIP;
	ip3 <= RSTIP;
	iip <= RSTIP;
	ir <= NOP;//_INSN;
	xir <= NOP;//_INSN;
	mir <= NOP;//_INSN;

	ximm <= 'd0;
	mcimm <= 'd0;
	for (n = 0; n < NLANES; n = n + 1) begin
		xa[n] <= 'd0;
		xb[n] <= 'd0;
		xc[n] <= 'd0;
		mca[n] <= 'd0;
		mcb[n] <= 'd0;
		mcc[n] <= 'd0;
	end
	for (n = 0; n < NTHREADS; n = n + 1) begin
		istk_depth[n] <= 3'd1;
		cause[n][0] <= FLT_NONE;
		cause[n][1] <= FLT_NONE;
		cause[n][2] <= FLT_NONE;
		cause[n][3] <= FLT_NONE;
		ipStack[n] <= {8{RSTIP}};
		sp_sel[n] <= 3'd3;
		ou_rollback_bitmaps[n] <= 'd0;
		rollback_ip[n] <= RSTIP;
	end
	for (n = 0; n < NTHREADS; n = n + 1)
		for (n10 = 0; n10 < 8; n10 = n10 + 1)
			status[n][n10] <= {8{32'hFF000CE0}};
	ithread <= 'd0;
	ip_thread <= 'd0;
	mca_busy <= 'd0;
	thread_busy <= 'd0;
	for (n = 0; n < NTHREADS; n = n + 1) begin
		dcb[n] <= 'd0;
		rfb1[n] <= 'd0;
		rfb2[n] <= 'd0;
		exb[n] <= 'd0;
		agb[n] <= 'd0;
		oub[n] <= 'd0;
		wbb[n] <= 'd0;
	end
	dthread_v <= 'd1;
	memreq <= 'd0;
	last_adr <= 'd0;
	xrid <= 'd15;
	mc_rid <= 'd15;
	commit_thread <= 0;
	commit_mask <= 64'h0000;
	commit_wr <= 'd0;
	commit_wrv <= 1'b0;
	commit_tgt <= 6'b0;
	for (n10 = 0; n10 < NLANES; n10 = n10 + 1)
		commit_bus[n10] <= 'd0;
	for (n10 = 0; n10 < NTHREADS; n10 = n10 + 1) begin
		thread[n10].imiss <= 5'b00111;
		thread[n10].ip <= RSTIP;
		thread[n10].miss_ip <= RSTIP;
		thread[n10].sleep <= 1'b0;
	end
	rfndx1_v <= 1'b0;
	ra0 <= 'd0;
	ra1 <= 'd0;
	ra2 <= 'd0;
	ra3 <= 'd0;
	ra4 <= 'd0;
	ip_thread1 <= 'd0;
	ip_thread2 <= 'd0;
	ip_thread3 <= 'd0;
	ip_thread4 <= 'd0;
	ip_thread5 <= 'd0;
	dbg_cr <= 'd0;
	dbg_sr <= 'd0;
	dbg_am[0] <= 'd0;
	dbg_am[1] <= 'd0;
	dbg_am[2] <= 'd0;
	dbg_am[3] <= 'd0;
	retired <= 'd0;
	imiss_count <= 'd0;
	rollback_ipv <= 'd0;
	mcrid_v <= 1'b0;
	rd_trace <= 1'b0;
end
endtask

task tOnce;
integer n;
begin
	rd_trace <= 1'b0;
	memreq.wr <= 1'b0;
	memresp_fifo_rd <= 1'b0;
	for (n = 0; n < NTHREADS; n = n + 1) begin
		mem_rollback[n] <= 1'b0;
		ou_rollback[n] <= 1'b0;
		if (ou_rollback[n])
			ou_rollback_bitmaps[n] <= 'd0;
	end
end
endtask

task tSpSel;
input [3:0] thread;
input [5:0] i;
output [5:0] o;
begin
	if (i==6'd31)
		case(sp_sel[thread])
		3'd1:	o <= 6'd52;
		3'd2:	o <= 6'd53;
		3'd3:	o <= 6'd54;
		3'd4:	o <= 6'd55;
		default:	o <= 6'd31;
		endcase
	else
		o <= i;
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Schedulers
//
// The following selectors use round-robin selection.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// itndx selects which thread will fetch the instruction. Usually the select
// will circle around through all the threads due to the round-robin select.
// However if an instruction fifo is almost full it will not be selected.
// The fifo might become full if the thread is executing long running
// operations.

reg [NTHREADS-1:0] itsel;
generate begin : gItsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			itsel[g] = !ififo_almost_full[g] & cr0[g];// & ~thread[g].imiss;
end
endgenerate

rfPhoenix_round_robin_select rr1
(
	.rst(rst_i),
	.clk(clk_g),
	.i(itsel),
	.o(itndx),
	.ov(itndx_v)
);

// dcndx selects a decoded instruction from the fifo for register fetch.
// The execution buffer for the thread must be empty and the scoreboard
// must indicate there are no dependencies on the instruction. There also
// must be an instruction in the fifo. dcndx will not issue to the same
// thread in two consecutive clock cycles to allow the thread status to
// update. It will also not issue if the fifo is being updated.
// dcndx is also used indirectly to select the thread for register fetch
// as the decoded register select signals are fed to the register file.
// Two cycles later the register file values are available the thread to
// update will have been dcndx delayed by two cycles. 

reg [NTHREADS-1:0] dcsel;
generate begin : gDcsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			dcsel[g] = sb_can_issue[g] &&							// There are no dependencies
				(!dcb[g].v || (exndx_v && exndx==g)) &&	// The buffer is empty or is going to be empty.
				cr0[g] &&	// The thread is enabled
				!ififo_empty[g] &&	// There is something in the fifo
				!ou_stall[g];// &&				// No stall at end of pipe
				//!sb_will_issue[g];	// and did not just issue
end
endgenerate

rfPhoenix_round_robin_select rr2
(
	.rst(rst_i),
	.clk(clk_g),
	.i(dcsel),
	.o(dcndx),
	.ov(dcndx_v)
);

generate begin : gIssue
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			sb_will_issue[g] = g==dcndx && dcndx_v &&
									!sb_issue[g] &&
									!ififo_empty[g]
									;
end
endgenerate

// exndx selects the thread to move to the execution stage. To be selected the
// register file values must have been fetched.

reg [NTHREADS-1:0] exsel;
generate begin : gExsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			exsel[g] = cr0[g] &&	// Thread enabled
			(
				!rfb2[g].v ||
				rfb2[g].regfetched	// Register fetched
			);
end
endgenerate

rfPhoenix_round_robin_select2 rr3
(
	.rst(rst_i),
	.clk(clk_g),
	.i(exsel),
	.o(exndx),
	.ov(exndx_v)
);

// Select thread for address generation

reg [NTHREADS-1:0] agsel;
generate begin : gAgsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			agsel[g] = (exb[g].out|exb[g].executed) & cr0[g];
end
endgenerate

rfPhoenix_round_robin_select2 rr4
(
	.rst(rst_i),
	.clk(clk_g),
	.i(agsel),
	.o(agndx),
	.ov(agndx_v)
);

// Pick an rob entry thats had its address generated.

// Copy into a bus, just wires.

reg [NTHREADS-1:0] aggen;
generate begin : gAggen
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			aggen[g] = agb[g].agen & cr0[g];
end
endgenerate


rfPhoenix_round_robin_select2 rr5
(
	.rst(rst_i),
	.clk(clk_g),
	.i(aggen),
	.o(oundx),
	.ov(oundx_v)
);

always_comb
begin
	ou_stall = 'd0;
	if (oundx_v)
		ou_stall[oundx] =
							agb[oundx].agen && !(!memreq_full && !req_icload) &&
						 	agb[oundx].dec.mem; // It is a memory operation
end

// Pick a finished instruction. wbndx selects which thread is written back to
// the register file.

// Copy into a bus, just wires.
reg [NTHREADS-1:0] oubfin;
generate begin : gOubfin
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			oubfin[g] = oub[g].executed & cr0[g];
end
endgenerate

rfPhoenix_round_robin_select2 rr6
(
	.rst(rst_i),
	.clk(clk_g),
	.i(oubfin),
	.o(wbndx),
	.ov(wbndx_v)
);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Instruction Pointers
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tIfIp;
begin
	if (itndx_v)
		thread_ip[itndx] <= thread_ip[itndx] + 4'd5;
	if (ip_thread2_v && !ihit) begin
		if (thread[ip_thread2].imiss[0]==1'b0)
			thread_ip[ip_thread2] <= ip1;
		else
			thread_ip[ip_thread2] <= thread[ip_thread2].miss_ip;
	end
end
endtask

task tExIp;
begin
	case(exb[exndx].ifb.insn.any.opcode)
	NOP:	thread_ip[exndx] <= exb[exndx].ifb.ip - 4'd4;
	CALL:	thread_ip[exndx] <= exb[exndx].dec.imm;
	BSR:	thread_ip[exndx] <= exb[exndx].dec.imm + exb[exndx].ifb.ip;
	RET:	thread_ip[exndx] <= exb[exndx].a[0];
	Bcc:	if (takb) 
					thread_ip[exndx] <= exb[exndx].dec.imm + exb[exndx].ifb.ip;
				else
					tIfIp();
	FBcc:	if (takb)
					thread_ip[exndx] <= exb[exndx].dec.imm + exb[exndx].ifb.ip;
				else
					tIfIp();
	default:	tIfIp();
	endcase
end
endtask

integer n14;
always_ff @(posedge clk_g)
if (rst_i) begin
	for (n14 = 0; n14 < NTHREADS; n14 = n14 + 1)
		thread_ip[n14] <= RSTIP;
end
else begin
	if (wbndx_v) begin
		if (|wbb[wbndx].cause)
			thread_ip[wbndx] <= tvec[2'd3];
		else
			case(1'b1)
			oub[wbndx].dec.brk:	thread_ip[wbndx] <= tvec[2'd3];
			oub[wbndx].dec.rti:	thread_ip[wbndx] <= ipStack[wbndx][31:0];
			oub[wbndx].dec.rex:	thread_ip[wbndx] <= tvec[oub[wbndx].ifb.insn[7:6]] + {omode[wbndx],6'h00};
			default:	
				if (oub[wbndx].dec.mem && oub[wbndx].dec.need_steps && oub[wbndx].count < vl && oub[wbndx].mask != 'd0)
					thread_ip[wbndx] <= oub[wbndx].ifb.ip;
				else if (exndx_v)
					tExIp();
				else
					tIfIp();
			endcase
	end
	else if (exndx_v)
		tExIp();
	else
		tIfIp();
end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// IF Stage
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tInsnFetch;
integer n;
begin
	begin
		{ic_ifb.pfx,ic_ifb.insn} <= ic_line >> {ip_icline[5:0],3'b0};
		ic_ifb.ip <= ip_icline;
		ic_ifb.v <= ihit2 && ip_thread2_v;
		ic_ifb.sp_sel <= sp_sel[ip_thread2];
		ic_ifb.thread <= ip_thread2;
		// External interrupt has highest priority.
		if (irq_i > status[ip_thread2][0].ipl && gie[ip_thread2] && status[ip_thread2][0].mie)
			ic_ifb.cause <= CauseCode'({irq_i,8'h00}|FLT_IRQ);
		else if (dbg_cr[0] && dbg_cr[9:8]==2'b00 && dbg_cr[31:28]==ip_thread2 && dbg_adr[0]==ip_icline) begin
			ic_ifb.cause <= FLT_DBG;
			dbg_sr[0] <= 1'b1;
		end
		else if (dbg_cr[1] && dbg_cr[13:12]==2'b00 && dbg_cr[31:28]==ip_thread2 && dbg_adr[1]==ip_icline) begin
			ic_ifb.cause <= FLT_DBG;
			dbg_sr[1] <= 1'b1;
		end
		else if (dbg_cr[2] && dbg_cr[17:16]==2'b00 && dbg_cr[31:28]==ip_thread2 && dbg_adr[2]==ip_icline) begin
			ic_ifb.cause <= FLT_DBG;
			dbg_sr[2] <= 1'b1;
		end
		else if (dbg_cr[3] && dbg_cr[21:20]==2'b00 && dbg_cr[31:28]==ip_thread2 && dbg_adr[3]==ip_icline) begin
			ic_ifb.cause <= FLT_DBG;
			dbg_sr[3] <= 1'b1;
		end
		else if (status[ip_thread2][0].ssm)
			ic_ifb.cause <= FLT_SSM;
		else
			ic_ifb.cause <= FLT_NONE;
		// 2 cycle pipeline delay reading the I$.
		// 1 for tag lookup and way determination
		// 1 for cache line lookup
		ip <= thread_ip[itndx];
		ip1 <= ip;
		for (n = 0; n < NTHREADS; n = n + 1)
			thread[n].imiss <= {thread[n].imiss[3:0],1'b0};
		if (thread[itndx].imiss[2:1]!=2'b00)
			ic_ifb.v <= 1'b0;
		thread_hist[0][itndx] <= thread[itndx];
		for (n = 1; n < 4; n = n + 1)
			thread_hist[n][itndx] <= thread_hist[n-1][itndx];
		if (itndx_v) begin
			thread[itndx].ip <= thread[itndx].ip + 4'd5;
		end
		ip_thread1 <= itndx;			// tag lookup ip_thread1 lined up with ip
		ip_thread2 <= ip_thread1;	// data fetch
		ip_thread3 <= ip_thread2;	// ip_thread3 lined up with ip_icline
		ip_thread4 <= ip_thread3;
		ip_thread5 <= ip_thread4;
		ip_thread1_v <= itndx_v;
		ip_thread2_v <= ip_thread1_v;
		ip_thread3_v <= ip_thread2_v;
		if (ip_thread2_v) begin
			if (!ihit) begin
				ic_ifb.v <= 1'b0;
				$display("Miss %d ip=%h", ip_thread2, ip1);
				if (thread[ip_thread2].imiss[0]==1'b0) begin
					thread[ip_thread2].imiss <= 5'b00111;
					thread[ip_thread2].ip <= ip1;
					thread[ip_thread2].miss_ip <= ip1;
				end
				else begin
					thread[ip_thread2].ip <= thread[ip_thread2].miss_ip;
					thread[ip_thread2].imiss[0] <= 1'b1;
				end
			end
		end
		// On a miss, request a cache line load from the memory system. This
		// should eventually cause a hit for the thread.
		// The old cache line is passed back for the victim buffer.
		if (!ihit2 && ip_thread3_v) begin
			if (!memreq_full) begin
				if (ip_icline[31:6] != last_adr[31:6] || imiss_count > 30) begin
					imiss_count <= 'd0;
					last_adr <= ip_icline;
					tid <= tid + 2'd1;
					memreq.tid <= tid;
					memreq.thread <= ip_thread3;
					memreq.wr <= 1'b1;
					memreq.func <= MR_ICACHE_LOAD;
					memreq.omode <= status[ip_thread3][0].om;
					memreq.asid <= asid[ip_thread3];
					memreq.adr <= {ip_icline[31:6],6'd0};
					memreq.vcadr <= {ic_tag,6'b0};
					memreq.res <= ic_line;
					memreq.sz <= ic_valid ? tetra : nul;
					// But, which line do we need?
					memreq.hit <= {ihito2,ihite2};
				end
				else
					imiss_count <= imiss_count + 2'd1;
			end
		end
	end
end
endtask


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DC Stage
// - most decoding is handled by a decode module above.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tDecode;
begin
	rfndx1_v <= 1'b0;
	rfndx1 <= dcndx;
	if (!ou_stall[dcndx] && dcndx_v) begin
		dcb[dcndx].thread <= dcndx;
		dcb[dcndx].regfetched <= 1'b0;
		dcb[dcndx].executed <= 1'b0;
		if (rollback[dcndx]) begin
			dcb[dcndx].v <= 1'b0;
			dcb[dcndx].ifb <= 'd0;
			dcb[dcndx].dec <= 'd0;
			ra0 <= 'd0;
			ra1 <= 'd0;
			ra2 <= 'd0;
			ra3 <= 'd0;
			ra4 <= 'd0;
		end
		else begin
			dcb[dcndx].v <= dc_ifb[dcndx].v;
			dcb[dcndx].ifb <= dc_ifb[dcndx];
			dcb[dcndx].dec <= dco[dcndx];
			ra0 <= fnSpSel(dcndx,dco[dcndx].Ra.num);
			ra1 <= fnSpSel(dcndx,dco[dcndx].Rb.num);
			ra2 <= fnSpSel(dcndx,dco[dcndx].Rc.num);
			ra3 <= dco[dcndx].Rm.num;
			ra4 <= fnSpSel(dcndx,dco[dcndx].Rt.num);
			rfndx1_v <= 1'b1;
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF Stage
// Forces instructions to be ignored until the rollback target address is seen.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Value csro;
always_comb
	tReadCSR(csro,rfndx1,rfb1[rfndx1].dec.imm[13:0]);
	
task tRegf;
integer n;
begin
	//exb[rfndx].cause <= dcause;
	rfb2[rfndx2].a <= rfb1[rfndx1].dec.Ra.vec ? vrfo0 : {NLANES{rfo0}};
	rfb2[rfndx2].b <= rfb1[rfndx1].dec.Rb.vec ? vrfo1 : {NLANES{rfo1}};
	if (rfb1[rfndx1].dec.csr)
		rfb2[rfndx2].c <= {NLANES{csro}};
	else
		rfb2[rfndx2].c <= rfb1[rfndx1].dec.Rc.vec ? vrfo2 : {NLANES{rfo2}};
	rfb2[rfndx2].mask <= rfo3;
	//rfb2[rfndx2].t <= rfb1[rfndx1].dec.Rt.vec ? vrfo4 : {NLANES{rfo4}};
	rfb2[rfndx2].regfetched <= 1'b1;//rfb1[rfndx1].v & ~rollback[rfndx1];
	if ((rfb1[rfndx1].dec.rfwr & ~rfb1[rfndx1].dec.Rt.vec) | (rfb1[rfndx1].dec.vrfwr & rfb1[rfndx1].dec.Rt.vec))
		ou_rollback_bitmaps[rfndx1][rfb1[rfndx1].dec.Rt] <= 1'b1;
end
endtask

task tRegfetch;
integer n;
begin
	if (!ou_stall[rfndx1]) begin
		rfndx2_v <= rfndx1_v;
		rfndx2 <= rfndx1;
		// RF stage #1, not much to do but propagate.
		if (rfndx1_v)
			rfb1[rfndx1] <= dcb[rfndx1];
		if (rfndx2_v)
			rfb2[rfndx2] <= rfb1[rfndx2];
		// RF stage #2
		if (rfndx2_v) begin
			if (rollback_ipv[rfndx1] && rfb1[rfndx1].ifb.ip != rollback_ip[rfndx1]) begin
				rfb2[rfndx2].v <= 1'b0;
				rfb2[rfndx2].dec.rfwr <= 1'b0;
				rfb2[rfndx2].dec.vrfwr <= 1'b0;
				rfb2[rfndx2].executed <= 1'b0;
			end
			else if (rollback_ipv[rfndx1] && rfb1[rfndx1].ifb.ip == rollback_ip[rfndx1]) begin
				rollback_ipv[rfndx2] <= 1'b0;
				tRegf();
			end
			else begin
				tRegf();
			end
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// EX stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tExCall;
integer n;
begin
	case(rfb2[exndx].ifb.insn.any.opcode)
	PFX:
		begin
			exb[exndx].out <= 1'b0;
			exb[exndx].executed <= 1'b1;
		end
	NOP:
		begin
			exb[exndx].out <= 1'b0;
			exb[exndx].executed <= 1'b1;
			thread[exndx].ip <= rfb2[exndx].ifb.ip - 4'd4;
		end
	CALL:
		begin
			thread[exndx].ip <= rfb2[exndx].dec.imm;
			rollback_ip[exndx] <= rfb2[exndx].dec.imm;
			rollback_ipv[exndx] <= 1'b1;
			exb[exndx].res <= 'd0;
			exb[exndx].res[0] <= rfb2[exndx].ifb.ip + 4'd5;
			ou_rollback[exndx] <= 1'b1;
			ou_rollback_bitmaps[exndx][rfb2[exndx].dec.Rt] <= 1'b1;
			exb[exndx].out <= 1'b0;
			exb[exndx].executed <= 1'b1;
		end
	BSR:
		begin
			thread[exndx].ip <= rfb2[exndx].dec.imm + rfb2[exndx].ifb.ip;
			rollback_ip[exndx] <= rfb2[exndx].dec.imm + rfb2[exndx].ifb.ip;
			rollback_ipv[exndx] <= 1'b1;
			exb[exndx].res <= 'd0;
			exb[exndx].res[0] <= rfb2[exndx].ifb.ip + 4'd5;
			ou_rollback[exndx] <= 1'b1;
			ou_rollback_bitmaps[exndx][rfb2[exndx].dec.Rt] <= 1'b1;
			exb[exndx].out <= 1'b0;
			exb[exndx].executed <= 1'b1;
		end
	RET:
		begin
			exb[exndx].out <= 1'b0;
			exb[exndx].executed <= 1'b1;
			thread[exndx].ip <= rfb2[exndx].a[0];
			rollback_ip[exndx] <= rfb2[exndx].a[0];
			rollback_ipv[exndx] <= 1'b1;
			ou_rollback[exndx] <= 1'b1;
			ou_rollback_bitmaps[exndx][rfb2[exndx].dec.Rt] <= 1'b1;
		end
	default:	;
	endcase
end
endtask

task tExBranch;
begin
	if (rfb2[exndx].dec.br) begin
		exb[exndx].out <= 1'b0;
		exb[exndx].executed <= 1'b1;
		if (takb) begin
			thread[exndx].ip <= rfb2[exndx].ifb.ip + rfb2[exndx].dec.imm;
			rollback_ip[exndx] <= rfb2[exndx].ifb.ip + rfb2[exndx].dec.imm;
			rollback_ipv[exndx] <= 1'b1;
			ou_rollback[exndx] <= 1'b1;
		end
	end
end
endtask

task tExecute;
begin
	if (!ou_stall[exndx] && exndx_v) begin
		exb[exndx] <= rfb2[exndx];
		exb[exndx].regfetched <= 1'b0;
		exb[exndx].out <= 1'b1;
		exb[exndx].retry <= 'd0;
		if (rfb2[exndx].dec.multicycle) begin
			mcrid_v <= 1'b1;
			mc_rid <= exndx;
			mcv_ridi <= exndx;
			mir <= rfb2[exndx].ifb.insn;
			mca <= rfb2[exndx].a;
			mcb <= rfb2[exndx].b;
			mcc <= rfb2[exndx].c;
			mct <= rfb2[exndx].t;
			mcimm <= rfb2[exndx].dec.imm;
			mcm <= rfb2[exndx].mask;
		end
		else begin
			xir <= rfb2[exndx].ifb.insn;
			xa <= rfb2[exndx].a;
			xb <= rfb2[exndx].b;
			xc <= rfb2[exndx].c;
			xt <= rfb2[exndx].t;
			xta <= rfb2[exndx].dec.Ta;
			xtb <= rfb2[exndx].dec.Tb;
			xtt <= rfb2[exndx].dec.Tt;
			xm <= rfb2[exndx].ifb.insn.r2.m ? rfb2[exndx].mask : 16'hFFFF;
			ximm <= rfb2[exndx].dec.imm;
			xasid <= asid[exndx];
			xhmask <= hmask;
//			$display("Decode %d:", dcndx);
//			$display("  insn=%h a=%h b=%h c=%h i=%h", dc_ifb[dcndx].ifb.insn, dc_ifb[dcndx].a, dc_ifb[dcndx].b, dc_ifb[dcndx].c, dc_ifb[dcndx].dec.imm);
//			$display("Execute %d:", exndx);
//			$display("  insn=%h a=%h b=%h c=%h i=%h", exb[exndx].ifb.insn, exb[exndx].a, exb[exndx].b, exb[exndx].c, exb[exndx].dec.imm);
		end
		tExCall();
		tExBranch();
	end
	if (mcv_rido < NTHREADS) begin
		exb[mcv_rido].res <= mc_vres;
		exb[mcv_rido].out <= 1'b0;
		exb[mcv_rido].executed <= 1'b1;
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Address generation
// - address generation is fed from the writeback stage if a vector scatter /
//   gather operation is taking place and the number of steps has not reached
//   the vector length.
// - step and count will be the same unless a compressed load / store operation
//   is taking place. In which case step is applied to the memory address
//   generation and count is used to fetch or store the data element.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Address tmpadr;

task tAgen;	// placeholder task
integer n;
begin
	if (!ou_stall[agndx]) begin
		if (wbb[agndx].dec.need_steps && wbb[agndx].dec.mem && wbb[agndx].count != 'd0)
			agb[agndx] <= wbb[agndx];
		else
			agb[agndx] <= exb[agndx];
		agb[agndx].agen <= 1'b1;
		casez({exb[agndx].dec.storer|exb[agndx].dec.loadr,exb[agndx].dec.Rb.vec,exb[agndx].dec.Ra.vec})
		3'b000:	tmpadr <= exb[agndx].a[0] + exb[agndx].b[0];
		3'b001: tmpadr <= exb[agndx].a[exb[agndx].step] + exb[agndx].b[0];
		3'b010:	tmpadr <= exb[agndx].a[0] + exb[agndx].b[exb[agndx].step];
		3'b011:	tmpadr <= exb[agndx].a[exb[agndx].step] + exb[agndx].b[exb[agndx].step];
		3'b1?0:	tmpadr <= exb[agndx].a[0] + exb[agndx].dec.imm;
		3'b1?1:	tmpadr <= exb[agndx].a[exb[agndx].step] + exb[agndx].dec.imm;
		endcase
		thread[agndx].sleep <= TRUE;
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OU stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tOuLoad;
begin
	if (agb[oundx].dec.load) begin
		if (dbg_cr[0] && dbg_cr[9:8]==2'b11 && dbg_cr[31:28]==ip_thread2 && dbg_adr[0]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[0] <= 1'b1;
		end
		else if (dbg_cr[1] && dbg_cr[13:12]==2'b11 && dbg_cr[31:28]==ip_thread2 && dbg_adr[1]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[1] <= 1'b1;
		end
		else if (dbg_cr[2] && dbg_cr[17:16]==2'b11 && dbg_cr[31:28]==ip_thread2 && dbg_adr[2]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[2] <= 1'b1;
		end
		else if (dbg_cr[3] && dbg_cr[21:20]==2'b11 && dbg_cr[31:28]==ip_thread2 && dbg_adr[3]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[3] <= 1'b1;
		end
		tid <= tid + 2'd1;
		memreq.tid <= tid;
		// Skip masked memory operation when mask is zero
		if (oub[oundx].dec.need_steps && oub[oundx].mask=='d0)
			memreq.wr <= 1'b0;
		else begin
			memreq.wr <= 1'b1;
			thread[oundx].sleep = TRUE;
		end
		memreq.func <= agb[oundx].dec.loadu ? MR_LOADZ : MR_LOAD;
		if (agb[oundx].dec.ldsr)
			memreq.func2 <= MR_LDR;
		else
			memreq.func2 <= MR_NOP;
		memreq.sz <= agb[oundx].dec.memsz;
		memreq.omode <= mprv[oundx] ? status[oundx][1].om : status[oundx][0].om;
		memreq.asid <= asid[oundx];
		memreq.adr <= tmpadr;
		case(agb[oundx].dec.memsz)
		byt:	memreq.sel <= 64'h1;
		wyde:	memreq.sel <= 64'h3;
		tetra:	memreq.sel <= 64'hF;
		vect:	memreq.sel <= 64'hFFFFFFFFFFFFFFFF;
		default:	memreq.sel <= 64'hF;
		endcase
		// Try the same address again on a cache miss.
		if (oub[oundx].cause != FLT_DCM) begin
			if (oub[oundx].dec.memsz==vect) begin
				if (oub[oundx].dec.loadr) begin
					if (oub[oundx].dec.Ra.vec) begin
						if (oub[oundx].step < NLANES-1) begin
							oub[oundx].step <= oub[oundx].step + 2'd1;
							memreq.sel <= 64'hF;
						end
					end
					else begin
						memreq.func2 <= MR_LDV;
						memreq.sel <= 64'hFFFFFFFFFFFFFFFF;
					end
				end
				else begin
					memreq.sel <= 64'hF;
					if (oub[oundx].step < NLANES-1 && oub[oundx].dec.loadn)
						oub[oundx].step <= oub[oundx].step + 2'd1;
				end
			end
		end
		else if (oub[oundx].retry < 3'd5) begin
			oub[oundx].retry <= oub[oundx].retry + 2'd1;
			oub[oundx].cause <= FLT_NONE;
		end
	end
end
endtask

task tOuStore;
begin
	if (agb[oundx].dec.store) begin
		if (dbg_cr[0] && dbg_cr[8]==1'b1 && dbg_cr[31:28]==ip_thread2 && dbg_adr[0]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[0] <= 1'b1;
		end
		else if (dbg_cr[1] && dbg_cr[12]==1'b1 && dbg_cr[31:28]==ip_thread2 && dbg_adr[1]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[1] <= 1'b1;
		end
		else if (dbg_cr[2] && dbg_cr[16]==1'b1 && dbg_cr[31:28]==ip_thread2 && dbg_adr[2]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[2] <= 1'b1;
		end
		else if (dbg_cr[3] && dbg_cr[20]==1'b1 && dbg_cr[31:28]==ip_thread2 && dbg_adr[3]==tmpadr) begin
			oub[oundx].cause <= FLT_DBG;
			dbg_sr[3] <= 1'b1;
		end
		tid <= tid + 2'd1;
		memreq.tid <= tid;
		// Skip masked memory operation when mask is zero
		if (oub[oundx].dec.need_steps && oub[oundx].mask=='d0)
			memreq.wr <= 1'b0;
		else begin
			memreq.wr <= 1'b1;
			thread[oundx].sleep = TRUE;
		end
		memreq.func <= MR_STORE;
		if (agb[oundx].dec.stcr)
			memreq.func2 <= MR_STC;
		else
			memreq.func2 <= MR_NOP;
		memreq.sz <= agb[oundx].dec.memsz;
		memreq.omode <= mprv[oundx] ? status[oundx][1].om : status[oundx][0].om;
		memreq.asid <= asid[oundx];
		memreq.adr <= tmpadr;
		if (agb[oundx].dec.need_steps)
			memreq.res <= {1024'd0,agb[oundx].c[agb[oundx].count]};
		else
			memreq.res <= {1024'd0,agb[oundx].c};
		case(agb[oundx].dec.memsz)
		byt:	memreq.sel <= 64'h1;
		wyde:	memreq.sel <= 64'h3;
		tetra:	memreq.sel <= 64'hF;
		default:	memreq.sel <= 64'hF;
		endcase
		// BIU works with 128-bit chunks for stores.
		if (agb[oundx].dec.memsz==vect) begin
			memreq.sel <= agb[oundx].ifb.insn.r2.m ? (
				{	{4{agb[oundx].mask[15]}},
					{4{agb[oundx].mask[14]}},
					{4{agb[oundx].mask[13]}},
					{4{agb[oundx].mask[12]}},
					{4{agb[oundx].mask[11]}},
					{4{agb[oundx].mask[10]}},
					{4{agb[oundx].mask[9]}},
					{4{agb[oundx].mask[8]}},
					{4{agb[oundx].mask[7]}},
					{4{agb[oundx].mask[6]}},
					{4{agb[oundx].mask[5]}},
					{4{agb[oundx].mask[4]}},
					{4{agb[oundx].mask[3]}},
					{4{agb[oundx].mask[2]}},
					{4{agb[oundx].mask[1]}},
					{4{agb[oundx].mask[0]}}} >> {agb[oundx].step,2'h0}) & 64'hFFFF
				 	: 64'hFFFF;
			if (agb[oundx].dec.storen)
				memreq.sz <= tetra;
			// For a scatter store select the current vector element, otherwise select entire vector (set above).
			if (agb[oundx].dec.storen) begin
				memreq.sel <= 64'h000000000000000F;	// 32 bit at a time
				// Dont bother storing if masked
				if (agb[oundx].ifb.insn.r2.m && !agb[oundx].mask[agb[oundx].step])
					memreq.wr <= 1'b0;
				memreq.res <= agb[oundx].t[agb[oundx].step];
			end
			if (oub[oundx].dec.storer) begin
				if (oub[oundx].dec.Ra.vec && oub[oundx].step < NLANES-1)
					oub[oundx].step <= oub[oundx].step + 5'd1;
				else if (agb[oundx].step < NLANES-4)
					oub[oundx].step <= agb[oundx].step + 5'd16;
			end
			// For scatter increment step
			if (oub[oundx].step < NLANES-1 && oub[oundx].dec.storen)
				oub[oundx].step <= oub[oundx].step + 2'd1;
		end
	end
end
endtask


wire req_icload = !ihit2 && !memreq_full && (ip_icline[31:6] != last_adr[31:6] || imiss_count > 10);

task tOut;
begin
	if (!ou_stall[oundx]) begin
		oub[oundx] <= agb[oundx];
		oub[oundx].agen <= 1'b1;
		// We want this before CALL is processed which also sets res.
		if (oundx_v && !exb[oundx].dec.cjb)
			oub[oundx].res <= vres;
		if (oundx_v) begin
			oub[oundx].out <= (agb[oundx].dec.load|agb[agndx].dec.store) ? ~agb[agndx].agen : 1'b0;
			oub[oundx].executed <= (agb[oundx].dec.load|agb[agndx].dec.store) ? agb[agndx].agen : 1'b1;
			if ((agb[oundx].dec.Ra.vec | ((agb[oundx].dec.storen|agb[oundx].dec.loadn) & agb[oundx].dec.Rb.vec)) && agb[oundx].dec.memsz==vect) begin
				if (agb[oundx].step < NLANES-1) begin
					oub[oundx].out <= 1'b1;
					oub[oundx].agen <= 1'b0;
					oub[oundx].executed <= 1'b0;
				end
			end
			if (agb[oundx].agen) begin
				if (!memreq_full && !req_icload) begin
					tOuLoad();
					tOuStore();
					oub[oundx].agen <= 1'b0;
				end
				// If the load/store could not be queued backout the decoded and out
				// indicators so the instruction will be reselected for execution.
				else begin
					if (agb[oundx].dec.load|agb[oundx].dec.store) begin
						oub[oundx].regfetched <= 1'b1;
						oub[oundx].executed <= 1'b0;
						oub[oundx].out <= 1'b0;
						oub[oundx].agen <= 1'b0;
					end
				end
			end
`ifdef IS_SIM
			$display("Out %d:", exndx);
			$display("  res=%h",vres);
`endif		
		end
		if (mcrid_v) begin
			if (exb[mc_rid].dec.is_vector ? mcv_done : mc_done) begin
				mcrid_v <= 1'b0;
				mca_busy[0] <= 1'b0;
				oub[mc_rid].out <= 1'b0;
				oub[mc_rid].executed <= 1'b1;
	//			if (exb[mc_rid].dec.Tt)
					oub[mc_rid].res <= mc_vres;
	//			else
	//				exb[mc_rid].res <= mc_res;
			end
		end
	end
end
endtask


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MEM stage 
// - process responses coming back from the BIU for requests sent by EX stage.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tMemory;
integer n;
begin
	if (!memresp_fifo_empty)
		memresp_fifo_rd <= 1'b1;
	if (memresp_fifo_rd & memresp_fifo_v) begin
		thread[memresp.thread].sleep <= FALSE;
		if (|memresp.cause && oub[memresp.thread].cause=='d0)
			oub[memresp.thread].cause <= memresp.cause;
		// Clear the imiss status. The thread might still miss again if the I$
		// has not updated before the thread is selected again, but at least
		// it can be prevented from being selected for a few cycles while the
		// imiss request is processed.
		if (memresp.func==MR_ICACHE_LOAD)
			;
//			for (n = 0; n < NTHREADS; n = n + 1)
//				thread[n].imiss <= 1'b0;
		else begin
			oub[memresp.thread].out <= 1'b0;
			if (oub[memresp.thread].out) begin
				oub[memresp.thread].executed <= 1'b1;
				// If a gather load
				if (oub[memresp.thread].dec.loadn && oub[memresp.thread].dec.memsz==vect) begin
					if (oub[memresp.thread].step < NLANES-1) begin
						oub[memresp.thread].out <= 1'b1;
						oub[memresp.thread].executed <= 1'b0;
					end
					oub[memresp.thread].res[memresp.step] <= memresp.res;
				end
				// Other load
				else if (oub[memresp.thread].dec.load) begin
					oub[memresp.thread].res <= memresp.res;
				end
				// Scatter store
				else if (oub[memresp.thread].dec.storen && oub[memresp.thread].dec.memsz==vect) begin
					if (oub[memresp.thread].step!=NLANES-1) begin
						oub[memresp.thread].out <= 1'b1;
						oub[memresp.thread].executed <= 1'b0;
					end
				end
				else if (oub[memresp.thread].dec.storer && oub[memresp.thread].dec.memsz==vect) begin
					if (oub[memresp.thread].step!=NLANES-4) begin
						oub[memresp.thread].out <= 1'b1;
						oub[memresp.thread].executed <= 1'b0;
					end
				end
				// Other store / Other Op
			end
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// WB Stage
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// REX instruction

task tWbRex;
begin
	// Exception if trying to switch to higher mode
	if (omode[wbndx] <= oub[wbndx].ifb.insn[7:6]) begin
		tWbException(oub[wbndx].ifb.ip,FLT_PRIV,1);
	end
	else begin
		status[wbndx][0].om <= oub[wbndx].ifb.insn[7:6];	// omode
		status[wbndx][0].pl <= oub[wbndx].a[0][7:0];
		cause[wbndx][oub[wbndx].ifb.insn[7:6]] <= cause[wbndx][2'd3];
		badaddr[wbndx][oub[wbndx].ifb.insn[7:6]] <= badaddr[wbndx][2'd3];
		ip <= tvec[oub[wbndx].ifb.insn[7:6]] + {omode[wbndx],6'h00};
		// Don't allow stack redirection for interrupt processing.
		if (sp_sel[wbndx] != 3'd4)
			case(status[wbndx][0].om)
			2'd0:	sp_sel[wbndx] <= 3'd0;
			2'd1:	sp_sel[wbndx] <= 3'd1;
			2'd2:	sp_sel[wbndx] <= 3'd2;
			2'd3:	sp_sel[wbndx] <= 3'd3;
			endcase
	end
end
endtask

// RTI processing at the WB stage.
task tWbRti;
integer n;
begin
	if (|istk_depth[wbndx]) begin
		// unstack
		for (n = 0; n < 7; n = n + 1)
			status[wbndx][n] <= status[wbndx][n+1];
		// Set some reasonable underflow values
		status[wbndx][7].pl <= 8'hFF;
		status[wbndx][7].om <= 2'b11;
		status[wbndx][7].ipl <= 3'b111;
		status[wbndx][7].uie <= 1'b0;
		status[wbndx][7].sie <= 1'b0;
		status[wbndx][7].hie <= 1'b0;
		status[wbndx][7].mie <= 1'b0;
		status[wbndx][7].die <= 1'b0;
		status[wbndx][7].trace_en <= 1'b0;
		ipStack[wbndx] <= {RSTIP,ipStack[wbndx][255:32]};
		thread[wbndx].ip <= ipStack[wbndx][31:0];
		istk_depth[wbndx] <= istk_depth[wbndx] - 2'd1;
		case(status[wbndx][0].om)
		2'd0:	sp_sel[wbndx] <= 3'd0;
		2'd1:	sp_sel[wbndx] <= 3'd1;
		2'd2:	sp_sel[wbndx] <= 3'd2;
		2'd3:	sp_sel[wbndx] <= 3'd3;
		endcase
	end
	else
		tWbException(oub[wbndx].ifb.ip,FLT_RTI,1);
end
endtask

task tWbException;
input CodeAddress ip;
input CauseCode cc;
input keepIrq;
integer n;
begin
	if (istk_depth[wbndx] < 3'd7) begin
		for (n = 1; n < 8; n = n + 1)
			status[wbndx][n] <= status[wbndx][n-1];
		status[wbndx][0].om <= 2'b11;		// select machine operating mode
		if (keepIrq || cc[10:8]==3'd0)
			status[wbndx][0].ipl <= status[wbndx][0].ipl;
		else
			status[wbndx][0].ipl <= cc[10:8];
		status[wbndx][0].uie <= 1'b0;	// disable all irqs
		status[wbndx][0].sie <= 1'b0;
		status[wbndx][0].hie <= 1'b0;
		status[wbndx][0].mie <= 1'b0;
		status[wbndx][0].die <= 1'b0;
		status[wbndx][0].trace_en <= 1'b0;
		status[wbndx][0].pl <= 8'hFF;	// select max priv level
		ipStack[wbndx] <= ipStack[wbndx] << 32;
		ipStack[wbndx][31:0] <= ip;
		istk_depth[wbndx] <= istk_depth[wbndx] + 2'd1;
		thread[wbndx].ip <= tvec[2'd3];
		cause[wbndx][omode[wbndx]] <= cc;
		badaddr[wbndx][omode[wbndx]] <= oub[wbndx].badAddr;
		exb[wbndx].cause <= FLT_NONE;
		if (exb[wbndx].cause & 12'h8FF==FLT_IRQ)
			sp_sel[wbndx] <= 3'd4;
		else
			sp_sel[wbndx] <= 3'd3;
	end
	if (oub[oundx].dec.mem) begin
		mem_rollback[wbndx] <= 1'b1;
		ou_rollback[wbndx] <= 1'b1;
	end
	else begin
		ou_rollback[wbndx] <= 1'b1;
	end
end
endtask

task tWriteback;
begin
	if (wbndx_v) begin
		wbb[wbndx] <= oub[wbndx];
`ifdef IS_SIM
		$display("Writeback %d:", wbndx);
`endif		
		// Normally we do not want to update the machine state on an exception.
		// However for single-step mode we do.
		if (|oub[wbndx].cause && oub[wbndx].cause != FLT_SSM)
			tWbException(oub[wbndx].ifb.ip,oub[wbndx].cause,0);
		else begin
			if (oub[wbndx].cause==FLT_SSM)
				tWbException(oub[wbndx].ifb.ip,oub[wbndx].cause,1);
`ifdef IS_SIM				
			$display("  ip=%h ir=%h", oub[wbndx].ifb.ip, oub[wbndx].ifb.insn);
			if (oub[oundx].dec.rfwr)
				$display("  %s=%h", fnRegName(oub[wbndx].dec.Rt), oub[wbndx].res);
`endif				
			commit_thread <= wbndx;
			commit_mask[1:0] <= {2{oub[wbndx].mask[0]}};
			commit_mask[3:2] <= {2{oub[wbndx].mask[1]}};
			commit_mask[5:4] <= {2{oub[wbndx].mask[2]}};
			commit_mask[7:6] <= {2{oub[wbndx].mask[3]}};
			commit_mask[9:8] <= {2{oub[wbndx].mask[4]}};
			commit_mask[11:10] <= {2{oub[wbndx].mask[5]}};
			commit_mask[13:12] <= {2{oub[wbndx].mask[6]}};
			commit_mask[15:14] <= {2{oub[wbndx].mask[7]}};
			commit_mask[17:16] <= {2{oub[wbndx].mask[8]}};
			commit_mask[19:18] <= {2{oub[wbndx].mask[9]}};
			commit_mask[21:20] <= {2{oub[wbndx].mask[10]}};
			commit_mask[23:22] <= {2{oub[wbndx].mask[11]}};
			commit_mask[25:24] <= {2{oub[wbndx].mask[12]}};
			commit_mask[27:26] <= {2{oub[wbndx].mask[13]}};
			commit_mask[29:28] <= {2{oub[wbndx].mask[14]}};
			commit_mask[31:30] <= {2{oub[wbndx].mask[15]}};
			commit_mask[33:32] <= {2{oub[wbndx].mask[16]}};
			commit_mask[35:34] <= {2{oub[wbndx].mask[17]}};
			commit_mask[37:36] <= {2{oub[wbndx].mask[18]}};
			commit_mask[39:38] <= {2{oub[wbndx].mask[19]}};
			commit_mask[41:40] <= {2{oub[wbndx].mask[20]}};
			commit_mask[43:42] <= {2{oub[wbndx].mask[21]}};
			commit_mask[45:44] <= {2{oub[wbndx].mask[22]}};
			commit_mask[47:46] <= {2{oub[wbndx].mask[23]}};
			commit_mask[49:48] <= {2{oub[wbndx].mask[24]}};
			commit_mask[51:50] <= {2{oub[wbndx].mask[25]}};
			commit_mask[53:52] <= {2{oub[wbndx].mask[26]}};
			commit_mask[55:54] <= {2{oub[wbndx].mask[27]}};
			commit_mask[57:46] <= {2{oub[wbndx].mask[28]}};
			commit_mask[59:48] <= {2{oub[wbndx].mask[29]}};
			commit_mask[61:60] <= {2{oub[wbndx].mask[30]}};
			commit_mask[63:62] <= {2{oub[wbndx].mask[31]}};
			commit_wr <= {4{oub[wbndx].dec.rfwr}};
			commit_wrv <= oub[wbndx].dec.vrfwr;
			commit_tgt <= oub[wbndx].dec.Rt;
			commit_bus <= oub[wbndx].res;
			case(1'b1)
			oub[wbndx].dec.popq:
				case(oub[wbndx].dec.imm[3:0])
				4'd15:	rd_trace <= 1'b1;
				default:	;
				endcase
			oub[wbndx].dec.brk:	tWbException(oub[wbndx].ifb.ip + 4'd5,FLT_BRK,1);	// BRK instruction
			//exb[wbndx].dec.irq: tWbException(exb[wbndx].ifb.ip,exb[wbndx].cause);	// hardware irq
			//exb[wbndx].dec.flt: tWbException(exb[wbndx].ifb.ip,exb[wbndx].cause);	// processing fault (divide by zero, tlb miss, ...)
			oub[wbndx].dec.rti:	tWbRti();
			oub[wbndx].dec.rex:	tWbRex();
			oub[wbndx].dec.csrrw:	tWriteCSR(oub[wbndx].a,wbndx,oub[wbndx].dec.imm[13:0]);
			oub[wbndx].dec.csrrc:	tClrbitCSR(oub[wbndx].a,wbndx,oub[wbndx].dec.imm[13:0]);
			oub[wbndx].dec.csrrs:	tSetbitCSR(oub[wbndx].a,wbndx,exb[wbndx].dec.imm[13:0]);
			endcase
			// Check for a vector memory instruction that needs to repeat.
			// Instructions will keep flowing into the pipeline while the vector
			// operation is taking place. Treat this like a branch and rollback the
			// incoming instructions.
			if (oub[wbndx].dec.mem & oub[wbndx].dec.need_steps) begin
				if (oub[wbndx].count < vl && oub[wbndx].mask != 'd0) begin
					wbb[wbndx].count <= oub[wbndx].count + 2'd1;
					if (oub[wbndx].mask[oub[wbndx].count] || !oub[wbndx].dec.compress)
						wbb[wbndx].step <= oub[wbndx].step + 2'd1;
					ou_rollback[wbndx] <= 1'b1;
					oub[wbndx].mask[oub[wbndx].count] <= 1'b0;
				end
				else
					wbb[wbndx] <= 'd0;
			end
			// Writing to machine stack pointer globally enables interrupts.
			if (oub[wbndx].dec.Rt==7'd47 && oub[wbndx].dec.rfwr)
				gie[wbndx] <= 1'b1;
			if (ic_ifb.v && ic_ifb.insn.any.opcode==PFX)
				retired <= retired + 2'd2;
			else
				retired <= retired + 2'd1;
		end
//		if (!dcndx_v || dcndx!=wbndx)
//			oub[wbndx] <= 'd0;
	end
	if ((oub[wbndx].dec.rfwr & ~commit_tgt.vec) | (oub[wbndx].dec.vrfwr & commit_tgt.vec))
		ou_rollback_bitmaps[commit_thread][commit_tgt] <= 1'b0;
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tRollback;
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1)
		if (rollback[n])
			exb[n] <= 'd0;
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// CSR Read / Update tasks
//
// Important to use the correct assignment type for the following, otherwise
// The read won't happen until the clock cycle.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tReadCSR;
output Value res;
input Tid thread;
input [13:0] regno;
begin
	if (regno[13:12] <= omode[thread]) begin
		casez({2'b00,regno[13:0]})
		CSR_MHARTID: res = {hartid_i[31:3],thread};
//		CSR_MCR0:	res = cr0|(dce << 5'd30);
		CSR_PTBR:	res = ptbr[thread];
//		CSR_HMASK:	res = hmask;
//		CSR_KEYS:	res = keys2[regno[0]];
//		CSR_FSTAT:	res = fpscr;
		CSR_ASID:	res = asid[thread];
		CSR_MBADADDR:	res = badaddr[thread][regno[13:12]];
		CSR_TICK:	res = tick;
		CSR_CAUSE:	res = cause[thread][regno[13:12]];
		CSR_MTVEC:	res = tvec[regno[1:0]];
		CSR_MEIP:		res = ipStack[thread][31:0];
		CSR_TIME:	res = wc_time[31:0];
		CSR_USTATUS:	res = status[thread][0].uie;
		CSR_SSTATUS:	res = {2'b01,8'h00,status[thread][0][1:0]};
		CSR_HSTATUS:	res = {2'b10,7'h00,status[thread][0][2:0]};
		CSR_MSTATUS:	res = status[thread][0];
		CSR_MDBAD:		res = dbg_adr[regno[1:0]];
		CSR_MDBAM:		res = dbg_am[regno[1:0]];
		CSR_MDBCR:		res = dbg_cr;
		CSR_MDBSR:		res = dbg_sr;
		default:	res = 'd0;
		endcase
	end
	else
		res = 'd0;
end
endtask

task tWriteCSR;
input Value val;
input Tid thread;
input [13:0] regno;
begin
	if (regno[13:12] <= omode[thread]) begin
		casez({2'b00,regno[13:0]})
		CSR_MCR0:		cr0 <= val;
		CSR_PTBR:		ptbr[thread] <= val;
//		CSR_HMASK:	hmask <= val;
//		CSR_SEMA:		sema <= val;
//		CSR_KEYS:		keys2[regno[0]] <= val;
//		CSR_FSTAT:	fpscr <= val;
		CSR_ASID: 	asid[thread] <= val;
		CSR_MBADADDR:	badaddr[thread][regno[13:12]] <= val;
		CSR_CAUSE:	cause[thread][regno[13:12]] <= CauseCode'(val[11:0]);
		CSR_MTVEC:	tvec[regno[1:0]] <= val;
		CSR_MTIME:	begin wc_time_dat <= val; ld_time <= 1'b1; end
		CSR_USTATUS:	status[thread][0][0] <= val[0];
		CSR_SSTATUS:	status[thread][0][1:0] <= val[1:0];
		CSR_HSTATUS:	status[thread][0][2:0] <= val[2:0];
		CSR_MSTATUS:	status[thread][0] <= val;
		CSR_MDBAD:		dbg_adr[regno[1:0]] <= val;
		CSR_MDBAM:		dbg_am[regno[1:0]] <= val;
		CSR_MDBCR:		dbg_cr <= val;
		CSR_MDBSR:		dbg_sr <= val;
		default:	;
		endcase
	end
end
endtask

task tSetbitCSR;
input Value val;
input Tid thread;
input [13:0] regno;
begin
	if (regno[13:12] <= omode[thread]) begin
		casez({2'b00,regno[13:0]})
		CSR_MCR0:			cr0[val[5:0]] <= 1'b1;
		CSR_USTATUS:	status[thread][0][0] <= status[thread][0][0] | val[0];
		CSR_SSTATUS:	status[thread][0][1:0] <= status[thread][0][1:0] | val[1:0];
		CSR_HSTATUS:	status[thread][0][2:0] <= status[thread][0][2:0] | val[2:0];
		CSR_MSTATUS:	status[thread][0] <= status[thread][0] | val;
		CSR_MDBCR:		dbg_cr <= dbg_cr | val;
		CSR_MDBSR:		dbg_sr <= dbg_sr | val;
		default:	;
		endcase
	end
end
endtask

task tClrbitCSR;
input Value val;
input Tid thread;
input [13:0] regno;
begin
	if (regno[13:12] <= omode[thread]) begin
		casez({2'b00,regno[13:0]})
		/*
		CSR_IE:
			case(regno[13:12])
			2'd0:	ie_reg[thread][0] <= ie_reg[thread][0] & ~val[0];
			2'd1: ie_reg[thread][1:0] <= ie_reg[thread][1:0] & ~val[1:0];
			2'd2:	ie_reg[thread][2:0] <= ie_reg[thread][2:0] & ~val[2:0];
			2'd3:	ie_reg[thread][4:0] <= ie_reg[thread][4:0] & ~val[4:0];
			endcase
		*/
		CSR_MCR0:			cr0[val[5:0]] <= 1'b0;
		CSR_USTATUS:	status[thread][0][0] <= status[thread][0][0] & ~val[0];
		CSR_SSTATUS:	status[thread][0][1:0] <= status[thread][0][1:0] & ~val[1:0];
		CSR_HSTATUS:	status[thread][0][2:0] <= status[thread][0][2:0] & ~val[2:0];
		CSR_MSTATUS:	status[thread][0] <= status[thread][0] & ~val;
		CSR_MDBCR:		dbg_cr <= dbg_cr & ~val;
		CSR_MDBSR:		dbg_sr <= dbg_sr & ~val;
		default:	;
		endcase
	end
end
endtask

// =========================================================================
// Debug Code
// =========================================================================

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Disassembler for debugging. It helps to have some output to allow 
// visual tracking in the simulation run.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

function [31:0] fnRegName;
input [5:0] Rn;
begin
	case(Rn)
	6'd0:	fnRegName = "zero";
	6'd1:	fnRegName = "a0";
	6'd2:	fnRegName = "a1";
	6'd3:	fnRegName = "t0";
	6'd4:	fnRegName = "t1";
	6'd5:	fnRegName = "t2";
	6'd6:	fnRegName = "t3";
	6'd7:	fnRegName = "t4";
	6'd8:	fnRegName = "t5";
	6'd9:	fnRegName = "t6";
	6'd10:	fnRegName = "t7";
	6'd11:	fnRegName = "s0";
	6'd12:	fnRegName = "s1";
	6'd13:	fnRegName = "s2";
	6'd14:	fnRegName = "s3";
	6'd15:	fnRegName = "s4";
	6'd16:	fnRegName = "s5";
	6'd17:	fnRegName = "s6";
	6'd18:	fnRegName = "s7";
	6'd19:	fnRegName = "s8";
	6'd20:	fnRegName = "s9";
	6'd21:	fnRegName = "a2";
	6'd22:	fnRegName = "a3";
	6'd23:	fnRegName = "a4";
	6'd24:	fnRegName = "a5";
	6'd25:	fnRegName = "a6";
	6'd26:	fnRegName = "a7";
	6'd27:	fnRegName = "gp3";
	6'd28:	fnRegName = "gp2";
	6'd29:	fnRegName = "gp";
	6'd30:	fnRegName = "fp";
	6'd31:	fnRegName = "sp";
	6'd32:	fnRegName = "vm0";
	6'd33:	fnRegName = "vm1";
	6'd34:	fnRegName = "vm2";
	6'd35:	fnRegName = "vm3";
	6'd36:	fnRegName = "vm4";
	6'd37:	fnRegName = "vm5";
	6'd38:	fnRegName = "vm6";
	6'd39:	fnRegName = "vm7";
	6'd40:	fnRegName = "vm8";
	6'd41:	fnRegName = "vm9";
	6'd42:	fnRegName = "vm10";
	6'd43:	fnRegName = "vm11";
	6'd44:	fnRegName = "vm12";
	6'd45:	fnRegName = "vm13";
	6'd46:	fnRegName = "vm14";
	6'd47:	fnRegName = "vm15";
	6'd48:	fnRegName = "lc";
	6'd49:	fnRegName = "lk1";
	6'd50:	fnRegName = "lk2";
	6'd51:	fnRegName = "r43";
	6'd52:	fnRegName = "ssp";
	6'd53:	fnRegName = "hsp";
	6'd54:	fnRegName = "msp";
	6'd55:	fnRegName = "isp";
	6'd56:	fnRegName = "f0";
	6'd57:	fnRegName = "f1";
	6'd58:	fnRegName = "f2";
	6'd59:	fnRegName = "f3";
	6'd60:	fnRegName = "f4";
	6'd61:	fnRegName = "f5";
	6'd62:	fnRegName = "f6";
	6'd63:	fnRegName = "f7";
	endcase
end
endfunction

task tDisplayRegs;
integer n;
begin
`ifdef IS_SIM
	// The heirarchical reference to the register file here prevents synthsis
	// from using RAM resources to implement the register file. So this block
	// is enabled only for simulation.
	$display("GPRs");
	for (n = 0; n < NTHREADS*NREGS; n = n + 8) begin
		// Do not bother with display of regs for disabled threads.
		if (cr0[n >> $clog2(NREGS)]) begin
			if ((n % NREGS)==0)
				$display("  Thread:%d", n / NREGS);
			$display("%s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  ",
				fnRegName(n), ugprs1.ugpr0.mem[n],
				fnRegName(n+1), ugprs1.ugpr0.mem[n+1],
				fnRegName(n+2), ugprs1.ugpr0.mem[n+2],
				fnRegName(n+3), ugprs1.ugpr0.mem[n+3],
				fnRegName(n+4), ugprs1.ugpr0.mem[n+4],
				fnRegName(n+5), ugprs1.ugpr0.mem[n+5],
				fnRegName(n+6), ugprs1.ugpr0.mem[n+6],
				fnRegName(n+7), ugprs1.ugpr0.mem[n+7]
				);				
		end
	end
	$display("");
`endif
end
endtask

task tDisplayPipe;
integer n,n1;
begin
`ifdef IS_SIM
	for (n = 0; n < NTHREADS; n = n + 1) begin
		if (cr0[n]) begin
			$display("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
			$display("Thread: %d", n);
			$display("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
			$display("Insn Fetch %d:", ip_thread4);
			$display("  ip_insn=%h  insn=%h postfix=%h", ic_ifb.ip, ic_ifb.insn, ic_ifb.pfx);
			$display("  ip_thread4=%h", ip_thread4);
			for (n1 = 0; n1 < NTHREADS; n1 = n1 + 1)
				$display("  thread[%d].ip=%h", n1[3:0], thread_ip[n1]);
			$display("DecodeBuffer:");
			$display("  1:%d: %c ip=%h ir=%h oc=%0s", n[3:0],
				dc_ifb[n].v ? "v":"-",
				dc_ifb[n].ip,
				dc_ifb[n].insn,
				dc_ifb[n].insn.any.opcode.name()
			);
			$display("  2:%d: %c ip=%h ir=%h oc=%0s", n[3:0],
				dcb[n].v ? "v":"-",
				dcb[n].ifb.ip,
				dcb[n].ifb.insn,
				dcb[n].ifb.insn.any.opcode.name()
			);
			$display("Regfetch %d,%d:", rfndx1,rfndx2);
			$display("  1:%c ip=%h Ra%d Rb%d csro[%h]=%h",
				rfndx1_v?"v":"-",rfb1[rfndx1].ifb.ip, rfb1[rfndx1].dec.Ra, rfb1[rfndx1].dec.Rb,rfb1[rfndx1].dec.imm[13:0],csro);
			$display("  2:%c ip=%h Ra%d=%h Rb%d=%h",
				rfndx2_v?"v":"-",rfb2[rfndx2].ifb.ip, rfb2[rfndx2].dec.Ra, rfo0, rfb2[rfndx2].dec.Rb, rfo1);
			$display("ExecuteBuffer:");
			$display("  %d: %c%c%c%c ip=%h ir=%h oc=%0s res=%h a=%h b=%h c=%h t=%h i=%h", n[3:0],
				exb[n].v ? "v":"-",
				exb[n].regfetched ? "r": "-",
				exb[n].out ? "o" : "-",
				exb[n].executed ? "x" : "-",
				exb[n].ifb.ip,
				exb[n].ifb.insn,
				exb[n].ifb.insn.any.opcode.name(),
				exb[n].res,
				exb[n].a,
				exb[n].b,
				exb[n].c,
				exb[n].t,
				exb[n].dec.imm
			);
			$display("Address Generation");
			$display("  %c ip=%h oc=%0s tmpadr=%h",
				agb[n].v ? "v" : "-",
				agb[n].ifb.ip,
				agb[n].ifb.insn.any.opcode.name(),
				tmpadr
			);
		end
	end
`endif
end
endtask

endmodule
