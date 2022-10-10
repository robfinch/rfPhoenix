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

import const_pkg::*;
import wishbone_pkg::*;
import rfPhoenixPkg::*;
import rfPhoenixMmupkg::*;

module rfPhoenix(hartid_i, rst_i, clk_i, wc_clk_i, clock,
		nmi_i, irq_i, icause_i, wb_req, wb_resp, state_o, trigger_o, wcause);
input [31:0] hartid_i;
input rst_i;
input clk_i;
input wc_clk_i;
input clock;					// MMU clock algorithm
input nmi_i;
input [2:0] irq_i;
input [8:0] icause_i;
output wb_write_request128_t wb_req;
input wb_read_response128_t wb_resp;
output [5:0] state_o;
output reg trigger_o;
output cause_code_t wcause;

parameter IC_LATENCY = 2;

integer n2,n3,n4,n10;

wire clk_g = clk_i;
pipeline_reg_t [NTHREADS-1:0] dcb, rfb1, rfb2, rfb3, exb, agb, oub, wbb, wbb2;

wire [1:0] bte_o;
wire [2:0] cti_o;
wb_tranid_t tid_o;
wire [2:0] seg_o;
wire vpa_o;
wire vda_o;
wire cyc_o;
wire stb_o;
wire we_o;
wire [15:0] sel_o;
address_t adr_o;
wire [127:0] dat_o;
wb_tranid_t tid_i;
reg bok_i;
reg next_i;
reg ack_i;
reg stall_i;
reg err_i;
reg [127:0] dat_i;
reg rb_i;
always_comb
	next_i <= wb_resp.next;
always_comb
	ack_i <= wb_resp.ack;
always_comb
	dat_i <= wb_resp.dat;
always_comb
	tid_i <= wb_resp.tid;
always_comb
	stall_i <= wb_resp.stall;
always_comb
	wb_req.cyc <= cyc_o;
always_comb
	wb_req.bte <= wb_burst_type_t'(bte_o);
always_comb
	wb_req.tid <= tid_o;
always_comb
	wb_req.cti <= wb_cycle_type_t'(cti_o);
always_comb
	wb_req.seg <= wb_segment_t'(seg_o);
always_comb
	wb_req.stb <= stb_o;
always_comb
	wb_req.we <= we_o;
always_comb
	wb_req.sel <= sel_o;
always_comb
	wb_req.adr <= adr_o;
always_comb
	wb_req.csr <= csr_o;
always_comb
	wb_req.dat <= dat_o;

reg [NTHREADS-1:0] gie;
reg [7:0] vl = 8'd8;
tid_t xrid,mc_rid,mc_rid1,mc_rid2,mc_rido;
order_tag_t [NTHREADS-1:0] insn_otag;
reg exndx_v,exndx1_v;
reg dcndx_v, wbndx_v;
reg agndx_v,oundx_v;
reg itndx_v;
reg rfndx1_v, rfndx2_v, rfndx3_v;
tid_t dcndx,itndx,exndx,exndx1,exndx2,oundx,wbndx,wbndx2,rfndx1,rfndx2,rfndx3,agndx;
reg xrid_v,mcrid_v;
reg [NTHREADS-1:0] dcb_v, rfb3_v, exb_v;
tid_t mcv_ridi, mcv_rido;
tid_t ithread, dthread, xthread, commit_thread;
reg dthread_v;
reg [3:0] commit_wr;
reg commit_wrv;
reg [63:0] commit_mask;
regspec_t commit_tgt;
vector_value_t commit_bus;
tid_t ip_thread, ip_thread1, ip_thread2, ip_thread3, ip_thread4, ip_thread5;
reg ip_thread_v,ip_thread1_v, ip_thread2_v, ip_thread3_v;
ThreadInfo_t [NTHREADS-1:0] thread;
ThreadInfo_t [NTHREADS-1:0] thread_hist [0:3];
code_address_t [NTHREADS-1:0] thread_ip;
reg [31:0] ip, ip2, ip3, ip4;
reg [NTHREADS-1:0] thread_busy;
code_address_t iip, dip, ip_icline, ip_insn, ip1;
instruction_t ir,dir,xir,bxir,mir,insn,mir1,mir2;
instruction_t rf_insn;
postfix_t pfx,irpfx,rf_pfx;
decode_bus_t deco;
decode_bus_t [NTHREADS-1:0] dco;
reg [6+TidMSB+1:0] ra0,ra1,ra2,ra3,ra4;
reg [6+TidMSB+1:0] ra0d,ra1d,ra2d,ra3d,ra4d;
value_t rfo0, rfo1, rfo2, rfo3, rfo4;
value_t ximm,mcimm,bxa,bxc;
ASID xasid;
vector_value_t vrfo0, vrfo1, vrfo2, vrfo3, vrfo4;
vector_value_t xa,xb,xc,xt,xm;
reg xta,xtb,xtt;
vector_value_t mca,mcb,mcc,mct,mcm;
vector_value_t mca1,mcb1,mcc1,mct1,mcm1;
vector_value_t mca2,mcb2,mcc2,mct2,mcm2;
reg [2:0] mca_busy;
value_t res,mc_res,mc_res1,mc_res2;
vector_value_t vres,mc_vres,mc_vres1,mc_vres2;
wire mc_done, mcv_done;
wire mc_done1, mcv_done1;
wire mc_done2, mcv_done2;
wire ihit,ihite,ihito;
reg ihit1,ihit2,ihit3,ihite2,ihito2,ihite1,ihito1;
memory_arg_t memreq;
memory_arg_t memresp;
wire memreq_full;
reg memresp_fifo_rd;
wire memresp_fifo_empty;
wire memresp_fifo_v;
wire [$bits(ICacheLine)*2-1:0] ic_line;
reg [$bits(ICacheLine)*2-1:0]  ic_line2;
wire ic_valid;
//reg [31:0] ptbr;
wire ipage_fault;
reg clr_ipage_fault;
wire itlbmiss;
reg clr_itlbmiss;
wire dce;
//reg [9:0] asid;
operating_mode_t omode [0:NTHREADS-1];
reg [NTHREADS-1:0] Usermode;
reg [NTHREADS-1:0] MUsermode;
wire takb;
wire [NTHREADS-1:0] ififo_almost_full;
reg [2:0] sp_sel [0:NTHREADS-1];
reg [2:0] sp_sel2;
reg [2:0] ic_sp_sel [0:NTHREADS-1];
reg [2:0] istk_depth [0:NTHREADS-1];
instruction_t [NTHREADS-1:0] exc_bucket;
code_address_t [NTHREADS-1:0] exc_ip;
cause_code_t icause,dcause;
reg [7:0] tid;
code_address_t last_adr;
reg [15:0] exv;
wire [25:0] ic_tage, ic_tago;
reg [25:0] ic_tag2e, ic_tag2o;
pipeline_reg_t [NTHREADS-1:0] dcbuf;
pipeline_reg_t [NTHREADS-1:0] mceb;
reg [NTHREADS-1:0] mem_rollback, ou_rollback, rollback, rolledback1, rolledback2;
regs_bitmap_t mem_rollback_bitmaps [0:NTHREADS-1];
regs_bitmap_t ou_rollback_bitmaps [0:NTHREADS-1];
regs_bitmap_t rollback_bitmaps [0:NTHREADS-1];
code_address_t [NTHREADS-1:0] rollback_ip;
reg [NTHREADS-1:0] rollback_ipv;
reg [NTHREADS-1:0] sb_will_issue, sb_issue;
wire [NTHREADS-1:0] sb_can_issue;
reg [NTHREADS-1:0] clr_ififo;
wire [NTHREADS-1:0] ififo_empty;
instruction_fetchbuf_t ic_ifb;
instruction_fetchbuf_t [NTHREADS-1:0] dc_ifb, dc1_ifb;
reg [5:0] imiss_count;
reg [NTHREADS-1:0] ou_stall, ex_stall, mc_stall, stall_pipe;
// pipeline fifo signals
reg exbrf_wr, memf_wr, mcbf_wr;
reg exbrf_rd, memf_rd, mcbf_rd;
pipeline_reg_t exbr, exbrf, memf, mcbi, mcbo, mcbf;
pipeline_reg_t memp;
wire exbrf_full, memf_full, mcbf_full;
wire exbrf_empty, memf_empty, mcbf_empty;
wire exbrf_v, memf_v, mcbf_v;
prec_t xprc, mprc;
code_address_t [NTHREADS-1:0] last_ip;

// CSRs
reg [31:0] cr0;
reg [31:0] ptbr [0:NTHREADS-1];
reg [9:0] asid [0:NTHREADS-1];
reg [31:0] hmask,xhmask;
address_t badaddr [0:NTHREADS-1][0:3];
cause_code_t cause [0:NTHREADS-1][0:3];
code_address_t tvec [0:3];
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
address_t [3:0] dbg_adr;
address_t [3:0] dbg_am;

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

reg stall_dec1,stall_dec2,stall_dec3,stall_dec4,stall_dec5;
reg stall_dec;
always_comb
	stall_dec2 =  (dco[dcndx].hasRa && dco[dcndx].Ra==dcb[rfndx1].dec.Rt && dcb[rfndx1].v && (dcb[rfndx1].dec.rfwr|dcb[rfndx1].dec.vrfwr) ||
							 dco[dcndx].hasRb && dco[dcndx].Rb==dcb[rfndx1].dec.Rt && dcb[rfndx1].v && (dcb[rfndx1].dec.rfwr|dcb[rfndx1].dec.vrfwr) ||
							 dco[dcndx].hasRc && dco[dcndx].Rc==dcb[rfndx1].dec.Rt && dcb[rfndx1].v && (dcb[rfndx1].dec.rfwr|dcb[rfndx1].dec.vrfwr))
							 ;
reg stall_rf,stall_rf1,stall_rf2,stall_rf3;
always_comb
	stall_rf2 = (rfb2[rfndx2].dec.Rc==exb[rfndx2].dec.Rt && (exb[rfndx2].dec.rfwr|exb[rfndx2].dec.vrfwr) && exb[rfndx2].v) ||
						(rfb2[rfndx2].dec.Rb==exb[rfndx2].dec.Rt && (exb[rfndx2].dec.rfwr|exb[rfndx2].dec.vrfwr) && exb[rfndx2].v) ||
						(rfb2[rfndx2].dec.Ra==exb[rfndx2].dec.Rt && (exb[rfndx2].dec.rfwr|exb[rfndx2].dec.vrfwr) && exb[rfndx2].v)
						;
always_ff @(posedge clk_g)
	stall_rf1 <= stall_rf2;
always_ff @(posedge clk_g)
	stall_rf3 <= stall_rf1;
always_comb
	stall_rf <= FALSE;//(stall_rf2|stall_rf1) & ~stall_rf3;

always_comb
	stall_dec <= (stall_dec2|stall_dec1|stall_dec3|stall_dec4) & !stall_dec5;
always_ff @(posedge clk_g)
	stall_dec1 <= stall_dec2;
always_ff @(posedge clk_g)
	stall_dec3 <= stall_dec1;
always_ff @(posedge clk_g)
	stall_dec4 <= stall_dec3;
always_ff @(posedge clk_g)
	stall_dec5 <= stall_dec4;

always_comb
	stall_pipe = ou_stall | ex_stall;

genvar g;

function [6:0] fnSpSel;
input tid_t thread;
input [6:0] i;
begin
	if (i==7'd31)
		case(sp_sel[thread])
		3'd1:	fnSpSel = 7'd60;
		3'd2:	fnSpSel = 7'd61;
		3'd3:	fnSpSel = 7'd62;
		3'd4:	fnSpSel = 7'd63;
		default:	fnSpSel = 7'd31;
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
	ir = OP_NOP;//_INSN;
	xir = OP_NOP;//_INSN;
	mir = OP_NOP;//_INSN;
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
	.tlbclk(clk_i),
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
	.ic_tage(ic_tage),
	.ic_tago(ic_tago),
	.fifoToCtrl_i(memreq),
	.fifoToCtrl_full_o(memreq_full),
	.fifoToCtrl_wack(memreq_wack),
	.fifoFromCtrl_o(memresp),
	.fifoFromCtrl_rd(memresp_fifo_rd),
	.fifoFromCtrl_empty(memresp_fifo_empty),
	.fifoFromCtrl_v(memresp_fifo_v),
	.bte_o(bte_o),
	.tid_o(tid_o),
	.cti_o(cti_o),
	.seg_o(seg_o),
	.cyc_o(cyc_o),
	.stb_o(stb_o),
	.stall_i(stall_i),
	.ack_i(ack_i),
	.next_i(next_i),
	.tid_i(tid_i),
	.we_o(we_o),
	.sel_o(sel_o),
	.adr_o(adr_o),
	.dat_i(dat_i),
	.dat_o(dat_o),
	.csr_o(csr_o),
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
	.deco(deco)
);


wire [3:0] mrt, ort;
ffo12 ufo9 (.i({11'd0,mem_rollback}), .o(mrt));
ffo12 ufo10 (.i({11'd0,ou_rollback}), .o(ort));
assign mem_rollback_thread = mrt;
assign ou_rollback_thread = ort;

always_comb
	for (n2 = 0; n2 < NTHREADS; n2 = n2 + 1) begin
		if (mem_rollback[n2]) begin
			rollback[n2] = TRUE;
			rollback_bitmaps[n2] = mem_rollback_bitmaps[n2];
		end
		else if (ou_rollback[n2]) begin
			rollback[n2] = TRUE;
			rollback_bitmaps[n2] = ou_rollback_bitmaps[n2];
		end
		else begin
			rollback[n2] = FALSE;
			rollback_bitmaps[n2] = 'd0;
		end
	end
	
generate begin : gScoreboard
for (g = 0; g < NTHREADS; g = g + 1) begin

	rfPhoenix_scoreboard uscb1
	(
		.rst(rst_i),
		.clk(clk_g),
		.db(ififo_empty[g] ? 'd0 : dco[g]),
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
	.ir(bxir),
	.a(bxa),
	.b(bxc),
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
	.rthread(dcndx),
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
	.prc(xprc),
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
	.i(mcbi),
	.o(mcbo),
	.done(mcv_done),
	.ridi(mcv_ridi),
	.rido(mcv_rido)
);

// Responses coming back from the memory pipeline can vary in latency. It is
// basically unknown, so some means of capturing results while the pipelines
// are active is needed. Fifos are used to store data until the writeback
// stage is ready for it.

rfPhoenix_pipeline_fifo uplf1
(
	.rst(rst_i),
	.clk(clk_g),
	.wr(exbrf_wr),
	.pin(exbr),
	.rd(exbrf_rd),
	.pout(exbrf),
	.cnt(),
	.almost_full(exbrf_full),
	.full(),
	.empty(exbrf_empty),
	.v(exbrf_v)
);

rfPhoenix_pipeline_fifo uplf2
(
	.rst(rst_i),
	.clk(clk_g),
	.wr(memf_wr),
	.pin(memp),
	.rd(memf_rd),
	.pout(memf),
	.cnt(),
	.almost_full(memf_full),
	.full(),
	.empty(memf_empty),
	.v(memf_v)
);

rfPhoenix_pipeline_fifo uplf3
(
	.rst(rst_i),
	.clk(clk_g),
	.wr(mcbf_wr),
	.pin(mcbo),
	.rd(mcbf_rd),
	.pout(mcbf),
	.cnt(),
	.almost_full(mcbf_full),
	.full(),
	.empty(mcbf_empty),
	.v(mcbf_v)
);

always_ff @(posedge clk_g)
	ic_line2 <= ic_line;
always_ff @(posedge clk_g)
	ip_insn <= ip_icline;
always_ff @(posedge clk_g)
	ihit1 <= ihit;
always_ff @(posedge clk_g)
	ihit2 <= ihit1;
always_ff @(posedge clk_g)
	ihit3 <= ihit2;
always_ff @(posedge clk_g)
	ihite1 <= ihite;
always_ff @(posedge clk_g)
	ihite2 <= ihite1;
always_ff @(posedge clk_g)
	ihito1 <= ihito;
always_ff @(posedge clk_g)
	ihito2 <= ihito1;
always_ff @(posedge clk_g)
	ic_tag2e <= ic_tage;
always_ff @(posedge clk_g)
	ic_tag2o <= ic_tago;
reg [NTHREADS-1:0] wr_ififo;
wire [$bits(dco)+$bits(dc_ifb)-1:0] ififo_out [0:NTHREADS-1];
generate begin
for (g = 0; g < NTHREADS; g = g + 1) begin
	always_comb
		clr_ififo[g] <= rollback[g];
	always_ff @(posedge clk_g)
	if (rst_i)
		sb_issue[g] <= 'd0;
	else
		sb_issue[g] <= sb_will_issue[g];
	always_comb
		wr_ififo[g] <= ic_ifb.thread==g && ic_ifb.v && ic_ifb.insn.any.opcode!=OP_PFX && ic_ifb.ip != last_ip[g];
	always_ff @(posedge clk_g)
	if (rst_i)
		last_ip[g] <= 'd0;
	else if (ic_ifb.thread==g && ic_ifb.v && ic_ifb.insn.any.opcode!=OP_PFX)
		last_ip[g] <= ic_ifb.ip;

	rfPhoenix_fifo #(.WID($bits({deco,ic_ifb})), .DEP(16)) ufifo1
	(
		.rst(rst_i|rollback[g]),
		.clk(clk_g),
		.wr(wr_ififo[g]),
		.di(rollback[g] ? 'd0 : {deco,ic_ifb}),
		.rd(sb_will_issue[g]),
		.dout(ififo_out[g]),		
		.cnt(),
		.almost_full(ififo_almost_full[g]),
		.full(),
		.empty(ififo_empty[g]),
		.v()
	);
	always_comb
		{dco[g],dc_ifb[g]} = rollback[g] ? 'd0 : ififo_out[g];

/*
	rfPhoenix_insn_fifo #(.DEP(16)) ufifo1
	(
		.rst(rst_i|rollback[g]),
		.clk(clk_g),
		.wr(1'b0),//wr_ififo[g]),
		.decin(rollback[g] ? 'd0 : deco),
		.ifbin(rollback[g] ? 'd0 : ic_ifb),
		.rd(1'b0),//sb_issue[g]),
		.decout(dco[g]),
		.ifbout(dc_ifb[g]),
		.cnt(),
		.full(),
		.almost_full(ififo_almost_full[g]),
		.empty(ififo_empty[g]),
		.v()
	);
*/
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
	tvec[2'd0] <= RSTIP;
	tvec[2'd1] <= RSTIP;
	tvec[2'd2] <= RSTIP;
	tvec[2'd3] <= RSTIP;
	ir <= OP_NOP;//_INSN;
	xir <= OP_NOP;//_INSN;
	mir <= OP_NOP;//_INSN;

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
		insn_otag[n10] <= 'd1;
		tmpadr[n10] <= 'd0;
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
	memp <= 'd0;
	ic_ifb <= 'd0;
	exndx2 <= 'd0;
end
endtask

task tOnce;
integer n;
begin
	rd_trace <= 1'b0;
	memreq.wr <= 1'b0;
	memresp_fifo_rd <= 1'b0;
	for (n = 0; n < NTHREADS; n = n + 1) begin
		mem_rollback[n] <= FALSE;
		ou_rollback[n] <= FALSE;
		if (ou_rollback[n])
			ou_rollback_bitmaps[n] <= 'd0;
	end
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
// The thread selected here also flows into instruction decode which is
// then added to the instruction fifo for the thread.

reg [NTHREADS-1:0] itsel;
generate begin : gItsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			itsel[g] = !ififo_almost_full[g] & cr0[g];
end
endgenerate

roundRobin rr1
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,itsel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(itndx)
);

assign itndx_v = |itsel;

// dcndx selects a decoded instruction from the fifo for register fetch.
// The execution buffer for the thread must be empty and the scoreboard
// must indicate there are no dependencies on the instruction. There also
// must be an instruction in the fifo. dcndx will not issue if there is
// a pipeline stall due to a conflict for the memory fifo.
// dcndx is also used indirectly to select the thread for register fetch
// as the decoded register select signals are fed to the register file.
// Two cycles later the register file values are available the thread to
// update will have been dcndx delayed by two cycles. 

reg [NTHREADS-1:0] dcsel;
generate begin : gDcsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			dcsel[g] = //!dcsel2[g] &&
				sb_can_issue[g] &&							// There are no dependencies
				(!dcb[g].v || !stall_pipe[g]) &&	// The buffer is empty or is going to be empty.
				cr0[g] &&	// The thread is enabled
				!ififo_empty[g];// &&	// There is something in the fifo
				//!stall_pipe[g];// &&				// No stall at end of pipe
				//!stall_dec && !stall_dec1 && 
				//sb_will_issue[g];
end
endgenerate

tid_t dcndx1;
roundRobin rr2
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,dcsel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(dcndx)
);

always_comb
	dcndx_v <= |dcsel;

generate begin : gIssue
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			sb_will_issue[g] = dcndx_v && dcndx==g;//g==dcndx && dcndx_v;// &&	!sb_issue[g];
end
endgenerate

reg [NTHREADS-1:0] mcsel;
generate begin : gMcsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			mcsel[g] = 	exb[g].dec.multicycle &&	// multi-cycle
									exb_v[g];						// and it is valid
end
endgenerate

wire tid_t mcndx;
roundRobin rr3
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,mcsel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(mcndx)
);

// Generate stall signal for threads that could have sent instructions down
// the execute pipe but did not get selected.
always_comb
	mc_stall <= (mcsel & ~(16'd1 << mcndx)) | ({NTHREADS{mcbf_full}} & mcsel);

// exndx selects the thread to move to the execution stage. To be selected the
// register file values must have been fetched.

reg [NTHREADS-1:0] exsel;
generate begin : gExsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			exsel[g] = 	!rfb3[g].dec.mem &&	// not memory
									!rfb3[g].dec.multicycle &&	// not multi-cycle
									 rfb3_v[g];						// and it is valid
end
endgenerate

roundRobin rr4
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,exsel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(exndx)
);

always_comb
	exndx_v = |exsel;
always_ff @(posedge clk_g)
	exndx1_v <= exndx_v;
always_ff @(posedge clk_g)
	exndx1 <= exndx;

// Generate stall signal for threads that could have sent instructions down
// the execute pipe but did not get selected.
always_comb
	ex_stall <= (exsel & ~(16'd1 << exndx)) | ({NTHREADS{exbrf_full}} & exsel) | mc_stall;

// Select thread for address generation
/*
reg [NTHREADS-1:0] agsel;
generate begin : gAgsel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			agsel[g] = (exb[g].out|exb[g].executed) & cr0[g];
end
endgenerate

roundRobin rr4
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,agsel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(agndx)
);

assign agndx_v = |agsel;
*/
// Pick an rob entry thats had its address generated.

// Copy into a bus, just wires.
/*
reg [NTHREADS-1:0] aggen;
generate begin : gAggen
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			aggen[g] = agb[g].agen & cr0[g];
end
endgenerate

*/
reg [NTHREADS-1:0] ousel;
generate begin : gOusel
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			ousel[g] = agb[g].dec.mem && agb[g].v;// &&
//				!(memreq_full || req_icload)
//				;
end
endgenerate

roundRobin rr5
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,ousel}),
	.lock(8'h00),
	.sel(),
	.sel_enc(oundx)
);

wire req_icload = !ihit1 && !memreq_full && (ip1[31:5] != last_adr[31:5] || imiss_count > 30);

always_comb
	ou_stall = (ousel & ~({16'd0,|ousel} << oundx)) | (({16'd0,(memreq_full||req_icload) && agb[oundx].dec.mem}) << oundx);

// The following is dead code. The instruction for writeback is now chosen from
// a pipeline fifo.

// Pick a finished instruction. wbndx selects which thread is written back to
// the register file.

// Copy into a bus, just wires.
/*
reg [NTHREADS-1:0] oubfin;
generate begin : gOubfin
	for (g = 0; g < NTHREADS; g = g + 1)
		always_comb
			oubfin[g] = oub[g].executed & cr0[g];
end
endgenerate

roundRobin rr6
(
	.rst(rst_i),
	.clk(clk_g),
	.ce(1'b1),
	.req({8'h00,oubfin}),
	.lock(8'h00),
	.sel(),
	.sel_enc(wbndx)
);

always_comb
	wbndx_v <= |oubfin;
*/

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Instruction Pointers
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tIfIp;
begin
	if (itndx_v) begin
		if (ihit) begin
			thread_ip[itndx] <= thread_ip[itndx] + 4'd5;
		/* The following for when instructions do not cross cache lines.
			if (thread_ip[itndx][5:0] < 6'd55)
				thread_ip[itndx][5:0] <= thread_ip[itndx][5:0] + 4'd5;
			else begin
				thread_ip[itndx][5:0] <= 6'd0;
				thread_ip[itndx][31:6] <= thread_ip[itndx][31:6] + 2'd1;
			end
		*/
		end
	end
	/*
	if (ip_thread2_v && !ihit1) begin
		if (thread[ip_thread2].imiss[0]==1'b0)
			thread_ip[ip_thread2] <= ip_icline;
		else
			thread_ip[ip_thread2] <= thread[ip_thread2].miss_ip;
	end
	*/
end
endtask

task tExIp;
begin
	case(exb[exndx].ifb.insn.any.opcode)
	OP_CALL:	thread_ip[exndx] <= exb[exndx].dec.imm;
	OP_BSR:	thread_ip[exndx] <= exb[exndx].dec.imm + exb[exndx].ifb.ip;
	OP_RET:	
		if (exb[exndx].ifb.insn[39])
			thread_ip[exndx] <= exb[exndx].a[0] + exb[exndx].dec.imm;
		else
			thread_ip[exndx] <= exb[exndx].a[0];
	OP_Bcc:
		if (takb) 
			thread_ip[exndx] <= exb[exndx].dec.imm + exb[exndx].ifb.ip;
		else
			tIfIp();
	OP_FBcc:
		if (takb)
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
		{ic_ifb.pfx2,ic_ifb.pfx,ic_ifb.insn} <= ic_line >> {ip_icline[4:0],3'b0};
		ic_ifb.ic <= ic_line[511:510];
		ic_ifb.ip <= ip_icline;
		ic_ifb.v <= ihit3 && ip_thread2_v;
		ic_ifb.sp_sel <= sp_sel[ip_thread2];
		ic_ifb.thread <= ip_thread2;
		ic_ifb.tag <= insn_otag[ip_thread2];
		if (ihit3)
			insn_otag[ip_thread2] <= insn_otag[ip_thread2] + 2'd1;
		// External interrupt has highest priority.
		if (irq_i > status[ip_thread2][0].ipl && gie[ip_thread2] && status[ip_thread2][0].mie)
			ic_ifb.cause <= cause_code_t'({irq_i,8'h00}|FLT_IRQ);
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
//		if (itndx_v) begin
//			thread[itndx].ip <= thread[itndx].ip + 4'd5;
//		end
		ip_thread1 <= itndx;			// tag lookup ip_thread1 lined up with ip
		ip_thread2 <= ip_thread1;	// data fetch
		ip_thread3 <= ip_thread2;	// ip_thread3 lined up with ip_icline
		ip_thread4 <= ip_thread3;
		ip_thread5 <= ip_thread4;
		ip_thread1_v <= itndx_v;
		ip_thread2_v <= ip_thread1_v;
		ip_thread3_v <= ip_thread2_v;
				/*
		if (ip_thread2_v) begin
			if (!ihit1) begin
				ic_ifb.v <= 1'b0;
				$display("Miss %d ip=%h", ip_thread2, ip_icline);
				if (thread[ip_thread2].imiss[0]==1'b0) begin
					thread[ip_thread2].imiss <= 5'b00111;
					thread[ip_thread2].ip <= ip_icline;
					thread[ip_thread2].miss_ip <= ip_icline;
				end
				else begin
					thread[ip_thread2].ip <= thread[ip_thread2].miss_ip;
					thread[ip_thread2].imiss[0] <= 1'b1;
				end
			end
		end
				*/
		// On a miss, request a cache line load from the memory system. This
		// should eventually cause a hit for the thread.
		// The old cache line is passed back for the victim buffer.
		if (!ihit1 && ip_thread2_v) begin
			if (!memreq_full) begin
				if (ip1[31:5] != last_adr[31:5] || imiss_count > 30) begin
					imiss_count <= 'd0;
					last_adr <= ip1;
					tid <= tid + 2'd1;
					memreq.tid <= tid;
					memreq.thread <= ip_thread2;
					memreq.wr <= 1'b1;
					memreq.func <= MR_ICACHE_LOAD;
					memreq.omode <= status[ip_thread2][0].om;
					memreq.asid <= asid[ip_thread2];
					memreq.adr <= {ip1[31:5],5'd0};
					memreq.vcadr <= {ip1[31:5],5'd0};//{ic_tag,6'b0};
					memreq.res <= ic_line;
					memreq.sz <= ic_valid ? tetra : nul;
					// But, which line do we need?
					memreq.hit <= {ihito1,ihite1};
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
// Only one thread is selected for further processing. That means the other
// threads are invalid at this stage. If no thread can be selected then a
// pipeline bubble results.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tDecode;
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1) begin
		if (stall_pipe[n])
			dcb[n] <= dcb[n];
		else begin
			dcb[n].thread <= n;
//				dcb[n].v <= dc_ifb[n].v;
			dcb[n].ifb <= dc_ifb[n];
			dcb[n].dec <= dco[n];
			// Needed for virtualization
			if (dco[n].csr && omode[n]!=OM_MACHINE)
				dcb[n].cause <= FLT_CSR;
			if (n==dcndx) begin
				ra0 <= {dcndx,fnSpSel(n,dco[n].Ra)};
				ra1 <= {dcndx,fnSpSel(n,dco[n].Rb)};
				ra2 <= {dcndx,fnSpSel(n,dco[n].Rc)};
				ra3 <= {dcndx,dco[n].Rm.num};
//				ra4 <= {dcndx,fnSpSel(n,dco[n].Rt)};
			end
		end
	end
end
endtask

always_comb
	for (n4 = 0; n4 < NTHREADS; n4 = n4 + 1) begin
		dcb_v[n4] <= (dcb[n4].ifb.ip != dc_ifb[n4].ip) && dc_ifb[n4].v;
		
		rfb3_v[n4] <= (rfb3[n4].ifb.ip != dcb[n4].ifb.ip);// && dc_ifb[n4].v;
			
		exb_v[n4] <= (exb[n4].ifb.ip != rfb3[n4].ifb.ip);// && (dcb_v[n4]|dc_ifb[n4].v);
//			(rfb3_v[n4] || (rfb3[n4].ifb.ip == dcb[n4].ifb.ip && dcb_v[n4]));
	end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF Stage
// Forces instructions to be ignored until the rollback target address is seen.
// There are two stages to register fetch since two clocks are required to read
// the register file.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

always_ff @(posedge clk_g)
	ra0d <= stall_pipe ? ra0d : ra0;
always_ff @(posedge clk_g)
	ra1d <= stall_pipe ? ra1d : ra1;
always_ff @(posedge clk_g)
	ra2d <= stall_pipe ? ra2d : ra2;
always_ff @(posedge clk_g)
	ra3d <= stall_pipe ? ra3d : ra3;

value_t csro;
always_comb
	tReadCSR(csro,rfndx1,rfb1[rfndx1].dec.imm[13:0]);
	
vector_value_t opera,operb,operc;
always_comb
	if (ra0d==7'd0 && !ra0d[6])
		opera = 'd0;
	else if (ra0d==7'd59 && !ra0d[6])
		opera = {NLANES{dcb[n].ifb.ip}};
//	else if (dcb[n].dec.Ra == exb[n].dec.Rt)
//		opera = vres;
	else if (ra0d[6])
		opera = vrfo0;
	else
		opera = {NLANES{rfo0}};

always_comb
	if (ra1d==7'd0 && !ra1d[6])
		operb = 'd0;
	else if (ra1d==7'd59 && !ra1d[6])
		operb = {NLANES{dcb[n].ifb.ip}};
//	else if (dcb[n].dec.Rb == exb[n].dec.Rt)
//		operb = vres;
	else if (ra1d[6])
		operb = vrfo1;
	else
		operb = {NLANES{rfo1}};

always_comb
	if (ra2d==7'd0 && !ra2d[6])
		operc = 'd0;
	else if (ra2d==7'd59 && !ra2d[6])
		operc = {NLANES{dcb[n].ifb.ip}};
	else if (dcb[n].dec.csr)
		operc = {NLANES{csro}};
//	else if (dcb[n].dec.Rc == exb[n].dec.Rt)
//		operc = vres;
	else if (ra2d[6])
		operc = vrfo2;
	else
		operc = {NLANES{rfo2}};

task tRegf;
input tid_t n;
begin
	rfb3[n].a <= opera;
	rfb3[n].b <= operb;
	rfb3[n].c <= operc;
	rfb3[n].mask <= rfo3;
	if (REGFILE_LATENCY==2) begin
		if ((dcb[n].dec.rfwr & ~dcb[n].dec.Rt.vec) | (dcb[n].dec.vrfwr & dcb[n].dec.Rt.vec))
			ou_rollback_bitmaps[n][dcb[n].dec.Rt] <= 1'b1;
	end
	// ToDo: update for more latencies.
	else if (REGFILE_LATENCY==3) begin
	end
	else if (REGFILE_LATENCY==4) begin
	end
end
endtask

// There will always be register values available two cycles after the decode
// stage regardless of a pipeline stall. So, a pipeline stall is not checked for
// at regfetch.

task tRegfetch;
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1) begin

		if (REGFILE_LATENCY==2) begin
			rfndx3 <= dcndx;
			rfb3[n] <= dcb[n];
			if (rollback[n] && rollback_ipv[n] && dcb[n].ifb.ip == rollback_ip[n]) begin
				rollback_ipv[n] <= INV;
				if (n==rfndx3)
					tRegf(n);
			end
			else begin
				if (n==rfndx3)
					tRegf(n);
			end
		end
	
		else if (REGFILE_LATENCY==3) begin
			rfndx2 <= dcndx;
			rfndx3 <= rfndx2;
			rfb2[n] <= dcb[n];
			rfb2[n].v <= dcb_v[n];
			rfb3[n] <= rfb2[n];
			if (rollback[n] && rollback_ipv[n] && rfb2[n].ifb.ip == rollback_ip[n]) begin
				rollback_ipv[n] <= INV;
				if (n==rfndx3)
					tRegf(n);
			end
			else begin
				if (n==rfndx3)
					tRegf(n);
			end
		end

		else if (REGFILE_LATENCY==4) begin
			rfndx1 <= dcndx;
			rfndx2 <= rfndx1;
			rfndx3 <= rfndx2;
			rfb1[n] <= dcb;
			rfb1[n].v <= dcb_v[n];
			rfb2[n] <= rfb1[n];
			rfb3[n] <= rfb2[n];
			if (rollback[n] && rollback_ipv[n] && rfb3[n].ifb.ip == rollback_ip[n]) begin
				rollback_ipv[n] <= INV;
				if (n==rfndx3)
					tRegf(n);
			end
			else begin
				if (n==rfndx3)
					tRegf(n);
			end
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// EX stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tExCall;
input tid_t exndx;
begin
	if (rfb3_v[exndx])
	case(rfb3[exndx].ifb.insn.any.opcode)
	OP_PFX:	;
	OP_NOP:	;
	OP_CALL:
		begin
			thread[exndx].ip <= rfb3[exndx].dec.imm;
			rollback_ip[exndx] <= rfb3[exndx].dec.imm;
			rollback_ipv[exndx] <= 1'b1;
			exb[exndx].res <= 'd0;
			exb[exndx].res[0] <= rfb3[exndx].ifb.ip + 4'd5;
			ou_rollback[exndx] <= 1'b1;
			ou_rollback_bitmaps[exndx][rfb3[exndx].dec.Rt] <= 1'b1;
		end
	OP_BSR:
		begin
			thread[exndx].ip <= rfb3[exndx].dec.imm + rfb3[exndx].ifb.ip;
			rollback_ip[exndx] <= rfb3[exndx].dec.imm + rfb3[exndx].ifb.ip;
			rollback_ipv[exndx] <= 1'b1;
			exb[exndx].res <= 'd0;
			exb[exndx].res[0] <= rfb3[exndx].ifb.ip + 4'd5;
			ou_rollback[exndx] <= 1'b1;
			ou_rollback_bitmaps[exndx][rfb3[exndx].dec.Rt] <= 1'b1;
		end
	OP_RET:
		begin
			thread[exndx].ip <= rfb3[exndx].a[0];
			rollback_ip[exndx] <= rfb3[exndx].a[0];
			rollback_ipv[exndx] <= VAL;
			ou_rollback[exndx] <= TRUE;
			ou_rollback_bitmaps[exndx][rfb3[exndx].dec.Rt] <= TRUE;
		end
	default:	;
	endcase
end
endtask

task tExBranch;
input tid_t exndx;
begin
	if (rfb3[exndx].dec.br & rfb3_v[exndx]) begin
		if (takb) begin
			thread[exndx].ip <= rfb3[exndx].ifb.ip + rfb3[exndx].dec.imm;
			rollback_ip[exndx] <= rfb3[exndx].ifb.ip + rfb3[exndx].dec.imm;
			rollback_ipv[exndx] <= TRUE;
			ou_rollback[exndx] <= TRUE;
		end
	end
end
endtask


// This combo to feed branch evaluation to avoid needing another pipeline stage.
always_comb
	for (n3 = 0; n3 < NTHREADS; n3 = n3 + 1) begin
		if (n3==exndx && exndx_v) begin
			bxir <= rfb3[n3].ifb.insn;
			bxa <= rfb3[n3].a[0];
			bxc <= rfb3[n3].c[0];
		end
	end

task tExecute;
integer n;
begin
	mcbi.v <= 1'b0;
	exbr.v <= FALSE;
	exbrf_wr <= FALSE;

	for (n = 0; n < NTHREADS; n = n + 1) begin

		if (stall_pipe[n])
			exb[n] <= exb[n];
		else
			exb[n] <= rfb3[n];

		if (!stall_pipe[n]) begin
			exb[n].regfetched <= 1'b0;
			exb[n].retry <= 'd0;

			if (rfb3[n].dec.multicycle) begin
				if (n==mcndx && |mcsel) begin
					mcbi <= rfb3[n];
					mcbi.v <= 1'b1;
					mir <= rfb3[n].ifb.insn;
					mprc <= rfb3[n].dec.prc;
					mca <= rfb3[n].a;
					mcb <= rfb3[n].b;
					mcc <= rfb3[n].c;
					mct <= rfb3[n].t;
					mcimm <= rfb3[n].dec.imm;
					mcm <= rfb3[n].mask;
				end
			end
			else begin
				if (n==exndx && exndx_v) begin
					xir <= rfb3[n].ifb.insn;
					xprc <= rfb3[n].dec.prc;
					xa <= rfb3[n].a;
					xb <= rfb3[n].b;
					xc <= rfb3[n].c;
					xt <= rfb3[n].t;
					xta <= rfb3[n].dec.Ta;
					xtb <= rfb3[n].dec.Tb;
					xtt <= rfb3[n].dec.Tt;
					xm <= rfb3[n].mask;
					ximm <= rfb3[n].dec.imm;
					xasid <= asid[n];
					xhmask <= hmask;
				end
			end
			tExCall(n);
			tExBranch(n);
		end
	end

	if (exndx1_v)
		if (!stall_pipe[exndx1] & exb_v[exndx1] & ~exb[exndx1].dec.mem) begin
			exbr <= exb[exndx1];
			exbr.v <= exb_v[exndx1];
			exbr.res <= vres;
			exbrf_wr <= TRUE;
		end

	mcbf_wr <= FALSE;
	if (|mcsel) begin
		//mcb.tag <= 
		mcbf_wr <= TRUE;
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

address_t [NTHREADS-1:0] tmpadr;

task tAgen;	// placeholder task
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1) begin
		if (stall_pipe[n])
			agb[n] <= agb[n];
		else begin
			if (wbb[n].dec.need_steps && wbb[n].dec.mem && wbb[n].count != 'd0)
				agb[n] <= wbb[n];
			else if (exb[n].dec.mem) begin
				agb[n] <= exb[n];
				agb[n].v <= exb_v[n];
			end
			else
				agb[n].v <= INV;
			// Get result
			if (!exb[n].dec.cjb)
				agb[n].res <= vres;
			agb[n].agen <= 1'b1;
			casez({exb[n].dec.storer|exb[n].dec.loadr,exb[n].dec.Rb.vec,exb[n].dec.Ra.vec})
			3'b000:	tmpadr[n] <= exb[n].a[0] + exb[n].b[0];
			3'b001: tmpadr[n] <= exb[n].a[exb[n].step] + exb[n].b[0];
			3'b010:	tmpadr[n] <= exb[n].a[0] + exb[n].b[exb[n].step];
			3'b011:	tmpadr[n] <= exb[n].a[exb[n].step] + exb[n].b[exb[n].step];
			3'b1?0:	tmpadr[n] <= exb[n].a[0] + exb[n].dec.imm;
			3'b1?1:	tmpadr[n] <= exb[n].a[exb[n].step] + exb[n].dec.imm;
			endcase
			if (exb[n].dec.mem)
				thread[n].sleep <= TRUE;
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OU stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tOuLoad;
input tid_t oundx;
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
		memreq.tag <= agb[oundx].ifb.tag;
		// Skip masked memory operation when mask is zero
		if (agb[oundx].dec.need_steps && agb[oundx].mask=='d0)
			memreq.wr <= 1'b0;
		else begin
			memreq.wr <= 1'b1;
			thread[oundx].sleep <= TRUE;
		end
		memreq.func <= agb[oundx].dec.loadu ? MR_LOADZ : MR_LOAD;
		if (agb[oundx].dec.ldsr)
			memreq.func2 <= MR_LDR;
		else
			memreq.func2 <= MR_NOP;
		memreq.load <= agb[oundx].dec.load;
		memreq.store <= agb[oundx].dec.store;
		memreq.need_steps <= agb[oundx].dec.need_steps;
		memreq.sz <= agb[oundx].dec.memsz;
		memreq.omode <= mprv[oundx] ? status[oundx][1].om : status[oundx][0].om;
		memreq.asid <= asid[oundx];
		memreq.adr <= tmpadr[oundx];
		case(agb[oundx].dec.memsz)
		byt:	memreq.sel <= 64'h1;
		wyde:	memreq.sel <= 64'h3;
		tetra:	memreq.sel <= 64'hF;
		vect:	memreq.sel <= 64'hFFFFFFFFFFFFFFFF;
		default:	memreq.sel <= 64'hF;
		endcase
		// Try the same address again on a cache miss.
		// Cannot get a cache miss anymore.
		if (agb[oundx].cause != FLT_DCM) begin
			if (agb[oundx].dec.memsz==vect) begin
				if (agb[oundx].dec.loadr) begin
					if (agb[oundx].dec.Ra.vec) begin
						if (agb[oundx].step < NLANES-1) begin
							oub[oundx].step <= agb[oundx].step + 2'd1;
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
					if (agb[oundx].step < NLANES-1 && agb[oundx].dec.loadn)
						oub[oundx].step <= oub[oundx].step + 2'd1;
				end
			end
		end
		// This bit of code should be dead.
		else if (agb[oundx].retry < 3'd5) begin
			oub[oundx].retry <= agb[oundx].retry + 2'd1;
			oub[oundx].cause <= FLT_NONE;
		end
	end
end
endtask

task tOuStore;
input tid_t oundx;
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
		memreq.tag <= agb[oundx].ifb.tag;
		// Skip masked memory operation when mask is zero
		if (agb[oundx].dec.need_steps && agb[oundx].mask=='d0)
			memreq.wr <= FALSE;
		else begin
			memreq.wr <= TRUE;
			thread[oundx].sleep <= TRUE;
		end
		memreq.func <= MR_STORE;
		if (agb[oundx].dec.stcr)
			memreq.func2 <= MR_STC;
		else
			memreq.func2 <= MR_NOP;
		memreq.load <= agb[oundx].dec.load;
		memreq.store <= agb[oundx].dec.store;
		memreq.need_steps <= agb[oundx].dec.need_steps;
		memreq.sz <= agb[oundx].dec.memsz;
		memreq.omode <= mprv[oundx] ? status[oundx][1].om : status[oundx][0].om;
		memreq.asid <= asid[oundx];
		memreq.adr <= tmpadr[oundx];
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
			memreq.sel <= (
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
					{4{agb[oundx].mask[0]}}} >> {agb[oundx].step,2'h0}) & 64'hFFFF;
			if (agb[oundx].dec.storen)
				memreq.sz <= tetra;
			// For a scatter store select the current vector element, otherwise select entire vector (set above).
			if (agb[oundx].dec.storen) begin
				memreq.sel <= 64'h000000000000000F;	// 32 bit at a time
				// Dont bother storing if masked
				if (!agb[oundx].mask[agb[oundx].step])
					memreq.wr <= 1'b0;
				memreq.res <= agb[oundx].t[agb[oundx].step];
			end
			if (agb[oundx].dec.storer) begin
				if (agb[oundx].dec.Ra.vec && agb[oundx].step < NLANES-1)
					oub[oundx].step <= agb[oundx].step + 5'd1;
				else if (agb[oundx].step < NLANES-4)
					oub[oundx].step <= agb[oundx].step + 5'd16;
			end
			// For scatter increment step
			if (agb[oundx].step < NLANES-1 && agb[oundx].dec.storen)
				oub[oundx].step <= agb[oundx].step + 2'd1;
		end
	end
end
endtask


task tOut;
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1) begin
		if (stall_pipe[n])
			oub[n] <= oub[n];
		else begin
			oub[n] <= agb[n];
			oub[n].agen <= 1'b1;
			if ((agb[n].dec.Ra.vec | ((agb[n].dec.storen|agb[n].dec.loadn) & agb[n].dec.Rb.vec)) && agb[n].dec.memsz==vect) begin
				if (agb[n].step < NLANES-1) begin
					oub[n].agen <= 1'b0;
				end
			end
			if (agb[n].agen & agb[n].v) begin
				if (!memreq_full && !req_icload) begin
					if (|ousel) begin
						if (n==oundx) begin
							tOuLoad(oundx);
							tOuStore(oundx);
						end
					end
				end
				// If the load/store could not be queued backout the decoded and out
				// indicators so the instruction will be reselected for execution.
				else begin
					if (agb[n].dec.load|agb[n].dec.store) begin
						oub[n].regfetched <= 1'b1;
						oub[n].agen <= 1'b0;
					end
				end
			end
			if (mcrid_v) begin
				if (exb[mc_rid].dec.is_vector ? mcv_done : mc_done) begin
					mcrid_v <= 1'b0;
					mca_busy[0] <= 1'b0;
		//			if (exb[mc_rid].dec.Tt)
						oub[mc_rid].res <= mc_vres;
		//			else
		//				exb[mc_rid].res <= mc_res;
				end
			end
		end
	end
end
endtask


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MEM stage 
// - process responses coming back from the BIU for requests sent by EX stage.
// - the responses are sent to a fifo at the tail of the memory pipe
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tMemory;
integer n;
begin
	memf_wr <= FALSE;
	if (!memresp_fifo_empty && !memf_full)
		memresp_fifo_rd <= 1'b1;
	if (memresp_fifo_rd) begin
		thread[memresp.thread].sleep <= FALSE;
		// For a load the tag, target register, result and cause code are needed
		// For a store the tag and cause code are needed.
		// The writeback stage wants to see a pipeline register as input, but to
		// conserver hardware the memory pipeline does not include everything from
		// the main pipeline. So, some of the field information needs to be
		// updated here.
		memf_wr <= TRUE;	// update the pipeline fifo
		memp <= 'd0;
		memp.ifb.ip <= memresp.ip;
		memp.ifb.thread <= memresp.thread;
		memp.badAddr <= memresp.adr;
		memp.ifb.tag <= memresp.tag;
		memp.dec.Rt <= memresp.tgt;	// Needed for a load
		memp.cause <= memresp.cause;
		memp.dec.rfwr <= memresp.wr_tgt && memresp.tgt.vec==1'b0;
		memp.dec.vrfwr <= memresp.wr_tgt && memresp.tgt.vec==1'b1;
		memp.dec.mem <= TRUE;
		memp.dec.need_steps <= memresp.need_steps;
		memp.count <= memresp.count;
		memp.step <= memresp.step;
		memp.mask <= {64{1'b1}}; // ToDo fix this
	
		// Clear the imiss status. The thread might still miss again if the I$
		// has not updated before the thread is selected again, but at least
		// it can be prevented from being selected for a few cycles while the
		// imiss request is processed.
		if (memresp.func==MR_ICACHE_LOAD)
			;
//			for (n = 0; n < NTHREADS; n = n + 1)
//				thread[n].imiss <= 1'b0;
		else begin
			begin
				// If a gather load
				if (memresp.load && memresp.need_steps && memresp.sz==vect) begin
					if (memresp.step < NLANES-1) begin
						;
					end
					memp.res[memresp.step] <= memresp.res;
				end
				// Other load
				else if (memresp.load) begin
					memp.res <= memresp.res;
				end
				// Scatter store
				else if (memresp.store && memresp.need_steps && memresp.sz==vect) begin
					memp.res <= memresp.res;
					if (memp.step!=NLANES-1) begin
						;
					end
				end
				// Other store / Other Op
				else
					memp.res <= memresp.res;
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
	if (omode[wbndx] <= wbb[wbndx].ifb.insn[7:6]) begin
		tWbException(wbb[wbndx].ifb.ip,FLT_PRIV,1);
	end
	else begin
		status[wbndx][0].om <= operating_mode_t'(wbb[wbndx].ifb.insn[7:6]);	// omode
		status[wbndx][0].pl <= wbb[wbndx].a[0][7:0];
		cause[wbndx][wbb[wbndx].ifb.insn[7:6]] <= cause[wbndx][2'd3];
		badaddr[wbndx][wbb[wbndx].ifb.insn[7:6]] <= badaddr[wbndx][2'd3];
		ip <= tvec[wbb[wbndx].ifb.insn[7:6]] + {omode[wbndx],6'h00};
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
		status[wbndx][7].om <= OM_MACHINE;
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
		tWbException(wbb[wbndx].ifb.ip,FLT_RTI,1);
end
endtask

task tWbException;
input code_address_t ip;
input cause_code_t cc;
input keepIrq;
integer n;
begin
	if (istk_depth[wbndx] < 3'd7) begin
		for (n = 1; n < 8; n = n + 1)
			status[wbndx][n] <= status[wbndx][n-1];
		status[wbndx][0].om <= OM_MACHINE;		// select machine operating mode
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
	if (wbb[oundx].dec.mem) begin
		mem_rollback[wbndx] <= TRUE;
		ou_rollback[wbndx] <= TRUE;
	end
	else begin
		ou_rollback[wbndx] <= TRUE;
	end
end
endtask

reg select_exbr;
reg select_memf;
reg select_mcbf;
always_comb
begin
	select_exbr = !exbrf_empty && ((exbrf.ifb.tag <= memf.ifb.tag) || !memf.v || (exbrf.ifb.thread != memf.ifb.thread)) &&
		((exbrf.ifb.tag <= mcbf.ifb.tag) || !mcbf.v || (exbrf.ifb.thread != mcbf.ifb.thread));
	select_memf = (!memf_empty && ((memf.ifb.tag <= mcbf.ifb.tag) || !mcbf.v || (memf.ifb.thread != mcbf.ifb.thread))) &&
		!select_exbr;
	select_mcbf = !mcbf_empty && !select_memf && !select_exbr;
end

task tWriteback;
integer n;
begin
	exbrf_rd <= FALSE;
	memf_rd <= FALSE;
	mcbf_rd <= FALSE;
	commit_wr <= FALSE;
	commit_wrv <= FALSE;
	commit_tgt <= 'd0;
//	if (wbndx_v & !stall_pipe[wbndx])
	begin
		wbndx2 <= wbndx;
		wbb2 <= wbb;

		// Decide which pipeline to update from. Order tags are used to determine the 
		// order of instructions. The instruction with the oldest tag must be updated
		// first.
		if (select_exbr) begin
			wbb[exbrf.ifb.thread] <= exbrf;
			wbndx <= exbrf.ifb.thread;
			exbrf_rd <= TRUE;
		end
		else if (select_memf) begin
			wbb[memf.ifb.thread] <= memf;
			wbndx <= memf.ifb.thread;
			memf_rd <= TRUE;
		end
		else if (select_mcbf) begin
			wbb[mcbf.ifb.thread] <= mcbf;
			wbndx <= mcbf.ifb.thread;
			mcbf_rd <= TRUE;
		end
		else begin// nothing to update
			for (n = 0; n < NTHREADS; n = n + 1)
				wbb[n] <= 'd0;
		end

`ifdef IS_SIM
		$display("Writeback %d:", wbndx);
`endif		
		// Normally we do not want to update the machine state on an exception.
		// However for single-step mode we do.
		if (|wbb[wbndx].cause && wbb[wbndx].cause != FLT_SSM)
			tWbException(wbb[wbndx].ifb.ip,wbb[wbndx].cause,0);
		else begin
			if (wbb[wbndx].cause==FLT_SSM)
				tWbException(wbb[wbndx].ifb.ip,wbb[wbndx].cause,1);
`ifdef IS_SIM				
			$display("  ip=%h ir=%h", wbb[wbndx].ifb.ip, wbb[wbndx].ifb.insn);
			if (oub[oundx].dec.rfwr)
				$display("  %s=%h", fnRegName(wbb[wbndx].dec.Rt), wbb[wbndx].res);
`endif				
			commit_thread <= wbndx;
			commit_mask[1:0] <= {2{wbb[wbndx].mask[0]}};
			commit_mask[3:2] <= {2{wbb[wbndx].mask[1]}};
			commit_mask[5:4] <= {2{wbb[wbndx].mask[2]}};
			commit_mask[7:6] <= {2{wbb[wbndx].mask[3]}};
			commit_mask[9:8] <= {2{wbb[wbndx].mask[4]}};
			commit_mask[11:10] <= {2{wbb[wbndx].mask[5]}};
			commit_mask[13:12] <= {2{wbb[wbndx].mask[6]}};
			commit_mask[15:14] <= {2{wbb[wbndx].mask[7]}};
			commit_mask[17:16] <= {2{wbb[wbndx].mask[8]}};
			commit_mask[19:18] <= {2{wbb[wbndx].mask[9]}};
			commit_mask[21:20] <= {2{wbb[wbndx].mask[10]}};
			commit_mask[23:22] <= {2{wbb[wbndx].mask[11]}};
			commit_mask[25:24] <= {2{wbb[wbndx].mask[12]}};
			commit_mask[27:26] <= {2{wbb[wbndx].mask[13]}};
			commit_mask[29:28] <= {2{wbb[wbndx].mask[14]}};
			commit_mask[31:30] <= {2{wbb[wbndx].mask[15]}};
			commit_mask[33:32] <= {2{wbb[wbndx].mask[16]}};
			commit_mask[35:34] <= {2{wbb[wbndx].mask[17]}};
			commit_mask[37:36] <= {2{wbb[wbndx].mask[18]}};
			commit_mask[39:38] <= {2{wbb[wbndx].mask[19]}};
			commit_mask[41:40] <= {2{wbb[wbndx].mask[20]}};
			commit_mask[43:42] <= {2{wbb[wbndx].mask[21]}};
			commit_mask[45:44] <= {2{wbb[wbndx].mask[22]}};
			commit_mask[47:46] <= {2{wbb[wbndx].mask[23]}};
			commit_mask[49:48] <= {2{wbb[wbndx].mask[24]}};
			commit_mask[51:50] <= {2{wbb[wbndx].mask[25]}};
			commit_mask[53:52] <= {2{wbb[wbndx].mask[26]}};
			commit_mask[55:54] <= {2{wbb[wbndx].mask[27]}};
			commit_mask[57:46] <= {2{wbb[wbndx].mask[28]}};
			commit_mask[59:48] <= {2{wbb[wbndx].mask[29]}};
			commit_mask[61:60] <= {2{wbb[wbndx].mask[30]}};
			commit_mask[63:62] <= {2{wbb[wbndx].mask[31]}};
			commit_wr <= {4{wbb[wbndx].dec.rfwr}};
			commit_wrv <= wbb[wbndx].dec.vrfwr;
			commit_tgt <= wbb[wbndx].dec.Rt;
			commit_bus <= wbb[wbndx].res;
			case(1'b1)
			wbb[wbndx].dec.popq:
				case(wbb[wbndx].dec.imm[3:0])
				4'd15:	rd_trace <= 1'b1;
				default:	;
				endcase
			wbb[wbndx].dec.brk:	tWbException(wbb[wbndx].ifb.ip + 4'd5,FLT_BRK,1);	// BRK instruction
			//exb[wbndx].dec.irq: tWbException(exb[wbndx].ifb.ip,exb[wbndx].cause);	// hardware irq
			//exb[wbndx].dec.flt: tWbException(exb[wbndx].ifb.ip,exb[wbndx].cause);	// processing fault (divide by zero, tlb miss, ...)
			wbb[wbndx].dec.rti:	tWbRti();
			wbb[wbndx].dec.rex:	tWbRex();
			wbb[wbndx].dec.csrrw:	tWriteCSR(wbb[wbndx].a,wbndx,wbb[wbndx].dec.imm[13:0]);
			wbb[wbndx].dec.csrrc:	tClrbitCSR(wbb[wbndx].a,wbndx,wbb[wbndx].dec.imm[13:0]);
			wbb[wbndx].dec.csrrs:	tSetbitCSR(wbb[wbndx].a,wbndx,exb[wbndx].dec.imm[13:0]);
			default:	;
			endcase
			// Check for a vector memory instruction that needs to repeat.
			// Instructions will keep flowing into the pipeline while the vector
			// operation is taking place. Treat this like a branch and rollback the
			// incoming instructions.
			if (wbb[wbndx].dec.mem & wbb[wbndx].dec.need_steps) begin
				if (wbb[wbndx].count < vl && wbb[wbndx].mask != 'd0) begin
					wbb[wbndx].count <= wbb[wbndx].count + 2'd1;
					if (wbb[wbndx].mask[wbb[wbndx].count] || !wbb[wbndx].dec.compress)
						wbb[wbndx].step <= wbb[wbndx].step + 2'd1;
					ou_rollback[wbndx] <= 1'b1;
					wbb[wbndx].mask[wbb[wbndx].count] <= 1'b0;
				end
				else
					wbb[wbndx] <= 'd0;
			end
			// Writing to machine stack pointer globally enables interrupts.
			if (wbb[wbndx].dec.Rt==7'd47 && wbb[wbndx].dec.rfwr)
				gie[wbndx] <= 1'b1;
			if (ic_ifb.v && ic_ifb.insn.any.opcode==OP_PFX)
				retired <= retired + 2'd2;
			else if (wbb[wbndx].v)
				retired <= retired + 2'd1;
		end
//		if (!dcndx_v || dcndx!=wbndx)
//			wbb[wbndx] <= 'd0;
	end
	if ((wbb[wbndx].dec.rfwr & ~commit_tgt.vec) | (wbb[wbndx].dec.vrfwr & commit_tgt.vec))
		ou_rollback_bitmaps[commit_thread][commit_tgt] <= 1'b0;
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tRollback;
integer n;
begin
	for (n = 0; n < NTHREADS; n = n + 1)
		if (rollback[n]) begin
			dcb[n].v <= INV;
			rfb1[n].v <= INV;
			rfb2[n].v <= INV;
			exb[n].v <= INV;
		end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// CSR Read / Update tasks
//
// Important to use the correct assignment type for the following, otherwise
// The read won't happen until the clock cycle.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tReadCSR;
output value_t res;
input tid_t thread;
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
input value_t val;
input tid_t thread;
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
		CSR_CAUSE:	cause[thread][regno[13:12]] <= cause_code_t'(val[11:0]);
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
input value_t val;
input tid_t thread;
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
input value_t val;
input tid_t thread;
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
	6'd32:	fnRegName = "t8";
	6'd33:	fnRegName = "t9";
	6'd34:	fnRegName = "t10";
	6'd35:	fnRegName = "t11";
	6'd36:	fnRegName = "s10";
	6'd37:	fnRegName = "s11";
	6'd38:	fnRegName = "s12";
	6'd39:	fnRegName = "s13";
	6'd40:	fnRegName = "r40";
	6'd41:	fnRegName = "r41";
	6'd42:	fnRegName = "r42";
	6'd43:	fnRegName = "r43";
	6'd44:	fnRegName = "r44";
	6'd45:	fnRegName = "r45";
	6'd46:	fnRegName = "r46";
	6'd47:	fnRegName = "r47";
	6'd48:	fnRegName = "vm0";
	6'd49:	fnRegName = "vm1";
	6'd50:	fnRegName = "vm2";
	6'd51:	fnRegName = "vm3";
	6'd52:	fnRegName = "vm4";
	6'd53:	fnRegName = "vm5";
	6'd54:	fnRegName = "vm6";
	6'd55:	fnRegName = "vm7";
	6'd56:	fnRegName = "lc";
	6'd57:	fnRegName = "lk1";
	6'd58:	fnRegName = "lk2";
	6'd59:	fnRegName = "ip";
	6'd60:	fnRegName = "ssp";
	6'd61:	fnRegName = "hsp";
	6'd62:	fnRegName = "msp";
	6'd63:	fnRegName = "isp";
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
			$display("  %c ip_insn=%h  insn=%h postfix=%h", ic_ifb.v ? "v":"-",ic_ifb.ip, ic_ifb.insn, ic_ifb.pfx);
			$display("  ip_thread4=%h", ip_thread4);
			$display("  ihit=%d", ihit);
			for (n1 = 0; n1 < NTHREADS; n1 = n1 + 1)
				$display("  thread[%d].ip=%h", n1[3:0], thread_ip[n1]);
			$display("DecodeBuffer:");
			$display("  1:%d: %c ip=%h.%d ir=%h oc=%0s", n[3:0],
				dc_ifb[n].v ? "v":"-",
				dc_ifb[n].ip,
				dc_ifb[n].tag,
				dc_ifb[n].insn,
				dc_ifb[n].insn.any.opcode.name()
			);
			$display("  2:%d: %c ip=%h.%d ir=%h oc=%0s Ra%d Rb%d Rc%d", n[3:0],
				dcb_v[n] ? "v":"-",
				dcb[n].ifb.ip,
				dcb[n].ifb.tag,
				dcb[n].ifb.insn,
				dcb[n].ifb.insn.any.opcode.name(),
				ra0,
				ra1,
				ra2
			);
			$display("Regfetch %d,%d:", rfndx1,rfndx2);
			if (REGFILE_LATENCY > 3) begin
				$display("  1:%c ip=%h.%d %0s Ra%d=%h Rb%d=%h Rc%d=%h csro[%h]=%h",
					rfb1[n].v?"v":"-",
					rfb1[n].ifb.ip,
					rfb1[n].ifb.tag,
					rfb1[n].ifb.insn.any.opcode.name(),
					rfb1[n].dec.Ra,
					opera,
					rfb1[n].dec.Rb,
					operb,
					rfb1[n].dec.Rc,
					operc,
					rfb1[n].dec.imm[13:0],
					csro
				);
			end
			if (REGFILE_LATENCY > 2) begin
				$display("  2:%c ip=%h.%d %0s Ra%d=%h Rb%d=%h Rc%d=%h",
					rfb2[n].v?"v":"-",
					rfb2[n].ifb.ip,
					rfb2[n].ifb.tag,
					rfb2[n].ifb.insn.any.opcode.name(),
					rfb2[n].dec.Ra,
					opera,
					rfb2[n].dec.Rb,
					operb,
					rfb2[n].dec.Rc,
					operc
				);
			end
			$display("  3:%c ip=%h.%d %0s Ra%d=%h Rb%d=%h Rc%d=%h",
				rfb3_v[n]?"v":"-",
				rfb3[n].ifb.ip,
				rfb3[n].ifb.tag,
				rfb3[n].ifb.insn.any.opcode.name(),
				rfb3[n].dec.Ra,
				opera,
				rfb3[n].dec.Rb,
				operb,
				rfb3[n].dec.Rc,
				operc
			);
			$display("Execute:");
			$display("  %d: %c%c ip=%h.%d ir=%h oc=%0s res=%h a=%h b=%h c=%h t=%h i=%h", n[3:0],
				exb_v[n] ? "v":"-",
				exb[n].regfetched ? "r": "-",
				exb[n].ifb.ip,
				exb[n].ifb.tag,
				exb[n].ifb.insn,
				exb[n].ifb.insn.any.opcode.name(),
				exb[n].res,
				exb[n].a,
				exb[n].b,
				exb[n].c,
				exb[n].t,
				exb[n].dec.imm
			);
			$display("Execute Pipe Input:");
			$display("  %d ip=%h, oc=%0s,res=%h",
				n[3:0],
				exbr.ifb.ip,
				exbr.ifb.insn.any.opcode.name(),
				exbr.res
			);
			$display("Address Generation");
			$display("  %c ip=%h oc=%0s tmpadr=%h res=%h Rc=%h",
				agb[n].v ? "v" : "-",
				agb[n].ifb.ip,
				agb[n].ifb.insn.any.opcode.name(),
				tmpadr[n],
				vres,
				agb[n].c
			);
			$display("Out");
			$display("  res=%h",oub[n].res);
			$display("Writeback");
			$display("  res=%h",wbb[n].res);
		end
	end
`endif
end
endtask

endmodule
