`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2013-2022  Robert Finch, Waterloo
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

`define IS_SIM	1'b1

import rfPhoenixPkg::*;

module rfPhoenix(hartid_i, rst_i, clk_i, clk2x_i, clk2d_i, wc_clk_i, clock,
		nmi_i, irq_i, icause_i,
		vpa_o, vda_o, bte_o, cti_o, bok_i, cyc_o, stb_o, lock_o, ack_i,
    err_i, we_o, sel_o, adr_o, dat_i, dat_o, cr_o, sr_o, rb_i, state_o, trigger_o,
    wcause);
input [63:0] hartid_i;
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

wire clk_g = clk_i;
ExecuteBuffer [NTHREADS-1:0] eb;
ExecuteBuffer ebq [0:NTHREADS-1];

// The following var indicates that r0 has been written for the thread.
// Only one write of r0 is allowed, to set the value to zero.
reg [NTHREADS-1:0] rz;
reg [NTHREADS-1:0] gie;
Tid exndx,oundx,wbndx,xrid,mc_rid,mc_rid1,mc_rid2,mc_rido;
reg exndx_v,oundx_v,wbndx_v,itndx_v;
Tid dcndx, rfndx, itndx;
reg dcndx_v,xrid_v;
reg [NTHREADS-1:0] rfndx_v;
Tid mcv_ridi, mcv_rido;
Tid ithread, rthread, dthread, xthread, commit_thread;
reg rthread_v, dthread_v;
reg commit_wr, commit_wrv;
reg [15:0] commit_mask;
Regspec commit_tgt;
VecValue commit_bus;
Tid ip_thread, ip_thread1, ip_thread2, ip_thread3, ip_thread4, ip_thread5;
reg [31:0] ips [0:NTHREADS-1];
reg [31:0] ip, ip2, ip3, ip4;
reg [NTHREADS-1:0] thread_busy;
CodeAddress iip, dip, ip_icline, ip_insn;
Instruction ir,dir,xir,mir,insn,mir1,mir2;
Instruction rf_insn;
Postfix pfx,irpfx,rf_pfx;
DecodeBus [NTHREADS-1:0] deco;
Regspec ra0,ra1,ra2,ra3,ra4;
Value rfo0, rfo1, rfo2, rfo3, rfo4;
Value ximm,mcimm;
ASID xasid;
VecValue vrfo0, vrfo1, vrfo2, vrfo3, vrfo4;
VecValue xa,xb,xc,xt,xm;
reg xtt;
VecValue mca,mcb,mcc,mct,mcm;
VecValue mca1,mcb1,mcc1,mct1,mcm1;
VecValue mca2,mcb2,mcc2,mct2,mcm2;
reg [2:0] mca_busy;
Value res,mc_res,mc_res1,mc_res2;
VecValue vres,mc_vres,mc_vres1,mc_vres2;
wire mc_done, mcv_done;
wire mc_done1, mcv_done1;
wire mc_done2, mcv_done2;
wire ihit;
reg ihit2;
MemoryRequest memreq;
MemoryResponse memresp;
wire memreq_full;
reg memresp_fifo_rd;
wire memresp_fifo_empty;
wire memresp_fifo_v;
wire [639:0] ic_line;
reg [639:0] ic_line2;
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
reg [2:0] ic_sp_sel [0:NTHREADS-1];
reg [2:0] istk_depth [0:NTHREADS-1];
Instruction [NTHREADS-1:0] exc_bucket;
CodeAddress [NTHREADS-1:0] exc_ip;
CauseCode icause,dcause;
reg [7:0] tid;
CodeAddress last_adr;
reg [15:0] exv;
Tid tmpndx, tmpndx3, tmpndx4, tmpndx8, tmpndx9, prev_exndx;
Tid prev_oundx, prev_wbndx, prev_dcndx, prev_itndx;
wire [25:0] ic_tag;
reg [25:0] ic_tag2;
ExecuteBuffer [NTHREADS-1:0] dcbuf;
ExecuteBuffer [NTHREADS-1:0] mceb;
reg [NTHREADS-1:0] mem_rollback, ou_rollback, rollback;
Tid mem_rollback_thread;
Tid ou_rollback_thread;
wire [127:0] mem_rollback_bitmap;
reg [127:0] ou_rollback_bitmap [0:NTHREADS-1];
reg [127:0] rollback_bitmap;
reg [NTHREADS-1:0] sb_will_issue;
wire [NTHREADS-1:0] sb_can_issue;

// CSRs
reg [31:0] cr0;
reg [31:0] ptbr [0:NTHREADS-1];
reg [9:0] asid [0:NTHREADS-1];
reg [31:0] hmask,xhmask;
Address badaddr [0:NTHREADS-1][0:3];
CauseCode cause [0:NTHREADS-1][0:3];
CodeAddress tvec [0:3];
reg [63:0] plStack [0:NTHREADS-1];
reg [63:0] pmStack [0:NTHREADS-1];
reg [255:0] ipStack [0:NTHREADS-1];
reg [31:0] status;
reg [31:0] tick;
reg [63:0] wc_time;
reg [31:0] wc_time_dat;
reg ld_time, clr_wc_time_irq;

genvar g;

function [5:0] fnSpSel;
input [3:0] thread;
input [5:0] i;
begin
	if (i==6'd31)
		case(sp_sel[thread])
		3'd1:	fnSpSel = 6'd44;
		3'd2:	fnSpSel = 6'd45;
		3'd3:	fnSpSel = 6'd46;
		3'd4:	fnSpSel = 6'd47;
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
		omode[n5] = pmStack[n5][7:6];
		Usermode[n5] = omode[n5]==2'b00;
		MUsermode[n5] = omode[n5]==2'b00;
	end

integer n;
initial begin
	tid = 8'd1;
	rz = 'd0;
	gie = 'd0;
	ip = RSTIP;
	iip = RSTIP;
	ir = NOP;//_INSN;
	xir = NOP;//_INSN;
	mir = NOP;//_INSN;
	xa = 'd0;
	xb = 'd0;
	xc = 'd0;
	ximm = 'd0;
	mca = 'd0;
	mcb = 'd0;
	mcc = 'd0;
	mcimm = 'd0;
	for (n = 0; n < NTHREADS; n = n + 1)
		ips[n] = RSTIP;
	ithread = 'd0;
	ip_thread = 'd0;
	mca_busy = 'd0;
	thread_busy = 'd0;
	for (n = 0; n < REB_ENTRIES; n = n + 1)
		eb[n] = 'd0;
	rthread_v = 'd0;
	dthread_v = 'd0;
	memreq = 'd0;
end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

wire memreq_wack;

rfPhoenixBiu ubiu
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
	.rollback(|mem_rollback),
	.rollback_thread(mem_rollback_thread),
	.rollback_bitmap(mem_rollback_bitmap)
);


generate begin : gDecoders
for (g = 0; g < NTHREADS; g = g + 1)
	rfPhoenix_decoder udec1
	(
		.ifb(dc_ifb[g]),
		.sp_sel(sp_sel[g]),
		.deco(deco[g])
	);
end
endgenerate

generate begin : gScoreboard
for (g = 0; g < NTHREADS; g = g + 1) begin
	always_comb
	begin
		rollback[g] <= 'd0;
		rollback_bitmap[g] = 'd0;
		if (mem_rollback[g] && g==mem_rollback_thread) begin
			rollback[g] = 1'b1;
			rollback_bitmap[g] = mem_rollback_bitmap;
		end
		else if (ou_rollback[g]) begin
			rollback[g] <= 1'b1;
			rollback_bitmap[g] <= ou_rollback_bitmap[g];
		end
	end

	rfPhoenix_scoreboard uscb1
	(
		.rst(rst_i),
		.clk(clk_g),
		.db(deco[g]),
		.wb_v(eb[g].dec.rfwr),
		.wb_Rt(eb[g].dec.Rt),
		.will_issue(sb_will_issue[g]),
		.can_issue(sb_can_issue[g]),
		.rollback(rollback[g]),
		.rollback_bitmap(rollback_bitmap[g])
	);
end
end
endgenerate

rfPhoenix_branch_eval ube1
(
	.ir(xir),
	.a(xa[0]),
	.b(xt[0]),
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
	.rthread(rthread),
	.ra0(ra0),
	.ra1(ra1),
	.ra2(ra2),
	.ra3(ra3),
	.ra4(ra4),
	.o0(rfo0),
	.o1(rfo1),
	.o2(rfo2),
	.o3(rfo3),
	.o4(rfo4)
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
	.rthread(rthread),
	.ra0(ra0),
	.ra1(ra1),
	.ra2(ra2),
	.ra3(ra3),
	.ra4(ra4),
	.o0(vrfo0),
	.o1(vrfo1),
	.o2(vrfo2),
	.o3(),				// mask register port not needed here
	.o4(vrfo4)
);

rfPhoenixVecAlu uvalu1 (
	.ir(xir),
	.a(xa),
	.b(xb),
	.c(xc),
	.imm(ximm),
	.Tt(xtt),
	.asid(xasid),
	.hmask(xhmask),
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

task tDisplayRegs;
integer n;
begin
`ifdef IS_SIM
	// The heirarchical reference to the register file here prevents synthsis
	// from using RAM resources to implement the register file. So this block
	// is enabled only for simulation.
	$display("GPRs");
	for (n = 0; n < NTHREADS*64; n = n + 8) begin
		if ((n % 64)==0)
			$display("  Thread:%d", n / 64);
		$display("%s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  %s:%h  ",
			fnRegName(n), ugprs1.regfile[n],
			fnRegName(n+1), ugprs1.regfile[n+1],
			fnRegName(n+2), ugprs1.regfile[n+2],
			fnRegName(n+3), ugprs1.regfile[n+3],
			fnRegName(n+4), ugprs1.regfile[n+4],
			fnRegName(n+5), ugprs1.regfile[n+5],
			fnRegName(n+6), ugprs1.regfile[n+6],
			fnRegName(n+7), ugprs1.regfile[n+7]
			);
	end
	$display("");
`endif
end
endtask

task tDisplayEb;
integer n;
begin
	$display("  ExecuteBuffer:");
	for (n = 0; n < NTHREADS; n = n + 1) begin
		$display("  %d: %c%c%c%c%c ip=%h ir=%h res=%h a=%h b=%h c=%h i=%h", n[3:0],
			eb[n].v ? "v":"-",
			eb[n].decoded ? "d" : "-",
			eb[n].regfetched ? "r": "-",
			eb[n].out ? "o" : "-",
			eb[n].executed ? "x" : "-",
			eb[n].ifb.ip,
			eb[n].ifb.insn,
			eb[n].res,
			eb[n].a,
			eb[n].b,
			eb[n].c,
			eb[n].dec.imm
		);
	end
end
endtask

reg [NTHREADS-1:0] clr_ififo;
wire [NTHREADS-1:0] ififo_empty;
InstructionFetchbuf ic_ifb;
InstructionFetchbuf [NTHREADS-1:0] dc_ifb, dc1_ifb;

always_ff @(posedge clk_g)
begin
	{ic_ifb.pfx,ic_ifb.insn} <= ic_line >> {ip_icline[5:0],3'b0};
	ic_ifb.ip <= ip_icline;
	ic_ifb.v <= 1'b1;
	ic_ifb.sp_sel <= sp_sel[ip_thread5];
	if (irq_i > pmStack[ip_thread5][3:1] && gie[ip_thread5])
		ic_ifb.cause <= {1'b1,irq_i,FLT_BRK};
	else
		ic_ifb.cause <= 'd0;
end
always_ff @(posedge clk_g)
	ic_line2 <= ic_line;
always_ff @(posedge clk_g)
	ip_insn <= ip_icline;
always_ff @(posedge clk_g)
	ihit2 <= ihit;
always_ff @(posedge clk_g)
	ic_tag2 <= ic_tag;

reg [NTHREADS-1:0] wr_ififo;
generate begin
for (g = 0; g < NTHREADS; g = g + 1) begin
	always_comb
		clr_ififo[g] = rollback[g];
	always_comb
		sb_will_issue[g] = g==dcndx && dcndx_v && !ififo_empty[g];
	always_comb
		wr_ififo[g] = ihit2 && ip_thread5==g;

	rfPhoenix_fifo #(.WID($bits(InstructionFetchbuf)), .DEP(16)) ufifo1
	(
		.rst(rst_i|clr_ififo[g]),
		.clk(clk_g),
		.wr(wr_ififo[g]),
		.di(ic_ifb),
		.rd(sb_will_issue[g]),
		.dout(dc_ifb[g]),
		.cnt(),
		.full(),
		.almost_full(ififo_almost_full[g]),
		.empty(ififo_empty[g]),
		.v()
	);
	
end
end
endgenerate

integer n10;
always_ff @(posedge clk_g)
if (rst_i) begin
	for (n10 = 0; n10 < NTHREADS; n10 = n10 + 1) begin
		eb[n10] <= 'd0;
		rfndx_v[n10] <= 1'b0;
	end
	ra0 <= 'd0;
	ra1 <= 'd0;
	ra2 <= 'd0;
	ra3 <= 'd0;
	ra4 <= 'd0;
end
else begin
	for (n10 = 0; n10 < NTHREADS; n10 = n10 + 1)
		if (sb_will_issue[n10]) begin
			eb[n10].v <= 1'b1;
			eb[n10].thread <= n10;
			eb[n10].ifb <= dc_ifb[n10];
			eb[n10].dec <= deco[n10];
			eb[n10].decoded <= 1'b1;
			ra0 <= fnSpSel(n10,deco[n10].Ra.num);
			ra1 <= fnSpSel(n10,deco[n10].Rb.num);
			ra2 <= fnSpSel(n10,deco[n10].Rc.num);
			ra3 <= deco[n10].Rm.num;
			ra4 <= fnSpSel(n10,deco[n10].Rt.num);
			rfndx <= n10;
			rfndx_v[n10] <= 1'b1;
		end
		else
			rfndx_v[n10] <= 1'b0;
end		

always_ff @(posedge clk_g)
if (rst_i) begin
	tReset();
end
else begin
	$display("=======================================");
	$display("=======================================");
	$display("Time %d", $time);
	$display("=======================================");
	$display("=======================================");
	$display("  exndx=%d exv=%h", exndx, exv);
	tDisplayRegs();
	tDisplayEb();
	tOnce();
	tInsnFetch();
	tRegfetch();
	tExecute();
	tOut();
	tMemory();
	tWriteback();
end

task tReset;
integer n;
begin
	tid <= 8'd1;
	rz <= 'd0;
	gie <= 'd0;
	ip <= RSTIP;
	ip2 <= RSTIP;
	ip3 <= RSTIP;
	iip <= RSTIP;
	ir <= NOP;//_INSN;
	xir <= NOP;//_INSN;
	mir <= NOP;//_INSN;
	xa <= 'd0;
	xb <= 'd0;
	xc <= 'd0;
	ximm <= 'd0;
	mca <= 'd0;
	mcb <= 'd0;
	mcc <= 'd0;
	mcimm <= 'd0;
	for (n = 0; n < NTHREADS; n = n + 1) begin
		ips[n] <= RSTIP;
		cause[n][0] <= 'd0;
		cause[n][1] <= 'd0;
		cause[n][2] <= 'd0;
		cause[n][3] <= 'd0;
		exc_bucket[n] <= 'd0;
		plStack[n] <= 64'hFFFFFFFFFFFFFFFF;
		pmStack[n] <= 64'hCECECECECECECECE;
		ipStack[n] <= {8{RSTIP}};
		sp_sel[n] <= 3'd3;
	end
	ithread <= 'd0;
	ip_thread <= 'd0;
	mca_busy <= 'd0;
	thread_busy <= 'd0;
	for (n = 0; n < NTHREADS; n = n + 1)
		eb[n] <= 'd0;
	rthread_v <= 'd1;
	dthread_v <= 'd1;
	memreq <= 'd0;
	last_adr <= 'd0;
	xrid <= 'd15;
	mc_rid <= 'd15;
end
endtask

task tOnce;
integer n;
begin
	memreq.wr <= 1'b0;
	memresp_fifo_rd <= 1'b1;
	for (n = 0; n < NTHREADS; n = n + 1) begin
		mem_rollback[n] <= 1'b0;
		ou_rollback[n] <= 1'b0;
		if (ou_rollback[n])
			ou_rollback_bitmap[n] <= 'd0;
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
		3'd1:	o <= 6'd44;
		3'd2:	o <= 6'd45;
		3'd3:	o <= 6'd46;
		3'd4:	o <= 6'd47;
		default:	o <= 6'd31;
		endcase
	else
		o <= i;
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Schedulers
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// The following selectors use round-robin selection.

Tid pit1;
Tid pit2;
Tid pit3;
Tid pit4;
Tid pit5;
Tid pit6;
Tid pit7;

always_comb
	pit1 = prev_itndx + 3'd1;
always_comb
	pit2 = prev_itndx + 3'd2;
always_comb
	pit3 = prev_itndx + 3'd3;
always_comb
	pit4 = prev_itndx + 4'd4;
always_comb
	pit5 = prev_itndx + 3'd5;
always_comb
	pit6 = prev_itndx + 3'd6;
always_comb
	pit7 = prev_itndx + 3'd7;

always_ff @(posedge clk_g)
if (rst_i) begin
	itndx <= 'd0;
	itndx_v <= 'd0;
	prev_itndx <= 'd0;
end
else begin
	itndx_v <= 1'b0;
	itndx <= 'd0;
	if (!ififo_almost_full[pit1]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd1;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit2]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd2;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit3]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd3;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit4]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd4;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit5]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd5;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit6]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd6;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[pit7]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx + 3'd7;
		itndx_v <= 1'b1;
	end
	else if (!ififo_almost_full[prev_itndx]) begin
		prev_itndx <= itndx;
		itndx <= prev_itndx;
		itndx_v <= 1'b1;
	end
end

Tid dit1;
Tid dit2;
Tid dit3;
Tid dit4;
Tid dit5;
Tid dit6;
Tid dit7;

always_comb
	dit1 = prev_dcndx + 3'd1;
always_comb
	dit2 = prev_dcndx + 3'd2;
always_comb
	dit3 = prev_dcndx + 3'd3;
always_comb
	dit4 = prev_dcndx + 3'd4;
always_comb
	dit5 = prev_dcndx + 3'd5;
always_comb
	dit6 = prev_dcndx + 3'd6;
always_comb
	dit7 = prev_dcndx + 3'd7;

always_ff @(posedge clk_g)
if (rst_i) begin
	prev_dcndx <= 'd0;
	dcndx <= 'd0;
	dcndx_v <= 'd0;
end
else begin
	if (sb_can_issue[dit1]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd1;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit2]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd2;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit3]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd3;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit4]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd4;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit5]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd5;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit6]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd6;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[dit7]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx + 3'd7;
		dcndx_v <= 1'b1;
	end
	else if (sb_can_issue[prev_dcndx]) begin
		prev_dcndx <= dcndx;
		dcndx <= prev_dcndx;
		dcndx_v <= 1'b1;
	end
end

Tid exn1;
Tid exn2;
Tid exn3;
Tid exn4;
Tid exn5;
Tid exn6;
Tid exn7;

always_comb
	exn1 = prev_exndx + 3'd1;
always_comb
	exn2 = prev_exndx + 3'd2;
always_comb
	exn3 = prev_exndx + 3'd3;
always_comb
	exn4 = prev_exndx + 3'd4;
always_comb
	exn5 = prev_exndx + 3'd5;
always_comb
	exn6 = prev_exndx + 3'd6;
always_comb
	exn7 = prev_exndx + 3'd7;

// Pick a rob entry to execute.
always_ff @(posedge clk_g)
if (rst_i) begin
	prev_exndx <= 'd0;
	exndx_v <= 'd0;
	exndx <= 'd0;
end
else begin
	exndx_v <= 1'b0;
	if (eb[exn1].regfetched & ~eb[exn1].out) begin
		prev_exndx <= exndx;
		exndx <= exn1;
		exndx_v <= 1'b1;
	end
	else if (eb[exn2].regfetched & ~eb[exn2].out) begin
		prev_exndx <= exndx;
		exndx <= exn2;
		exndx_v <= 1'b1;
	end
	else if (eb[exn3].regfetched & ~eb[exn3].out) begin
		prev_exndx <= exndx;
		exndx <= exn3;
		exndx_v <= 1'b1;
	end
	else if (eb[exn4].regfetched & ~eb[exn4].out) begin
		prev_exndx <= exndx;
		exndx <= exn4;
		exndx_v <= 1'b1;
	end
	else if (eb[exn5].regfetched & ~eb[exn5].out) begin
		prev_exndx <= exndx;
		exndx <= exn5;
		exndx_v <= 1'b1;
	end
	else if (eb[exn6].regfetched & ~eb[exn6].out) begin
		prev_exndx <= exndx;
		exndx <= exn6;
		exndx_v <= 1'b1;
	end
	else if (eb[exn7].regfetched & ~eb[exn7].out) begin
		prev_exndx <= exndx;
		exndx <= exn7;
		exndx_v <= 1'b1;
	end
	else if (eb[prev_exndx].regfetched & ~eb[prev_exndx].out) begin
		prev_exndx <= exndx;
		exndx <= prev_exndx;
		exndx_v <= 1'b1;
	end
end

// Pick an rob entry thats out.

Tid oun1;
Tid oun2;
Tid oun3;
Tid oun4;
Tid oun5;
Tid oun6;
Tid oun7;

always_comb
	oun1 = prev_oundx + 3'd1;
always_comb
	oun2 = prev_oundx + 3'd2;
always_comb
	oun3 = prev_oundx + 3'd3;
always_comb
	oun4 = prev_oundx + 3'd4;
always_comb
	oun5 = prev_oundx + 3'd5;
always_comb
	oun6 = prev_oundx + 3'd6;
always_comb
	oun7 = prev_oundx + 3'd7;

always_ff @(posedge clk_g)
if (rst_i) begin
	prev_oundx <= 'd0;
	oundx_v <= 'd0;
	oundx <= 'd0;
end
else begin
	oundx_v <= 1'b0;
	if (eb[oun1].out) begin
		prev_oundx <= oundx;
		oundx <= oun1;
		oundx_v <= 1'b1;
	end
	else if (eb[oun2].out) begin
		prev_oundx <= oundx;
		oundx <= oun2;
		oundx_v <= 1'b1;
	end
	else if (eb[oun3].out) begin
		prev_oundx <= oundx;
		oundx <= oun3;
		oundx_v <= 1'b1;
	end
	else if (eb[oun4].out) begin
		prev_oundx <= oundx;
		oundx <= oun4;
		oundx_v <= 1'b1;
	end
	else if (eb[oun5].out) begin
		prev_oundx <= oundx;
		oundx <= oun5;
		oundx_v <= 1'b1;
	end
	else if (eb[oun6].out) begin
		prev_oundx <= oundx;
		oundx <= oun6;
		oundx_v <= 1'b1;
	end
	else if (eb[oun7].out) begin
		prev_oundx <= oundx;
		oundx <= oun7;
		oundx_v <= 1'b1;
	end
	else if (eb[prev_oundx].out) begin
		prev_oundx <= oundx;
		oundx <= prev_oundx;
		oundx_v <= 1'b1;
	end
end

// Pick a finished rob entry.
Tid wbn1;
Tid wbn2;
Tid wbn3;
Tid wbn4;
Tid wbn5;
Tid wbn6;
Tid wbn7;
always_comb
	wbn1 = prev_wbndx + 3'd1;
always_comb
	wbn2 = prev_wbndx + 3'd2;
always_comb
	wbn3 = prev_wbndx + 3'd3;
always_comb
	wbn4 = prev_wbndx + 3'd4;
always_comb
	wbn5 = prev_wbndx + 3'd5;
always_comb
	wbn6 = prev_wbndx + 3'd6;
always_comb
	wbn7 = prev_wbndx + 3'd7;

always_ff @(posedge clk_g)
if (rst_i) begin
	prev_wbndx <= 'd0;
	wbndx_v <= 'd0;
	wbndx <= 'd0;
end
else begin
	wbndx_v <= 1'b0;
	if (eb[wbn1].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn1;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn2].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn2;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn3].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn3;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn4].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn4;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn5].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn5;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn6].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn6;
		wbndx_v <= 1'b1;
	end
	else if (eb[wbn7].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= wbn7;
		wbndx_v <= 1'b1;
	end
	else if (eb[prev_wbndx].executed) begin
		prev_wbndx <= wbndx;
		wbndx <= prev_wbndx;
		wbndx_v <= 1'b1;
	end
end

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// IF Stage
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tInsnFetch;
integer n;
begin
	begin
		// 3 cycle pipeline delay reading the I$.
		// 1 for tag lookup and way determination
		// 2 for cache line lookup
		if (itndx_v) begin
			ip <= ips[itndx];
			ips[itndx] <= ips[itndx] + 4'd5;
		end
		ip_thread1 <= itndx;
		ip_thread2 <= ip_thread1;
		ip_thread3 <= ip_thread2;
		ip_thread4 <= ip_thread3;
		ip_thread5 <= ip_thread4;
		if (!ihit2)
			ips[ip_thread5] <= ic_ifb.ip;
		$display("  ip_insn=%h  insn=%h postfix=%h", ip_icline, ic_ifb.insn, ic_ifb.pfx);
		$display("  ip_thread5=%h", ip_thread5);
		for (n = 0; n < NTHREADS; n = n + 1)
			$display("  ips[%d]=%h", n[3:0], ips[n]);
		// On a miss, request a cache line load from the memory system. This
		// should eventually cause a hit for the thread.
		// Suppress consecutive requests for the same cache line load.
		// The old cache line is passed back for the victim buffer.
		if (!ihit && !memreq_full) begin
			if (ip_icline[31:6] != last_adr[31:6]) begin
				last_adr <= ip_icline;
				tid <= tid + 2'd1;
				memreq.tid <= tid;
				memreq.thread <= ip_thread4;
				memreq.wr <= 1'b1;
				memreq.func <= MR_ICACHE_LOAD;
				memreq.adr <= {ip_icline[31:6],6'd0};
				memreq.vcadr <= {ic_tag,6'b0};
				memreq.dat <= ic_line;
				memreq.sz <= ic_valid ? tetra : nul;
			end
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DC Stage
// - decoding is handled by a decode module above.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF Stage
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Value csro;
always_comb
	tReadCSR(csro,rfndx,eb[rfndx].dec.imm[13:0]);

task tRegfetch;
begin
	if (rfndx_v[rfndx])
	begin
		//eb[rfndx].cause <= dcause;
		eb[rfndx].a <= eb[rfndx].dec.Ra.vec ? vrfo0 : {NLANES{rfo0}};
		eb[rfndx].b <= eb[rfndx].dec.Rb.vec ? vrfo1 : {NLANES{rfo1}};
		if (eb[rfndx].dec.csr)
			eb[rfndx].c <= {NLANES{csro}};
		else
			eb[rfndx].c <= eb[rfndx].dec.Rc.vec ? vrfo2 : {NLANES{rfo2}};
		eb[rfndx].mask <= rfo3;
		eb[rfndx].t <= eb[rfndx].dec.Rt.vec ? vrfo4 : {NLANES{rfo4}};
		eb[rfndx].regfetched <= 1'b1;
		ou_rollback_bitmap[rfndx][eb[rfndx].dec.Rt] <= 1'b1;
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// EX stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tExLoad;
begin
	if (eb[exndx].dec.load) begin
		tid <= tid + 2'd1;
		memreq.tid <= tid;
		memreq.wr <= 1'b1;
		memreq.func <= eb[exndx].dec.loadu ? MR_LOADZ : MR_LOAD;
		if (eb[exndx].dec.ldr)
			memreq.func2 <= MR_LDR;
		memreq.sz <= eb[exndx].dec.memsz;
		memreq.asid = asid[exndx];
		case(eb[exndx].dec.memsz)
		byt:	memreq.sel <= 64'h1;
		wyde:	memreq.sel <= 64'h3;
		tetra:	memreq.sel <= 64'hF;
		vect:	memreq.sel <= 64'hFFFFFFFFFFFFFFFF;
		default:	memreq.sel <= 64'hF;
		endcase
		if (eb[exndx].dec.memsz==vect) begin
			if (eb[exndx].dec.loadr) begin
				memreq.func2 <= MR_LDV;
				memreq.sel <= 64'hFFFFFFFFFFFFFFFF;
				memreq.adr <= eb[exndx].a[0] + eb[exndx].dec.imm;
			end
			else begin
				memreq.sel <= 64'hF;
				memreq.adr <= eb[exndx].a[0] + eb[exndx].b[eb[exndx].step];
				if (eb[exndx].step != NLANES-1 && eb[exndx].dec.loadn)
					eb[exndx].step <= eb[exndx].step + 2'd1;
			end
		end
		else
			memreq.adr <= eb[exndx].dec.loadr ? eb[exndx].a[0] + eb[exndx].dec.imm : eb[exndx].a[0] + eb[exndx].b[0];
	end
end
endtask

task tExStore;
begin
	if (eb[exndx].dec.store) begin
		tid <= tid + 2'd1;
		memreq.tid <= tid;
		memreq.wr <= 1'b1;
		memreq.func <= MR_STORE;
		if (eb[exndx].dec.stc)
			memreq.func2 <= MR_STC;
		memreq.sz <= eb[exndx].dec.memsz;
		memreq.asid = asid[exndx];
		memreq.dat <= eb[exndx].t;
		case(eb[exndx].dec.memsz)
		byt:	memreq.sel <= 64'h1;
		wyde:	memreq.sel <= 64'h3;
		tetra:	memreq.sel <= 64'hF;
		default:	memreq.sel <= 64'hF;
		endcase
		// BIU works with 128-bit chunks for stores.
		if (eb[exndx].dec.memsz==vect) begin
			memreq.sel <= eb[exndx].ifb.insn.r2.m ? (
				{	{4{eb[exndx].mask[15]}},
					{4{eb[exndx].mask[14]}},
					{4{eb[exndx].mask[13]}},
					{4{eb[exndx].mask[12]}},
					{4{eb[exndx].mask[11]}},
					{4{eb[exndx].mask[10]}},
					{4{eb[exndx].mask[9]}},
					{4{eb[exndx].mask[8]}},
					{4{eb[exndx].mask[7]}},
					{4{eb[exndx].mask[6]}},
					{4{eb[exndx].mask[5]}},
					{4{eb[exndx].mask[4]}},
					{4{eb[exndx].mask[3]}},
					{4{eb[exndx].mask[2]}},
					{4{eb[exndx].mask[1]}},
					{4{eb[exndx].mask[0]}}} >> {eb[exndx].step,2'h0}) & 64'hFFFF
				 	: 64'hFFFF;
			if (eb[exndx].dec.storen)
				memreq.sz <= tetra;
			memreq.adr <= eb[exndx].dec.storer ? 
				eb[exndx].a[0] + eb[exndx].dec.imm + {eb[exndx].step,5'b0} :
				eb[exndx].a[0] + eb[exndx].b[eb[exndx].step];
			// For a scatter store select the current vector element, otherwise select entire vector (set above).
			if (eb[exndx].dec.storen) begin
				memreq.sel <= 64'h000000000000000F;	// 32 bit at a time
				// Dont bother storing if masked
				if (eb[exndx].ifb.insn.r2.m && !eb[exndx].mask[eb[exndx].step])
					memreq.wr <= 1'b0;
				memreq.dat <= eb[exndx].t[eb[exndx].step];
			end
			if (eb[exndx].step!=NLANES-4 && eb[exndx].dec.storer)
				eb[exndx].step <= eb[exndx].step + 2'd4;
			// For scatter increment step
			if (eb[exndx].step!=NLANES-1 && eb[exndx].dec.storen)
				eb[exndx].step <= eb[exndx].step + 2'd1;
		end
		else
			memreq.adr <= eb[exndx].dec.storer ? eb[exndx].a[0] + eb[exndx].dec.imm : eb[exndx].a[0] + eb[exndx].b[0];
	end
end
endtask

task tExecute;
begin
	xrid_v <= 1'b0;
	if (exndx_v) begin
		eb[exndx].decoded <= 1'b0;
		eb[exndx].regfetched <= 1'b0;
		eb[exndx].out <= 1'b1;
		if (eb[exndx].dec.multicycle) begin
			mc_rid <= exndx;
			mcv_ridi <= exndx;
			mir <= eb[exndx].ifb.insn;
			mca <= eb[exndx].a;
			mcb <= eb[exndx].b;
			mcc <= eb[exndx].c;
			mct <= eb[exndx].t;
			mcimm <= eb[exndx].dec.imm;
			mcm <= eb[exndx].mask;
		end
		else begin
			xrid <= exndx;
			xrid_v <= 1'b1;
			xir <= eb[exndx].ifb.insn;
			xa <= eb[exndx].a;
			xb <= eb[exndx].b;
			xc <= eb[exndx].c;
			xt <= eb[exndx].t;
			xtt <= eb[exndx].dec.Tt;
			xm <= eb[exndx].ifb.insn.r2.m ? eb[exndx].mask : 16'hFFFF;
			ximm <= eb[exndx].dec.imm;
			xasid <= asid[exndx];
			xhmask <= hmask;
			if (!memreq_full && ihit) begin
				tExLoad();
				tExStore();
			end
			// If the load/store could not be queued backout the decoded and out
			// indicators so the instruction will be reselected for execution.
			else begin
				if (eb[exndx].dec.load|eb[exndx].dec.store) begin
					eb[exndx].decoded <= 1'b1;
					eb[exndx].regfetched <= 1'b1;
					eb[exndx].out <= 1'b0;
				end
			end
		end
	end
	if (mcv_rido < NTHREADS) begin
		eb[mcv_rido].res <= mc_vres;
		eb[mcv_rido].out <= 1'b0;
		eb[mcv_rido].executed <= 1'b1;
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OU stage 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tOuCall;
begin
	if (xrid_v) begin
		if (eb[xrid].out) begin
			eb[xrid].out <= 1'b0;
			eb[xrid].executed <= 1'b1;
			case(eb[xrid].ifb.insn.any.opcode)
			NOP:				ips[eb[xrid].thread] <= ips[eb[xrid].thread] - 4'd4;
			CALLA,JMP:
				begin
					ips[eb[xrid].thread] <= eb[xrid].ifb.insn.call.target;
					if (eb[xrid].dec.rfwr)
						eb[xrid].res <= ips[eb[xrid].thread] + 4'd5;
					ou_rollback[xrid] <= 1'b1;
					ou_rollback_bitmap[eb[xrid].dec.Rt] <= 1'b1;
					ou_rollback_thread[xrid] <= xrid;
				end
			CALLR,BRA:
				begin
					ips[eb[xrid].thread] <= eb[xrid].ifb.insn.call.target + ips[eb[xrid].thread];
					if (eb[xrid].dec.rfwr)
						eb[xrid].res <= ips[eb[xrid].thread] + 4'd5;
					ou_rollback[xrid] <= 1'b1;
					ou_rollback_bitmap[eb[xrid].dec.Rt] <= 1'b1;
					ou_rollback_thread[xrid] <= xrid;
				end
			default:	;
			endcase
		end
	end
end
endtask

task tOuBranch;
begin
	if (xrid_v) begin
		if (eb[xrid].out) begin
			eb[xrid].out <= 1'b0;
			eb[xrid].executed <= 1'b1;
			if (eb[xrid].dec.br) begin
				if (takb) begin
					ips[eb[xrid].thread] <= ips[eb[xrid].thread] + eb[xrid].dec.imm;
					ou_rollback[xrid] <= 1'b1;
					ou_rollback_thread[xrid] <= xrid;
				end
			end
		end
	end
end
endtask

task tOut;
begin
	tOuCall();
	tOuBranch();
	if (xrid_v) begin
		eb[xrid].out <= 1'b0;
		eb[xrid].executed <= 1'b1;
		if (eb[xrid].dec.storen && eb[xrid].dec.memsz==vect) begin
			if (eb[xrid].step!=NLANES-1) begin
				eb[xrid].out <= 1'b1;
				eb[xrid].executed <= 1'b0;
			end
		end
//		if (eb[xrid].dec.Tt)
			eb[xrid].res <= vres;
//		else if (!eb[xrid].dec.cjb)
//			eb[xrid].res <= res;
	end
	if (mc_rid < NTHREADS) begin
		if (eb[mc_rid].dec.is_vector ? mcv_done : mc_done) begin
			mca_busy[0] <= 1'b0;
			eb[mc_rid].out <= 1'b0;
			eb[mc_rid].executed <= 1'b1;
//			if (eb[mc_rid].dec.Tt)
				eb[mc_rid].res <= mc_vres;
//			else
//				eb[mc_rid].res <= mc_res;
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MEM stage 
// - process responses coming back from the BIU for requests sent by EX stage.
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

task tMemory;
begin
	if (memresp_fifo_v) begin
		eb[memresp.thread].out <= 1'b0;
		if (eb[memresp.thread].out) begin
			eb[memresp.thread].executed <= 1'b1;
			// If a gather load
			if (eb[memresp.thread].dec.loadn && eb[memresp.thread].dec.memsz==vect) begin
				if (eb[memresp.thread].step!=NLANES-1) begin
					eb[memresp.thread].out <= 1'b1;
					eb[memresp.thread].executed <= 1'b0;
				end
				eb[memresp.thread].res[memresp.step] <= memresp.res;
			end
			// Other load
			else if (eb[memresp.thread].dec.load)
				eb[memresp.thread].res <= memresp.res;
			// Scatter store
			else if (eb[memresp.thread].dec.storen && eb[memresp.thread].dec.memsz==vect) begin
				if (eb[memresp.thread].step!=NLANES-1) begin
					eb[memresp.thread].out <= 1'b1;
					eb[memresp.thread].executed <= 1'b0;
				end
			end
			else if (eb[memresp.thread].dec.storer && eb[memresp.thread].dec.memsz==vect) begin
				if (eb[memresp.thread].step!=NLANES-4) begin
					eb[memresp.thread].out <= 1'b1;
					eb[memresp.thread].executed <= 1'b0;
				end
			end
			// Other store / Other Op
		end
	end
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// WB Stage
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// RTI processing at the WB stage.
task tWbRti;
begin
	if (|istk_depth[wbndx]) begin
		pmStack[wbndx] <= {8'hCE,pmStack[wbndx][63:8]};	// restore operating mode, irq level
		plStack[wbndx] <= {8'hFF,plStack[wbndx][63:8]};	// restore privilege level
		ipStack[wbndx] <= {RSTIP,ipStack[wbndx][255:32]};
		ips[wbndx] <= ipStack[wbndx][31:0];
		istk_depth[wbndx] <= istk_depth[wbndx] - 2'd1;
		case(pmStack[wbndx][15:14])
		2'd0:	sp_sel[wbndx] <= 3'd0;
		2'd1:	sp_sel[wbndx] <= 3'd1;
		2'd2:	sp_sel[wbndx] <= 3'd2;
		2'd3:	sp_sel[wbndx] <= 3'd3;
		endcase
	end
	else
		tWbException(eb[wbndx].ifb.ip,{4'h0,FLT_RTI});
end
endtask

task tWbException;
input CodeAddress ip;
input CauseCode cc;
begin
	if (istk_depth[wbndx] < 3'd7) begin
		pmStack[wbndx] <= pmStack[wbndx] << 8;
		pmStack[wbndx][7:6] <= 2'b11;		// select machine operating mode
		pmStack[wbndx][3:1] <= cause[wbndx][omode[wbndx]][10:8];
		pmStack[wbndx][0] <= 1'b0;			// disable all irqs
		plStack[wbndx] <= plStack[wbndx] << 8;
		plStack[wbndx][7:0] <= 8'hFF;		// select max priv level
		ipStack[wbndx] <= ipStack[wbndx] << 32;
		ipStack[wbndx][31:0] <= ip;
		istk_depth[wbndx] <= istk_depth[wbndx] + 2'd1;
		ips[wbndx] <= tvec[2'd3];
		cause[wbndx][omode[wbndx]] <= cc;
		badaddr[wbndx][omode[wbndx]] <= eb[wbndx].badAddr;
		eb[wbndx].cause <= 'd0;
		if (eb[wbndx].cause[7:0]==FLT_IRQ)
			sp_sel[wbndx] <= 3'd4;
		else
			sp_sel[wbndx] <= 3'd3;
	end
	if (eb[wbndx].dec.mem) begin
		mem_rollback[wbndx] <= 1'b1;
		mem_rollback_thread <= wbndx;
		ou_rollback[wbndx] <= 1'b1;
		ou_rollback_thread <= wbndx;
	end
	else begin
		ou_rollback[wbndx] <= 1'b1;
		ou_rollback_thread <= wbndx;
	end
end
endtask

task tWriteback;
begin
	if (wbndx_v) begin
		$display("Writeback");
		if (|eb[wbndx].cause)
			tWbException(eb[wbndx].ifb.ip,eb[wbndx].cause);
		else begin
			$display("  thread:%d ip=%h ir=%h", wbndx, eb[wbndx].ifb.ip, eb[wbndx].ifb.insn);
			if (eb[wbndx].dec.rfwr)
				$display("  %s=%h", fnRegName(eb[wbndx].dec.Rt), eb[wbndx].res);
			commit_thread <= wbndx;
			commit_mask <= eb[wbndx].ifb.insn.r2.m ? eb[wbndx].mask : 16'hFFFF;
			commit_wr <= eb[wbndx].dec.rfwr;
			commit_wrv <= eb[wbndx].dec.vrfwr;
			commit_tgt <= eb[wbndx].dec.Rt;
			commit_bus <= eb[wbndx].res;
			case(1'b1)
			eb[wbndx].dec.brk:	tWbException(eb[wbndx].ifb.ip + 4'd5,eb[wbndx].cause);	// BRK instruction
			eb[wbndx].dec.irq: tWbException(eb[wbndx].ifb.ip,eb[wbndx].cause);	// hardware irq
			eb[wbndx].dec.flt: tWbException(eb[wbndx].ifb.ip,eb[wbndx].cause);	// processing fault (divide by zero, tlb miss, ...)
			eb[wbndx].dec.rti:	tWbRti();
			eb[wbndx].dec.csrrw:	tWriteCSR(eb[wbndx].a,wbndx,eb[wbndx].dec.imm[13:0]);
			eb[wbndx].dec.csrrc:	tClrbitCSR(eb[wbndx].a,wbndx,eb[wbndx].dec.imm[13:0]);
			eb[wbndx].dec.csrrs:	tSetbitCSR(eb[wbndx].a,wbndx,eb[wbndx].dec.imm[13:0]);
			endcase
			if (eb[wbndx].dec.Rt=='d0 && eb[wbndx].dec.rfwr)
				rz[wbndx] <= 1'b1;
		end
		eb[wbndx] <= 'd0;
	end
	ou_rollback_bitmap[commit_thread][commit_tgt] <= 1'b0;
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
		CSR_MHARTID: res = {hartid_i[31:2],thread};
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
		CSR_MPLSTACK:	res = plStack[thread];
		CSR_MPMSTACK:	res = pmStack[thread];
		CSR_TIME:	res = wc_time[31:0];
		CSR_MSTATUS:	res = status[3];
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
		CSR_CAUSE:	cause[thread][regno[13:12]] <= val[11:0];
		CSR_MTVEC:	tvec[regno[1:0]] <= val;
		CSR_MPLSTACK:	plStack[thread] <= val;
		CSR_MPMSTACK:	pmStack[thread] <= val;
		CSR_MTIME:	begin wc_time_dat <= val; ld_time <= 1'b1; end
		CSR_MSTATUS:	status[3] <= val;
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
		CSR_MPMSTACK:	pmStack[thread] <= pmStack[thread] | val;
		CSR_MSTATUS:	status[3] <= status[3] | val;
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
		CSR_MCR0:			cr0[val[5:0]] <= 1'b0;
		CSR_MPMSTACK:	pmStack[thread] <= pmStack[thread] & ~val;
		CSR_MSTATUS:	status[3] <= status[3] & ~val;
		default:	;
		endcase
	end
end
endtask

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
	6'd40:	fnRegName = "lc";
	6'd41:	fnRegName = "lk1";
	6'd42:	fnRegName = "lk2";
	6'd43:	fnRegName = "r43";
	6'd44:	fnRegName = "ssp";
	6'd45:	fnRegName = "hsp";
	6'd46:	fnRegName = "msp";
	6'd47:	fnRegName = "isp";
	6'd48:	fnRegName = "t8";
	6'd49:	fnRegName = "t9";
	6'd50:	fnRegName = "t10";
	6'd51:	fnRegName = "t11";
	6'd52:	fnRegName = "s10";
	6'd53:	fnRegName = "s11";
	6'd54:	fnRegName = "s12";
	6'd55:	fnRegName = "s13";
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

endmodule
