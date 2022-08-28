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
sReorderEntry rob [0:NTHREADS-1];

// The following var indicates that r0 has been written for the thread.
// Only one write of r0 is allowed, to set the value to zero.
reg [NTHREADS-1:0] rz;
reg [3:0] exndx,oundx,wbndx,xrid,mc_rid,mc_rid1,mc_rid2,mc_rido;
reg exndx_v,oundx_v,wbndx_v;
reg [3:0] rfndx;
reg rfndx_v;
reg [3:0] mcv_ridi, mcv_rido;
reg [3:0] ithread, rthread, dthread, xthread, commit_thread;
reg rthread_v, dthread_v;
reg commit_wr, commit_wrv;
reg [15:0] commit_mask;
Regspec commit_tgt;
VecValue commit_bus;
reg [3:0] ip_thread, ip_thread2, ip_thread3;
reg [31:0] ips [0:15];
reg [31:0] ip, ip2, ip3;
reg [15:0] thread_busy;
CodeAddress iip, dip;
Instruction ir,dir,xir,mir,insn,mir1,mir2;
Postfix pfx,irpfx;
sDecodeBus ddec,deco;
Regspec ra0,ra1,ra2,ra3,ra4;
Value rfo0, rfo1, rfo2, rfo3, rfo4;
Value ximm,mcimm;
VecValue vrfo0, vrfo1, vrfo2, vrfo3, vrfo4;
VecValue xa,xb,xc,xt,xm;
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
sMemoryRequest memreq;
sMemoryResponse memresp;
wire memreq_full;
reg memresp_fifo_rd;
wire memresp_fifo_empty;
wire memresp_fifo_v;
wire [639:0] ic_line;
wire ic_valid;
reg [31:0] ptbr;
wire ipage_fault;
reg clr_ipage_fault;
wire itlbmiss;
reg clr_itlbmiss;
wire dce;
reg [9:0] asid;
wire [1:0] omode;
wire UserMode = omode==2'b00;
wire MUserMode = omode==2'b00;
wire takb;

wire pipe_advance = rfndx_v;

integer n;
initial begin
	rz = 'd0;
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
		rob[n] = 'd0;
	rthread_v = 'd0;
	dthread_v = 'd0;
	ra0 = 'd0;
	ra1 = 'd0;
	ra2 = 'd0;
	ra3 = 'd0;
	ra4 = 'd0;
	ddec = 'd0;
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
	.UserMode(UserMode),
	.MUserMode(MUserMode),
	.omode(omode),
	.ASID(asid),
	.bounds_chk(),
	.pe(pe),
	.ip(ip),
	.ihit(ihit),
	.ifStall(1'b0),
	.ic_line(ic_line),
	.ic_valid(ic_valid),
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
	.ptbr(ptbr),
	.ipage_fault(ipage_fault),
	.clr_ipage_fault(clr_ipage_fault),
	.itlbmiss(itlbmiss),
	.clr_itlbmiss(clr_itlbmiss)
);


rfPhoenix_decoder udec1
(
	.ir(ir),
	.pfx(irpfx),
	.deco(deco)
);

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

/*
rfPhoenixAlu usalu1 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(xir),
	.a(xa[0]),
	.b(xb[0]),
	.c(xc[0]),
	.imm(ximm),
	.o(res)
);

rfPhoenixMcAlu usalu2 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir),
	.a(mca[0]),
	.b(mcb[0]),
	.c(mcc[0]),
	.imm(mcimm),
	.o(mc_res),
	.done(mc_done),
	.ridi(mc_ridi),
	.rido(mc_rido)
);
*/
/*
rfPhoenixMcAlu usalu3 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir1),
	.a(mca1[0]),
	.b(mcb1[0]),
	.c(mcc1[0]),
	.o(mc_res1),
	.done(mc_done1)
);
*/
/*
rfPhoenixMcAlu usalu4 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir2),
	.a(mca2[0]),
	.b(mcb2[0]),
	.c(mcc2[0]),
	.o(mc_res2),
	.done(mc_done2)
);
*/

rfPhoenixVecAlu uvalu1 (
	.ir(xir),
	.a(xa),
	.b(xb),
	.c(xc),
	.imm(ximm),
	.o(vres)
);

/**********************************
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
***********************************/
/*
rfPhoenixMcVecAlu uvalu3 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir1),
	.a(mca1),
	.b(mcb1),
	.c(mcc1),
	.o(mc_vres1),
	.done(mcv_done1)
);
*/
/*
rfPhoenixMcVecAlu uvalu4 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir2),
	.a(mca2),
	.b(mcb2),
	.c(mcc2),
	.o(mc_vres2),
	.done(mcv_done2)
);
*/
always_comb
	{pfx,insn} = ic_line >> {ip3[5:0],3'b0};

always_ff @(posedge clk_g)
if (rst_i) begin
	tReset();
end
else begin
	tOnce();
	tInsnFetch();
	tDecode();
	tRegfetch();
	tExecute();
	tOut();
	tMemory();
	tWriteback();
end

task tReset;
integer n;
begin
	rz <= 'd0;
	ip <= RSTIP;
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
	for (n = 0; n < NTHREADS; n = n + 1)
		ips[n] <= RSTIP;
	ithread <= 'd0;
	ip_thread <= 'd0;
	mca_busy <= 'd0;
	thread_busy <= 'd0;
	for (n = 0; n < REB_ENTRIES; n = n + 1)
		rob[n] <= 'd0;
	rthread_v <= 'd0;
	dthread_v <= 'd0;
	ra0 <= 'd0;
	ra1 <= 'd0;
	ra2 <= 'd0;
	ra3 <= 'd0;
	ra4 <= 'd0;
	ddec <= 'd0;
	memreq <= 'd0;
end
endtask

task tOnce;
begin
	memreq.wr <= 1'b0;
	memresp_fifo_rd <= 1'b1;
end
endtask

task tInsnFetch;
begin
	if (pipe_advance) begin
		// 3 cycle pipeline delay reading the I$.
		// 1 for tag lookup and way determination
		// 2 for cache line lookup
		ip <= ips[ithread];
		ip2 <= ip;
		ip3 <= ip2;
		ip_thread <= ithread;
		ip_thread2 <= ip_thread;
		ip_thread3 <= ip_thread2;
		ithread <= ithread + 2'd1;
		if (ihit & ~thread_busy[ip_thread3]) begin
			ips[ip_thread3] <= ips[ip_thread3] + (pfx.opcode==PFX ? 4'd8 : 4'd5);
			ir <= insn;
			irpfx <= pfx;
			iip <= ip3;
			rthread <= ip_thread3;
			rthread_v <= 1'b1;
			thread_busy[ip_thread3] <= 1'b1;
			ra0 <= insn.r2.Ra;
			ra1 <= insn.r2.Rb;
			ra2 <= insn.f3.Rc;
			ra3 <= insn.r2.mask;
			ra4 <= insn.r2.Rt;
		end
		else begin
			rthread_v <= 1'b0;
			// On a miss, request a cache line load from the memory system. This
			// should eventuallly cause a hit for the thread.
			if (!ihit && !memreq_full) begin
				memreq.wr <= 1'b1;
				memreq.func <= MR_ICACHE_LOAD;
				memreq.adr <= ip3;
				memreq.dat <= ic_line;
				memreq.sz <= ic_valid ? tetra : nul;
			end
		end
	end
end
endtask

task tDecode;
begin
	if (pipe_advance) begin
		dip <= iip;
		dir <= ir;
		dthread <= rthread;
		dthread_v <= rthread_v;
		ddec <= deco;
	end
end
endtask

task tRegfetch;
begin
	if (pipe_advance) begin
		if (dthread_v) begin
			rob[rfndx].v <= 1'b1;
			rob[rfndx].ip <= dip;
			rob[rfndx].ir <= dir;
			rob[rfndx].thread <= dthread;
			rob[rfndx].a <= ddec.Ta ? vrfo0 : {NLANES{rfo0}};
			rob[rfndx].b <= ddec.Tb ? vrfo1 : {NLANES{rfo1}};
			rob[rfndx].c <= ddec.Tc ? vrfo2 : {NLANES{rfo2}};
			rob[rfndx].mask <= rfo3;
			rob[rfndx].t <= ddec.Tt ? vrfo4 : {NLANES{rfo4}};
			rob[rfndx].dec <= ddec;
			rob[rfndx].decoded <= 1'b1;
		end
	end
end
endtask

/*
// Pick an empty rob entry for update.
ffz12 uffz1 (
	.i({rob[11].v,rob[10].v,rob[9].v,rob[8].v,
			rob[7].v,rob[6].v,rob[5].v,rob[4].v,
			rob[3].v,rob[2].v,rob[1].v,rob[0].v}),
	.o(rfndx)
);
*/
always_comb
	rfndx = dthread;
always_comb
	rfndx_v = dthread_v;

/*
// Pick a rob entry to execute.
ffo12 uffz2 (
	.i({rob[11].decoded & ~rob[11].out,rob[10].decoded & ~rob[10].out,rob[9].decoded & ~rob[9].out,rob[8].decoded & ~rob[8].out,
			rob[7].decoded & ~rob[7].out,rob[6].decoded & ~rob[6].out,rob[5].decoded & ~rob[5].out,rob[4].decoded & ~rob[4].out,
			rob[3].decoded & ~rob[3].out,rob[2].decoded & ~rob[2].out,rob[1].decoded & ~rob[1].out,rob[0].decoded & ~rob[0].out}),
	.o(exndx)
);
*/
integer n2;
always_comb
begin
	exndx_v = 1'b0;
	exndx = 'd0;
	for (n2 = 0; n2 < NTHREADS; n2 = n2 + 1)
		if (rob[n2].decoded & ~rob[n2].out) begin
			exndx = n2;
			exndx_v = 1'b1;
		end
end

integer n3;
always_comb
begin
	oundx_v = 1'b0;
	oundx = 'd0;
	for (n3 = 0; n3 < NTHREADS; n3 = n3 + 1)
		if (rob[n3].out) begin
			oundx = n3;
			oundx_v = 1'b1;
		end
end

integer n4;
always_comb
begin
	wbndx_v = 1'b0;
	wbndx = 'd0;
	for (n4 = 0; n4 < NTHREADS; n4 = n4 + 1)
		if (rob[n4].executed) begin
			wbndx = n4;
			wbndx_v = 1'b1;
		end
end
/*
// Pick an rob entry thats out.
ffo12 uffz3 (
	.i({rob[11].out,rob[10].out,rob[9].out,rob[8].out,
			rob[7].out,rob[6].out,rob[5].out,rob[4].out,
			rob[3].out,rob[2].out,rob[1].out,rob[0].out}),
	.o(oundx)
);
*/

/*
// Pick a finished rob entry.
ffo12 uffz4 (
	.i({rob[11].executed,rob[10].executed,rob[9].executed,rob[8].executed,
			rob[7].executed,rob[6].executed,rob[5].executed,rob[4].executed,
			rob[3].executed,rob[2].executed,rob[1].executed,rob[0].executed}),
	.o(wbndx)
);
*/

task tExecute;
begin
	if (exndx_v) begin
		rob[exndx].decoded <= 1'b0;
		rob[exndx].out <= 1'b1;
		if (rob[exndx].dec.multicycle) begin
			rob[exndx].out <= 1'b1;
			mc_rid <= exndx;
			mcv_ridi <= exndx;
			mir <= rob[exndx].ir;
			mca <= rob[exndx].a;
			mcb <= rob[exndx].b;
			mcc <= rob[exndx].c;
			mct <= rob[exndx].t;
			mcimm <= rob[exndx].dec.imm;
			mcm <= rob[exndx].mask;
		end
		else begin
			xrid <= exndx;
			xir <= rob[exndx].ir;
			xa <= rob[exndx].a;
			xb <= rob[exndx].b;
			xc <= rob[exndx].c;
			xt <= rob[exndx].t;
			xm <= rob[exndx].ir.r2.m ? rob[exndx].mask : 16'hFFFF;
			ximm <= rob[exndx].dec.imm;
			if (rob[exndx].dec.load) begin
				if (!memreq_full && ihit) begin
					memreq.wr <= 1'b1;
					memreq.func <= rob[exndx].dec.loadu ? MR_LOADZ : MR_LOAD;
					memreq.sz <= rob[exndx].dec.memsz;
					if (rob[exndx].dec.memsz==vect) begin
						if (rob[exndx].dec.loadr)
							memreq.adr <= rob[exndx].a[0] + rob[exndx].dec.imm;
						else begin
							memreq.adr <= rob[exndx].a[0] + rob[exndx].b[rob[exndx].step];
							if (rob[exndx].step != NLANES-1 && rob[exndx].dec.loadn)
								rob[exndx].step <= rob[exndx].step + 2'd1;
						end
					end
					else
						memreq.adr <= rob[exndx].dec.loadr ? rob[exndx].a[0] + rob[exndx].dec.imm : rob[exndx].a[0] + rob[exndx].b[0];
				end
			end
			else if (rob[exndx].dec.store) begin
				if (!memreq_full && ihit) begin
					memreq.wr <= 1'b1;
					memreq.func <= MR_STORE;
					memreq.sz <= rob[exndx].dec.memsz;
					memreq.dat <= rob[exndx].t;
					if (rob[exndx].dec.memsz==vect) begin
						if (rob[exndx].dec.storen)
							memreq.sz <= tetra;
						memreq.adr <= rob[exndx].dec.storer ? rob[exndx].a[0] + rob[exndx].dec.imm : rob[exndx].a[0] + rob[exndx].b[rob[exndx].step];
						// For a scatter store select the current vector element, otherwise select entire vector (set above).
						if (rob[exndx].dec.memsz==vect && rob[exndx].dec.storen)
							memreq.dat <= rob[exndx].t[rob[exndx].step];
						// For scatter increment step
						if (rob[exndx].step!=NLANES-1 && rob[exndx].dec.storen)
							rob[exndx].step <= rob[exndx].step + 2'd1;
					end
					else
						memreq.adr <= rob[exndx].dec.storer ? rob[exndx].a[0] + rob[exndx].dec.imm : rob[exndx].a[0] + rob[exndx].b[0];
				end
			end
		end
	end
	tExCall();
	tExBranch();
	/*
	if (mc_rido < 4'd12) begin
		rob[mc_rido].res <= mc_res;
		rob[mc_rido].out <= 1'b0;
		rob[mc_rido].executed <= 1'b1;
	end
	*/
	if (mcv_rido < NTHREADS) begin
		rob[mcv_rido].res <= mc_vres;
		rob[mcv_rido].out <= 1'b0;
		rob[mcv_rido].executed <= 1'b1;
	end
end
endtask

task tExCall;
begin
	if (xrid < NTHREADS) begin
		if (rob[xrid].out) begin
			rob[xrid].out <= 1'b0;
			rob[xrid].executed <= 1'b1;
			case(rob[xrid].ir.any.opcode)
			NOP:				ips[rob[xrid].thread] <= ips[rob[xrid].thread] - 4'd4;
			CALLA,JMP:	begin ips[rob[xrid].thread] <= rob[xrid].ir.call.target; if (rob[xrid].dec.rfwr) rob[xrid].res <= ips[rob[xrid].thread] + 4'd5; end
			CALLR,BRA:	begin ips[rob[xrid].thread] <= rob[xrid].ir.call.target + ips[rob[xrid].thread]; if (rob[xrid].dec.rfwr) rob[xrid].res <= ips[rob[xrid].thread] + 4'd5; end
			default:	;
			endcase
		end
	end
end
endtask

task tExBranch;
begin
	if (xrid < NTHREADS) begin
		if (rob[xrid].out) begin
			rob[xrid].out <= 1'b0;
			rob[xrid].executed <= 1'b1;
			if (rob[xrid].dec.br) begin
				if (takb)
					ips[rob[xrid].thread] <= ips[rob[xrid].thread] + rob[xrid].dec.imm;
			end
		end
	end
end
endtask

task tMemory;
begin
	if (memresp_fifo_v) begin
		if (rob[memresp.rid].out) begin
			// If a gather load
			if (rob[memresp.rid].dec.loadn && rob[memresp.rid].dec.memsz==vect) begin
				if (rob[memresp.rid].step!=NLANES-1) begin
					rob[memresp.rid].out <= 1'b1;
					rob[memresp.rid].executed <= 1'b0;
				end
				rob[memresp.rid].res[memresp.step] <= memresp.res;
			end
			// Other load
			else if (rob[memresp.rid].dec.load) begin
				rob[memresp.rid].res <= memresp.res;
				rob[memresp.rid].out <= 1'b0;
				rob[memresp.rid].executed <= 1'b1;
			end
			// Scatter store
			else if (rob[memresp.rid].dec.storen && rob[memresp.rid].dec.memsz==vect) begin
				if (rob[memresp.rid].step!=NLANES-1) begin
					rob[memresp.rid].out <= 1'b1;
					rob[memresp.rid].executed <= 1'b0;
				end
			end
			// Other store
			else if (rob[memresp.rid].dec.store) begin
				rob[memresp.rid].out <= 1'b0;
				rob[memresp.rid].executed <= 1'b1;
			end	
			// Some other op
			else begin
				rob[memresp.rid].out <= 1'b0;
				rob[memresp.rid].executed <= 1'b1;
			end
		end
	end
end
endtask

task tOut;
begin
	if (xrid < NTHREADS) begin
		rob[xrid].out <= 1'b0;
		rob[xrid].executed <= 1'b1;
		if (rob[xrid].dec.storen && rob[xrid].dec.memsz==vect) begin
			if (rob[xrid].step!=NLANES-1) begin
				rob[xrid].out <= 1'b1;
				rob[xrid].executed <= 1'b0;
			end
		end
//		if (rob[xrid].dec.Tt)
			rob[xrid].res <= vres;
//		else if (!rob[xrid].dec.cjb)
//			rob[xrid].res <= res;
	end
	if (mc_rid < NTHREADS) begin
		if (rob[mc_rid].dec.is_vector ? mcv_done : mc_done) begin
			mca_busy[0] <= 1'b0;
			rob[mc_rid].out <= 1'b0;
			rob[mc_rid].executed <= 1'b1;
//			if (rob[mc_rid].dec.Tt)
				rob[mc_rid].res <= mc_vres;
//			else
//				rob[mc_rid].res <= mc_res;
		end
	end
	/*
	if (mc_rid1 < 4'd12) begin
		if (rob[mc_rid1].dec.is_vector ? mcv_done1 : mc_done1) begin
			mca_busy[1] <= 1'b0;
			rob[mc_rid1].out <= 1'b0;
			rob[mc_rid1].executed <= 1'b1;
			if (rob[mc_rid1].dec.Tt)
				rob[mc_rid1].res <= mc_vres1;
			else
				rob[mc_rid1].res <= mc_res1;
		end
	end
	*/
	/*
	if (mc_rid2 < 4'd12) begin
		if (rob[mc_rid2].dec.is_vector ? mcv_done2 : mc_done2) begin
			mca_busy[2] <= 1'b0;
			rob[mc_rid2].out <= 1'b0;
			rob[mc_rid2].executed <= 1'b1;
			if (rob[mc_rid2].dec.Tt)
				rob[mc_rid2].res <= mc_vres2;
			else
				rob[mc_rid2].res <= mc_res2;
		end
	end
	*/
end
endtask

task tWriteback;
begin
	if (wbndx_v) begin
		thread_busy[wbndx] <= 1'b0;
		commit_thread <= wbndx;
		commit_mask <= rob[wbndx].ir.r2.m ? rob[wbndx].mask : 16'hFFFF;
		commit_wr <= rob[wbndx].dec.rfwr;
		commit_wrv <= rob[wbndx].dec.vrfwr;
		commit_tgt <= rob[wbndx].dec.Rt;
		commit_bus <= rob[wbndx].res;
		if (rob[wbndx].dec.Rt=='d0 && rob[wbndx].dec.rfwr)
			rz[wbndx] <= 1'b1;
		rob[wbndx] <= 'd0;
	end
end
endtask

endmodule
