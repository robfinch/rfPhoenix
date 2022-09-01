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
sReorderEntry rob [0:NTHREADS-1];
sReorderEntry robq [0:NTHREADS-1];

// The following var indicates that r0 has been written for the thread.
// Only one write of r0 is allowed, to set the value to zero.
reg [NTHREADS-1:0] rz;
reg [NTHREADS-1:0] gie;
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
reg [3:0] ip_thread, ip_thread1, ip_thread2, ip_thread3, ip_thread4, ip_thread5;
reg [31:0] ips [0:NTHREADS-1];
reg [31:0] ip, ip2, ip3, ip4;
reg [NTHREADS-1:0] thread_busy;
CodeAddress iip, dip, ip_icline, ip_insn;
Instruction ir,dir,xir,mir,insn,mir1,mir2;
Postfix pfx,irpfx;
sDecodeBus ddec,deco;
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
sMemoryRequest memreq;
sMemoryResponse memresp;
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
reg [2:0] sp_sel [0:NTHREADS-1];
reg [2:0] istk_depth [0:NTHREADS-1];
reg [2:0] ir_sp_sel;
Instruction [NTHREADS-1:0] exc_bucket;
CodeAddress [NTHREADS-1:0] exc_ip;
CauseCode icause,dcause;
reg [7:0] tid;
CodeAddress last_adr;
reg [15:0] exv;
reg [3:0] tmpndx, prev_exndx;

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

wire pipe_advance = rfndx_v;

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
	.clr_itlbmiss(clr_itlbmiss)
);


rfPhoenix_decoder udec1
(
	.ir(ir),
	.sp_sel(ir_sp_sel),
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

task tDisplayRob;
integer n;
begin
	$display("  ROB:");
	for (n = 0; n < NTHREADS; n = n + 1) begin
		$display("  %d: %c%c%c%c ip=%h ir=%h res=%h a=%h b=%h c=%h i=%h", n[3:0],
			rob[n].v ? "v":"-",
			rob[n].decoded ? "d" : "-",
			rob[n].out ? "o" : "-",
			rob[n].executed ? "x" : "-",
			rob[n].ip,
			rob[n].ir,
			rob[n].res,
			rob[n].a,
			rob[n].b,
			rob[n].c,
			rob[n].dec.imm
		);
	end
end
endtask

always_ff @(posedge clk_g)
	{pfx,insn} <= ic_line >> {ip_icline[5:0],3'b0};
always_ff @(posedge clk_g)
	ic_line2 <= ic_line;
always_ff @(posedge clk_g)
	ip_insn <= ip_icline;
always_ff @(posedge clk_g)
	ihit2 <= ihit;

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
	prev_exndx <= exndx;
	tDisplayRegs();
	tDisplayRob();
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
	end
	ithread <= 'd0;
	ip_thread <= 'd0;
	mca_busy <= 'd0;
	thread_busy <= 'd0;
	for (n = 0; n < NTHREADS; n = n + 1)
		rob[n] <= 'd0;
	rthread_v <= 'd1;
	dthread_v <= 'd1;
	ra0 <= 'd0;
	ra1 <= 'd0;
	ra2 <= 'd0;
	ra3 <= 'd0;
	ra4 <= 'd0;
	ddec <= 'd0;
	memreq <= 'd0;
	last_adr <= 'd0;
	xrid <= 'd15;
	mc_rid <= 'd15;
	prev_exndx <= 'd0;
end
endtask

task tOnce;
begin
	memreq.wr <= 1'b0;
	memresp_fifo_rd <= 1'b1;
end
endtask

task tSpSel;
input [5:0] i;
output [5:0] o;
begin
	if (i==6'd31)
		case(sp_sel[ip_thread3])
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

task tInsnFetch;
integer n;
begin
	begin
		// 3 cycle pipeline delay reading the I$.
		// 1 for tag lookup and way determination
		// 2 for cache line lookup
		ithread <= ithread + 2'd1;
		if (ithread==NTHREADS-1)
			ithread <= 'd0;
		/*
		if (thread_busy[ithread]) begin
			ip <= 32'hFFFFFFFC;
			ip_thread1 <= 4'd15;
		end
		else begin
			thread_busy[ithread] <= 1'b1;
			ip <= ips[ithread];
			ip_thread1 <= ithread;
		end
		*/
		ip <= ips[ithread];
		ip_thread1 <= ithread;
		ip_thread2 <= ip_thread1;
		ip_thread3 <= ip_thread2;
		ip_thread4 <= ip_thread3;
		ip_thread5 <= ip_thread4;
		$display("  ip_insn=%h  insn=%h postfix=%h", ip_insn, insn, pfx);
		$display("  ip_thread5=%h", ip_thread5);
		for (n = 0; n < NTHREADS; n = n + 1)
			$display("  ips[%d]=%h", n[3:0], ips[n]);
		icause <= 'd0;
		rthread <= ip_thread5;
		rthread_v <= 1'b1;
		if (ihit2 && !thread_busy[ip_thread5]) begin
			if (exc_bucket[ip_thread5].any.opcode!=6'd0) begin
				ir <= exc_bucket[ip_thread5];
				iip <= exc_ip[ip_thread5];
			end
			else if (irq_i > pmStack[ip_thread5][3:1] && gie[ip_thread5]) begin
				icause <= {1'b1,irq_i,FLT_BRK};
				ir <= insn;
				iip <= ip_insn;
			end
			else begin
				$display("pfx.opcode=%h",pfx.opcode);
				if (pfx.opcode==PFX) begin
					ips[ip_thread5] <= ips[ip_thread5] + 4'd8;
					$display("  %h", ips[ip_thread5] + 4'd8);
				end
				else begin
					ips[ip_thread5] <= ips[ip_thread5] + 4'd5;
					$display("  %h", ips[ip_thread5] + 4'd5);
				end
				ir <= insn;
				iip <= ip_insn;
			end
			irpfx <= pfx;
			ir_sp_sel <= sp_sel[ip_thread5];
			rthread <= ip_thread5;
			rthread_v <= 1'b1;
			thread_busy[ip_thread5] <= 1'b1;
			tSpSel(insn.r2.Ra,ra0);
			tSpSel(insn.r2.Rb,ra1);
			tSpSel(insn.f3.Rc,ra2);
			ra3 <= insn.r2.mask;
			tSpSel(insn.r2.Rt,ra4);
		end
		else begin
			ir <= BRA;
			irpfx <= NOP;
//			rthread_v <= 1'b0;
			rthread <= 4'd15;
			// On a miss, request a cache line load from the memory system. This
			// should eventually cause a hit for the thread.
			// Suppress consecutive requests for the same cache line load.
			// The old cache line is passed back for the victim buffer.
			if (!ihit2 && !memreq_full) begin
				if (ip_insn[31:6] != last_adr[31:6]) begin
					last_adr <= ip_insn;
					tid <= tid + 2'd1;
					memreq.tid <= tid;
					// The following should cause an imiss to be executed like a
					// load or store.
//					rob[ip_thread5] <= 'd0;
//					rob[ip_thread5].v <= 1'b1;
//					rob[ip_thread5].decoded <= 1'b1;
//					rob[ip_thread5].imiss <= 1'b1;
//					rob[ip_thread5].res <= ic_line2;
//					rob[ip_thread5].badAddr <= ip_insn;
//					rob[ip_thread5].count <= ic_valid ? tetra : nul;
					memreq.rid <= ip_thread5;
					memreq.wr <= 1'b1;
					memreq.func <= MR_ICACHE_LOAD;
					memreq.adr <= {ip_insn[31:6],6'd0};
					memreq.vcadr <= ip_insn;
					memreq.dat <= ic_line2;
					memreq.sz <= ic_valid ? tetra : nul;
				end
			end
		end
	end
	/*
	else begin
		if (ihit && !thread_busy[ip_thread3])
			rthread_v <= 1'b1;
	end
	*/
end
endtask

task tDecode;
begin
	begin
		dip <= iip;
		dir <= ir;
		dcause <= icause;
		dthread <= rthread;
		dthread_v <= rthread_v;
		ddec <= deco;
	end
end
endtask

Value csro;
always_comb
	tReadCSR(csro,rfndx,ddec.imm[13:0]);

task tRegfetch;
begin
	begin
		if (rfndx < NTHREADS) begin
			rob[rfndx].v <= 1'b1;
			rob[rfndx].cause <= dcause;
			rob[rfndx].ip <= dip;
			rob[rfndx].ir <= dir;
			rob[rfndx].thread <= dthread;
			rob[rfndx].a <= ddec.Ta ? vrfo0 : {NLANES{rfo0}};
			rob[rfndx].b <= ddec.Tb ? vrfo1 : {NLANES{rfo1}};
			if (ddec.csrrd|ddec.csrrw|ddec.csrrc|ddec.csrrs)
				rob[rfndx].c <= {NLANES{csro}};
			else
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
	tmpndx = 'd0;
	exv = 16'h0000;
	for (n2 = 0; n2 < NTHREADS; n2 = n2 + 1)
		exv[n2] = rob[n2].decoded & ~rob[n2].out;
	for (n2 = 0; n2 < 16; n2 = n2 + 1) begin
		tmpndx = prev_exndx[3:0]+n2[3:0];
		if (exv[tmpndx] && !exndx_v) begin
			exndx = tmpndx;
			exndx_v = 1'b1;
		end
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
			xtt <= rob[exndx].dec.Tt;
			xm <= rob[exndx].ir.r2.m ? rob[exndx].mask : 16'hFFFF;
			ximm <= rob[exndx].dec.imm;
			xasid <= asid[exndx];
			xhmask <= hmask;
			/*
			if ((rob[exndx].dec.load|rob[exndx].dec.store|rob[exndx].imiss) && omode[exndx]!=2'd3 && rob[exndx].agen==1'b0) begin
				tlbrid <= exndx;
				tlba <= rob[exndx].a;
				tlbb <= rob[exndx].b;
			end
			else
			*/
			begin
				/*
				if (rob[exndx].imiss) begin
					tid <= tid + 2'd1;
					memreq.tid <= tid;
					memreq.rid <= ip_thread5;
					memreq.wr <= 1'b1;
					memreq.func <= MR_ICACHE_LOAD;
					memreq.adr <= {rob[exndx].badAddr[31:6],6'd0};
					memreq.vcadr <= rob[exndx].badAddr;
					memreq.dat <= rob[exndx].res;	// the cache line
					memreq.sz <= rob[exndx].count;
				end
				else 
				*/
				if (rob[exndx].dec.load) begin
					if (!memreq_full && ihit) begin
						tid <= tid + 2'd1;
						memreq.tid <= tid;
						memreq.wr <= 1'b1;
						memreq.func <= rob[exndx].dec.loadu ? MR_LOADZ : MR_LOAD;
						memreq.sz <= rob[exndx].dec.memsz;
						memreq.asid = asid[exndx];
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
						tid <= tid + 2'd1;
						memreq.tid <= tid;
						memreq.wr <= 1'b1;
						memreq.func <= MR_STORE;
						memreq.sz <= rob[exndx].dec.memsz;
						memreq.asid = asid[exndx];
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
end
endtask

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RTI instruction
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
		tWbException(rob[wbndx].ip,{4'h0,FLT_RTI});
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
		badaddr[wbndx][omode[wbndx]] <= rob[wbndx].badAddr;
		rob[wbndx].cause <= 'd0;
	end
end
endtask

task tWriteback;
begin
	if (wbndx_v) begin
		$display("Writeback");
		thread_busy[wbndx] <= 1'b0;
		if (|rob[wbndx].cause)
			tWbException(rob[wbndx].ip,rob[wbndx].cause);
		else begin
			exc_bucket[wbndx] <= 'd0;
			$display("  thread:%d ip=%h ir=%h", wbndx, rob[wbndx].ip, rob[wbndx].ir);
			if (rob[wbndx].dec.rfwr)
				$display("  %s=%h", fnRegName(rob[wbndx].dec.Rt), rob[wbndx].res);
			commit_thread <= wbndx;
			commit_mask <= rob[wbndx].ir.r2.m ? rob[wbndx].mask : 16'hFFFF;
			commit_wr <= rob[wbndx].dec.rfwr;
			commit_wrv <= rob[wbndx].dec.vrfwr;
			commit_tgt <= rob[wbndx].dec.Rt;
			commit_bus <= rob[wbndx].res;
			case(1'b1)
			rob[wbndx].dec.brk:	tWbException(rob[wbndx].ip + 4'd5,rob[wbndx].cause);	// BRK instruction
			rob[wbndx].dec.irq: tWbException(rob[wbndx].ip,rob[wbndx].cause);	// hardware irq
			rob[wbndx].dec.flt: tWbException(rob[wbndx].ip,rob[wbndx].cause);	// processing fault (divide by zero, tlb miss, ...)
			rob[wbndx].dec.rti:	tWbRti();
			rob[wbndx].dec.csrrw:	tWriteCSR(rob[wbndx].a,wbndx,rob[wbndx].dec.imm[13:0]);
			rob[wbndx].dec.csrrc:	tClrbitCSR(rob[wbndx].a,wbndx,rob[wbndx].dec.imm[13:0]);
			rob[wbndx].dec.csrrs:	tSetbitCSR(rob[wbndx].a,wbndx,rob[wbndx].dec.imm[13:0]);
			endcase
			if (rob[wbndx].dec.Rt=='d0 && rob[wbndx].dec.rfwr)
				rz[wbndx] <= 1'b1;
		end
		rob[wbndx] <= 'd0;
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
output Value res;
input [3:0] thread;
input [13:0] regno;
begin
	if (regno[13:12] <= omode[thread]) begin
		casez({2'b00,regno[13:0]})
		CSR_MHARTID: res = {hartid_i[31:4],thread};
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
input [3:0] thread;
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
input [3:0] thread;
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
input [3:0] thread;
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
