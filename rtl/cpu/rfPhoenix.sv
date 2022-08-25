
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
sReorderEntry rob [0:11];

reg mc_busy;
reg [3:0] rfndx,exndx,oundx,wbndx,xrid,mc_rid;
reg [3:0] ithread, rthread, dthread, xthread, commit_thread;
reg rthread_v, dthread_v;
reg commit_wr, commit_wrv;
reg [15:0] commit_mask;
Regspec commit_tgt;
VecValue commit_bus;
reg [3:0] ip_thread;
reg [31:0] ips [0:15];
reg [31:0] ip;
reg [15:0] thread_busy;
CodeAddress iip, dip;
Instruction ir,xir,mir,insn;
sDecodeBus ddec,deco;
Regspec ra0,ra1,ra2,ra3,ra4;
Value rfo0, rfo1, rfo2, rfo3, rfo4;
VecValue vrfo0, vrfo1, vrfo2, vrfo3, vrfo4;
VecValue xa,xb,xc,xt,xm;
VecValue mca,mcb,mcc,mct,mcm;
Value res,mc_res;
VecValue vres,mc_vres;
wire mc_done, mcv_done;
wire ihit;
sMemoryRequest memreq;
sMemoryResponse memresp;
wire memreq_full;
reg memresp_fifo_rd;
wire memresp_fifo_empty;
wire memresp_fifo_v;
wire [1023:0] ic_line;
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

wire pipe_advance = (rfndx < 4'd12);

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
	.o3(vrfo3),
	.o4(vrfo4)
);

rfPhoenixAlu usalu1 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(xir),
	.a(xa[0]),
	.b(xb[0]),
	.c(xc[0]),
	.o(res)
);

rfPhoenixMcAlu usalu2 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir),
	.a(mca[0]),
	.b(mcb[0]),
	.c(mcc[0]),
	.o(mc_res),
	.done(mc_done)
);

rfPhoenixVecAlu uvalu1 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(xir),
	.a(xa),
	.b(xb),
	.c(xc),
	.o(vres)
);

rfPhoenixMcVecAlu uvalu2 (
	.rst(rst_i),
	.clk(clk_g),
	.ir(mir),
	.a(mca),
	.b(mcb),
	.c(mcc),
	.o(mc_vres),
	.done(mcv_done)
);

always_comb
	insn = ic_line >> {ip[5:0],3'b0};

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
	ip <= RSTIP;
	iip <= RSTIP;
	ir <= NOP_INSN;
	for (n = 0; n < NTHREADS; n = n + 1)
		ips[n] <= RSTIP;
	ithread <= 'd0;
	ip_thread <= 'd0;
	mc_busy <= 'd0;
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
end
endtask

task tInsnFetch;
begin
	if (pipe_advance) begin
		ip <= ips[ithread];
		ip_thread <= ithread;
		ithread <= ithread + 2'd1;
		if (ihit & ~thread_busy[ip_thread]) begin
			ips[ip_thread] <= ips[ip_thread] + 4'd5;
			ir <= insn;
			iip <= ip;
			rthread <= ip_thread;
			rthread_v <= 1'b1;
			thread_busy[ip_thread] <= 1'b1;
			ra0 <= insn.r2.Ra;
			ra1 <= insn.r2.Rb;
			ra2 <= insn.f3.Rc;
			ra3 <= insn.r2.mask;
			ra4 <= insn.r2.Rt;
		end
		else begin
			rthread_v <= 1'b0;
		end
	end
end
endtask

task tDecode;
begin
	if (pipe_advance) begin
		dip <= iip;
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
			rob[rfndx].thread <= dthread;
			rob[rfndx].a <= ddec.Ta ? vrfo0 : rfo0;
			rob[rfndx].b <= ddec.Tb ? vrfo1 : rfo1;
			rob[rfndx].c <= ddec.Tc ? vrfo2 : rfo2;
			rob[rfndx].mask <= rfo3;
			rob[rfndx].t <= ddec.Tt ? vrfo4 : rfo4;
			rob[rfndx].dec <= ddec;
			rob[rfndx].decoded <= 1'b1;
		end
	end
end
endtask

// Pick an empty rob entry for update.
ffz12 uffz1 (
	.i({rob[11].v,rob[10].v,rob[9].v,rob[8].v,
			rob[7].v,rob[6].v,rob[5].v,rob[4].v,
			rob[3].v,rob[2].v,rob[1].v,rob[0].v}),
	.o(rfndx)
);

// Pick a rob entry to execute.
ffo12 uffz2 (
	.i({rob[11].decoded & ~rob[11].out,rob[10].decoded & ~rob[10].out,rob[9].decoded & ~rob[9].out,rob[8].decoded & ~rob[8].out,
			rob[7].decoded & ~rob[7].out,rob[6].decoded & ~rob[6].out,rob[5].decoded & ~rob[5].out,rob[4].decoded & ~rob[4].out,
			rob[3].decoded & ~rob[3].out,rob[2].decoded & ~rob[2].out,rob[1].decoded & ~rob[1].out,rob[0].decoded & ~rob[0].out}),
	.o(exndx)
);

// Pick an rob entry thats out.
ffo12 uffz3 (
	.i({rob[11].out,rob[10].out,rob[9].out,rob[8].out,
			rob[7].out,rob[6].out,rob[5].out,rob[4].out,
			rob[3].out,rob[2].out,rob[1].out,rob[0].out}),
	.o(oundx)
);

// Pick a finished rob entry.
ffo12 uffz4 (
	.i({rob[11].executed,rob[10].executed,rob[9].executed,rob[8].executed,
			rob[7].executed,rob[6].executed,rob[5].executed,rob[4].executed,
			rob[3].executed,rob[2].executed,rob[1].executed,rob[0].executed}),
	.o(wbndx)
);

task tExecute;
begin
	if (exndx < 4'd12) begin
		rob[exndx].decoded <= 1'b0;
		rob[exndx].out <= 1'b1;
		if (rob[exndx].dec.multicycle) begin
			rob[exndx].out <= !mc_busy;
			if (!mc_busy) begin
				mc_busy <= 1'b1;
				mc_rid <= exndx;
				mir <= rob[exndx].ir;
				mca <= rob[exndx].a;
				mcb <= rob[exndx].b;
				mcc <= rob[exndx].c;
				mct <= rob[exndx].t;
				mcm <= rob[exndx].mask;
			end
		end
		else begin
			xrid <= exndx;
			xir <= rob[exndx].ir;
			xa <= rob[exndx].a;
			xb <= rob[exndx].b;
			xc <= rob[exndx].c;
			xt <= rob[exndx].t;
			xm <= rob[exndx].ir.r2.m ? rob[exndx].mask : 16'hFFFF;
			if (rob[exndx].load) begin
				if (!memreq_full) begin
					memreq.wr <= 1'b1;
					memreq.func <= rob[exndx].dec.loadu ? MR_LOADZ : MR_LOAD;
					memreq.sz <= rob[exndx].dec.memsz;
					if (rob[exndx].dec.memsz==vect) begin
						if (rob[exndx].dec.loadr)
							memreq.adr <= rob[exndx].dec.loadr ? rob[exndx].a[0] + rob[exndx].dec.imm;
						else begin
							memreq.adr <= rob[exndx].a[0] + rob[exndx].b[rob[exndx].step];
							if (rob[exndx].step != 4'd15 && rob[exndx].dec.loadn)
								rob[exndx].step <= rob[exndx].step + 2'd1;
						end
					end
					else
						memreq.adr <= rob[exndx].dec.loadr ? rob[exndx].a[0] + rob[exndx].dec.imm : rob[exndx].a[0] + rob[exndx].b[0];
				end
			end
			else if (rob[exndx].store) begin
				if (!memreq_full) begin
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
						if (rob[exndx].step!=4'd15 && rob[exndx].dec.storen)
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
end
endtask

task tExCall;
begin
	if (xrid < 4'd12) begin
		if (rob[xrid].out) begin
			rob[xrid].out <= 1'b0;
			rob[xrid].executed <= 1'b1;
			case(rob[xrid].ir.any.opcode)
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
	if (xrid < 4'd12) begin
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
				if (rob[memresp.rid].step!=4'd15) begin
					rob[memresp.rid].out <= 1'b1;
					rob[memresp.rid].executed <= 1'b0;
				end
				rob[memresp.rid].res[memresp.step] <= memresp.dat;
			end
			// Other load
			else if (rob[memresp.rid].dec.load) begin
				rob[memresp.rid].res <= memresp.dat;
				rob[memresp.rid].out <= 1'b0;
				rob[memresp.rid].executed <= 1'b1;
			end
			// Scatter store
			else if (rob[memresp.rid].dec.storen && rob[memresp.rid].dec.memsz==vect) begin
				if (rob[memresp.rid].step!=4'd15) begin
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

task tOut;
begin
	if (xrid < 4'd12) begin
		rob[xrid].out <= 1'b0;
		rob[xrid].executed <= 1'b1;
		if (rob[xrid].dec.storen && rob[xrid].dec.memsz==vect) begin
			if (rob[xrid].step!=4'd15) begin
				rob[xrid].out <= 1'b1;
				rob[xrid].executed <= 1'b0;
			end
		end
		if (rob[xrid].dec.Tt)
			rob[xrid].res <= vres;
		else if (!rob[xrid].dec.cjb)
			rob[xrid].res <= res;
	end
	if (mc_rid < 4'd12) begin
		if (rob[mc_rid].dec.is_vector ? mcv_done : mc_done) begin
			mc_busy <= 1'b0;
			rob[mc_rid].out <= 1'b0;
			rob[mc_rid].executed <= 1'b1;
			if (rob[mc_rid].dec.Tt)
				rob[mc_rid].res <= mc_vres;
			else
				rob[mc_rid].res <= mc_res;
		end
	end
end
endtask

task tWriteback;
begin
	if (wbndx < 4'd12) begin
		thread_busy[rob[wbndx].thread] <= 1'b0;
		commit_thread <= rob[wbndx].thread;
		commit_mask <= rob[wbndx].ir.r2.m ? rob[wbndx].mask : 16'hFFFF;
		commit_wr <= rob[wbndx].dec.rfwr;
		commit_wrv <= rob[wbndx].dec.vrfwr;
		commit_tgt <= rob[wbndx].dec.Rt;
		commit_bus <= rob[wbndx].res;
		rob[wbndx] <= 'd0;
	end
end
endtask

endmodule
