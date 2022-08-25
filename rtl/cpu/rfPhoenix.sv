
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
reg [3:0] rfndx,exndx,oundx,wbndx;
reg [3:0] ithread, rthread, dthread, commit_thread;
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
	tInsnFetch();
	tDecode();
	tRegfetch();
	tExecute();
	tOut();
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
				mir <= rob[exndx].ir;
				mca <= rob[exndx].a;
				mcb <= rob[exndx].b;
				mcc <= rob[exndx].c;
				mct <= rob[exndx].t;
				mcm <= rob[exndx].mask;
			end
		end
		else begin
			xir <= rob[exndx].ir;
			xa <= rob[exndx].a;
			xb <= rob[exndx].b;
			xc <= rob[exndx].c;
			xt <= rob[exndx].t;
			xm <= rob[exndx].ir.r2.m ? rob[exndx].mask : 16'hFFFF;
			case(rob[exndx].ir.any.opcode)
			CALLA,JMP:	ip[rob[exndx].thread] <= rob[exndx].ir.call.target;
			CALLR,BRA:	ip[rob[exndx].thread] <= rob[exndx].ir.call.target + ip[rob[exndx].thread];
			default:	;
			endcase
		end
	end
end
endtask

task tOut;
begin
	if (oundx < 4'd12) begin
		if (!rob[oundx].dec.multicycle) begin
			rob[oundx].out <= 1'b0;
			rob[oundx].executed <= 1'b1;
			if (rob[oundx].dec.Tt)
				rob[oundx].res <= vres;
			else
				rob[oundx].res <= res;
		end
		else if (rob[oundx].dec.is_vector ? mcv_done : mc_done) begin
			mc_busy <= 1'b0;
			rob[oundx].out <= 1'b0;
			rob[oundx].executed <= 1'b1;
			if (rob[oundx].dec.Tt)
				rob[oundx].res <= mc_vres;
			else
				rob[oundx].res <= mc_res;
		end
	end
end
endtask

task tWriteback;
begin
	if (wbndx < 4'd12) begin
		thread_busy[rob[wbndx].thread] <= 1'b0;
		commit_thread <= rob[wbndx].thread;
		commit_mask <= rob[wbndx].mask;
		commit_wr <= rob[wbndx].dec.rfwr;
		commit_wrv <= rob[wbndx].dec.vrfwr;
		commit_tgt <= rob[wbndx].dec.Rt;
		commit_bus <= rob[wbndx].res;
		rob[wbndx] <= 'd0;
	end
end
endtask

endmodule
