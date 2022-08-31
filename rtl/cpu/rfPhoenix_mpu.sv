`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2017-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//		
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
//
// ============================================================================
//

module rfPhoenix_mpu(hartid_i, rst_i,
  clk4x_i, clk2x_i, clk_i, tm_clk_i, div_clk_i,
	pit_clk2, pit_gate2, pit_out2,
	irq_o,
    i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12,i13,i14,i15,i16,i17,i18,i19,
    i20,i21,i22,i23,i24,i25,i26,i27,i28,
	cti_o,bte_o,bok_i,cyc_o,stb_o,ack_i,err_i,we_o,sel_o,adr_o,dat_o,dat_i,
	sr_o, cr_o, rb_i, wcause);
input [63:0] hartid_i;
input rst_i;
input clk2x_i;
input clk4x_i;
input div_clk_i;
input clk_i;
input tm_clk_i;
input pit_clk2;
input pit_gate2;
output pit_out2;
output [2:0] irq_o;
input i1;
input i2;
input i3;
input i4;
input i5;
input i6;
input i7;
input i8;
input i9;
input i10;
input i11;
input i12;
input i13;
input i14;
input i15;
input i16;
input i17;
input i18;
input i19;
input i20;
input i21;
input i22;
input i23;
input i24;
input i25;
input i26;
input i27;
input i28;
output reg [2:0] cti_o;
output reg [1:0] bte_o;
input bok_i;
output reg cyc_o;
output reg stb_o;
input ack_i;
input err_i;
output reg we_o;
output reg [15:0] sel_o;
output reg [31:0] adr_o;
output reg [127:0] dat_o;
input [127:0] dat_i;
output sr_o;
output cr_o;
input rb_i;
output [15:0] wcause;

wire [2:0] cti;
wire [1:0] bte;
wire cyc,stb,we;
wire [15:0] sel;
(* mark_debug="true" *)
wire [63:0] adr;
reg [127:0] dati;
wire [127:0] dato;
wire nmi;
wire [2:0] irq;
wire [7:0] cause;
wire pic_ack;
wire [31:0] pic_dato;
wire pit_ack;
wire [31:0] pit_dato;
wire pit_out0, pit_out1;
wire pit_irq;
wire [31:0] pet_out;
wire crd_ack;
wire [63:0] crd_dato;
reg ack;
wire [63:0] ipt_dato;
wire ipt_ack;
wire [31:0] pcr;
wire [63:0] pcr2;
wire icl;           // instruction cache load
wire exv,rdv,wrv;
wire pulse60;
wire sptr_o;
wire [127:0] pkeys;

always @(posedge clk_i)
	bte_o <= bte;
always @(posedge clk_i)
	cti_o <= cti;
always @(posedge clk_i)
	cyc_o <= cyc;
always @(posedge clk_i)
	stb_o <= stb;
always @(posedge clk_i)
	we_o <= we;
always @(posedge clk_i)
	sel_o <= sel;
always @(posedge clk_i)
	adr_o <= adr;
always @(posedge clk_i)
	dat_o <= dato;

wire cs_pit = adr[31:12]==20'hFF960;

// Need to recreate the a3 address bit for 64 bit peripherals.
wire [31:0] adr64 = {adr[31:4],|sel[15:8],3'b00};
reg [63:0] dat64;
always @*
case(sel)
16'h00FF:	dat64 <= dato[63:0];
16'hFF00:	dat64 <= dato[127:64];
default:	dat64 <= dato[63:0];
endcase

// Precision Event Timers
/*
Thor2022_pet #(.NTIMER(8), .BITS(48)) upet1
(
	.rst_i(rst_i),
	.clk_i(clk_i),
	.cs_i(cs_pit),
	.cyc_i(cyc_o),
	.stb_i(stb_o),
	.ack_o(pit_ack),
	.sel_i(sel_o[15:8]|sel_o[7:0]),
	.we_i(we_o),
	.adr_i(adr64[9:0]),
	.dat_i(dat64),
	.dat_o(pit_dato),
	.cclk_i(clk_i),
	.out(pet_out)
);
*/
assign pit_out0 = pet_out[0];
assign pit_out1 = pet_out[1];
assign pit_out2 = pet_out[2];
assign pit_out3 = pet_out[3];
wire pet_irq = |pet_out[31:1];

wire irq3;

rfPhoenix_pic upic1
(
	.rst_i(rst_i),		// reset
	.clk_i(clk_i),		// system clock
	.cyc_i(cyc_o),
	.stb_i(stb_o),
	.ack_o(pic_ack),    // controller is ready
	.wr_i(we_o),		// write
	.adr_i(adr32),		// address
	.dat_i(dat32),
	.dat_o(pic_dato),
	.vol_o(),			// volatile register selected
	.i1(i1),
	.i2(i2),
	.i3(i3),
	.i4(i4),
	.i5(i5),
	.i6(i6),
	.i7(i7),
	.i8(i8),
	.i9(i9),
	.i10(i10),
	.i11(i11),
	.i12(i12),
	.i13(i13),
	.i14(i14),
	.i15(i15),
	.i16(i16),
	.i17(i17),
	.i18(i18),
	.i19(i19),
	.i20(i20),
	.i21(i21),
	.i22(i22),
	.i23(i23),
	.i24(i24),
	.i25(i25),
	.i26(i26),
	.i27(i27),
	.i28(i28),
	.i29(i29),			// 
	.i30(pet_irq),	// 
	.i31(pit_out0),	// time slice interrupt
	.irqo({irq3,irq}),
	.nmii(1'b0),
	.nmio(nmi),
	.causeo(cause)
);

assign irq_o = irq;

assign crd_dato = 64'd0;
assign crd_ack = 1'b0;

assign pit_dato = 32'h0;
always @(posedge clk_i)
	dati <= {4{pic_dato}}|{4{pit_dato}}|dat_i;
/*
casez({pic_ack,pit_ack,ack_i})
3'b1??:	dati <= {4{pic_dato}};
3'b01?:	dati <= {4{pit_dato}};
3'b001:	dati <= dat_i;
default:  dati <= dati;
endcase
*/
assign pit_ack = 1'b0;
always @(posedge clk_i)
	ack <= ack_i|pic_ack|pit_ack;

rfPhoenix ucpu1
(
  .hartid_i(hartid_i),
  .rst_i(rst_i),
  .clk_i(clk_i),
  .clk2x_i(clk2x_i),
//    .clk4x_i(clk4x_i),
  .wc_clk_i(tm_clk_i),
  .clock(pit_out3),		// MMU clock algorithm
//    .div_clk_i(div_clk_i),
	.nmi_i(nmi),
  .irq_i(irq),
  .icause_i(cause),
  .cti_o(cti),
  .bte_o(bte),
  .bok_i(bok_i),
	.vpa_o(),
  .cyc_o(cyc),
  .stb_o(stb),
  .ack_i(ack),
//    .err_i(err_i),
  .we_o(we),
  .sel_o(sel),
  .adr_o(adr),
  .dat_o(dato),
  .dat_i(dati),
//    .icl_o(icl),
  .sr_o(sr_o),
  .cr_o(cr_o),
  .rb_i(rb_i),
  .wcause(wcause)
);

endmodule
