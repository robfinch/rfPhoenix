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
import wishbone_pkg::*;

module rfPhoenix_mpu(hartid_i, rst_i,
  clk4x_i, clk2x_i, clk_i, tm_clk_i, div_clk_i,
	pit_clk2, pit_gate2, pit_out2,
	irq_o,
    i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12,i13,i14,i15,i16,i17,i18,i19,
    i20,i21,i22,i23,i24,i25,i26,i27,i28,
  wb_req, wb_resp,
  bok_i,err_i,rb_i, wcause);
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
output wb_write_request128_t wb_req;
input wb_read_response128_t wb_resp;
input bok_i;
input err_i;
input rb_i;
output [15:0] wcause;

wb_write_request128_t iwb_req;
wb_read_response128_t iwb_resp;

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

always_ff @(posedge clk_i)
	wb_req <= iwb_req;

wire cs_pit = iwb_req.adr[31:12]==20'hFF960;

// Need to recreate the a3 address bit for 64 bit peripherals.
wire [31:0] adr64 = {iwb_req.adr[31:4],|iwb_req.sel[15:8],3'b00};
reg [63:0] dat64;
always_comb
case(sel)
16'h00FF:	dat64 <= iwb_req.dat[63:0];
16'hFF00:	dat64 <= iwb_req.dat[127:64];
default:	dat64 <= iwb_req.dat[63:0];
endcase
reg [31:0] dat32;
always_comb
case(sel)
16'h000F:	dat32 <= iwb_req.dat[31:0];
16'h00F0:	dat32 <= iwb_req.dat[63:32];
16'h0F00:	dat32 <= iwb_req.dat[95:64];
16'hF000:	dat32 <= iwb_req.dat[127:96];
default:	dat32 <= iwb_req.dat[31:0];
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
	.cyc_i(iwb_req.cyc),
	.stb_i(iwb_req.stb),
	.ack_o(pic_ack),    // controller is ready
	.wr_i(iwb_req.we),		// write
	.adr_i(iwb_req.adr[31:0]),		// address
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
assign pit_ack = 1'b0;

assign pit_dato = 32'h0;
always_ff @(posedge clk_i)
begin
	iwb_resp <= wb_resp;
	iwb_resp.dat <= {4{pic_dato}}|{4{pit_dato}}|wb_resp.dat;
	iwb_resp.ack <= wb_resp.ack|pic_ack|pit_ack;
end

/*
casez({pic_ack,pit_ack,ack_i})
3'b1??:	dati <= {4{pic_dato}};
3'b01?:	dati <= {4{pit_dato}};
3'b001:	dati <= dat_i;
default:  dati <= dati;
endcase
*/

rfPhoenix ucpu1
(
  .hartid_i(hartid_i),
  .rst_i(rst_i),
  .clk_i(clk_i),
  .wc_clk_i(tm_clk_i),
  .clock(pit_out3),		// MMU clock algorithm
	.nmi_i(nmi),
  .irq_i(irq),
  .icause_i(cause),
  .wb_req(iwb_req),
  .wb_resp(iwb_resp),
  .wcause(wcause)
);

endmodule
