
module rfPhoenix_tb();

reg rst;
reg clk;
wire [6:0] state;
wb_write_request128_t req;
wb_read_response128_t resp;

initial begin
	clk = 1'b0;
	rst = 1'b0;
	#10 rst = 1'b1;
	#300 rst = 1'b0;
end

always #10 clk = ~clk;

rfPhoenix_mpu ucpu (
	.hartid_i(32'h10),
	.rst_i(rst),
	.clk_i(clk),
	.tm_clk_i(clk),
	.div_clk_i(1'b0),
	.clk4x_i(1'b0),
	.clk2x_i(1'b0),
//	.clock(1'b0),
	.pit_clk2(1'b0),
	.pit_gate2(1'b0),
	.pit_out2(),
//	.nmi_i(1'b0),
	.irq_o(),
	.i1(1'b0),
	.i2(1'b0),
	.i3(1'b0),
	.i4(1'b0),
	.i5(1'b0),
	.i6(1'b0),
	.i7(1'b0),
	.i8(1'b0),
	.i9(1'b0),
	.i10(1'b0),
	.i11(1'b0),
	.i12(1'b0),
	.i13(1'b0),
	.i14(1'b0),
	.i15(1'b0),
	.i16(1'b0),
	.i17(1'b0),
	.i18(1'b0),
	.i19(1'b0),
	.i20(1'b0),
	.i21(1'b0),
	.i22(1'b0),
	.i23(1'b0),
	.i24(1'b0),
	.i25(1'b0),
	.i26(1'b0),
	.i27(1'b0),
	.i28(1'b0),
//	.icause_i('d0),
	.wb_req(req),
	.wb_resp(resp),
//	.state_o(state),
//	.trigger_o(),
	.bok_i(1'b0),
	.err_i(1'b0),
	.rb_i(1'b0),
	.wcause()
);

scratchmem128 umem1
(
	.rst_i(rst), 
	.clk_i(clk),
	.cti_i(req.cti),
	.tid_i(req.tid),
	.tid_o(resp.tid),
	.cs_i(req.cyc),
	.cyc_i(req.cyc),
	.stb_i(req.stb),
	.next_o(resp.next),
	.ack_o(resp.ack),
	.we_i(req.we),
	.sel_i(req.sel),
	.adr_i(req.adr[17:0]),
	.dat_i(req.dat),
	.dat_o(resp.dat),
	.ip('d0),
	.sp('d0)
);

endmodule
