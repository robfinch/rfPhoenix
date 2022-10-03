// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	gpr_regfile.sv
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
// ============================================================================

//import const_pkg::*;
import rfPhoenixPkg::*;

module gpr_regfile(clk, wr, wa, i, ra, o);
parameter ZERO_BYPASS = 1'b0;
input clk;
input [3:0] wr;
input [5+TidMSB+1:0] wa;
input value_t i;
input [5+TidMSB+1:0] ra;
output value_t o;

// Tools do not infer RAMs quite as well as explicitly declaring them. All
// the RAMs use the same number of block RAMs so it is tempting to have
// multiple register sets where there are fewer threads.

//`ifdef IS_SIM

integer k;

   // xpm_memory_sdpram: Simple Dual Port RAM
   // Xilinx Parameterized Macro, version 2020.2

   xpm_memory_sdpram #(
      .ADDR_WIDTH_A(5+TidMSB+2),
      .ADDR_WIDTH_B(5+TidMSB+2),
      .AUTO_SLEEP_TIME(0),
      .BYTE_WRITE_WIDTH_A(8),
      .CASCADE_HEIGHT(0),
      .CLOCKING_MODE("common_clock"), // String
      .ECC_MODE("no_ecc"),            // String
      .MEMORY_INIT_FILE("none"),      // String
      .MEMORY_INIT_PARAM("0"),        // String
      .MEMORY_OPTIMIZATION("true"),   // String
      .MEMORY_PRIMITIVE("block"),      // String
      .MEMORY_SIZE(8192),             // DECIMAL
      .MESSAGE_CONTROL(0),            // DECIMAL
      .READ_DATA_WIDTH_B($bits(value_t)),
      .READ_LATENCY_B(1),
      .READ_RESET_VALUE_B("0"),       // String
      .RST_MODE_A("SYNC"),            // String
      .RST_MODE_B("SYNC"),            // String
      .SIM_ASSERT_CHK(0),             // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
      .USE_EMBEDDED_CONSTRAINT(0),    // DECIMAL
      .USE_MEM_INIT(1),               // DECIMAL
      .WAKEUP_TIME("disable_sleep"),  // String
      .WRITE_DATA_WIDTH_A($bits(value_t)),
      .WRITE_MODE_B("no_change")      // String
   )
   xpm_memory_sdpram_inst (
      .dbiterrb(),             // 1-bit output: Status signal to indicate double bit error occurrence
                               // on the data output of port B.

      .doutb(o),               // READ_DATA_WIDTH_B-bit output: Data output for port B read operations.
      .sbiterrb(),             // 1-bit output: Status signal to indicate single bit error occurrence
                               // on the data output of port B.

      .addra(wa),            // ADDR_WIDTH_A-bit input: Address for port A write operations.
      .addrb(ra),            // ADDR_WIDTH_B-bit input: Address for port B read operations.
      .clka(clk),              // 1-bit input: Clock signal for port A. Also clocks port B when
                               // parameter CLOCKING_MODE is "common_clock".

      .clkb(clk),              // 1-bit input: Clock signal for port B when parameter CLOCKING_MODE is
                               // "independent_clock". Unused when parameter CLOCKING_MODE is
                               // "common_clock".

      .dina(i),                // WRITE_DATA_WIDTH_A-bit input: Data input for port A write operations.
      .ena(wr),                // 1-bit input: Memory enable signal for port A. Must be high on clock
                               // cycles when write operations are initiated. Pipelined internally.

      .enb(1'b1),              // 1-bit input: Memory enable signal for port B. Must be high on clock
                                       // cycles when read operations are initiated. Pipelined internally.

      .injectdbiterra(1'b0), 	// 1-bit input: Controls double bit error injection on input data when
                              // ECC enabled (Error injection capability is not available in
                              // "decode_only" mode).

      .injectsbiterra(1'b0), 	// 1-bit input: Controls single bit error injection on input data when
                              // ECC enabled (Error injection capability is not available in
                              // "decode_only" mode).

      .regceb(1'b1),	        // 1-bit input: Clock Enable for the last register stage on the output
                      	      // data path.

      .rstb(1'b0),             // 1-bit input: Reset signal for the final port B output register stage.
                              // Synchronously resets output port doutb to the value specified by
                              // parameter READ_RESET_VALUE_B.

      .sleep(1'b0),           // 1-bit input: sleep signal to enable the dynamic power saving feature.
      .wea(wr)                // WRITE_DATA_WIDTH_A/BYTE_WRITE_WIDTH_A-bit input: Write enable vector
                              // for port A input data port dina. 1 bit wide when word-wide writes are
                              // used. In byte-wide write configurations, each bit controls the
                              // writing one byte of dina to address addra. For example, to
                              // synchronously write only bits [15-8] of dina when WRITE_DATA_WIDTH_A
                              // is 32, wea would be 4'b0010.

   );

/*

(* ram_style = "block" *)
value_t [NTHREADS*NREGS-1:0] mem;
initial begin
	for (k = 0; k < NTHREADS*NREGS; k = k + 1)
		mem[k] <= 32'd0;
end
reg [5+TidMSB+1:0] rar;
always_ff @(posedge clk)
	rar <= ra;
always_ff @(posedge clk)
begin
	if (wr[0]) mem[wa][ 7: 0] <= i[ 7: 0];
	if (wr[1]) mem[wa][15: 8] <= i[15: 8];
	if (wr[2]) mem[wa][23:16] <= i[23:16];
	if (wr[3]) mem[wa][31:24] <= i[31:24];
end
always_comb
	o = mem[rar];
*/
/*
`else

reg rstb;
always_comb
	rstb <= 1'b0;

generate begin : gRegfile
case(NTHREADS)
1,2,3,4:
//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG
blk_mem256x32 bmem0 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb(ra),  // input wire [7 : 0] addrb
  .doutb(o),  // output wire [31 : 0] doutb
  .rstb(rstb)
);
5,6,7,8:
blk_mem512x32 bmem1 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb(ra),  // input wire [7 : 0] addrb
  .doutb(o),  // output wire [31 : 0] doutb
  .rstb(rstb)
);
9,10,11,12,13,14,15,16:
blk_mem1024x32 bmem2 (
  .clka(clk),    // input wire clka
  .ena(wr),      // input wire ena
  .wea(wr),      // input wire [0 : 0] wea
  .addra(wa),  // input wire [7 : 0] addra
  .dina(i),    // input wire [31 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(1'b1),      // input wire enb
  .addrb(ra),  // input wire [7 : 0] addrb
  .doutb(o),  // output wire [31 : 0] doutb
  .rstb(rstb)
);
endcase
end
endgenerate

//`endif
*/
endmodule
