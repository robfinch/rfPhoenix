// ============================================================================
//        __
//   \\__/ o\    (C) 2018-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
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
//
module char_ram(clk_i, cs_i, we_i, adr_i, dat_i, dat_o, dot_clk_i, ce_i,
  fontAddress_i, char_code_i, maxScanpix_i, maxscanline_i, scanline_i, bmp_o);
parameter pFontFile = "char_bitmaps_12x18.mem";
input clk_i;
input cs_i;
input we_i;
input [15:0] adr_i;
input [7:0] dat_i;
output [7:0] dat_o;
input dot_clk_i;
input ce_i;
input [15:0] fontAddress_i;
input [12:0] char_code_i;
input [5:0] maxScanpix_i;
input [5:0] maxscanline_i;
input [5:0] scanline_i;
output reg [63:0] bmp_o;

wire [7:0] memo;
reg [15:0] rcc, rcc0, rcc1;
reg [2:0] rcc200, rcc201, rcc202;
reg [63:0] bmp1;
reg [4:0] bndx, stpndx;
reg [7:0] bmp [0:7];

wire pe_cs;
edge_det ued1 (.rst(1'b0), .clk(clk_i), .ce(1'b1), .i(cs_i), .pe(pe_cs), .ne(), .ee());


// xpm_memory_tdpram: True Dual Port RAM
// Xilinx Parameterized Macro, version 2020.2

xpm_memory_tdpram #(
  .ADDR_WIDTH_A(16),
  .ADDR_WIDTH_B(16),
  .AUTO_SLEEP_TIME(0),
  .BYTE_WRITE_WIDTH_A(8),
  .BYTE_WRITE_WIDTH_B(8),
  .CASCADE_HEIGHT(0),
  .CLOCKING_MODE("independent_clock"), // String
  .ECC_MODE("no_ecc"),            // String
  .MEMORY_INIT_FILE(pFontFile),	  // String
  .MEMORY_INIT_PARAM(""),        // String
  .MEMORY_OPTIMIZATION("true"),   // String
  .MEMORY_PRIMITIVE("block"),      // String
  .MEMORY_SIZE(524288),
  .MESSAGE_CONTROL(0),
  .READ_DATA_WIDTH_A(8),
  .READ_DATA_WIDTH_B(8),
  .READ_LATENCY_A(2),
  .READ_LATENCY_B(1),
  .READ_RESET_VALUE_A("0"),       // String
  .READ_RESET_VALUE_B("0"),       // String
  .RST_MODE_A("SYNC"),            // String
  .RST_MODE_B("SYNC"),            // String
  .SIM_ASSERT_CHK(0),             // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
  .USE_EMBEDDED_CONSTRAINT(0),    // DECIMAL
  .USE_MEM_INIT(1),
  .WAKEUP_TIME("disable_sleep"),  // String
  .WRITE_DATA_WIDTH_A(8),
  .WRITE_DATA_WIDTH_B(8),
  .WRITE_MODE_A("no_change"),     // String
  .WRITE_MODE_B("no_change")      // String
)
xpm_memory_tdpram_inst (
  .dbiterra(),             // 1-bit output: Status signal to indicate double bit error occurrence
                                   // on the data output of port A.

  .dbiterrb(),             // 1-bit output: Status signal to indicate double bit error occurrence
                                   // on the data output of port A.

  .douta(dat_o),                   // READ_DATA_WIDTH_A-bit output: Data output for port A read operations.
  .doutb(memo),                    // READ_DATA_WIDTH_B-bit output: Data output for port B read operations.
  .sbiterra(),             // 1-bit output: Status signal to indicate single bit error occurrence
                                   // on the data output of port A.

  .sbiterrb(),             // 1-bit output: Status signal to indicate single bit error occurrence
                                   // on the data output of port B.

  .addra(adr_i),                   // ADDR_WIDTH_A-bit input: Address for port A write and read operations.
  .addrb(rcc1), 	                  // ADDR_WIDTH_B-bit input: Address for port B write and read operations.
  .clka(clk_i),                     // 1-bit input: Clock signal for port A. Also clocks port B when
                                   // parameter CLOCKING_MODE is "common_clock".

  .clkb(~dot_clk_i),               // 1-bit input: Clock signal for port B when parameter CLOCKING_MODE is
                                   // "independent_clock". Unused when parameter CLOCKING_MODE is
                                   // "common_clock".

  .dina(dat_i),                     // WRITE_DATA_WIDTH_A-bit input: Data input for port A write operations.
  .dinb(8'h00),                     // WRITE_DATA_WIDTH_B-bit input: Data input for port B write operations.
  .ena(cs_i),                       // 1-bit input: Memory enable signal for port A. Must be high on clock
                                   // cycles when read or write operations are initiated. Pipelined
                                   // internally.

  .enb(1'b1),                       // 1-bit input: Memory enable signal for port B. Must be high on clock
                                   // cycles when read or write operations are initiated. Pipelined
                                   // internally.

  .injectdbiterra(1'b0), // 1-bit input: Controls double bit error injection on input data when
                                   // ECC enabled (Error injection capability is not available in
                                   // "decode_only" mode).

  .injectdbiterrb(1'b0), // 1-bit input: Controls double bit error injection on input data when
                                   // ECC enabled (Error injection capability is not available in
                                   // "decode_only" mode).

  .injectsbiterra(1'b0), // 1-bit input: Controls single bit error injection on input data when
                                   // ECC enabled (Error injection capability is not available in
                                   // "decode_only" mode).

  .injectsbiterrb(1'b0), // 1-bit input: Controls single bit error injection on input data when
                                   // ECC enabled (Error injection capability is not available in
                                   // "decode_only" mode).

  .regcea(cs_i),                 // 1-bit input: Clock Enable for the last register stage on the output
                                   // data path.

  .regceb(1'b1),                 // 1-bit input: Clock Enable for the last register stage on the output
                                   // data path.

  .rsta(1'b0),                     // 1-bit input: Reset signal for the final port A output register stage.
                                   // Synchronously resets output port douta to the value specified by
                                   // parameter READ_RESET_VALUE_A.

  .rstb(1'b0),                     // 1-bit input: Reset signal for the final port B output register stage.
                                   // Synchronously resets output port doutb to the value specified by
                                   // parameter READ_RESET_VALUE_B.

  .sleep(1'b0),                   // 1-bit input: sleep signal to enable the dynamic power saving feature.
  .wea(we_i),                       // WRITE_DATA_WIDTH_A/BYTE_WRITE_WIDTH_A-bit input: Write enable vector
                                   // for port A input data port dina. 1 bit wide when word-wide writes are
                                   // used. In byte-wide write configurations, each bit controls the
                                   // writing one byte of dina to address addra. For example, to
                                   // synchronously write only bits [15-8] of dina when WRITE_DATA_WIDTH_A
                                   // is 32, wea would be 4'b0010.

  .web(1'b0)                        // WRITE_DATA_WIDTH_B/BYTE_WRITE_WIDTH_B-bit input: Write enable vector
                                   // for port B input data port dinb. 1 bit wide when word-wide writes are
                                   // used. In byte-wide write configurations, each bit controls the
                                   // writing one byte of dinb to address addrb. For example, to
                                   // synchronously write only bits [15-8] of dinb when WRITE_DATA_WIDTH_B
                                   // is 32, web would be 4'b0010.

);

// Char code is already delated two clocks relative to ce
// Assume that characters are always going to be at least four clocks wide.
// Clock #0
always_ff @(posedge dot_clk_i)
  if (ce_i)
    rcc <= char_code_i*maxscanline_i+scanline_i;
// Clock #1
always @(posedge dot_clk_i)
  case(maxScanpix_i[5:3])
  3'b111: rcc0 <= {rcc,3'b0};
  3'b110:	rcc0 <= {rcc,2'b0} + {rcc,1'b0} + rcc;
  3'b101: rcc0 <= {rcc,2'b0} + {rcc,1'b0};
  3'b100: rcc0 <= {rcc,2'b0} + rcc;
  3'b011: rcc0 <= {rcc,2'b0};
  3'b010: rcc0 <= {rcc,1'b0} + rcc;
  3'b001: rcc0 <= {rcc,1'b0};
  3'b000: rcc0 <=  rcc;
  endcase
// Clock #2
always_ff @(posedge dot_clk_i)
  if (ce_i) begin
    rcc1 <= {fontAddress_i[15:3],3'b0}+rcc0;
    bndx <= 5'd0;
  end
  else begin
    if (bndx[2:0] <= maxScanpix_i[5:3]) begin
      bmp[bndx[2:0]] <= memo;
      rcc1 <= rcc1 + 1'd1;
      bndx <= bndx + 1'd1;
    end
  end
always @(posedge dot_clk_i)
  if (ce_i)
 	  bmp_o <= {bmp[7],bmp[6],bmp[5],bmp[4],bmp[3],bmp[2],bmp[1],bmp[0]};

endmodule
