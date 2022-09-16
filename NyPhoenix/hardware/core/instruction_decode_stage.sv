//
// Copyright 2011-2015 Jeff Bush
// Modified 2022 Rob Finch for rfPhoenix ISA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

`include "defines.svh"

import defines::*;

//
// Instruction Pipeline - Instruction Decode Stage
// Populate the decoded_instruction_t structure with fields from
// the instruction. The structure contains control fields will be
// used later in the pipeline.
//
// This also handles interrupts. When an interrupt is pending, an existing
// instruction is replaced with one that has the trap flag set. This works
// similar to the way traps from earlier stages in the pipeline are handled
// and is necessary to make sure interrupts are precise (from the software
// perspective, the interrupt appears  to occur on a boundary between two
// instructions, with every prior instruction executing, and every subsequent
// instruction not). We must do it this way because:
// - Instructions can retire out of order
// - There may be pending instructions in the pipeline for the thread that
//   will cause a rollback in a subsequent cycle.
//
// Register port to operand mapping
//                                               store
//       format           op1     op2    mask    value
// +-------------------+-------+-------+-------+-------+
// | R - scalar/scalar |   s1  |   s2  |       |       |
// | R - vector/scalar |   v1  |   s2  |  s1   |       |
// | R - vector/vector |   v1  |   v2  |  s2   |       |
// | I - scalar        |   s1  |  imm  |  n/a  |       |
// | I - vector        |   v1  |  imm  |  s2   |       |
// | M - scalar        |   s1  |  imm  |  n/a  |  s2   |
// | M - block         |   s1  |  imm  |  s2   |  v2   |
// | M - scatter/gather|   v1  |  imm  |  s2   |  v2   |
// | C                 |   s1  |  imm  |       |       |
// | B                 |   s1  |       |       |       |
// +-------------------+-------+-------+-------+-------+
//

module instruction_decode_stage(
  input                         clk,
  input                         reset,

  // From ifetch_data_stage
  input                         ifd_instruction_valid,
  input instruction_t           ifd_instruction,
  input instruction_t						ifd_postfix,
  input                         ifd_inst_injected,
  input scalar_t                ifd_pc,
  input local_thread_idx_t      ifd_thread_idx,
  input                         ifd_alignment_fault,
  input                         ifd_supervisor_fault,
  input                         ifd_page_fault,
  input                         ifd_executable_fault,
  input                         ifd_tlb_miss,

  // From dcache_data_stage
  input local_thread_bitmap_t   dd_load_sync_pending,

  // From l1_l2_interface
  input local_thread_bitmap_t   sq_store_sync_pending,

  // To thread_select_stage
  output decoded_instruction_t  id_instruction,
  output logic                  id_instruction_valid,
  output local_thread_idx_t     id_thread_idx,

  // From io_request_queue
  input local_thread_bitmap_t   ior_pending,

  // From control_registers
  input local_thread_bitmap_t   cr_interrupt_en,
  input local_thread_bitmap_t   cr_interrupt_pending,

  // From on_chip_debugger
  input                         ocd_halt,

  // From writeback_stage
  input                         wb_rollback_en,
  input local_thread_idx_t      wb_rollback_thread_idx);

  localparam T = 1'b1;
  localparam F = 1'b0;
/*
  typedef enum logic[2:0] {
      IMM_ZERO,
      IMM_23_15,  // Masked immediate arithmetic
      IMM_23_10,  // Unmasked immediate arithmetic
      IMM_24_15,  // Masked memory access
      IMM_24_10,  // Unmasked memory access
      IMM_24_5,   // Small branch offset (multiply by four)
      IMM_24_0,   // Large branch offset (multiply by four)
      IMM_EXT_19  // 19 bit extended immediate value
  } imm_loc_t;

  typedef enum logic[1:0] {
      SCLR1_NONE,
      SCLR1_14_10,
      SCLR1_4_0
  } scalar1_loc_t;

  typedef enum logic[2:0] {
      SCLR2_NONE,
      SCLR2_19_15,
      SCLR2_14_10,
      SCLR2_9_5
  } scalar2_loc_t;

  struct packed {
      logic illegal;
      logic dest_vector;
      logic has_dest;
      imm_loc_t imm_loc;
      scalar1_loc_t scalar1_loc;
      scalar2_loc_t scalar2_loc;
      logic has_vector1;
      logic has_vector2;
      logic vector_sel2_9_5;    // Else is src2. Only for stores.
      logic op1_vector;
      op2_src_t op2_src;
      mask_src_t mask_src;
      logic store_value_vector;
      logic call;
  } dlut_out;
*/
  decoded_instruction_t decoded_instr_nxt;
  logic postfix;
  logic nop;
  logic cjb;
  logic br;
  logic fmt_r;
  logic fmt_i;
  logic fmt_m;
  logic getlane;
  logic compare;
  alu_op_t alu_op;
  memory_op_t memory_access_type;
  register_idx_t scalar_sel2;
  logic has_trap;
  logic syscall;
  logic breakpoint;
  logic raise_interrupt;
  local_thread_bitmap_t masked_interrupt_flags;
  logic unary_arith;

  // I originally tried to structure the instruction set so that this could
  // determine the format of the instruction from the first 7 bits. Those
  // index into this ROM table that returns the decoded information. This
  // has become less true as the instruction set has evolved. Also, synthesis
  // tools just turn this into random logic. Should revisit this at some point.
/*    
  always_comb
  begin
      unique casez (ifd_instruction[31:25])
          // Format R (register arithmetic)
          7'b110_000_?: dlut_out = {F, F, T, IMM_ZERO, SCLR1_4_0, SCLR2_19_15,   F, F, F, F, OP2_SRC_SCALAR2, MASK_SRC_ALL_ONES, F, F};
          7'b110_001_?: dlut_out = {F, T, T, IMM_ZERO, SCLR1_4_0, SCLR2_19_15,   T, F, F, T, OP2_SRC_SCALAR2, MASK_SRC_ALL_ONES, F, F};
          7'b110_010_?: dlut_out = {F, T, T, IMM_ZERO, SCLR1_14_10, SCLR2_19_15, T, F, F, T, OP2_SRC_SCALAR2, MASK_SRC_SCALAR1, F, F};
          7'b110_100_?: dlut_out = {F, T, T, IMM_ZERO, SCLR1_14_10, SCLR2_NONE,  T, T, F, T, OP2_SRC_VECTOR2, MASK_SRC_ALL_ONES, F, F};
          7'b110_101_?: dlut_out = {F, T, T, IMM_ZERO, SCLR1_4_0, SCLR2_14_10,   T, T, F, T, OP2_SRC_VECTOR2, MASK_SRC_SCALAR2, F, F};

          // Format I (immediate arithmetic)
          7'b0_00_????: dlut_out = {F, F, T, IMM_23_10, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b0_01_????: dlut_out = {F, T, T, IMM_23_10, SCLR1_4_0, SCLR2_NONE,   T, F, F, T, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b0_10_????: dlut_out = {F, F, T, IMM_EXT_19, SCLR1_4_0, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b0_11_????: dlut_out = {F, T, T, IMM_23_15, SCLR1_4_0, SCLR2_14_10,  T, F, F, T, OP2_SRC_IMMEDIATE, MASK_SRC_SCALAR2, F, F};

          // Format M (memory)
          // Store
          7'b10_0_0000: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0001: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0010: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     T, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0011: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0100: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0101: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0110: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_9_5,     F, F, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_0_0111: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    F, T, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, T, F};
          7'b10_0_1000: dlut_out = {F, F, F, IMM_24_15, SCLR1_4_0, SCLR2_14_10, F, T, T, F, OP2_SRC_IMMEDIATE, MASK_SRC_SCALAR2, T, F};
          7'b10_0_1101: dlut_out = {F, F, F, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, T, T, T, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, T, F};
          7'b10_0_1110: dlut_out = {F, F, F, IMM_24_15, SCLR1_4_0, SCLR2_14_10, T, T, T, T, OP2_SRC_IMMEDIATE, MASK_SRC_SCALAR2, T, F};

          // Load
          7'b10_1_0000: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0001: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0010: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0011: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0100: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0101: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0110: dlut_out = {F, F, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_0111: dlut_out = {F, T, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_1000: dlut_out = {F, T, T, IMM_24_15, SCLR1_4_0, SCLR2_14_10, T, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_SCALAR2, F, F};
          7'b10_1_1101: dlut_out = {F, T, T, IMM_24_10, SCLR1_4_0, SCLR2_NONE,    T, T, F, T, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b10_1_1110: dlut_out = {F, T, T, IMM_24_15, SCLR1_4_0, SCLR2_14_10, T, T, F, T, OP2_SRC_IMMEDIATE, MASK_SRC_SCALAR2, F, F};

          // Format C (cache control)
          7'b1110_000: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_9_5,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_001: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_010: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_011: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_100: dlut_out = {F, F, F,  IMM_24_15, SCLR1_NONE, SCLR2_NONE, F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_101: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_110: dlut_out = {F, F, F,  IMM_24_15, SCLR1_NONE, SCLR2_NONE, F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1110_111: dlut_out = {F, F, F,  IMM_24_15, SCLR1_4_0, SCLR2_9_5,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};

          // Format B (branch)
          7'b1111_000: dlut_out = {F, F, F, IMM_24_5, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1111_001: dlut_out = {F, F, F, IMM_24_5, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1111_010: dlut_out = {F, F, F, IMM_24_5, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1111_011: dlut_out = {F, F, F, IMM_24_0, SCLR1_NONE, SCLR2_NONE,  F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
          7'b1111_100: dlut_out = {F, F, T, IMM_24_0, SCLR1_NONE, SCLR2_NONE,  F, F, F, F, OP2_SRC_SCALAR2, MASK_SRC_ALL_ONES, F, T};
          7'b1111_110: dlut_out = {F, F, T, IMM_24_5, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_SCALAR2, MASK_SRC_ALL_ONES, F, T};
          7'b1111_111: dlut_out = {F, F, T, IMM_24_5, SCLR1_4_0, SCLR2_NONE,   F, F, F, F, OP2_SRC_SCALAR2, MASK_SRC_ALL_ONES, F, F};

          // Invalid instruction format
          default: dlut_out = {T, F, F, IMM_ZERO, SCLR1_NONE, SCLR2_NONE, F, F, F, F, OP2_SRC_IMMEDIATE, MASK_SRC_ALL_ONES, F, F};
      endcase
  end
*/
  //assign fmt_r = ifd_instruction[31:29] == 3'b110;    // register arithmetic
  assign fmt_r = ifd_instruction.any.opcode==OP_R2;
  //assign fmt_i = ifd_instruction[31] == 1'b0;    // immediate arithmetic
  assign fmt_i = ifd_instruction.any.opcode==OP_ADDI ||
  							 ifd_instruction.any.opcode==OP_SUBFI ||
  							 ifd_instruction.any.opcode==OP_ANDI ||
  							 ifd_instruction.any.opcode==OP_ORI ||
  							 ifd_instruction.any.opcode==OP_XORI ||
  							 ifd_instruction.any.opcode==OP_CMPI ||
  							 ifd_instruction.any.opcode==OP_CMP_EQI ||
  							 ifd_instruction.any.opcode==OP_CMP_NEI ||
  							 ifd_instruction.any.opcode==OP_CMP_LTI ||
  							 ifd_instruction.any.opcode==OP_CMP_GEI ||
  							 ifd_instruction.any.opcode==OP_CMP_LEI ||
  							 ifd_instruction.any.opcode==OP_CMP_GTI ||
  							 ifd_instruction.any.opcode==OP_CMP_LTUI ||
  							 ifd_instruction.any.opcode==OP_CMP_GEUI ||
  							 ifd_instruction.any.opcode==OP_CMP_LEUI ||
  							 ifd_instruction.any.opcode==OP_CMP_GTUI ||
  							 ifd_instruction.any.opcode==OP_FCMP_EQI ||
  							 ifd_instruction.any.opcode==OP_FCMP_NEI ||
  							 ifd_instruction.any.opcode==OP_FCMP_LTI ||
  							 ifd_instruction.any.opcode==OP_FCMP_GEI ||
  							 ifd_instruction.any.opcode==OP_FCMP_LEI ||
  							 ifd_instruction.any.opcode==OP_FCMP_GTI ||
  							 ifd_instruction.any.opcode==OP_LDB ||
  							 ifd_instruction.any.opcode==OP_LDBU ||
  							 ifd_instruction.any.opcode==OP_LDW ||
  							 ifd_instruction.any.opcode==OP_LDWU ||
  							 ifd_instruction.any.opcode==OP_LDT ||
  							 ifd_instruction.any.opcode==OP_STB ||
  							 ifd_instruction.any.opcode==OP_STW ||
  							 ifd_instruction.any.opcode==OP_STT
  							 ;
  assign fmt_m = ifd_instruction.any.opcode==OP_LDB ||
  							 ifd_instruction.any.opcode==OP_LDBU ||
  							 ifd_instruction.any.opcode==OP_LDW ||
  							 ifd_instruction.any.opcode==OP_LDWU ||
  							 ifd_instruction.any.opcode==OP_LDT ||
  							 ifd_instruction.any.opcode==OP_STB ||
  							 ifd_instruction.any.opcode==OP_STW ||
  							 ifd_instruction.any.opcode==OP_STT
  							 ;
  assign getlane = (fmt_r || fmt_i) && alu_op == OP_VEX;

  assign syscall = 1'b0;//fmt_i && 6'(ifd_instruction[28:24]) == OP_SYSCALL;
  assign breakpoint = ifd_instruction.any.opcode == OP_BRK;
  assign postfix = ifd_instruction.any.opcode == OP_PFX;
  assign nop = ifd_instruction.any.opcode == OP_NOP;
  assign has_trap = (ifd_instruction_valid
//      && (dlut_out.illegal || syscall || breakpoint || raise_interrupt))
      && (syscall || breakpoint || raise_interrupt))
      || ifd_alignment_fault || ifd_tlb_miss
      || ifd_supervisor_fault
      || ifd_page_fault || ifd_executable_fault;

  // Check for TLB miss first, since permission bits are not valid if there
  // is a TLB miss. The order of the remaining faults should match that in
  // dcache_data_stage for consistency.
  always_comb
  begin
    if (raise_interrupt)
      decoded_instr_nxt.trap_cause = {2'b00, TT_INTERRUPT};
    else if (ifd_tlb_miss)
      decoded_instr_nxt.trap_cause = {2'b00, TT_TLB_MISS};
    else if (ifd_page_fault)
      decoded_instr_nxt.trap_cause = {2'b00, TT_PAGE_FAULT};
    else if (ifd_supervisor_fault)
      decoded_instr_nxt.trap_cause = {2'b00, TT_SUPERVISOR_ACCESS};
    else if (ifd_alignment_fault)
      decoded_instr_nxt.trap_cause = {2'b00, TT_UNALIGNED_ACCESS};
    else if (ifd_executable_fault)
      decoded_instr_nxt.trap_cause = {2'b00, TT_NOT_EXECUTABLE};
    else if (dlut_out.illegal)
      decoded_instr_nxt.trap_cause = {2'b00, TT_ILLEGAL_INSTRUCTION};
    else if (syscall)
      decoded_instr_nxt.trap_cause = {2'b00, TT_SYSCALL};
    else if (breakpoint)
      decoded_instr_nxt.trap_cause = {2'b00, TT_BREAKPOINT};
    else
      decoded_instr_nxt.trap_cause = {2'b00, TT_RESET};
  end

  assign decoded_instr_nxt.injected = ifd_inst_injected;

  // Subtle: Certain instructions need to be issued twice, including I/O
  // requests and synchronized memory accesses. The first queues the
  // transaction and the second collects the result. Because the first
  // instruction updates internal state, bad things would happen if an
  // interrupt were dispatched between them. To avoid this, don't dispatch
  // an interrupt if the first instruction has been issued (indicated by
  // dd_load_sync_pending, ior_pending, or sq_store_sync_pending).
  assign masked_interrupt_flags = cr_interrupt_pending & cr_interrupt_en
      & ~ior_pending & ~dd_load_sync_pending & ~sq_store_sync_pending;
  assign raise_interrupt = masked_interrupt_flags[ifd_thread_idx] && !ocd_halt;
  assign decoded_instr_nxt.has_trap = has_trap;

  assign unary_arith = fmt_r && ifd_instruction.r2.func==R1;
  assign br = ifd_instruction.any.opcode==OP_Bcc || ifd_instruction.any.opcode==OP_FBcc;
	assign cjb = ifd_instruction.any.opcode==OP_CALL || ifd_instruction.any.opcode==OP_BSR;
	assign decoded_instr_nxt.cjb = ifd_instruction.any.opcode==OP_CALL || ifd_instruction.any.opcode==OP_BSR;

	always_comb
	begin
		decoded_instr_nxt.loadr = 1'b0;
		decoded_instr_nxt.loadn = 1'b0;
		decoded_instr_nxt.storer = 1'b0;
		decoded_instr_nxt.storen = 1'b0;
		case(ifd_instruction.any.opcode)
		OP_R2:
			case(ifd_instruction.r2.func)
			OP_LDBX,OP_LDBUX,OP_LDWX,OP_LDWUX,OP_LDTX:	decoded_instr_nxt.loadn = 1'b1;
			OP_STBX,OP_STWX,OP_STTX:	decoded_instr_nxt.storen = 1'b1;
			default:	;
			endcase
		OP_LDB,OP_LDBU,OP_LDDW,OP_LDWU,OP_LDT:	decoded_instr_nxt.loadr = 1'b1;
		OP_STB,OP_STW,OP_STT:	decoded_instr_nxt.storer = 1'b1;
		default:	;
		endcase
	end

	assign decoded_instr_nxt.load = decoded_instr_nxt.loadr|decoded_instr_nxt.loadn; 
	assign decoded_instr_nxt.store = decoded_instr_nxt.storer|decoded_instr_nxt.storen;
	assign decoded_instr_nxt.mem = decoded_instr_nxt.load|decoded_instr_nxt.store; 
    /*
     (alu_op == OP_CLZ
        || alu_op == OP_CTZ
        || alu_op == OP_MOVE
        || alu_op == OP_FTOI
        || alu_op == OP_RECIPROCAL
        || alu_op == OP_SEXT8
        || alu_op == OP_SEXT16
        || alu_op == OP_ITOF)
        && dlut_out.mask_src != MASK_SRC_SCALAR1;
    */
	assign decoded_instr_nxt.has_scalar1 = !decoded_instr_nxt.cjb && !postfix && !nop && !has_trap && !ifd_instruction.r2.Ra.vec;
	assign decoded_instr_nxt.has_scalar2 = !has_trap && !ifd_instruction.r2.Rb.vec && (
								(ifd_instruction.any.opcode==OP_R2 && ifd_instruction.r2.func!=OP_R1) ||
								ifd_instruction.any.opcode==OP_FMA ||
								ifd_instruction.any.opcode==OP_FMS ||
								ifd_instruction.any.opcode==OP_FNMA ||
								ifd_instruction.any.opcode==OP_FNMS
								)
								;
	assign decoded_instr_nxt.has_scalar3 = !has_trap && !ifd_instruction.f3.Rc.vec && (
								ifd_instruction.any.opcode==OP_FMA ||
								ifd_instruction.any.opcode==OP_FMS ||
								ifd_instruction.any.opcode==OP_FNMA ||
								ifd_instruction.any.opcode==OP_FNMS ||
								ifd_instruction.any.opcode==OP_Bcc ||
								ifd_instruction.any.opcode==OP_FBcc ||
								decoded_instr_nxt.store
								)
								;
//    assign decoded_instr_nxt.has_scalar1 = dlut_out.scalar1_loc != SCLR1_NONE && !nop
//        && !has_trap && !unary_arith;
  always_comb
  	decoded_instr_nxt.scalar_sel1 = ifd_instruction.r2.Ra.num;
  always_comb
  	decoded_instr_nxt.scalar_sel2 = ifd_instruction.r2.Rb.num;
  always_comb
  	decoded_instr_nxt.scalar_sel3 = ifd_instruction.f3.Rc.num;
    /*
    begin
        unique case (dlut_out.scalar1_loc)
            SCLR1_14_10: decoded_instr_nxt.scalar_sel1 = ifd_instruction[14:10];
            default: decoded_instr_nxt.scalar_sel1 = ifd_instruction[4:0]; //  src1
        endcase
    end
		*/
    //assign decoded_instr_nxt.has_scalar2 = dlut_out.scalar2_loc != SCLR2_NONE && !nop
     //   && !has_trap;

    // XXX: assigning this directly to decoded_instr_nxt.scalar_sel2 causes Verilator issues when
    // other blocks read it. Added another signal to work around this.
    /*
    always_comb
    begin
        unique case (dlut_out.scalar2_loc)
            SCLR2_14_10: scalar_sel2 = ifd_instruction[14:10];
            SCLR2_19_15: scalar_sel2 = ifd_instruction[19:15];
            SCLR2_9_5: scalar_sel2 = ifd_instruction[9:5];
            default: scalar_sel2 = 0;
        endcase
    end
		*/
    //assign decoded_instr_nxt.scalar_sel2 = scalar_sel2;
  assign decoded_instr_nxt.has_vector1 = ifd_instruction.r2.Ra.vec && !postfix && !nop && !has_trap;
  assign decoded_instr_nxt.vector_sel1 = ifd_instruction.r2.Ra.num;
  assign decoded_instr_nxt.has_vector2 = ifd_instruction.r2.Rb.vec && !postfix && !nop && !has_trap;
  assign decoded_instr_nxt.vector_sel2 = ifd_instruction.r2.Rb.num;
  assign decoded_instr_nxt.has_vector3 = ((decoded_instr_nxt.br|decoded_instr_nxt.store) ? ifd_instruction.r2.Rt.vec : ifd_instruction.r2.Rc.vec) && !postfix && !nop && !has_trap;
  assign decoded_instr_nxt.vector_sel3 = (decoded_instr_nxt.br|decoded_instr_nxt.store) ? ifd_instruction.r2.Rt.num : ifd_instruction.r2.Rc.num;
  assign decoded_instr_nxt.has_mask = !cjb && !br && !postfix && !nop && !has_trap;
  assign decoded_instr_nxt.mask_sel = {ifd_instruction.r2.m,ifd_instruction.r2.Rm};

	assign decoded_instr_nxt.has_dest = !postfix && !(decoded_instr_nxt.br|decoded_instr_nxt.store|nop) && !has_trap;

//		deco.Rtsrc = 	deco.hasRt & (deco.br|deco.store);

//    assign decoded_instr_nxt.has_dest = dlut_out.has_dest && !nop && !has_trap;

    assign decoded_instr_nxt.dest_vector = ifd_instruction.r2.Rt.vec && !compare
        && !getlane;
    assign decoded_instr_nxt.call = ifd_instruction.any.opcode==OP_CALL;
    assign decoded_instr_nxt.dest_reg = ifd_instruction.any.opcode==OP_CALL ?
    																		(ifd_instruction.call.Rt==2'b00 ? 6'd0 : {4'b1100,ifd_instruction.call.Rt}) :
    																		ifd_instruction.r2.Rt.num;
    /*
    always_comb
    begin
        if (fmt_i)
            alu_op = alu_op_t'({1'b0, ifd_instruction[28:24]});
        else if (dlut_out.call)
            alu_op = OP_MOVE;    // Treat a call as move ra, pc
        else
            alu_op = alu_op_t'(ifd_instruction[25:20]); // Format R
    end
		*/
    //assign decoded_instr_nxt.alu_op = alu_op;
    assign decoded_instr_nxt.insn = ifd_instruction;
//    assign decoded_instr_nxt.mask_src = dlut_out.mask_src;
//    assign decoded_instr_nxt.store_value_vector = dlut_out.store_value_vector;

  // Decode operand source ports, checking specifically for PC operands
  always_comb
  begin
    if (ifd_instruction.r2.Ra.vec)
      decoded_instr_nxt.op1_src = OP1_SRC_VECTOR1;
    else
      decoded_instr_nxt.op1_src = OP1_SRC_SCALAR1;
  end

  always_comb
  begin
  	if (fmt_i)
  		decoded_instr_nxt.op2_src = OP2_SRC_IMMEDIATE;
    else if (ifd_instruction.r2.Rb.vec)
      decoded_instr_nxt.op2_src = OP2_SRC_VECTOR2;
    else
      decoded_instr_nxt.op2_src = OP2_SRC_SCALAR2;
  end

  always_comb
  begin
    if (ifd_instruction.f3.Rc.vec)
      decoded_instr_nxt.op3_src = OP1_SRC_VECTOR3;
    else
      decoded_instr_nxt.op3_src = OP1_SRC_SCALAR3;
  end

//  assign decoded_instr_nxt.op2_src = dlut_out.op2_src;

	// Looks like a lot but there is really only a couple of fields.
	decoded_instr_nxt.immediate_value = 'd0;
	case(ifb.insn.any.opcode)
	R2:
		case(ifb.insn.r2.func)
		R1:
			case(ifb.insn.r1.func1)
			PEEKQ,POPQ,STATQ,RESETQ:	deco.imm =decoded_instr_nxt.immediate_value = ifd_instruction.any.r2.Ra[3:0];
			default:	decoded_instr_nxt.immediate_value = 'd0;
			endcase
		PUSHQ:	deco.imm = deco.Ra[3:0];
		default:	decoded_instr_nxt.immediate_value = 'd0;
		endcase
	ADDI,SUBFI,ANDI,ORI,XORI:
		decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ri.imm[15]}},ifd_instruction.ri.imm};
	CMPI,CMP_EQI,CMP_NEI,CMP_LTI,CMP_GEI,CMP_LEI,CMP_GTI,
	CMP_LTUI,CMP_GEUI,CMP_LEUI,CMP_GTUI:
		decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ri.imm[15]}},ifd_instruction.ri.imm};
	FCMP_EQI,FCMP_NEI,FCMP_LTI,FCMP_GEI,FCMP_LEI,FCMP_GTI:
		decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ri.imm[15]}},ifd_instruction.ri.imm};
	CALL:	decoded_instr_nxt.immediate_value = ifd_instruction.call.target;
	BSR:	decoded_instr_nxt.immediate_value = ifd_instruction.call.target;
	Bcc,FBcc:	decoded_instr_nxt.immediate_value = {{16{ifd_instruction.br.disp[15]}},ifd_instruction.br.disp};
	RET:	decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ri.imm[15]}},ifd_instruction.ri.imm};
	LDB,LDBU,LDW,LDWU,LDT,
	STB,STW,STT:
		decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ls.disp[15]}},ifd_instruction.ls.disp};
	CSR:	decoded_instr_nxt.immediate_value = {{16{ifd_instruction.ri.imm[15]}},ifd_instruction.ri.imm};
	default:	decoded_instr_nxt.immediate_value = 'd0;
	endcase	
	if (ifd_postfix.any.opcode==OP_PFX)
		decoded_instr_nxt.immediate_value[31:16] = ifd_postfix.pfx.imm;
/*
    always_comb
    begin
        unique case (dlut_out.imm_loc)
            IMM_EXT_19: decoded_instr_nxt.immediate_value = { ifd_instruction[23:10], ifd_instruction[4:0], 13'd0 };
            IMM_23_15: decoded_instr_nxt.immediate_value = scalar_t'($signed(ifd_instruction[23:15]));
            IMM_23_10: decoded_instr_nxt.immediate_value = scalar_t'($signed(ifd_instruction[23:10]));
            IMM_24_15: decoded_instr_nxt.immediate_value = scalar_t'($signed(ifd_instruction[24:15]));
            IMM_24_10: decoded_instr_nxt.immediate_value = scalar_t'($signed(ifd_instruction[24:10]));

            // Branch offsets are multiplied by four
            IMM_24_5: decoded_instr_nxt.immediate_value = scalar_t'($signed({ifd_instruction[24:5], 2'b00}));
            IMM_24_0: decoded_instr_nxt.immediate_value = scalar_t'($signed({ifd_instruction[24:0], 2'b00}));
            default: decoded_instr_nxt.immediate_value = 0;
        endcase
    end
*/
  assign decoded_instr_nxt.branch_type = branch_type_t'(ifd_instruction[27:25]);
  assign decoded_instr_nxt.branch = (ifd_instruction.any.opcode==OP_Bcc || ifd_instruction.any.opcode==OP_FBcc) && !has_trap;
  assign decoded_instr_nxt.pc = ifd_pc;

  always_comb
  begin
    if (has_trap)
      decoded_instr_nxt.pipeline_sel = PIPE_INT_ARITH;
    else
    begin
    	case(ifd_instruction.any.opcode)
    	OP_FMA,OP_FMS,OP_FNMA,OP_FNMS,OP_MULI:
    		decoded_instr_nxt.pipeline_sel = PIPE_FLOAT_ARITH;
    	OP_R2:
    		case (ifd_instruction.r2.func)
    		OP_MUL,
    		OP_FADD,OP_FSUB,OP_FMUL,OP_FTOI,OP_ITOF:
    			decoded_instr_nxt.pipeline_sel = PIPE_FLOAT_ARITH;
      	OP_LDBX,OP_LDBUX,OP_LDWX,OP_LDWUX,OP_LDTX,
      	OP_STBX,OP_STWX,OP_STTX:
      		decoded_instr_nxt.pipeline_sel = PIPE_MEM;
    		default:	decoded_instr_nxt.pipeline_sel = PIPE_INT_ARITH;
    		endcase
    	OP_LDB,OP_LDBU,OP_LDW,OP_LDWU,OP_LDT,
    	OP_STB,OP_STW,OP_STT:
    		decoded_instr_nxt.pipeline_sel = PIPE_MEM;
    	default:	decoded_instr_nxt.pipeline_sel = PIPE_INT_ARITH;
    	endcase
    end
  end
/*
typedef enum logic[3:0] {
    MEM_B                   = 4'b0000,  // Byte (8 bit)
    MEM_BX                  = 4'b0001,  // Byte, sign extended
    MEM_S                   = 4'b0010,  // Short (16 bit)
    MEM_SX                  = 4'b0011,  // Short, sign extended
    MEM_L                   = 4'b0100,  // Long (32 bit)
    MEM_SYNC                = 4'b0101,  // Synchronized
    MEM_CONTROL_REG         = 4'b0110,  // Control register
    MEM_BLOCK               = 4'b0111,  // Vector block
    MEM_BLOCK_M             = 4'b1000,
    MEM_SCGATH              = 4'b1101,  // Vector scatter/gather
    MEM_SCGATH_M            = 4'b1110
} memory_op_t;
*/
	always_comb
		case(ifd_instruction.any.opcode)
  	OP_R2:
  		case (ifd_instruction.r2.func)
  		OP_LDBX:	memory_acces_type = MEM_BX;
  		OP_LDBUX,OP_STBX:	memory_acces_type = MEM_B;
  		OP_LDWX:	memory_acces_type = MEM_SX;
  		OP_LDWUX,OP_STWX:	memory_acces_type = MEM_S;
  		OP_LDTX,OP_STTX:	
  			if (ifd_instruction.r2.Rb.vec & ifd_instruction.r2.Rt.vec)
  				memory_acces_type = MEM_SCGATH;
  			else if (ifd_instruction.r2.Rt.vec)
  				memory_acces_type = MEM_BLOCK;
  			else
  				memory_acces_type = MEM_L;
  		default:	memory_acces_type = MEM_L;
  		endcase
		OP_LDB:	memory_acces_type = MEM_BX;
		OP_LDBU,OP_STB:	memory_acces_type = MEM_B;
		OP_LDW:	memory_acces_type = MEM_SX;
		OP_LDWU,OP_STW:	memory_acces_type = MEM_S;
		OP_LDT,OP_STT:	memory_acces_type = MEM_L;
  	default:	memory_acces_type = MEM_L;
		endcase


//    assign memory_access_type = memory_op_t'(ifd_instruction[28:25]);
  assign decoded_instr_nxt.memory_access_type = memory_access_type;
  assign decoded_instr_nxt.memory_access = decoded_instr_nxt.mem
      && !has_trap;
//    assign decoded_instr_nxt.cache_control = ifd_instruction[31:28] == 4'b1110
//         && !has_trap;
//    assign decoded_instr_nxt.cache_control_op = cache_op_t'(ifd_instruction[27:25]);

  always_comb
  begin
      if (ifd_instruction[31:30] == 2'b10
          && (memory_access_type == MEM_SCGATH
          || memory_access_type == MEM_SCGATH_M))
      begin
          // Scatter/Gather access
          decoded_instr_nxt.last_subcycle = subcycle_t'(NUM_VECTOR_LANES - 1);
      end
      else
          decoded_instr_nxt.last_subcycle = 0;
  end

  assign decoded_instr_nxt.creg_index = control_register_t'(ifd_instruction[4:0]);

	always_comb
	begin
		compare = 1'b0;
		case(ifd_instruction.any.opcode)
  	OP_R2:
  		case (ifd_instruction.r2.func)
  		OP_CMP,
  		OP_CMP_EQ,
  		OP_CMP_NE,
  		OP_CMP_LT,
  		OP_CMP_GE,
  		OP_CMP_LE,
  		OP_CMP_GT,
  		OP_CMP_LTU,
  		OP_CMP_GEU,
  		OP_CMP_LEU,
  		OP_CMP_GTU,
  		OP_FCMP_EQ,
  		OP_FCMP_NE,
  		OP_FCMP_LT,
  		OP_FCMP_GE,
  		OP_FCMP_LE,
  		OP_FCMP_GT:	compare = 1'b1;
  		default:	compare = 1'b0;
  		endcase
  	OP_CMPI,
		OP_CMP_EQI,
		OP_CMP_NEI,
		OP_CMP_LTI,
		OP_CMP_GEI,
		OP_CMP_LEI,
		OP_CMP_GTI,
		OP_CMP_LTUI,
		OP_CMP_GEUI,
		OP_CMP_LEUI,
		OP_CMP_GTUI,
		OP_FCMP_EQI,
		OP_FCMP_NEI,
		OP_FCMP_LTI,
		OP_FCMP_GEI,
		OP_FCMP_LEI,
		OP_FCMP_GTI:	compare = 1'b1;
 		default:	compare = 1'b0;
  	endcase
  end

  assign decoded_instr_nxt.compare = compare;

  always_ff @(posedge clk)
  begin
    id_instruction <= decoded_instr_nxt;
    id_thread_idx <= ifd_thread_idx;
  end

  always_ff @(posedge clk, posedge reset)
  begin
    if (reset)
      id_instruction_valid <= '0;
    else
    begin
      // Piggyback ifetch faults and TLB misses inside instructions, marking
      // the instruction valid if these conditions occur
      id_instruction_valid <= (ifd_instruction_valid || has_trap)
        && (!wb_rollback_en || wb_rollback_thread_idx != ifd_thread_idx);
    end
  end
endmodule
