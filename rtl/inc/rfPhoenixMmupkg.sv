// ============================================================================
//        __
//   \\__/ o\    (C) 2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	rfPhoenixMmupkg.sv
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

package rfPhoenixMmupkg;

typedef struct packed
{
	logic m;						// modified indicator
	logic [511:0] data;
} DCacheLine;

typedef struct packed
{
	logic [511:0] data;
} ICacheLine;
/*
typedef struct packed
{
	logic [583:0] data;
} ICacheLine;
*/
typedef struct packed
{
	logic [19:0] at;
	rfPhoenixPkg::Address cta;
	rfPhoenixPkg::Address pmt;
	rfPhoenixPkg::Address nd;
	rfPhoenixPkg::Address start;
} REGION;

typedef struct packed
{
	logic vm;
	logic n;
	logic pm;
	logic [9:0] pad10b;
	logic e;
	logic [1:0] al;
	logic [15:0] pci;
	logic [7:0] pl;
	logic [23:0] key;
	logic [31:0] access_count;
	logic [15:0] acl;
	logic [15:0] share_count;
} PMTE;	// 128 bits

// Page Table Entry
typedef struct packed
{
	logic [18:0] ppn;
	logic [1:0] sw;
	logic m;
	logic a;
	logic g;
	logic c;
	logic [2:0] rwx;
	logic [2:0] lvl;
	logic v;
} PTE;	// 32 bits

typedef PTE PDE;

typedef struct packed
{
	logic [31:0] resv3;
	logic [31:0] adr;
	logic [18:0] vpn;
	logic resv1;
	logic [11:0] asid;
	PTE		pte;
} TLBE;	// 128 bits

// Small Hash Page Table Entry
typedef struct packed
{
	logic [9:0] asid;
	logic g;
	logic [3:0] bc;
	logic pad1b;
	logic [15:0] vpn;
	logic v;
	logic [2:0] lvl;
	logic [2:0] mb;
	logic [2:0] me;
	logic m;
	logic [2:0] rwx;
	logic a;
	logic c;
	logic [15:0] ppn;
} SHPTE;	// 64 bits

// Hash Page Table Entry
typedef struct packed
{
	logic [31:0] vpnhi;
	logic [31:0] ppnhi;
	logic [9:0] asid;
	logic g;
	logic [3:0] bc;
	logic pad1b;
	logic [15:0] vpn;
	logic v;
	logic [2:0] lvl;
	logic [2:0] mb;
	logic [2:0] me;
	logic m;
	logic [2:0] rwx;
	logic a;
	logic c;
	logic [15:0] ppn;
} HPTE;	// 64 bits

typedef struct packed
{
	logic v;
	rfPhoenixPkg::Address adr;
	PDE pde;
} PDCE;

`define PtePerPtg 8
`define PtgSize 2048
`define StripsPerPtg	10

integer PtePerPtg = `PtePerPtg;
integer PtgSize = `PtgSize;

typedef struct packed
{
	HPTE [`PtePerPtg-1:0] ptes;
} PTG;	// 1024 bits

typedef struct packed
{
	SHPTE [`PtePerPtg-1:0] ptes;
} SPTG;	// 512 bits

typedef struct packed
{
	logic v;
	rfPhoenixPkg::Address dadr;
	PTG ptg;
} PTGCE;
parameter PTGC_DEP = 8;

typedef enum logic [6:0] {
	MEMORY_INIT = 7'd0,
	MEMORY_IDLE = 7'd1,
	MEMORY_DISPATCH = 7'd2,
	MEMORY3 = 7'd3,
	MEMORY4 = 7'd4,
	MEMORY5 = 7'd5,
	MEMORY_ACK = 7'd6,
	MEMORY_NACK = 7'd7,
	MEMORY8 = 7'd8,
	MEMORY9 = 7'd9,
	MEMORY10 = 7'd10,
	MEMORY11 = 7'd11,
	MEMORY_ACKHI = 7'd12,
	MEMORY13 = 7'd13,
	DATA_ALIGN = 7'd14,
	MEMORY_KEYCHK1 = 7'd15,
	MEMORY_KEYCHK2 = 7'd16,
	KEYCHK_ERR = 7'd17,
	TLB1 = 7'd21,
	TLB2 = 7'd22,
	TLB3 = 7'd23,
	RGN1 = 7'd25,
	RGN2 = 7'd26,
	RGN3 = 7'd27,
	IFETCH0 = 7'd30,
	IFETCH1 = 7'd31,
	IFETCH2 = 7'd32,
	IFETCH3 = 7'd33,
	IFETCH4 = 7'd34,
	IFETCH5 = 7'd35,
	IFETCH6 = 7'd36,
	IFETCH1a = 7'd37,
	IFETCH1b = 7'd38,
	IFETCH3a = 7'd39,
	DFETCH2 = 7'd42,
	DFETCH5 = 7'd43,
	DFETCH6 = 7'd44,
	DFETCH7 = 7'd45,
	DSTORE1 = 7'd46,
	DSTORE2 = 7'd47,
	DSTORE3 = 7'd48,
	KYLD = 7'd51,
	KYLD2 = 7'd52,
	KYLD3 = 7'd53,
	KYLD4 = 7'd54,
	KYLD5 = 7'd55,
	KYLD6 = 7'd56,
	KYLD7 = 7'd57,
	MEMORY1 = 7'd60,
	MFSEL1 = 7'd61,
	MEMORY_ACTIVATE = 7'd62,
	MEMORY_ACTIVATE_HI = 7'd63,
	IPT_FETCH1 = 7'd64,
	IPT_FETCH2 = 7'd65,
	IPT_FETCH3 = 7'd66,
	IPT_FETCH4 = 7'd67,
	IPT_FETCH5 = 7'd68,
	IPT_RW_PTG2 = 7'd69,
	IPT_RW_PTG3 = 7'd70,
	IPT_RW_PTG4 = 7'd71,
	IPT_RW_PTG5 = 7'd72,
	IPT_RW_PTG6 = 7'd73,
	IPT_WRITE_PTE = 7'd75,
	IPT_IDLE = 7'd76,
	MEMORY5a = 7'd77,
	PT_FETCH1 = 7'd81,
	PT_FETCH2 = 7'd82,
	PT_FETCH3 = 7'd83,
	PT_FETCH4 = 7'd84,
	PT_FETCH5 = 7'd85,
	PT_FETCH6 = 7'd86,
	PT_RW_PTE1 = 7'd92,
	PT_RW_PTE2 = 7'd93,
	PT_RW_PTE3 = 7'd94,
	PT_RW_PTE4 = 7'd95,
	PT_RW_PTE5 = 7'd96,
	PT_RW_PTE6 = 7'd97,
	PT_RW_PTE7 = 7'd98,
	PT_WRITE_PTE = 7'd99,
	PMT_FETCH1 = 7'd101,
	PMT_FETCH2 = 7'd102,
	PMT_FETCH3 = 7'd103,
	PMT_FETCH4 = 7'd104,
	PMT_FETCH5 = 7'd105,
	PT_RW_PDE1 = 7'd108,
	PT_RW_PDE2 = 7'd109,
	PT_RW_PDE3 = 7'd110,
	PT_RW_PDE4 = 7'd111,
	PT_RW_PDE5 = 7'd112,
	PT_RW_PDE6 = 7'd113,
	PT_RW_PDE7 = 7'd114,
	PTG1 = 7'd115,
	PTG2 = 7'd116,
	PTG3 = 7'd117,
	MEMORY_UPD1 = 7'd118,
	MEMORY_UPD2 = 7'd119
} mem_state_t;

parameter IPT_CLOCK1 = 7'd1;
parameter IPT_CLOCK2 = 7'd2;
parameter IPT_CLOCK3 = 7'd3;

endpackage
