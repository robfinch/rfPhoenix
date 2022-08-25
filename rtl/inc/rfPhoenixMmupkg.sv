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

typedef struct packed
{
	rfPhoenixPkg::Address adr;
	rfPhoenixPkg::Address pmtadr;
	// PMT info
	// 256 bits follow
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
	
	logic [31:0] vpnphi;
	logic [31:0] ppnhi;
	logic [9:0] asid;
	logic g;
	logic [3:0] bc;
	logic pad1b;
	logic [15:0] vpn;
//	logic [9:0] pad10a;	
	logic v;
	logic [2:0] lvl;
	logic [2:0] mb;
	logic [2:0] me;
	logic m;
	logic [2:0] rwx;
	logic a;
	logic c;
	logic [15:0] ppn;
} TLBE;	// 288 bits

// Small Page Table Entry
typedef struct packed
{
	logic v;
	logic [2:0] lvl;
	logic [2:0] mb;
	logic [2:0] me;
	logic m;
	logic [2:0] rwx;
	logic c;
	logic [15:0] ppn;
} SPTE;	// 32 bits

// Page Table Entry
typedef struct packed
{
	logic [31:0] ppnhi;
	logic v;
	logic [2:0] lvl;
	logic [2:0] mb;
	logic [2:0] me;
	logic m;
	logic [2:0] rwx;
	logic c;
	logic [15:0] ppn;
} PTE;	// 32 bits

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

// Small Page Table Entry
typedef struct packed
{
	logic v;
	logic [2:0] lvl;
	logic [9:0] pad10a;
	logic c;
	logic [15:0] ppn;
} SPDE;	// 32 bits + address

typedef struct packed
{
	logic [31:0] ppnhi;
	logic v;
	logic [2:0] lvl;
	logic [9:0] pad10a;
	logic c;
	logic [15:0] ppn;
} PDE;	// 64 bits + address

typedef struct packed
{
	logic v;
	rfPhoenixPkg::Address adr;
	SPDE pde;
} SPDCE;

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

parameter MEMORY_INIT = 7'd0;
parameter MEMORY_IDLE = 7'd1;
parameter MEMORY_DISPATCH = 7'd2;
parameter MEMORY3 = 7'd3;
parameter MEMORY4 = 7'd4;
parameter MEMORY5 = 7'd5;
parameter MEMORY_ACKLO = 7'd6;
parameter MEMORY_NACKLO = 7'd7;
parameter MEMORY8 = 7'd8;
parameter MEMORY9 = 7'd9;
parameter MEMORY10 = 7'd10;
parameter MEMORY11 = 7'd11;
parameter MEMORY_ACKHI = 7'd12;
parameter MEMORY13 = 7'd13;
parameter DATA_ALIGN = 7'd14;
parameter MEMORY_KEYCHK1 = 7'd15;
parameter MEMORY_KEYCHK2 = 7'd16;
parameter KEYCHK_ERR = 7'd17;
parameter TLB1 = 7'd21;
parameter TLB2 = 7'd22;
parameter TLB3 = 7'd23;
parameter RGN1 = 7'd25;
parameter RGN2 = 7'd26;
parameter RGN3 = 7'd27;
parameter IFETCH0 = 7'd30;
parameter IFETCH1 = 7'd31;
parameter IFETCH2 = 7'd32;
parameter IFETCH3 = 7'd33;
parameter IFETCH4 = 7'd34;
parameter IFETCH5 = 7'd35;
parameter IFETCH6 = 7'd36;
parameter IFETCH1a = 7'd37;
parameter IFETCH1b = 7'd38;
parameter IFETCH3a = 7'd39;
parameter DFETCH2 = 7'd42;
parameter DFETCH3 = 7'd43;
parameter DFETCH4 = 7'd44;
parameter DFETCH5 = 7'd45;
parameter DFETCH6 = 7'd46;
parameter DFETCH7 = 7'd47;
parameter DFETCH8 = 7'd48;
parameter DFETCH9 = 7'd49;
parameter KYLD = 7'd51;
parameter KYLD2 = 7'd52;
parameter KYLD3 = 7'd53;
parameter KYLD4 = 7'd54;
parameter KYLD5 = 7'd55;
parameter KYLD6 = 7'd56;
parameter KYLD7 = 7'd57;
parameter MEMORY1 = 7'd60;
parameter MFSEL1 = 7'd61;
parameter MEMORY_ACTIVATE_LO = 7'd62;
parameter MEMORY_ACTIVATE_HI = 7'd63;
parameter IPT_FETCH1 = 7'd64;
parameter IPT_FETCH2 = 7'd65;
parameter IPT_FETCH3 = 7'd66;
parameter IPT_FETCH4 = 7'd67;
parameter IPT_FETCH5 = 7'd68;
parameter IPT_RW_PTG2 = 7'd69;
parameter IPT_RW_PTG3 = 7'd70;
parameter IPT_RW_PTG4 = 7'd71;
parameter IPT_RW_PTG5 = 7'd72;
parameter IPT_RW_PTG6 = 7'd73;
parameter IPT_WRITE_PTE = 7'd75;
parameter IPT_IDLE = 7'd76;
parameter MEMORY5a = 7'd77;
parameter PT_FETCH1 = 7'd81;
parameter PT_FETCH2 = 7'd82;
parameter PT_FETCH3 = 7'd83;
parameter PT_FETCH4 = 7'd84;
parameter PT_FETCH5 = 7'd85;
parameter PT_FETCH6 = 7'd86;
parameter PT_RW_PTE1 = 7'd92;
parameter PT_RW_PTE2 = 7'd93;
parameter PT_RW_PTE3 = 7'd94;
parameter PT_RW_PTE4 = 7'd95;
parameter PT_RW_PTE5 = 7'd96;
parameter PT_RW_PTE6 = 7'd97;
parameter PT_RW_PTE7 = 7'd98;
parameter PT_WRITE_PTE = 7'd99;
parameter PMT_FETCH1 = 7'd101;
parameter PMT_FETCH2 = 7'd102;
parameter PMT_FETCH3 = 7'd103;
parameter PMT_FETCH4 = 7'd104;
parameter PMT_FETCH5 = 7'd105;
parameter PT_RW_PDE1 = 7'd108;
parameter PT_RW_PDE2 = 7'd109;
parameter PT_RW_PDE3 = 7'd110;
parameter PT_RW_PDE4 = 7'd111;
parameter PT_RW_PDE5 = 7'd112;
parameter PT_RW_PDE6 = 7'd113;
parameter PT_RW_PDE7 = 7'd114;
parameter PTG1 = 7'd115;
parameter PTG2 = 7'd116;
parameter PTG3 = 7'd117;
parameter IPT_CLOCK1 = 7'd1;
parameter IPT_CLOCK2 = 7'd2;
parameter IPT_CLOCK3 = 7'd3;

endpackage
