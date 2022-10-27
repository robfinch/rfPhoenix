#include "vasm.h"

#define TRACE(x)		/* printf(x) */
#define TRACE2(x,y)	/* printf((x),(y)) */

char *cpu_copyright="vasm rfPhoenix cpu backend (c) in 2022 Robert Finch";

char *cpuname="rfPhoenix";
int bitsperbyte=8;
int bytespertaddr=4;
int abits=32;
/* The following indicates if instructions including postfixes can span cache
   lines. */
int span_cache_lines = 1;
static taddr sdreg = 45;
static taddr sd2reg = 44;
static taddr sd3reg = 43;
static __int64 regmask = 0x3fLL;

static insn_count = 0;
static byte_count = 0;

static insn_sizes1[20000];
static insn_sizes2[20000];
static int sz1ndx = 0;
static int sz2ndx = 0;
static short int argregs[14] = {1,2,29,30,31,32,33,34,35,36,37,38,-1,-1};
static short int tmpregs[14] = {3,4,5,6,7,8,9,10,11,12,13,14,-1,-1};
static short int saved_regs[14] = {15,16,17,18,19,20,21,22,23,24,25,26,27,28};

static char *regnames[64] = {
	"0", "a0", "a1", "t0", "t1", "t2", "t3", "t4",
	"t5", "t6", "t7", "t8", "t9", "t10", "t11", "s0",
	"s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8",
	"s9", "s10", "s11", "s12", "s13", "a2", "a3", "a4",
	"a5", "a6", "a7", "a8", "a9", "a10", "a11", "r39",
	"r40", "r41", "r42", "gp2", "gp1", "gp0", "fp", "sp",
	"vm0", "vm1", "vm2", "vm3", "vm4", "vm5", "vm6", "vm7",
	"lc", "lr1", "lr2", "pc", "ssp", "hsp", "msp", "isp"
};

static int regop[64] = {
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, 
	OP_REG, OP_LK, OP_LK, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG
};

mnemonic mnemonics[]={
	"abs",	{OP_REG,OP_REG,0,0,0}, {R3RR,CPU_ALL,0,0x0C000001LL,6},

	"add", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TB(1LL)|OP(2),6},	
	"add", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TA(1LL)|OP(2),6},	
	"add", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TB(1LL)|TA(1)|OP(2),6},	
	"add", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TT(1LL)|OP(2),6},	
	"add", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"add", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"add", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"add", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(4LL)|OP(2LL),6},	
	
	"add", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(4),6},	
	"add", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(4),6},	
	"add", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1)|OP(4),6},	
	"add", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(4),6},	

	"and", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TB(1LL)|OP(2),6},	
	"and", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TA(1LL)|OP(2),6},	
	"and", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TB(1LL)|TA(1)|OP(2),6},	
	"and", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TT(1LL)|OP(2),6},	
	"and", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"and", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"and", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"and", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(8LL)|OP(2),6},	
	
	"and", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(8),6},	
	"and", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(8),6},	
	"and", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1LL)|OP(8),6},	
	"and", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(8LL),6},	

//	"bbs",	{OP_LK,OP_REG,OP_IMM,OP_IMM,0}, {BL,CPU_ALL,0,0x00001F000822LL,6},
	"bbs",	{OP_REG,OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(5LL)|OP(28),6},

	"beq",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(6LL)|OP(28),6},
	"bge",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(1LL)|OP(28),6},
	"bgeu",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(3LL)|OP(28),6},
	"blt",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(0LL)|OP(28),6},
	"bltu",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(2LL)|OP(28),6},
	"bne",	{OP_REG,OP_REG,OP_IMM,0,0}, {B,CPU_ALL,0,SZ(2LL)|CND(7LL)|OP(28),6},
	
	"bra",	{OP_IMM,0,0,0,0}, {B2,CPU_ALL,0,OP(25),6},

	"brk",	{0,0,0,0,0}, {R2,CPU_ALL,0,RM(1LL)|TB(0LL)|RB(1LL)|TA(1LL)|RA(63LL)|TT(1LL)|RT(63LL)|OP(32),6},
	"bsr",	{OP_LK,OP_IMM,0,0,0}, {BL2,CPU_ALL,0,OP(25LL),6},
	"bsr",	{OP_IMM,0,0,0,0}, {B2,CPU_ALL,0,RT(1LL)|OP(25LL),6},

	"call",	{OP_LK,OP_IMM,0,0,0}, {JL2,CPU_ALL,0,OP(24LL),6},
	"call",	{OP_IMM,0,0,0,0}, {J2,CPU_ALL,0,OP(24LL),6},

	"cmp", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TB(1LL)|OP(2),6},	
	"cmp", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TA(1LL)|OP(2),6},	
	"cmp", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TB(1LL)|TA(1LL)|OP(2),6},	
	"cmp", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TT(1LL)|OP(2),6},	
	"cmp", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"cmp", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"cmp", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"cmp", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(13LL)|OP(2),6},	
	
	"cmp", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(13),6},	
	"cmp", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(13),6},	
	"cmp", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1LL)|OP(13),6},	
	"cmp", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(13LL),6},	

	"cntlz", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(0LL)|TA(1LL)|TT(1LL)|OP(2),6},
	"cntlz", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(0LL)|TA(1LL)|TT(0LL)|OP(2),6},
	"cntlz", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(0LL)|TA(0LL)|TT(1LL)|OP(2),6},
	"cntlz", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(0LL)|TA(0LL)|TT(0LL)|OP(2),6},

	"cntpop", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(2LL)|TA(1LL)|TT(1LL)|OP(2),6},
	"cntpop", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(2LL)|TA(1LL)|TT(0LL)|OP(2),6},
	"cntpop", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(2LL)|TA(0LL)|TT(1LL)|OP(2),6},
	"cntpop", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|FN(1LL)|RB(2LL)|TA(0LL)|TT(0LL)|OP(2),6},

	"com", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|IMM(0x7ffffLL)|TA(1LL)|OP(9),6},
	"com", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|IMM(0x7ffffLL)|TT(1LL)|OP(9),6},
	"com", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|IMM(0x7ffffLL)|TA(1LL)|TT(1LL)|OP(9),6},
	"com", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,SZ(2LL)|IMM(0x7ffffLL)|OP(9LL),6},

	"csrrc", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|IMM(0x8000LL)|OP(7LL),6},
	"csrrd", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(7LL),6},
	"csrrs", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|IMM(0x30000LL)|OP(7LL),6},
	"csrrw", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|IMM(0x10000LL)|OP(7LL),6},

	"eor", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|OP(2),6},	
	"eor", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TA(1LL)|OP(2),6},	
	"eor", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TA(1)|OP(2),6},	
	"eor", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TT(1LL)|OP(2),6},	
	"eor", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"eor", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"eor", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"eor", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|OP(2),6},	
	
	"eor", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(10),6},	
	"eor", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(10),6},	
	"eor", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1LL)|OP(10),6},	
	"eor", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(10LL),6},	

	"jmp",	{OP_IMM,0,0,0,0}, {J2,CPU_ALL,0,OP(24),6},
	"jsr",	{OP_LK,OP_IMM,0,0,0}, {JL2,CPU_ALL,0,OP(24),6},
	"jsr",	{OP_IMM,0,0,0,0}, {J2,CPU_ALL,0,OP(24),6},

	"load",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,SZ(2LL)|OP(44),6},	
	"load",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,SZ(2LL)|OP(44),6},	
	"load",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,SZ(2LL)|FN(44)|OP(2),6},	
	"load",	{OP_VREG,OP_SCNDX,OP_VMREG,0,0}, {SCNDX,SZ(2LL)|CPU_ALL,0,FN(44)|OP(2),6},	

	"loadu",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,SZ(2LL)|OP(45),6},	
	"loadu",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,SZ(2LL)|OP(45),6},	
	"loadu",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,SZ(2LL)|FN(45)|OP(2),6},	
	"loadu",	{OP_VREG,OP_SCNDX,OP_VMREG,0,0}, {SCNDX,SZ(2LL)|CPU_ALL,0,FN(45)|OP(2),6},	

/*****************************************/

//	"ldo",	{OP_REG,OP_SEL|OP_REGIND8,0,0,0}, {REGIND,CPU_ALL,0,0x87LL,4},	
	
	"ldi",  {OP_VMREG,OP_NEXTREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(9LL),6},
	"ldi",  {OP_REG,OP_NEXTREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(9LL),6},

	"lea",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,0xD4LL,6,0x04,6},
	"lea",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0xD4LL,6,0x04,6},
	"lea",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0x19LL,6},	

	"max",	{OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3RR,CPU_ALL,0,0x520000000002LL,6},	
	"memdb",	{0,0,0,0,0}, {BITS16,CPU_ALL,0,0xF9,2},
	"memsb",	{0,0,0,0,0}, {BITS16,CPU_ALL,0,0xF8,2},
	"min",	{OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3RR,CPU_ALL,0,0x500000000002LL,6},	

	"mov", {OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x13LL,4},	
	"move", {OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x13LL,4},	
//	"mov",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x0817F00000AALL,6},
//	"move",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x0817F00000AALL,6},
	"movsxb",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x0A120E0000AALL,6},
	"movsxt",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x0A123E0000AALL,6},
	"movsxw",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x0A121E0000AALL,6},
	"movzxb",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x08120E0000AALL,6},
	"movzxt",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x08123E0000AALL,6},
	"movzxw",	{OP_REG,OP_REG,0,0,0}, {MV,CPU_ALL,0,0x08121E0000AALL,6},

	"mrts",	{0,0,0,0,0}, {RTS,CPU_ALL,0,0x01F2LL, 2},

	"mul", {OP_VREG,OP_VREG,OP_IMM,OP_VMREG,0}, {RIL,CPU_ALL,0,0x1D2LL,6},	
	"mul", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RIL,CPU_ALL,0,0x1D2LL,6},
	"mul", {OP_VREG,OP_VREG,OP_VREG,OP_VMREG,0}, {R3,CPU_ALL,0,0x0C0000000102LL,6},	
	"mul", {OP_VREG,OP_VREG,OP_VREG,0,0}, {R3,CPU_ALL,0,0x0C0000000102LL,6},	
	"mul", {OP_REG,OP_REG,OP_REG,0,0}, {R3RR,CPU_ALL,0,0x0C0000000002LL,6},	
	"mul", {OP_REG,OP_REG,OP_IMM,0,0}, {RIL,CPU_ALL,0,0xD2,6,0x06LL,4},
	"muladd", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3RR,CPU_ALL,0,0x0C0000000002LL,6},	

	"mulh", {OP_REG,OP_REG,OP_REG,0,0}, {R3RR,CPU_ALL,0,0x1E0000000002LL,6},	

	"mulu", {OP_REG,OP_REG,OP_REG,0,0}, {R3RR,CPU_ALL,0,0x1C0000000002LL,6},	
	"mulu", {OP_REG,OP_REG,OP_IMM,0,0}, {RIL,CPU_ALL,0,0xDE,6,0x0ELL,4},

	"mux",	{OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3RR,CPU_ALL,0,0x680000000002LL,6},	

	"nand", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x000000000102LL,6},	
	"nand", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x000000000102LL,6},	
	"nand", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x000000000002LL,6},	

	"neg", {OP_REG,OP_NEXTREG,OP_REG,0,0}, {R2,CPU_ALL,0,0x0000000DLL,4},	
//	"neg",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A000001LL,4},

	"nop",	{0,0,0,0,0}, {BITS16,CPU_ALL,0,0x0B,6},

	"nor", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x020000000102LL,6},	
	"nor", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x020000000102LL,6},	
	"nor", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x020000000002LL,6},	
	"not", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,0x08000001LL,4},

	"or", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TB(1LL)|OP(2),6},	
	"or", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TA(1LL)|OP(2),6},	
	"or", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TB(1LL)|TA(1)|OP(2),6},	
	"or", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TT(1LL)|OP(2),6},	
	"or", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"or", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"or", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"or", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(9LL)|OP(2),6},	
	
	"or", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(9),6},	
	"or", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(9),6},	
	"or", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1LL)|OP(9),6},	
	"or", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(9LL),6},	

	"orc", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x060000000102LL,6},	
	"orc", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x060000000102LL,6},	
	"orc", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x060000000002LL,6},	
	"orn", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x060000000102LL,6},	
	"orn", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x060000000102LL,6},	
	"orn", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x060000000002LL,6},	

	"peekq",	{OP_REG,OP_NEXTREG,OP_IMM,0,0},{R3RR,CPU_ALL,0,0x140000000007LL,6},	
	"peekq",	{OP_REG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x140000000007LL,6},	

	"pfi",	{OP_REG,0,0,0,0},{R3RR,CPU_ALL,0,0x220000000007LL,6},	

	"pop",	{OP_REG,OP_REG,OP_REG,OP_REG,0},{R4,CPU_ALL,0,0x800000BELL,4},	
	"pop",	{OP_REG,OP_REG,OP_REG,0,0},{R3,CPU_ALL,0,0x600000BELL,4},	
	"pop",	{OP_REG,OP_REG,0,0,0},{R3,CPU_ALL,0,0x400000BELL,4},	
	"pop",	{OP_REG,0,0,0,0},{R3,CPU_ALL,0,0x000BCLL,2},	

	"popq",	{OP_REG,OP_NEXTREG,OP_IMM,0,0},{R3RR,CPU_ALL,0,0x120000000007LL,6},	
	"popq",	{OP_REG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x120000000007LL,6},	

	"ptghash", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,0x5E000001,4},

	"ptrdif",	{OP_REG,OP_REG,OP_REG,OP_IMM,0}, {R3RI,CPU_ALL,0,0x281000000002LL,6},
	"ptrdif",	{OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3RR,CPU_ALL,0,0x280000000002LL,6},

	"push",	{OP_REG,OP_REG,OP_REG,OP_REG,0},{R4,CPU_ALL,0,0x800000AELL,4},	
	"push",	{OP_REG,OP_REG,OP_REG,0,0},{R3,CPU_ALL,0,0x600000AELL,4},	
	"push",	{OP_REG,OP_REG,0,0,0},{R3,CPU_ALL,0,0x400000AELL,4},	
	"push",	{OP_REG,0,0,0,0},{R3,CPU_ALL,0,0x000ACLL,2},	

	"pushq",	{OP_NEXTREG,OP_REG,OP_IMM,0,0},{R3RR,CPU_ALL,0,0x100000000007LL,6},	
	"pushq",	{OP_NEXTREG,OP_REG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x100000000007LL,6},	

//	"rem", {OP_VREG,OP_VREG,OP_VREG,0,0}, {R3,CPU_ALL,0,0x200000000102LL,6},	
//	"rem", {OP_REG,OP_REG,OP_REG,0,0}, {R3RR,CPU_ALL,0,0x200000000002LL,6},	
//	"rem", {OP_REG,OP_REG,OP_IMM,0,0}, {RIL,CPU_ALL,0,0x42LL,6},

	"resetq",	{OP_NEXTREG,OP_NEXTREG,OP_IMM,0,0},{R3RR,CPU_ALL,0,0x180000000007LL,6},	
	"resetq",	{OP_NEXTREG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x180000000007LL,6},	

	"revbit",	{OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,0x50000001LL,4},

	"ret",	{OP_LK,OP_IMM,0,0,0}, {RTS,CPU_ALL,0,0x00F2LL, 2},
	"ret",	{OP_LK,0,0,0,0}, {RTS,CPU_ALL,0,0x00F2LL, 2},
	"ret",  {OP_NEXTREG,OP_NEXTREG,OP_IMM,0,0}, {RTS,CPU_ALL,0,SZ(2LL)|RA(57)|RT(47)|OP(26),6},
	"ret",	{0,0,0,0,0}, {RTS,CPU_ALL,0,SZ(2LL)|RA(57)|RT(47)|OP(26), 6},

	"rex",	{OP_IMM,OP_REG,0,0,0},{REX,CPU_ALL,0,0x200000000007LL,6},	

	"rte",	{OP_IMM,OP_REG,0,0,0},{RTE,CPU_ALL,0,0x260000000007LL,6},	

	"rol",	{OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x860000000002LL,6},	
	"rol",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x860000000002LL,6},	
	"rol",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x860000000002LL,6},	
	"rol",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_VMREG,0}, {R3,CPU_ALL,0,0x860000000002LL,6},	
	"rol",	{OP_REG,OP_REG,OP_IMM,0,0}, {SHIFTI,CPU_ALL,0,0x860000000002LL,6},
	"rol",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,0x5B,4},

	"ror",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x880000000002LL,6},	
	"ror",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x880000000002LL,6},	
	"ror",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_VMREG,0}, {R3,CPU_ALL,0,0x880000000002LL,6},	
	"ror",	{OP_REG,OP_REG,OP_IMM,0,0}, {SHIFTI,CPU_ALL,0,0x880000000002LL,6},
	"ror",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,0x5C,4},
	
	"rti",	{OP_IMM,0,0,0,0}, {RTS,CPU_ALL,0,0x00F0LL, 2},
	"rti",	{0,0,0,0,0}, {RTS,CPU_ALL,0,0x00F0LL, 2},

	"rts",	{OP_LK,OP_IMM,0,0,0}, {RTS,CPU_ALL,0,0x00F2LL, 2},
	"rts",  {OP_NEXTREG,OP_NEXTREG,OP_IMM,0,0}, {RTS,CPU_ALL,0,SZ(2LL)|RA(57)|RT(47)|OP(26),6},
	"rts",	{0,0,0,0,0}, {RTS,CPU_ALL,0,SZ(2LL)|RA(57)|RT(47)|OP(26), 6},
//	"rts",	{OP_LK,0,0,0,0}, {RTS,CPU_ALL,0,0x00F2LL, 2},

	"sei",	{OP_REG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x2E0000000007LL,6},	

	"sll",	{OP_REG,OP_REG,OP_IMM,OP_IMM,0}, {RI6,CPU_ALL,0,SZ(2LL)|0x0600000002,5},
	"sll",	{OP_REG,OP_REG,OP_IMM,0,0}, {RI6,CPU_ALL,0,SZ(2LL)|0x0600000002,5},
	"sll",	{OP_REG,OP_REG,OP_REG,OP_IMM,0}, {R2,CPU_ALL,0,SZ(2LL)|0x0780000002,5},
	"sll",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|0x0780000002,5},

	"sra",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x840000000002LL,6},	
	"sra",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x840000000002LL,6},	
	"sra",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_VMREG,0}, {R3,CPU_ALL,0,0x840000000002LL,6},	
//	"sra",	{OP_REG,OP_REG,OP_IMM,0,0}, {SHIFTI,CPU_ALL,0,0x840000000002LL,6},
	"sra",	{OP_REG,OP_REG,OP_IMM,0,0}, {RI6,CPU_ALL,0,0x6E,4},
	"sra",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,0x5A,4},

	"srl",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x820000000002LL,6},	
	"srl",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x820000000002LL,6},	
	"srl",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_VMREG,0}, {R3,CPU_ALL,0,0x820000000002LL,6},	
//	"srl",	{OP_REG,OP_REG,OP_IMM,0,0}, {SHIFTI,CPU_ALL,0,0x820000000002LL,6},
	"srl",	{OP_REG,OP_REG,OP_IMM,0,0}, {RI6,CPU_ALL,0,0x6D,4},
	"srl",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,0x59,4},

	"statq",	{OP_REG,OP_NEXTREG,OP_IMM,0,0},{R3RR,CPU_ALL,0,0x160000000007LL,6},	
	"statq",	{OP_REG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x160000000007LL,6},	

	"store",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,SZ(2LL)|OP(40LL),6},	
	"store",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,SZ(2LL)|OP(40LL),6},	
	"store",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,SZ(2LL)|FN(40LL)|OP(2LL),6},	

	"stptg",	{OP_REG,OP_REG,OP_REG,OP_REG,0},{R3RR,CPU_ALL,0,0x4A0000000007LL,6},	

	"sub", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TB(1LL)|OP(2LL),6},	
	"sub", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TA(1LL)|OP(2),6},	
	"sub", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TB(1LL)|TA(1LL)|OP(2),6},	
	"sub", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TT(1LL)|OP(2),6},	
	"sub", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"sub", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"sub", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|TB(1LL)|TA(1LL)|TT(1)|OP(2),6},	
	"sub", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(5LL)|OP(2LL),6},	

	"subf", {OP_VREG,OP_VREG,OP_IMM,OP_VMREG,0}, {RIL,CPU_ALL,0,SZ(2LL)|OP(5LL),6},
	"subf", {OP_REG,OP_REG,OP_IMM,0,0}, {RIL,CPU_ALL,0,SZ(2LL)|OP(5LL),6},
/* 0000_1010_0001_0001_1111_0000_0000_0000_0000_0000_AALL */

	"sxb",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A120E0000AALL,6},	
	"sxc",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A121E0000AALL,6},	/* alternate mnemonic for sxw */
	"sxo",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A123E0000AALL,6},
	"sxw",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A121E0000AALL,6},
	"sxt",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A123E0000AALL,6},

	"sync", {0,0,0,0,0}, {BITS16,CPU_ALL,0,0xF7LL,2},
	"sys",	{OP_IMM,0,0,0,0}, {BITS32,CPU_ALL,0,0xA5,4},

	/* Alternate mnemonic for eor */
	"xor", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|OP(2),6},	
	"xor", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TA(1LL)|OP(2),6},	
	"xor", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TA(1)|OP(2),6},	
	"xor", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TT(1LL)|OP(2),6},	
	"xor", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TT(1LL)|OP(2),6},	
	"xor", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"xor", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|TB(1LL)|TA(1LL)|TT(1LL)|OP(2),6},	
	"xor", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,SZ(2LL)|FN(10LL)|OP(2),6},	
	
	"xor", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|OP(10),6},	
	"xor", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TT(1LL)|OP(10),6},	
	"xor", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|TA(1LL)|TT(1LL)|OP(10),6},	
	"xor", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,SZ(2LL)|OP(10LL),6}
};

int mnemonic_cnt=sizeof(mnemonics)/sizeof(mnemonics[0]);

int rfPhoenix_data_operand(int n)
{
  if (n&OPSZ_FLOAT) return OPSZ_BITS(n)>32?OP_F64:OP_F32;
  if (OPSZ_BITS(n)<=8) return OP_D8;
  if (OPSZ_BITS(n)<=16) return OP_D16;
  if (OPSZ_BITS(n)<=32) return OP_D32;
  return OP_D64;
}

/* parse instruction and save extension locations */
char *parse_instruction(char *s,int *inst_len,char **ext,int *ext_len,
                        int *ext_cnt)
{
  char *inst = s;

	TRACE("+parse_instruction\n");
  while (*s && *s!='.' && !isspace((unsigned char)*s))
    s++;
  *inst_len = s - inst;
  if (*s =='.') {
    /* extension present */
    ext[*ext_cnt] = ++s;
    while (*s && *s!='.' && !isspace((unsigned char)*s))
      s++;
    ext_len[*ext_cnt] = s - ext[*ext_cnt];
    *ext_cnt += 1;
  }
  return (s);
}

/* fill in pointers to default qualifiers, return number of qualifiers */
int set_default_qualifiers(char **q,int *q_len)
{
  q[0] = "t";
  q_len[0] = 1;
  return (1);
}

/* check if a given value fits within a certain number of bits */
static int is_nbit(int64_t val, int64_t n)
{
	int64_t low, high;
  if (n > 63)
    return (1);
	low = -(1LL << (n - 1LL));
	high = (1LL << (n - 1LL));
	return (val >= low && val < high);
}

static int is_identchar(unsigned char ch)
{
	return (isalnum(ch) || ch == '_');
}

/* parse register names */
static int is_reg(char *p, char **ep, int* typ)
{
	int nn;
	int rg = -1;

	TRACE("+is_reg");	
	if (ep)
		*ep = p;
	if (*p == 'r' || *p == 'R') {
		if (isdigit((unsigned char)p[1]) && isdigit((unsigned char)p[2]) && !ISIDCHAR((unsigned char)p[3])) {
			rg = (p[1]-'0')*10 + p[2]-'0';
			if (rg < 64) {
				if (ep)
					*ep = &p[3];
				*typ = regop[rg];
				return (rg);
			}
			return (-1);
		}
		if (isdigit((unsigned char)p[1]) && !ISIDCHAR((unsigned char)p[2])) {
			rg = p[1]-'0';
			if (ep)
				*ep = &p[2];
			*typ = regop[rg];
			return (rg);
		}
	}
	for (nn = 0; nn < 64; nn++) {
		if (p[0] == regnames[nn][0] && p[1]== regnames[nn][1]) {
			if (!ISIDCHAR((unsigned char)p[2])) {
				if (regnames[nn][2]=='\0') {
					if (ep)
						*ep = &p[2];
					*typ = regop[nn];
					return (nn);
				}
				return (-1);
			}
			if (regnames[nn][2]=='\0')
				return (-1);
			if (regnames[nn][2]==p[2]) {
				if (!is_identchar((unsigned char)p[3])) {
					if (regnames[nn][3]=='\0') {
						*typ = regop[nn];
						if (ep)
							*ep = &p[3];
						return (nn);
					}
					return (-1);
				}
				if (regnames[nn][3]==p[3]) {
					if (!is_identchar((unsigned char)p[4])) {
						if (regnames[nn][4]=='\0') {
							if (ep)
								*ep = &p[4];
							*typ = regop[nn];
							return (nn);
						}
						return (-1);
					}
				}
			}
		}
	}
	return (-1);	
}

/* parse a vector register, v0 to v63 */
static int is_vreg(char *p, char **ep)
{
	int rg = -1;

	*ep = p;
	if (*p != 'v' && *p != 'V') {
		return (-1);
	}
	if (isdigit((unsigned char)p[1]) && isdigit((unsigned char)p[2]) && !ISIDCHAR((unsigned char)p[3])) {
		rg = (p[1]-'0')*10 + p[2]-'0';
		if (rg < 64) {
			*ep = &p[3];
			return (rg+64);
		}
		return (-1);
	}
	if (isdigit((unsigned char)p[1]) && !ISIDCHAR((unsigned char)p[2])) {
		rg = p[1]-'0';
		*ep = &p[2];
		return (rg+64);
	}
	return (-1);
}

/* parse a link register, lk0 to lk3 */
static int is_lkreg(char *p, char **ep)
{
	int rg = -1;

	*ep = p;
	if (*p != 'l' && *p != 'L') {
		return (-1);
	}
	if (p[1] != 'k' && p[1] != 'K') {
		return (-1);
	}
	if (isdigit((unsigned char)p[2]) && !ISIDCHAR((unsigned char)p[3])) {
		rg = p[2]-'0' + 40;
		if (rg < 4) {
			*ep = &p[3];
			return (rg);
		}
	}
	return (-1);
}

/* parse a code address register, ca0 to ca7 */
static int is_careg(char *p, char **ep)
{
	int rg = -1;

	*ep = p;
	/* IP */
	if ((p[0]=='I' || p[0]=='i') && (p[1]=='P' || p[1]=='p') && !ISIDCHAR((unsigned char)p[3])) {
		*ep = &p[3];
		return (7);
	}
	/* PC */
	if ((p[0]=='P' || p[0]=='p') && (p[1]=='C' || p[1]=='c') && !ISIDCHAR((unsigned char)p[3])) {
		*ep = &p[3];
		return (7);
	}
	if (*p != 'c' && *p != 'C') {
		return (-1);
	}
	if (p[1] != 'a' && p[1] != 'A') {
		return (-1);
	}
	if (isdigit((unsigned char)p[2]) && !ISIDCHAR((unsigned char)p[3])) {
		rg = p[2]-'0';
		if (rg < 8) {
			*ep = &p[3];
			return (rg);
		}
	}
	return (-1);
}

/* Parse a vector mask register, vm0 to vm7
	 The 'z' indicator follows the register number.
	 vm5z for instance indicates to zero out masked results, while
	 vm5 by itself indicates to not modify the masked result register.
*/
static int is_vmreg(char *p, char **ep)
{
	int rg = -1;
	int z = 0;

	*ep = p;
	if (*p != 'v' && *p != 'V') {
		return (-1);
	}
	if (p[1] != 'm' && p[1] != 'M') {
		return (-1);
	}
	if (isdigit((unsigned char)p[2])) {
		if (p[3]=='Z' || p[3]=='z') {
			p++;
			z = 1;
		}
		if (!ISIDCHAR((unsigned char)p[3])) {
			rg = p[2]-'0';
			if (rg < 8) {
				rg = rg + 32;
				*ep = &p[3];
				return (rg);
			}
		}
	}
	return (-1);
}

static int is_branch(mnemonic* mnemo)
{
	switch(mnemo->ext.format) {
	case B:
	case BL:
	case J:
	case JL:
	case B2:
	case BL2:
	case J2:
	case JL2:
	case B3:
	case BL3:
	case J3:
	case JL3:
		return (1);
	}
	return (0);	
}

int parse_operand(char *p,int len,operand *op,int requires)
{
	int rg, nrg;
	int rv = PO_NOMATCH;
	char ch;
	int dmm;

	TRACE("+parse_operand\n");
	op->attr = REL_NONE;
	op->value = NULL;

	if (requires==OP_NEXTREG) {
    op->type = OP_REG;
    op->basereg = 0;
    op->value = number_expr((taddr)0);
		return (PO_NEXT);
	}
	if (requires==OP_NEXT) {
    op->value = number_expr((taddr)0);
		return (PO_NEXT);
	}

  p=skip(p);
  if ((rg = is_reg(p, &p, &op->type)) >= 0) {
    op->basereg=rg;
    op->value = number_expr((taddr)rg);
  }
  else if ((rg = is_vreg(p, &p)) >= 0) {
    op->type=OP_VREG;
    op->basereg=rg;
    op->value = number_expr((taddr)rg);
  }
  else if ((rg = is_careg(p, &p)) >= 0) {
    op->type=OP_CAREG;
    op->basereg=rg;
    op->value = number_expr((taddr)rg);
  }
  else if ((rg = is_vmreg(p, &p)) >= 0) {
    op->type=OP_VMREG;
    op->basereg=rg;
    op->value = number_expr((taddr)rg);
  }
  else if ((rg = is_lkreg(p, &p)) >= 0) {
    op->type=OP_LK;
    op->basereg=rg;
    op->value = number_expr((taddr)rg);
  }
  else if(p[0]=='#'){
    op->type=OP_IMM;
    p=skip(p+1);
    op->value=parse_expr(&p);
  }else{
    int parent=0;
    expr *tree;
    op->type=-1;
    if (*p == '[') {
    	tree = number_expr((taddr)0);
    }
    else
    	tree=parse_expr(&p);
    if(!tree)
      return (PO_NOMATCH);
   	op->type = OP_IMM;
    if(*p=='['){
      parent=1;
      p=skip(p+1);
    }
    p=skip(p);
    if(parent){
    	if ((rg = is_reg(p, &p, &op->type)) >= 0) {
    		op->basereg = rg;
    		p = skip(p);
    		if (*p=='+') {
    			p = skip(p+1);
    			if ((nrg = is_reg(p, &p, &op->type)) >= 0) {
    				op->ndxreg = nrg;
		    		p = skip(p);
		    		op->type = OP_SCNDX;
    			}
    			else if ((nrg = is_vreg(p, &p)) >= 0) {
    				op->ndxreg = nrg;
		    		p = skip(p);
		    		op->type = OP_SCNDX;
    			}
    			else {
    				cpu_error(0);
    				return (0);
    			}
    		}
    		else {
    			op->type = OP_REGIND;
    		}
    	}
    	else if ((rg = is_careg(p, &p)) >= 0) {
    		op->basereg = rg;
    		op->type = OP_CAREGIND;
    	}
      if(*p!=']'){
				cpu_error(5);
				return (0);
      }
      else
				p=skip(p+1);
    }
    op->value=tree;
  }
	TRACE("p");
  if(requires & op->type) {
    return (PO_MATCH);
  }
  return (PO_NOMATCH);
}

operand *new_operand()
{
  operand *nw=mymalloc(sizeof(*nw));
  nw->type=-1;
  return nw;
}

static void fix_reloctype(dblock *db,int rtype)
{
  rlist *rl;

  for (rl=db->relocs; rl!=NULL; rl=rl->next)
    rl->type = rtype;
}


static int get_reloc_type(operand *op)
{
  int rtype = REL_NONE;

  if (OP_DATAM(op->type)) {  /* data relocs */
    return (REL_ABS);
  }

  else {  /* handle instruction relocs */
  	switch(op->format) {
  	
  	/* BEQ r1,r2,target */
  	case B:
  		if (op->number > 1)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_PC;
	          break;
	        case REL_PLT:
	          rtype = REL_PLTPC;
	          break;
	        case REL_LOCALPC:
	          rtype = REL_LOCALPC;
	          break;
	        default:
	          cpu_error(11);
	          break;
	      }
      break;

		/* BRA target */		
  	case B2:
      switch (op->attr) {
        case REL_NONE:
          rtype = REL_PC;
          break;
        case REL_PLT:
          rtype = REL_PLTPC;
          break;
        case REL_LOCALPC:
          rtype = REL_LOCALPC;
          break;
        default:
          cpu_error(11);
          break;
      }
      break;
  		
  	/* BEQZ r1,target */
  	case B3:
  		if (op->number > 0)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_PC;
	          break;
	        case REL_PLT:
	          rtype = REL_PLTPC;
	          break;
	        case REL_LOCALPC:
	          rtype = REL_LOCALPC;
	          break;
	        default:
	          cpu_error(11);
	          break;
	      }
      break;

		/* BEQ LK1,r1,r2,target */
  	case BL:
  		if (op->number > 2)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_PC;
	          break;
	        case REL_PLT:
	          rtype = REL_PLTPC;
	          break;
	        case REL_LOCALPC:
	          rtype = REL_LOCALPC;
	          break;
	        default:
	          cpu_error(11);
	          break;
	      }
      break;

		/* BRA	LK1,target */
  	case BL2:
  		if (op->number > 0)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_PC;
	          break;
	        case REL_PLT:
	          rtype = REL_PLTPC;
	          break;
	        case REL_LOCALPC:
	          rtype = REL_LOCALPC;
	          break;
	        default:
	          cpu_error(11);
	          break;
	      }
      break;

  	/* BEQZ LK1,r1,target */
  	case BL3:
  		if (op->number > 1)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_PC;
	          break;
	        case REL_PLT:
	          rtype = REL_PLTPC;
	          break;
	        case REL_LOCALPC:
	          rtype = REL_LOCALPC;
	          break;
	        default:
	          cpu_error(11);
	          break;
	      }
      break;

  	/* JEQ r1,r2,target */
    case J:
    	if (op->number > 1)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_ABS;
	          break;
	        case REL_PLT:
	        case REL_GLOBDAT:
	        case REL_SECOFF:
	          rtype = op->attr;
	          break;
	        default:
	          cpu_error(11); /* reloc attribute not supported by operand */
	          break;
	      }
      break;

		/* JEQ LK1,r1,r1,target */ 
    case JL:
    	if (op->number > 2)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_ABS;
	          break;
	        case REL_PLT:
	        case REL_GLOBDAT:
	        case REL_SECOFF:
	          rtype = op->attr;
	          break;
	        default:
	          cpu_error(11); /* reloc attribute not supported by operand */
	          break;
	      }
      break;

		/* JMP target */
    case J2:
      switch (op->attr) {
        case REL_NONE:
          rtype = REL_ABS;
          break;
        case REL_PLT:
        case REL_GLOBDAT:
        case REL_SECOFF:
          rtype = op->attr;
          break;
        default:
          cpu_error(11); /* reloc attribute not supported by operand */
          break;
      }
      break;

		/* JMP LK1,target */
    case JL2:
    	if (op->number > 0)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_ABS;
	          break;
	        case REL_PLT:
	        case REL_GLOBDAT:
	        case REL_SECOFF:
	          rtype = op->attr;
	          break;
	        default:
	          cpu_error(11); /* reloc attribute not supported by operand */
	          break;
	      }
      break;

		/* JEQZ r1,target */
    case J3:
    	if (op->number > 0)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_ABS;
	          break;
	        case REL_PLT:
	        case REL_GLOBDAT:
	        case REL_SECOFF:
	          rtype = op->attr;
	          break;
	        default:
	          cpu_error(11); /* reloc attribute not supported by operand */
	          break;
	      }
      break;

		/* JEQZ LK1,r1,target */
    case JL3:
    	if (op->number > 1)
	      switch (op->attr) {
	        case REL_NONE:
	          rtype = REL_ABS;
	          break;
	        case REL_PLT:
	        case REL_GLOBDAT:
	        case REL_SECOFF:
	          rtype = op->attr;
	          break;
	        default:
	          cpu_error(11); /* reloc attribute not supported by operand */
	          break;
	      }
      break;

    default:
      switch (op->attr) {
        case REL_NONE:
          rtype = REL_ABS;
          break;
        case REL_GOT:
        case REL_PLT:
        case REL_SD:
          rtype = op->attr;
          break;
        default:
          cpu_error(11); /* reloc attribute not supported by operand */
          break;
      }
  	}
  }
  return (rtype);
}

/* create a reloc-entry when operand contains a non-constant expression */
static taddr make_reloc(int reloctype,operand *op,section *sec,
                        taddr pc,rlist **reloclist, int *constexpr)
{
  taddr val;

	TRACE("M");
	*constexpr = 1;
  if (!eval_expr(op->value,&val,sec,pc)) {
  	*constexpr = 0;
    /* non-constant expression requires a relocation entry */
    symbol *base;
    int btype,pos,size,disp;
    taddr addend,mask;

    btype = find_base(op->value,&base,sec,pc);
    pos = disp = 0;

    if (btype > BASE_ILLEGAL) {
      if (btype == BASE_PCREL) {
        if (reloctype == REL_ABS)
          reloctype = REL_PC;
        else
          goto illreloc;
      }

      if (reloctype == REL_PC && !is_pc_reloc(base,sec)) {
        /* a relative branch - reloc is only needed for external reference */
				TRACE("m");
        return val-pc;
      }

      /* determine reloc size, offset and mask */
      if (OP_DATAM(op->type)) {  /* data operand */
        switch (op->type) {
          case OP_D8:
            size = 8;
            break;
          case OP_D16:
            size = 16;
            break;
          case OP_D32:
          case OP_F32:
            size = 32;
            break;
          case OP_D64:
          case OP_F64:
            size = 64;
            break;
          default:
            ierror(0);
            break;
        }
        addend = val;
        mask = -1;
      }
      else {  /* instruction operand */
        addend = (btype == BASE_PCREL) ? val + disp : val;
      	switch(op->format) {
      	/* Conditional jump */
      	case B:
      	case J:
      	case JL:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                           22,17,0,0x1ffffLL);
          break;
      	/* Unconditional jump */
      	case B2:
        case J2:
        case JL2:
		      add_extnreloc_masked(reloclist,base,val,reloctype,
                           12,36,0,0xfffffffffLL);
          break;
        case RI:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
        	break;
        case VRI:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
        	break;
        case DIRECT:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
        	break;
        case VDIRECT:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
        	break;
        case REGIND:
        	// ToDo: vector VREGIND
        	if (op->basereg==sdreg) {
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
	        }
        	else if (op->basereg==sd2reg) {
        		int org_sdr = sdreg;
        		sdreg = sd2reg;
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         20,19,0,0x7ffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,3,0,0x00380000LL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         54,10,0,0xffc00000LL);
						sdreg = org_sdr;        		
        	}
        	else if (op->basereg==sd3reg) {
        		int org_sdr = sdreg;
        		sdreg = sd3reg;
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         20,19,0,0x7ffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,3,0,0x00380000LL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         54,10,0,0xffc00000LL);
						sdreg = org_sdr;        		
        	}
        	else {
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         20,19,0,0x7ffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,3,0,0x00380000LL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         54,10,0,0xffc00000LL);
        	}
        	break;
        default:
        		/* relocation of address as data */
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
                          0,31,0,0x7fffffffLL);
					;
      	}
      }
    }
    else if (btype != BASE_NONE) {
illreloc:
      general_error(38);  /* illegal relocation */
    }
  }
  else {
     if (reloctype == REL_PC) {
       /* a relative reference to an absolute label */
			TRACE("n");
       return val-pc;
     }
  }

	TRACE("m");
  return val;
}


static void encode_reg(uint64_t* insn, operand *op, mnemonic* mnemo, int i)
{
	if (insn) {
		switch(mnemo->ext.format) {
		case BFR3IR:
			if (i==0)
				*insn = *insn| (RT(op->basereg & 0x3f));
			else if (i==1)
				*insn = *insn| (RA(op->basereg & regmask));
			if (i==3)			
				*insn = *insn| (RC(op->basereg & regmask));
			break;
		case R1:
		case BFR3II:
		case RI6:
			if (i==0)
				*insn = *insn| (RT(op->basereg));
			else if (i==1)
				*insn = *insn| (RA(op->basereg));
			break;
		case MV:
		case CSR:
		case R2:
		case R3RI:
		case BFR3RI:
		case SHIFTI:
			if (i==0)
				*insn = *insn| (RT(op->basereg));
			else if (i==1)
				*insn = *insn| (RA(op->basereg));
			else if (i==2)
				*insn = *insn| (RB(op->basereg));
			break;
		case R4:
		case R3:
		case R3RR:
			if (i==0)
				*insn = *insn| (RT(op->basereg));
			else if (i==1)
				*insn = *insn| (RA(op->basereg));
			else if (i==2)
				*insn = *insn| (RB(op->basereg));
			else if (i==3)
				*insn = *insn| (RC(op->basereg));
			break;
		case B:
		case J:
			if (i==0)
				*insn = *insn| (RT(op->basereg));
			else if (i==1)
				*insn = *insn| (RA(op->basereg ));
			break;
		case B3:
		case J3:
			if (i==0)
				*insn = *insn| (RA(op->basereg ));
			break;
		case BL:
		case JL:
			if (i==1)
				*insn = *insn| (RA(op->basereg ));
			else if (i==2)
				*insn = *insn| (RB(op->basereg ));
			else if (i==3)
				*insn = *insn| (RC(op->basereg ));
			break;
		case BL3:
		case JL3:
			if (i==1)
				*insn = *insn| (RA(op->basereg ));
			break;
		case REGIND:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			else if (i==1)
				*insn = *insn| (RA(op->basereg ));
			break;
		case SCNDX:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			else if (i==1)
				*insn = *insn| (RA(op->basereg ));
			else if (i==2)
				*insn = *insn| (RB(op->basereg ));
			break;
		case VRI:
		case RI:
		case RIL:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			else if (i==1)
				*insn = *insn| (RA(op->basereg ));
			break;
		case DIRECT:
		case VDIRECT:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			break;
		}				
	}
}

/* Encode a direct addresss */
static size_t encode_direct (
	uint64_t *postfix,
	uint64_t *insn,
	mnemonic* mnemo,
	taddr val, int constexpr)
{
	size_t isize;

	isize = 6;
	if (constexpr) {
		if (!is_nbit(val,19LL)) {
			if (postfix)
				*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
			isize = (6<<4)|6;
		}
		if (insn) {
			*insn = *insn | ((val & 0x7ffffLL) << 20LL);
		}
	}
	/* If not a constant expression then the value may be unknown. Allow for the
		 largest value given the number of address bits in use.
	*/
	else {
		if (abits <= 19LL)
			return (isize);
		// One postfix
		if (postfix)
			*postfix = OP(0x10);
		return ((6<<4)|6);
	}
	return (isize);
}

/* Encode a vector direct addresss */
static size_t encode_vdirect (
	uint64_t *postfix,
	uint64_t *insn,
	mnemonic* mnemo,
	taddr val, int constexpr)
{
	return (encode_direct(postfix,insn,mnemo,val,constexpr));
}

static size_t encode_r2 (
	uint64_t *insn,
	int i,
	taddr val,
	int constexpr)
{
	size_t isize;

	isize = 6;
	if (constexpr) {
		switch(i) {
		case 2:
			if (insn) {
				*insn = *insn | RB(val);
			}
			break;
		case 3:
			if (insn) {
				*insn = *insn | ((val & 1LL) << 33LL);
			}
			break;
		}
		return (isize);
	}
	if (insn)
		*insn = *insn | RB(val);
	return (isize);
}

static size_t encode_r3 (
	uint64_t *insn,
	int i,
	taddr val)
{
	size_t isize;

	isize = 6;
	if (i==2) {
		if (insn)
			*insn = *insn | RB(val);
	}
	else if (i==3)
		if (insn)
			*insn = *insn | RC(val);
	return (isize);
}

static size_t encode_shifti (
	uint64_t *insn,
	int i,
	taddr val)
{
	size_t isize;

	isize = 6;
	if (insn)
		*insn = *insn | RB(val);
	return (isize);
}

static size_t encode_ri6 (
	uint64_t *insn,
	int i,
	taddr val)
{
	size_t isize;

	isize = 6;
	switch(i) {
	case 2:
		if (insn)
			*insn = *insn | RB(val);
		break;
	case 3:
		if (insn)
			*insn = *insn | ((val & 0x1LL) << 33);
		break;
	}
	return (isize);
}

static size_t encode_vri (
	uint64_t *postfix,
	uint64_t *insn,
	mnemonic* mnemo,
	taddr val,
	int constexpr)
{
	size_t isize;

	isize = 5;
	if (constexpr) {
		if (!is_nbit(val,19LL)) {
			if (postfix)
				*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
			isize = (6<<4)|6;
		}
		if (insn) {
			*insn = *insn | ((val & 0x1ffffLL) << 20LL);
		}
		return (isize);
	}
	if (!is_nbit(val,19LL))
		return (isize);
	if (insn) {
		*insn = *insn | ((val & 0x1ffffLL) << 20LL);
	}
	return (isize);
}

static size_t encode_default (
	uint64_t *postfix,
	uint64_t *insn,
	mnemonic* mnemo,
	operand* op,
	taddr val,
	int constexpr)
{
	size_t isize;

	isize = 6;
	if (constexpr) {
		if (op->type & OP_IMM)
			isize = 6;
		if (!is_nbit(val,19LL)) {
			if (postfix)
				*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
			isize = (6<<4)|6;
		}
		if (insn) {
			*insn = *insn | ((val & 0x7ffffLL) << 20LL);
		}
		return (isize);
	}
	if (op->type & OP_IMM) {
		if (abits <= 19LL)
			;
		else
		{
			if (postfix)
				*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
			isize = (6<<4)|6;
		}
		if (insn) {
			*insn = *insn | ((val & 0x7ffffLL) << 20LL);
		}
	}
	return (isize);
}

static size_t encode_immed(uint64_t *postfix, uint64_t *insn, mnemonic* mnemo,
	operand *op, taddr hval, int constexpr, int i, char vector)
{
	int64_t val;

	if (mnemo->ext.flags & FLG_NEGIMM)
		hval = -hval;	/* ToDo: check here for value overflow */
	val = hval;
	switch(mnemo->ext.format) {
	case R2:
		return (encode_r2(insn,i,val,constexpr));
	case R3:
		return (encode_r3(insn,i,val));
	case DIRECT:
		return (encode_direct(postfix,insn,mnemo,hval,constexpr));
	case VDIRECT:
		return (encode_vdirect(postfix,insn,mnemo,hval,constexpr));
	case SHIFTI:
		return (encode_shifti(insn,i,val));
	case RI6:
		return (encode_ri6(insn,i,val));
	case VRI:
		return (encode_vri(postfix,insn,mnemo,hval,constexpr));
	default:
		return (encode_default(postfix,insn,mnemo,op,hval,constexpr));
	}
	return (0);	/* Cannot get here */
}

/* Evaluate branch operands excepting GPRs which are handled earlier.
	Returns 1 if the branch was processed, 0 if illegal branch format.
*/
static int encode_branch(uint64_t* insn, mnemonic* mnemo, operand* op, int64_t val, int* isize, int i)
{
	if (isize)
		*isize = 6;

	TRACE("evb:");
	switch(mnemo->ext.format) {

	case B:
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 1:
					*insn |= RT(val & 0x3fLL);
					break;
				case 2:
		  		uint64_t tgt;
		  		tgt = ((val & 0x1ffffLL) << 22LL);
		  		*insn |= tgt;
			  	break;
				}
			}
	  	return (1);
		}
		break;

	case BL:
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 2:
					*insn |= RB(val>>2)|((val & 3LL) << 12);
					break;
				case 3:
			  	if (insn) {
			  		uint64_t tgt;
			  		tgt = ((val & 0x1ffffLL) << 22LL);
			  		*insn |= tgt;
			  	}
			  	break;
				}
			}
	  	return (1);
		}
		break;

	case J:
		if (op->type == OP_IMM) {
	  	if (insn) {
	  		switch(i) {
	  		case 1:
					*insn |= RB(val>>2)|((val & 3LL) << 12);
					break;
				case 2:
		  		uint64_t tgt;
		  		tgt = ((val & 0x1ffffLL) << 22LL);
		  		*insn |= tgt;
		  		break;
	  		}
	  	}
	  	return (1);
		}
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= RC(op->basereg & 0x1f);
	  		tgt = (((val >> 1LL) & 0x7ffffLL) << 26LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case JL:
		if (op->type == OP_IMM) {
	  	if (insn) {
	  		switch(i) {
	  		case 2:
					*insn |= RB(val>>2)|((val & 3LL) << 12);
					break;
				case 3:
		  		uint64_t tgt;
		  		tgt = ((val & 0x1ffffLL) << 22LL);
		  		*insn |= tgt;
		  		break;
	  		}
	  	}
	  	return (1);
		}
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= RC(op->basereg & 0x1f);
		  		tgt = ((val & 0x1ffffLL) << 20LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case B2:
	  if (op->type==OP_IMM) {
	  	if (insn) {
	  		uint64_t tgt;
	  		tgt = ((val & 0xfffffffffLL) << 12LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case J2:
	  if (op->type==OP_IMM) {
	  	if (insn) {
	  		uint64_t tgt;
	  		//*insn |= CA(mnemo->ext.format==B2 ? 0x7 : 0x0);
	  		tgt = (val & 0xfffffffffLL) << 12;
	  		*insn |= tgt;
	  	}
	  	return (1);
		}
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= RC(op->basereg & 0x1f);
	    		tgt = (((val >> 1LL) & 0x1fffLL) << 11LL) | (((val >> 14LL) & 0x7ffffLL) << 26LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case BL2:
  	if (op->type==OP_IMM) {
	  	if (insn) {
    		uint64_t tgt;
	  		tgt = (val & 0xfffffffffLL) << 12;
    		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case JL2:
  	if (op->type==OP_IMM) {
	  	if (insn) {
    		uint64_t tgt;
	  		tgt = (val & 0xfffffffffLL) << 12;
    		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		tgt = (val & 0xfffffffffLL) << 12;
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case B3:
		*isize = 4;
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 1:
			  	if (insn) {
			  		uint64_t tgt;
			  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
			  		*insn |= tgt;
			  	}
			  	break;
				}
			}
	  	return (1);
		}
		break;

	case BL3:
		*isize = 5;
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 2:
			  	if (insn) {
			  		uint64_t tgt;
			  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
			  		*insn |= tgt;
			  	}
			  	break;
				}
			}
	  	return (1);
		}
		break;

	case J3:
		*isize = 5;
		/*
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= CA(op->basereg & 0x7);
	  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
		else 
		*/
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 1:
			  	if (insn) {
			  		uint64_t tgt;
			  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
			  		*insn |= tgt;
			  	}
			  	break;
				}
			}
	  	return (1);
		}
		break;

	case JL3:
		*isize = 6;
		/*
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= CA(op->basereg & 0x7);
	  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
		else
		*/
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 2:
			  	if (insn) {
			  		uint64_t tgt;
			  		tgt = (((val >> 1LL) & 0x1fLL) << 9LL) | (((val >> 6LL) & 0x1fffLL) << 19LL);
			  		*insn |= tgt;
			  	}
			  	break;
				}
			}
	  	return (1);
		}
		break;

  }
  TRACE("ebv0:");
  return (0);
}

/* evaluate expressions and try to optimize instruction,
   return size of instruction 

   Since the instruction may contain a postfix which varies in size, both the
   size of the instruction and the size of the postfix is returned. The size
   of the instruction is in byte 0 of the return value, the size of the 
   postfix is in byte 1. The total size may be calculated using a simple
   shift and sum.
*/
size_t encode_rfPhoenix_operands(instruction *ip,section *sec,taddr pc,
                     uint64_t *postfix, uint64_t *insn, dblock *db)
{
  mnemonic *mnemo = &mnemonics[ip->code];
  size_t isize;
  int i;
  operand op;
	int constexpr;
	int reg = 0;
	char vector_insn = 0;
	char opsz;				// operand size
	int64_t op1val;

	TRACE("Eto:");
	opsz = 't';
	if (ip->qualifiers[0])
		opsz = ip->qualifiers[0][0];

	isize = mnemo->ext.len;
  if (insn != NULL) {
    *insn = mnemo->ext.opcode;
   }

	if (postfix)
		*postfix = 0;

	// Detect a vector instruction
  for (i=0; i<MAX_OPERANDS && ip->op[i]!=NULL; i++) {
  	if (ip->op[i]->type==OP_VREG) {
  		vector_insn = 1;
  		break;
  	}
	}

  for (i=0; i<MAX_OPERANDS && ip->op[i]!=NULL; i++) {
    operand *pop;
    int reloctype;
    taddr hval;
    int64_t val;

		TRACE("F");
    op = *(ip->op[i]);
    /* reflect the format back into the operand */
    ip->op[i]->number = i;
    op.number = i;
    op.format = mnemo->ext.format;

      /* special case: operand omitted and use this operand's type + 1
         for the next operand */
    /*
    if (op.type == NEXT) {
      op = *(ip->op[++i]);
      op.type = mnemo->operand_type[i-1] + 1;
    }
	*/
		constexpr = 1;
    if ((reloctype = get_reloc_type(&op)) != REL_NONE) {
      if (db != NULL) {
        val = make_reloc(reloctype,&op,sec,pc,&db->relocs,&constexpr);
      }
      else {
        if (!eval_expr(op.value,&val,sec,pc)) {
          if (reloctype == REL_PC)
//          	hval = hsub(huge_zero(),pc);
						val -= pc;
        }
      }
    }
    else {
      if (!eval_expr(op.value,&val,sec,pc))
        if (insn != NULL) {
/*	    	printf("***A4 val:%lld****", val);
          cpu_error(2);  */ /* constant integer expression required */
        }
    }

		if (i==1) {
			op1val = val;
		}

		TRACE("Ethof:");
    if (db!=NULL && op.type==OP_REGIND && op.attr==REL_NONE) {
			TRACE("Ethof1:");
      if (op.basereg == sdreg) {  /* is it a small data reference? */
				TRACE("Ethof3:");
        fix_reloctype(db,REL_SD);
/*        else if (reg == sd2reg)*/  /* EABI small data 2 */
/*          fix_reloctype(db,REL_PPCEABI_SDA2); */
			}
    }

		TRACE("Etho2:");
		if (op.type==OP_REG || op.type==OP_VMREG) {
			encode_reg(insn, &op, mnemo, i);
		}
		else if (mnemo->operand_type[i]==OP_LK) {
			if (insn) {
 				switch(mnemo->ext.format) {
 				case JL:
 				case JL2:
 				case JL3:
 				case BL:
 				case BL2:
 				case BL3:
 				case RTS:
 					if (i==0)
 						*insn = *insn| RT(op.basereg & 0x3);
 					break;
				default:
 					cpu_error(18);
				}				
			}
		}
    else if (((mnemo->operand_type[i])&OP_IMM) && (op.type==OP_IMM) && !is_branch(mnemo)) {
			TRACE("Etho3:");
			isize = encode_immed(postfix, insn, mnemo, &op, val, constexpr, i, vector_insn);
    }
    else if (encode_branch(insn, mnemo, &op, val, &isize, i)) {
			TRACE("Etho4:");
    	;
    }
    else if ((mnemo->operand_type[i]&OP_REGIND) && op.type==OP_REGIND) {
			TRACE("Etho5:");
    	/* Check for short form */
    	if (constexpr) {
//	    	if (0 && is_nbit(val,8) && (mnemo->ext.opcode==0x86LL || mnemo->ext.opcode==0x93LL)) {
	    		{
		    		isize = 6;
		    		if (insn) {
			    		if (i==0)
			    			*insn |= (RT(op.basereg));
			    		else if (i==1) {
			    			*insn |= (RA(op.basereg));
			    			*insn |= (val & 0x1ffffLL) << 20LL;
			    		}
		    		}
		    	}
	    		if (!is_nbit(val,19) && abits > 19) {
	    			if (postfix)
							*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
						isize = (6<<4)|6;
					}
  		}
  		else {
    		if (insn) {
	    		if (i==0)
	    			*insn |= (RT(op.basereg));
	    		else if (i==1) {
	    			*insn |= (RA(op.basereg));
	    			*insn |= (val & 0x1ffffLL) << 20LL;
	    		}
    		}
    		if (!is_nbit(val,19) && abits > 19) {
    			if (postfix)
						*postfix = OP(0x10 | ((val >> 19LL) & 7LL)) | ((val >> 22LL) << 6LL);
					isize = (6<<4)|6;
				}
  		}
    }
    else if ((mnemo->operand_type[i]&OP_SCNDX) && op.type==OP_SCNDX) {
			TRACE("Etho6:");
    	isize = 4;
  		if (insn) {
  			*insn |= (RA(op.basereg));
  			/* *insn |= ((val & 0xffLL) << 21LL); */
  			*insn |= RB(op.ndxreg);
  		}
    }
	}
	
	// Set instruction size bits
	
	if (insn) {
		switch(opsz) {
		case 'b':
		case 'B':
			*insn &= 0xfc7FFFFFFFFFLL;
			break;
		case 'w':
		case 'W':
			*insn &= 0xfc7FFFFFFFFFLL;
			*insn |= 0x008000000000LL;
			break;
		case 't':
		case 'T':
			*insn &= 0xfc7FFFFFFFFFLL;
			*insn |= 0x010000000000LL;
			break;
		case 'o':
		case 'O':
			*insn &= 0xfc7FFFFFFFFFLL;
			*insn |= 0x018000000000LL;
			break;
		case 'h':
		case 'H':
			*insn &= 0xfc7FFFFFFFFFLL;
			*insn |= 0x020000000000LL;
			break;
		}
	}
	
	
	TRACE("G");
	return (isize);
}

/* Calculate the size of the current instruction; must be identical
   to the data created by eval_instruction. */
size_t instruction_size(instruction *ip,section *sec,taddr pc)
{
	uint64_t bc;

	TRACE("+instruction_size\n");
	size_t sz = encode_rfPhoenix_operands(ip,sec,pc,NULL,NULL,NULL);
	sz = (sz & 0xf) + ((sz >> 4) & 0xf) + (sz >> 8);
	bc = (pc & 0x3fLL) + sz;
	if (bc > 63LL && !span_cache_lines)
		sz = sz + (64LL-(pc & 0x3fLL));
	insn_sizes1[sz1ndx++] = sz;
	TRACE2("isize=%d ", sz);
  return (sz);
}


/* Convert an instruction into a DATA atom including relocations,
   when necessary. */
/* ToDo:
		There are lots of instructions that would fit into four bytes at the end of
		a cache line. Anything with a constant value where the high byte is zero will
		fit into the four byte space. But for now if the instruction and postfixes
		would span cache lines we simply output a four byte NOP.
*/
dblock *eval_instruction(instruction *ip,section *sec,taddr pc)
{
  dblock *db = new_dblock();
  unsigned char *d;
  uint64_t postfix;
  uint64_t insn;
  uint64_t bc;
  size_t sz;
  int toobig = 0;
  uint64_t insn_ol;		// Instructions on line

	TRACE("+eval_instruction\n");
	postfix = 0;
	sz = encode_rfPhoenix_operands(ip,sec,pc,&postfix,&insn,db);
	db->size = (sz & 0xf) + ((sz >> 4) & 0xf) + (sz >> 8);
	/* Is anything being added to the instruction stream? */
	if (db->size) {
		/* Include padding space if instruction plus postfixes will not fit on
		 	cache line. */
		bc = (pc & 0x3fLL) + db->size;
		if (bc > 63LL) {
			toobig = !span_cache_lines;
			if (toobig)
				db->size = db->size + (64LL-(pc & 0x3fLL));
		}
		d = db->data = mymalloc(db->size);
		/* Will it fit on the cache line? If it is too big fill the remainder of
			 the cache line with NOP bytes and set the number of instructions on the
			 line. */
		if (toobig) {
			bc = pc & 63LL;
			switch(bc) {
			case 45:	insn_ol = 0x00; break;	/* 10 instructions on line */
			case 50:	insn_ol = 0x40; break;	/* 11 instructions on line */
			case 55:	insn_ol = 0x80; break;	/* 12 instructions on line */
			case 60:	insn_ol = 0xC0; break;	/* 13 instructions on line */
			default:  insn_ol = 0;
			}
			while (bc < 64LL) {
				if (bc==63LL)
					d = setval(0,d,(size_t)1,(uint64_t)insn_ol);
				else
		   		d = setval(0,d,(size_t)1,(uint64_t)11);	/* NOP instruction */
				bc++;
			}
		}
		insn_sizes2[sz2ndx++] = db->size;
		if ((sz >> 4LL) & 0xfLL) {
	    d = setval(0,d,(sz >> 4LL) & 0xfLL,insn);
	    d = setval(0,d,sz & 0xfLL,postfix);
	    insn_count+=2;
	  }
	  else {
    	d = setval(0,d,sz & 0xfLL,insn);
    	insn_count++;
    }
    byte_count += db->size;
  }
  return (db);
}


/* Create a dblock (with relocs, if necessary) for size bits of data. */
dblock *eval_data(operand *op,size_t bitsize,section *sec,taddr pc)
{
  dblock *db = new_dblock();
  taddr val;
  tfloat flt;
  int constexpr = 1;

	TRACE("+eval_data\n");
  if ((bitsize & 7) || bitsize > 64)
    cpu_error(9,bitsize);  /* data size not supported */
  /*
	if (!OP_DATAM(op->type))
  	ierror(0);
	*/
  db->size = bitsize >> 3;
  db->data = mymalloc(db->size);

  if (type_of_expr(op->value) == FLT) {
    if (!eval_expr_float(op->value,&flt))
      general_error(60);  /* cannot evaluate floating point */

    switch (bitsize) {
      case 32:
        conv2ieee32(0,db->data,flt);
        break;
      case 64:
        conv2ieee64(0,db->data,flt);
        break;
      default:
        cpu_error(10);  /* data has illegal type */
        break;
    }
  }
  else {
    val = make_reloc(get_reloc_type(op),op,sec,pc,&db->relocs,&constexpr);

    switch (db->size) {
      case 1:
        db->data[0] = val & 0xff;
        break;
      case 2:
      case 4:
      case 8:
        setval(0,db->data,db->size,val);
        break;
      default:
        ierror(0);
        break;
    }
  }

  return db;
}

/* return true, if initialization was successfull */
int init_cpu()
{
	TRACE("+init_cpu");
	insn_count = 0;
	byte_count = 0;
  return 1;
}

/* return true, if the passed argument is understood */
int cpu_args(char *p)
{
	TRACE("+cpu_args");
  abits = 32;
  if (strncmp(p, "-abits=", 7)==0) {
  	abits = atoi(&p[7]);
  	if (abits < 16)
  		abits = 16;
  	else if (abits > 64)
  		abits = 64;
  	return (1);
  }
  return (0);
}

static taddr read_sdreg(char **s,taddr def)
{
  expr *tree;
  taddr val = def;

  *s = skip(*s);
  tree = parse_expr(s);
  simplify_expr(tree);
  if (tree->type==NUM && tree->c.val>=0 && tree->c.val<=63)
    val = tree->c.val;
  else
    cpu_error(13);  /* not a valid register */
  free_expr(tree);
  return val;
}


/* parse cpu-specific directives; return pointer to end of
   cpu-specific text */
char *parse_cpu_special(char *start)
{
  char *name=start,*s=start;

	TRACE("+parse_special");
  if (ISIDSTART(*s)) {
    s++;
    while (ISIDCHAR(*s))
      s++;
    if (s-name==6 && !strncmp(name,".sdreg",6)) {
      sdreg = read_sdreg(&s,sdreg);
      return s;
    }
    else if (s-name==7 && !strncmp(name,".sd2reg",7)) {
      sd2reg = read_sdreg(&s,sd2reg);
      return s;
    }
    else if (s-name==7 && !strncmp(name,".sd3reg",7)) {
      sd3reg = read_sdreg(&s,sd3reg);
      return s;
    }
  }
  return start;
}

/* To be inserted at the end of main() for debugging */

void at_end()
{
	int lmt = sz1ndx > sz2ndx ? sz2ndx : sz1ndx;
	int ndx;

	printf("Instructions: %d\n", insn_count);
	printf("Bytes: %d\n", byte_count);
	if (insn_count > 0)
		printf("%f bytes per instruction\n", (double)(byte_count)/(double)(insn_count));
	/*
	for (ndx = 0; ndx < lmt; ndx++) {
		printf("%csz1=%d, sz2=%d\n", insn_sizes1[ndx]!=insn_sizes2[ndx] ? '*' : ' ', insn_sizes1[ndx], insn_sizes2[ndx]);
	}
	*/
}
