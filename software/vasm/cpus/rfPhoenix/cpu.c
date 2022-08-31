#include "vasm.h"

#define TRACE(x)		/*printf(x)*/
#define TRACE2(x,y)	/*printf((x),(y))*/

char *cpu_copyright="vasm rfPhoenix cpu backend (c) in 2022 Robert Finch";

char *cpuname="rfPhoenix";
int bitsperbyte=8;
int bytespertaddr=4;
int abits=32;
static taddr sdreg = 29;
static taddr sd2reg = 28;
static taddr sd3reg = 27;
static __int64 regmask = 0x3fLL;

static insn_count = 0;
static byte_count = 0;

static insn_sizes1[20000];
static insn_sizes2[20000];
static int sz1ndx = 0;
static int sz2ndx = 0;
static short int argregs[14] = {1,2,21,22,23,24,25,26,48,49,50,51,-1,-1};
static short int tmpregs[14] = {3,4,5,6,7,8,9,10,-1,-1,-1,-1,-1,-1};
static short int saved_regs[14] = {11,12,13,14,15,16,17,18,19,20,52,53,54,55};

static char *regnames[64] = {
	"0", "a0", "a1", "t0", "t1", "t2", "t3", "t4",
	"t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4",
	"s5", "s6", "s7", "s8", "s9", "a2", "a3", "a4",
	"a5", "a6", "a7", "gp2", "gp1", "gp0", "fp", "sp",
	"vm0", "vm1", "vm2", "vm3", "vm4", "vm5", "vm6", "vm7",
	"lc", "lr1", "lr2", "r43", "ssp", "hsp", "msp", "isp",
	"t8", "t9", "t10", "t11", "s10", "s11", "s12", "s13",
	"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7"
};

static int regop[64] = {
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, OP_VMREG, 
	OP_REG, OP_LK, OP_LK, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, 
	OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG, OP_REG
};

mnemonic mnemonics[]={
	"abs",	{OP_REG,OP_REG,0,0,0}, {R3RR,CPU_ALL,0,0x0C000001LL,4},

	"add", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(4)|TB(1)|OP(2),5},	
	"add", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(4)|TA(1)|OP(2),5},	
	"add", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(4)|TB(1)|TA(1)|OP(2),5},	
	"add", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(4)|TT(1)|OP(2),5},	
	"add", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(4)|TB(1)|TT(1)|OP(2),5},	
	"add", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(4)|TA(1)|TT(1)|OP(2),5},	
	"add", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(4)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"add", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(4)|OP(2),5},	
	
	"add", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(4),5},	
	"add", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(4),5},	
	"add", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(4),5},	
	"add", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(4),5},	

	"and", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(8)|TB(1)|OP(2),5},	
	"and", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(8)|TA(1)|OP(2),5},	
	"and", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(8)|TB(1)|TA(1)|OP(2),5},	
	"and", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(8)|TT(1)|OP(2),5},	
	"and", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(8)|TB(1)|TT(1)|OP(2),5},	
	"and", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(8)|TA(1)|TT(1)|OP(2),5},	
	"and", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(8)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"and", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(8)|OP(2),5},	
	
	"and", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(8),5},	
	"and", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(8),5},	
	"and", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(8),5},	
	"and", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(8),5},	

//	"bbs",	{OP_LK,OP_REG,OP_IMM,OP_IMM,0}, {BL,CPU_ALL,0,0x00001F000822LL,6},
	"bbs",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(4)|OP(28),5},

	"beq",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(6)|OP(28),5},
	"bge",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(1)|OP(28),5},
	"bgeu",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(3)|OP(28),5},
	"blt",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(0)|OP(28),5},
	"bltu",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(2)|OP(28),5},
	"bne",	{OP_REG,OP_REG|OP_IMM,OP_IMM,0,0}, {B,CPU_ALL,0,CND(7)|OP(28),5},
	
	"bra",	{OP_IMM,0,0,0,0}, {B2,CPU_ALL,0,OP(25),5},

	"brk",	{0,0,0,0,0}, {R2,CPU_ALL,0,RM(1)|TB(0)|RB(1)|TA(1)|RA(63)|TT(1)|RT(63)|OP(32),5},
	"bsr",	{OP_LK,OP_IMM,0,0,0}, {BL2,CPU_ALL,0,OP(25),5},
	"bsr",	{OP_IMM,0,0,0,0}, {B2,CPU_ALL,0,RT(1)|OP(25),5},

	"cmp", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(13)|TB(1)|OP(2),5},	
	"cmp", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(13)|TA(1)|OP(2),5},	
	"cmp", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(13)|TB(1)|TA(1)|OP(2),5},	
	"cmp", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(13)|TT(1)|OP(2),5},	
	"cmp", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(13)|TB(1)|TT(1)|OP(2),5},	
	"cmp", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(13)|TA(1)|TT(1)|OP(2),5},	
	"cmp", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(13)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(13)|OP(2),5},	
	
	"cmp", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(13),5},	
	"cmp", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(13),5},	
	"cmp", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(13),5},	
	"cmp", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(13),5},	

	"cmp_eq", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(14)|TB(1)|OP(2),5},	
	"cmp_eq", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(14)|TA(1)|OP(2),5},	
	"cmp_eq", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(14)|TB(1)|TA(1)|OP(2),5},	
	"cmp_eq", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(14)|TT(1)|OP(2),5},	
	"cmp_eq", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(14)|TB(1)|TT(1)|OP(2),5},	
	"cmp_eq", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(14)|TA(1)|TT(1)|OP(2),5},	
	"cmp_eq", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(14)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_eq", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(14)|OP(2),5},	
	
	"cmp_eq", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(14),5},	
	"cmp_eq", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(14),5},	
	"cmp_eq", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(14),5},	
	"cmp_eq", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(14),5},	

	"cmp_ge", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(17)|TB(1)|OP(2),5},	
	"cmp_ge", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(17)|TA(1)|OP(2),5},	
	"cmp_ge", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(17)|TB(1)|TA(1)|OP(2),5},	
	"cmp_ge", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(17)|TT(1)|OP(2),5},	
	"cmp_ge", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(17)|TB(1)|TT(1)|OP(2),5},	
	"cmp_ge", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(17)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ge", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(17)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ge", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(17)|OP(2),5},	
	
	"cmp_ge", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(17),5},	
	"cmp_ge", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(17),5},	
	"cmp_ge", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(17),5},	
	"cmp_ge", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(17),5},	

	"cmp_geu", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(21)|TB(1)|OP(2),5},	
	"cmp_geu", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(21)|TA(1)|OP(2),5},	
	"cmp_geu", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(21)|TB(1)|TA(1)|OP(2),5},	
	"cmp_geu", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(21)|TT(1)|OP(2),5},	
	"cmp_geu", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(21)|TB(1)|TT(1)|OP(2),5},	
	"cmp_geu", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(21)|TA(1)|TT(1)|OP(2),5},	
	"cmp_geu", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(21)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_geu", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(21)|OP(2),5},	
	
	"cmp_geu", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(21),5},	
	"cmp_geu", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(21),5},	
	"cmp_geu", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(21),5},	
	"cmp_geu", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(21),5},	

	"cmp_gt", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(19)|TB(1)|OP(2),5},	
	"cmp_gt", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(19)|TA(1)|OP(2),5},	
	"cmp_gt", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(19)|TB(1)|TA(1)|OP(2),5},	
	"cmp_gt", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(19)|TT(1)|OP(2),5},	
	"cmp_gt", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(19)|TB(1)|TT(1)|OP(2),5},	
	"cmp_gt", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(19)|TA(1)|TT(1)|OP(2),5},	
	"cmp_gt", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(19)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_gt", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(19)|OP(2),5},	
	
	"cmp_gt", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(19),5},	
	"cmp_gt", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(19),5},	
	"cmp_gt", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(19),5},	
	"cmp_gt", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(19),5},	

	"cmp_gtu", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(23)|TB(1)|OP(2),5},	
	"cmp_gtu", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(23)|TA(1)|OP(2),5},	
	"cmp_gtu", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(23)|TB(1)|TA(1)|OP(2),5},	
	"cmp_gtu", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(23)|TT(1)|OP(2),5},	
	"cmp_gtu", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(23)|TB(1)|TT(1)|OP(2),5},	
	"cmp_gtu", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(23)|TA(1)|TT(1)|OP(2),5},	
	"cmp_gtu", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(23)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_gtu", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(23)|OP(2),5},	
	
	"cmp_gtu", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(23),5},	
	"cmp_gtu", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(23),5},	
	"cmp_gtu", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(23),5},	
	"cmp_gtu", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(23),5},	

	"cmp_le", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(18)|TB(1)|OP(2),5},	
	"cmp_le", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(18)|TA(1)|OP(2),5},	
	"cmp_le", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(18)|TB(1)|TA(1)|OP(2),5},	
	"cmp_le", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(18)|TT(1)|OP(2),5},	
	"cmp_le", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(18)|TB(1)|TT(1)|OP(2),5},	
	"cmp_le", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(18)|TA(1)|TT(1)|OP(2),5},	
	"cmp_le", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(18)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_le", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(18)|OP(2),5},	
	
	"cmp_le", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(18),5},	
	"cmp_le", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(18),5},	
	"cmp_le", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(18),5},	
	"cmp_le", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(18),5},	

	"cmp_leu", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(22)|TB(1)|OP(2),5},	
	"cmp_leu", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(22)|TA(1)|OP(2),5},	
	"cmp_leu", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(22)|TB(1)|TA(1)|OP(2),5},	
	"cmp_leu", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(22)|TT(1)|OP(2),5},	
	"cmp_leu", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(22)|TB(1)|TT(1)|OP(2),5},	
	"cmp_leu", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(22)|TA(1)|TT(1)|OP(2),5},	
	"cmp_leu", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(22)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_leu", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(22)|OP(2),5},	
	
	"cmp_leu", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(22),5},	
	"cmp_leu", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(22),5},	
	"cmp_leu", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(22),5},	
	"cmp_leu", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(22),5},	

	"cmp_lt", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(16)|TB(1)|OP(2),5},	
	"cmp_lt", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(16)|TA(1)|OP(2),5},	
	"cmp_lt", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(16)|TB(1)|TA(1)|OP(2),5},	
	"cmp_lt", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(16)|TT(1)|OP(2),5},	
	"cmp_lt", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(16)|TB(1)|TT(1)|OP(2),5},	
	"cmp_lt", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(16)|TA(1)|TT(1)|OP(2),5},	
	"cmp_lt", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(16)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_lt", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(16)|OP(2),5},	
	
	"cmp_lt", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(16),5},	
	"cmp_lt", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(16),5},	
	"cmp_lt", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(16),5},	
	"cmp_lt", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(16),5},	

	"cmp_ltu", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(20)|TB(1)|OP(2),5},	
	"cmp_ltu", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(20)|TA(1)|OP(2),5},	
	"cmp_ltu", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(20)|TB(1)|TA(1)|OP(2),5},	
	"cmp_ltu", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(20)|TT(1)|OP(2),5},	
	"cmp_ltu", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(20)|TB(1)|TT(1)|OP(2),5},	
	"cmp_ltu", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(20)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ltu", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(20)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ltu", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(20)|OP(2),5},	
	
	"cmp_ltu", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(20),5},	
	"cmp_ltu", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(20),5},	
	"cmp_ltu", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(20),5},	
	"cmp_ltu", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(20),5},	

	"cmp_ne", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(15)|TB(1)|OP(2),5},	
	"cmp_ne", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(15)|TA(1)|OP(2),5},	
	"cmp_ne", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(15)|TB(1)|TA(1)|OP(2),5},	
	"cmp_ne", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(15)|TT(1)|OP(2),5},	
	"cmp_ne", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(15)|TB(1)|TT(1)|OP(2),5},	
	"cmp_ne", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(15)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ne", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(15)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"cmp_ne", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(15)|OP(2),5},	
	
	"cmp_ne", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(15),5},	
	"cmp_ne", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(15),5},	
	"cmp_ne", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(15),5},	
	"cmp_ne", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(15),5},	

	"cntlz", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(0)|TA(1)|TT(1)|OP(2),5},
	"cntlz", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(0)|TA(1)|TT(0)|OP(2),5},
	"cntlz", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(0)|TA(0)|TT(1)|OP(2),5},
	"cntlz", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(0)|TA(0)|TT(0)|OP(2),5},

	"cntpop", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(2)|TA(1)|TT(1)|OP(2),5},
	"cntpop", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(2)|TA(1)|TT(0)|OP(2),5},
	"cntpop", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(2)|TA(0)|TT(1)|OP(2),5},
	"cntpop", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,FN(1)|RB(2)|TA(0)|TT(0)|OP(2),5},

	"com", {OP_REG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,IMM(0xffffLL)|TA(1)|OP(9),5},
	"com", {OP_VREG,OP_REG,0,0,0}, {R1,CPU_ALL,0,IMM(0xffffLL)|TT(1)|OP(9),5},
	"com", {OP_VREG,OP_VREG,0,0,0}, {R1,CPU_ALL,0,IMM(0xffffLL)|TA(1)|TT(1)|OP(9),5},
	"com", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,IMM(0xffffLL)|OP(9),5},

	"csrrc", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,IMM(0x8000LL)|OP(7),5},
	"csrrd", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(7),5},
	"csrrs", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,IMM(0xC000LL)|OP(7),5},
	"csrrw", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,IMM(0x4000LL)|OP(7),5},

	"eor", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|OP(2),5},	
	"eor", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TA(1)|OP(2),5},	
	"eor", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TA(1)|OP(2),5},	
	"eor", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TT(1)|OP(2),5},	
	"eor", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TT(1)|OP(2),5},	
	"eor", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TA(1)|TT(1)|OP(2),5},	
	"eor", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"eor", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(10)|OP(2),5},	
	
	"eor", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(10),5},	
	"eor", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(10),5},	
	"eor", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(10),5},	
	"eor", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(10),5},	

	"jmp",	{OP_IMM,0,0,0,0}, {J2,CPU_ALL,0,OP(24),5},
	"jsr",	{OP_LK,OP_IMM,0,0,0}, {JL2,CPU_ALL,0,OP(24),5},
	"jsr",	{OP_IMM,0,0,0,0}, {J2,CPU_ALL,0,OP(24),5},

	"ldb",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,OP(48),5},	
	"ldb",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,OP(48),5},	
	"ldb",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,FN(48)|OP(2),5},	
	"ldb",	{OP_VREG,OP_SCNDX,OP_VMREG,0,0}, {SCNDX,CPU_ALL,0,FN(48)|OP(2),5},	

/*****************************************/

	"ldbu",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0x81LL,6,0x79,4},	
	"ldbu",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xB1LL,4},	
	"ldbu",	{OP_VREG,OP_SCNDX,OP_VMREG,0,0}, {SCNDX,CPU_ALL,0,0x1B1LL,4},	

//	"ldo",	{OP_REG,OP_SEL|OP_REGIND8,0,0,0}, {REGIND,CPU_ALL,0,0x87LL,4},	
	
	"ldi",  {OP_REG,OP_NEXTREG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(9),5},

	"ldo",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,OP(55),5},
	"ldo",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0x86LL,5},	
	"ldo",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xB6LL,5},

	"ldos",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x7ELL,4,0x7ELL,4},
	"ldos",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x7ELL,4,0x7ELL,4},	
	"ldou",	{OP_REG,OP_SEL|OP_IMM,0,0}, {DIRECT,CPU_ALL,0,0x87LL},
	"ldou",	{OP_REG,OP_SEL|OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0x87LL,6},	
	"ldou",	{OP_REG,OP_SEL|OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xB7LL,4},	
	
	"ldt",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,OP(52),5},	
	"ldt",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,OP(52),5},	
	"ldt",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,FN(52)|OP(2),5},	

	"ldtu",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,OP(53),5},	
	"ldtu",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,OP(53),5},	
	"ldtu",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,FN(53)|OP(2),5},	

	"ldw",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0x82LL,6,0x7A,4},	
	"ldw",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xB2LL,4},	
	"ldwu",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0x83LL,6,0x7B,4},	
	"ldwu",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xB3LL,4},	

	"lea",	{OP_REG,OP_IMM,0,0}, {DIRECT,CPU_ALL,0,0xD4LL,6,0x04,4},
	"lea",	{OP_REG,OP_REGIND,0,0}, {REGIND,CPU_ALL,0,0xD4LL,6,0x04,4},
	"lea",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0x19LL,4},	

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

	"nop",	{0,0,0,0,0}, {BITS16,CPU_ALL,0,0x0B,1},

	"nor", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x020000000102LL,6},	
	"nor", {OP_VREG,OP_VREG,OP_VREG|OP_REG|OP_IMM7,OP_VREG|OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x020000000102LL,6},	
	"nor", {OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x020000000002LL,6},	
	"not", {OP_REG,OP_REG,0,0,0}, {R1,CPU_ALL,0,0x08000001LL,4},

	"or", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(9)|TB(1)|OP(2),5},	
	"or", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(9)|TA(1)|OP(2),5},	
	"or", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(9)|TB(1)|TA(1)|OP(2),5},	
	"or", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(9)|TT(1)|OP(2),5},	
	"or", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(9)|TB(1)|TT(1)|OP(2),5},	
	"or", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(9)|TA(1)|TT(1)|OP(2),5},	
	"or", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(9)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"or", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(9)|OP(2),5},	
	
	"or", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(9),5},	
	"or", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(9),5},	
	"or", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(9),5},	
	"or", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(9),5},	

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
	"ret",	{0,0,0,0,0}, {RTS,CPU_ALL,0,0x02F2LL, 2},

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
	"rts",	{OP_LK,0,0,0,0}, {RTS,CPU_ALL,0,0x00F2LL, 2},
	"rts",	{0,0,0,0,0}, {RTS,CPU_ALL,0,0x02F2LL, 2},

	"sei",	{OP_REG,OP_NEXTREG,OP_REG,0,0},{R3RR,CPU_ALL,0,0x2E0000000007LL,6},	

	"sll",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,OP_VMREG}, {R3,CPU_ALL,0,0x800000000002LL,6},	
	"sll",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_REG|OP_IMM7,0}, {R3,CPU_ALL,0,0x800000000002LL,6},	
	"sll",	{OP_REG,OP_REG,OP_REG|OP_IMM7,OP_VMREG,0}, {R2,CPU_ALL,0,0x58,4},
//	"sll",	{OP_REG,OP_REG,OP_IMM,0,0}, {SHIFTI,CPU_ALL,0,0x800000000002LL,6},
	"sll",	{OP_REG,OP_REG,OP_IMM,0,0}, {RI6,CPU_ALL,0,0x6C,4},
	"sll",	{OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,0x58,4},

	"sllp",	{OP_REG,OP_REG,OP_REG,OP_REG,0}, {R3,CPU_ALL,0,0x800000000002LL,6},	
	"sllp",	{OP_REG,OP_REG,OP_REG,OP_IMM,0}, {R3,CPU_ALL,0,0x800000000002LL,6},	

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

	"stb",	{OP_REG,OP_SEL|OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x90LL,6},	
	"stb",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x90LL,6,0xCALL,4},	
	"stb",	{OP_REG,OP_SEL|OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xC0LL,4},	
	"sth",	{OP_REG,OP_SEL|OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x94LL,6,0x95LL,4},	
	"sth",	{OP_REG,OP_SEL|OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x94LL,6,0x95LL,4},	
	"sth",	{OP_REG,OP_SEL|OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xC4LL,4},	
	"sthp",	{OP_REG,OP_SEL|OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x97LL,6},	
	"sthp",	{OP_REG,OP_SEL|OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x97LL,6},	
	"sthp",	{OP_REG,OP_SEL|OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xC7LL,4},	
	"sths",	{OP_REG,OP_SEL|OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x95LL,4,0x95LL,4},
	"sths",	{OP_REG,OP_SEL|OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x95LL,4,0x95LL,4},	
	"sto",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0x93LL,6,0x95LL,4},	
	"sto",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0x93LL,6,0xCDLL,4},	
	"sto",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,0xC3LL,4},	
	"stos",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,0xCDLL,4,0xCDLL,4},
	"stos",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,0xCDLL,4,0xCDLL,4},	
	"stptg",	{OP_REG,OP_REG,OP_REG,OP_REG,0},{R3RR,CPU_ALL,0,0x4A0000000007LL,6},	

	"stt",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,OP(58),5},	
	"stt",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,OP(58),5},	
	"stt",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,FN(58)|OP(2),5},	

	"stw",	{OP_REG,OP_IMM,0,0,0}, {DIRECT,CPU_ALL,0,OP(57),5},	
	"stw",	{OP_REG,OP_REGIND,0,0,0}, {REGIND,CPU_ALL,0,OP(57),5},	
	"stw",	{OP_REG,OP_SCNDX,0,0,0}, {SCNDX,CPU_ALL,0,FN(57)|OP(2),5},	

	"sub", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(5)|TB(1)|OP(2),5},	
	"sub", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(5)|TA(1)|OP(2),5},	
	"sub", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(5)|TB(1)|TA(1)|OP(2),5},	
	"sub", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(5)|TT(1)|OP(2),5},	
	"sub", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(5)|TB(1)|TT(1)|OP(2),5},	
	"sub", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(5)|TA(1)|TT(1)|OP(2),5},	
	"sub", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(5)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"sub", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(5)|OP(2),5},	

	"subf", {OP_VREG,OP_VREG,OP_IMM,OP_VMREG,0}, {RIL,CPU_ALL,0,0x1D5LL,6},
	"subf", {OP_REG,OP_REG,OP_IMM,0,0}, {RIL,CPU_ALL,0,0xD5LL,6},
/* 0000_1010_0001_0001_1111_0000_0000_0000_0000_0000_AALL */

	"sxb",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A120E0000AALL,6},	
	"sxc",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A121E0000AALL,6},	/* alternate mnemonic for sxw */
	"sxo",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A123E0000AALL,6},
	"sxw",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A121E0000AALL,6},
	"sxt",	{OP_REG,OP_REG,0,0,0}, {R3,CPU_ALL,0,0x0A123E0000AALL,6},

	"sync", {0,0,0,0,0}, {BITS16,CPU_ALL,0,0xF7LL,2},
	"sys",	{OP_IMM,0,0,0,0}, {BITS32,CPU_ALL,0,0xA5,4},

	/* Alternate mnemonic for eor */
	"xor", {OP_REG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|OP(2),5},	
	"xor", {OP_REG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TA(1)|OP(2),5},	
	"xor", {OP_REG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TA(1)|OP(2),5},	
	"xor", {OP_VREG,OP_REG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TT(1)|OP(2),5},	
	"xor", {OP_VREG,OP_REG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TT(1)|OP(2),5},	
	"xor", {OP_VREG,OP_VREG,OP_REG,0,0},	{R2,CPU_ALL,0,FN(10)|TA(1)|TT(1)|OP(2),5},	
	"xor", {OP_VREG,OP_VREG,OP_VREG,0,0},	{R2,CPU_ALL,0,FN(10)|TB(1)|TA(1)|TT(1)|OP(2),5},	
	"xor", {OP_REG,OP_REG,OP_REG,0,0}, {R2,CPU_ALL,0,FN(10)|OP(2),5},	
	
	"xor", {OP_REG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|OP(10),5},	
	"xor", {OP_VREG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,TT(1)|OP(10),5},	
	"xor", {OP_VREG,OP_VREG,OP_IMM,0,0}, {RI,CPU_ALL,0,TA(1)|TT(1)|OP(10),5},	
	"xor", {OP_REG,OP_REG,OP_IMM,0,0}, {RI,CPU_ALL,0,OP(10),5}	

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
  q[0] = "o";
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
				if (!ISIDCHAR((unsigned char)p[3])) {
					if (regnames[nn][3]=='\0') {
						*typ = regop[nn];
						if (ep)
							*ep = &p[3];
						return (nn);
					}
					return (-1);
				}
				if (regnames[nn][3]=='\0')
					return (-1);
				if (regnames[nn][3]==p[3]) {
					if (!ISIDCHAR((unsigned char)p[4])) {
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
	if (*p != 'r' && *p != 'R') {
		return (-1);
	}
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
      	case J:
      	case JL:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                           23,17,0,0x1ffffLL);
          break;
      	/* Unconditional jump */
        case J2:
        case JL2:
		      add_extnreloc_masked(reloclist,base,val,reloctype,
                           8,32,0,0xffffffffLL);
          break;
        case RI:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         23,16,0,0xffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,16,0,0xffff0000LL);
        	break;
        case DIRECT:
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         23,16,0,0xffffLL);
		      add_extnreloc_masked(reloclist,base,addend,reloctype,
                         48,16,0,0xffff0000LL);
        	break;
        case REGIND:
        	if (op->basereg==sdreg) {
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         23,16,0,0xffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,16,0,0xffff0000LL);
	        }
        	else if (op->basereg==sd2reg) {
        		int org_sdr = sdreg;
        		sdreg = sd2reg;
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         23,16,0,0xffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,16,0,0xffff0000LL);
						sdreg = org_sdr;        		
        	}
        	else if (op->basereg==sd3reg) {
        		int org_sdr = sdreg;
        		sdreg = sd3reg;
        		reloctype = REL_SD;
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         23,16,0,0xffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,16,0,0xffff0000LL);
						sdreg = org_sdr;        		
        	}
        	else {
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         23,16,0,0xffffLL);
			      add_extnreloc_masked(reloclist,base,addend,reloctype,
	                         48,16,0,0xffff0000LL);
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
				*insn = *insn| (RA(op->basereg));
			else if (i==1)
				*insn = *insn| (RB(op->basereg ));
			else if (i==2)
				*insn = *insn| (RC(op->basereg ));
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
		case RI:
		case RIL:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			else if (i==1)
				*insn = *insn| (RA(op->basereg ));
			break;
		case DIRECT:
			if (i==0)
				*insn = *insn| (RT(op->basereg ));
			break;
		}				
	}
}

static size_t encode_immed(uint64_t *postfix, uint64_t *postfix2, uint64_t *insn, mnemonic* mnemo,
	operand *op, taddr hval, int constexpr, int i, char vector)
{
	size_t isize;
	int64_t val;

	if (mnemo->ext.flags & FLG_NEGIMM)
		hval = -hval;	/* ToDo: check here for value overflow */
	val = hval;
	if (constexpr) {
		if (mnemo->ext.format==DIRECT) {
			isize = 5;
			/*if (mnemo->ext.short_opcode) {
				if (is_nbit(val,8)) {
					isize = 4;
				}
			} */
			if (!is_nbit(hval,16)) {
				if (postfix)
					*postfix = OP(1) | ((val >> 16LL) << 8);
				isize = (5<<8)|3;
			}
			if (insn) {
				*insn = *insn | ((val & 0xffffLL) << 23LL);
			}
		}
		else if (mnemo->ext.format==R2) {
			isize = 5;
			if (insn) {
				*insn = *insn | RB(val);
			}
		}
		else if (mnemo->ext.format==R3) {
			isize = 5;
			if (i==2) {
				if (insn)
					*insn = *insn | RB(val);
			}
			else if (i==3)
				if (insn)
					*insn = *insn | RC(val);
		}
		else if (mnemo->ext.format==SHIFTI) {
			isize = 5;
			if (insn)
				*insn = *insn | RB(val);
		}
		else if (mnemo->ext.format==RI6) {
			isize = 5;
			if (insn)
				*insn = *insn | RB(val & 0x3fLL);
		}

		else {
			if (op->type & OP_IMM)
			isize = 5;
			if (!is_nbit(hval,16)) {
				if (postfix)
					*postfix = OP(1) | ((val >> 16LL) << 8LL);
				isize = (5<<8)|3;
			}
			if (insn) {
				*insn = *insn | ((val & 0xffffLL) << 23LL);
			}
		}
	}
	else {
		if (mnemo->ext.format==DIRECT) {
			isize = 5;
			goto j2;
j1:
				if (insn) {
					*insn = *insn | ((val & 0xffffLL) << 23LL);
				}
				return (isize);
		}
		if (mnemo->ext.format==SHIFTI) {
			isize = 5;
			if (insn)
				*insn = *insn | RB(val);
		}
		else if (mnemo->ext.format==R2) {
			isize = 5;
				if (insn)
					*insn = *insn | RB(val);
		}
		else {
			if (op->type & OP_IMM) {
				isize = 5;
				if (!is_nbit(val,16))
					goto j2;
				goto j1;
			}
j2:	;
		}
	}
	return (isize);
}

/* Evaluate branch operands excepting GPRs which are handled earlier.
	Returns 1 if the branch was processed, 0 if illegal branch format.
*/
static int encode_branch(uint64_t* insn, mnemonic* mnemo, operand* op, int64_t val, int* isize, int i)
{
	*isize = 5;

	TRACE("evb:");
	switch(mnemo->ext.format) {

	case B:
		if (op->type == OP_IMM) {
			if (insn) {
				switch(i) {
				case 1:
					*insn |= RB(val>>2)|((val & 3LL) << 12);
					break;
				case 2:
		  		uint64_t tgt;
		  		tgt = ((val & 0x1ffffLL) << 23LL);
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
			  		tgt = ((val & 0x1ffffLL) << 23LL);
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
		  		tgt = ((val & 0x1ffffLL) << 23LL);
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
	  		tgt = (((val >> 1LL) & 0x7ffffLL) << 29LL);
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
		  		tgt = ((val & 0x1ffffLL) << 23LL);
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
		  		tgt = ((val & 0x1ffffLL) << 23LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case B2:
	  if (op->type==OP_IMM) {
	  	if (insn) {
	  		uint64_t tgt;
	  		tgt = ((val & 0x1ffffLL) << 23LL);
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
	  		tgt = (val & 0xffffffffLL) << 8;
	  		*insn |= tgt;
	  	}
	  	return (1);
		}
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		*insn |= RC(op->basereg & 0x1f);
	    		tgt = (((val >> 1LL) & 0x1fffLL) << 11LL) | (((val >> 14LL) & 0x7ffffLL) << 29LL);
	  		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case BL2:
  	if (op->type==OP_IMM) {
	  	if (insn) {
    		uint64_t tgt;
	  		tgt = (val & 0xffffffffLL) << 8;
    		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  break;

	case JL2:
  	if (op->type==OP_IMM) {
	  	if (insn) {
    		uint64_t tgt;
	  		tgt = (val & 0xffffffffLL) << 8;
    		*insn |= tgt;
	  	}
	  	return (1);
	  }
	  if (op->type==OP_REGIND) {
	  	if (insn) {
	  		uint64_t tgt;
	  		tgt = (val & 0xffffffffLL) << 8;
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
                     uint64_t *postfix, uint64_t *postfix2, uint64_t *insn, dblock *db)
{
  mnemonic *mnemo = &mnemonics[ip->code];
  size_t isize;
  int i;
  operand op;
	int constexpr;
	int reg = 0;
	char vector_insn = 0;
	int64_t op1val;

	TRACE("Eto:");
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
		if (op.type==OP_REG) {
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
			isize = encode_immed(postfix, postfix2, insn, mnemo, &op, hval, constexpr, i, vector_insn);
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
		    		isize = 5;
		    		if (insn) {
			    		if (i==0)
			    			*insn |= (RT(op.basereg));
			    		else if (i==1) {
			    			*insn |= (RA(op.basereg));
			    			*insn |= (val & 0x1ffffLL) << 23LL;
			    		}
		    		}
		    	}
	    		if (!is_nbit(val,16) && abits > 16) {
	    			if (postfix)
							*postfix = OP(1) | ((val >> 16LL) << 8LL);
						isize = (5<<8)|3;
					}
  		}
  		else {
    		if (insn) {
	    		if (i==0)
	    			*insn |= (RT(op.basereg));
	    		else if (i==1) {
	    			*insn |= (RA(op.basereg));
	    			*insn |= (val & 0x1ffffLL) << 23LL;
	    		}
    		}
    		if (!is_nbit(val,16) && abits > 16) {
    			if (postfix)
						*postfix = OP(1) | ((val >> 16LL) << 8LL);
					isize = (5<<8)|3;
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
	
	TRACE("G");
	return (isize);
}

/* Calculate the size of the current instruction; must be identical
   to the data created by eval_instruction. */
size_t instruction_size(instruction *ip,section *sec,taddr pc)
{
	TRACE("+instruction_size\n");
	size_t sz = encode_rfPhoenix_operands(ip,sec,pc,NULL,NULL,NULL,NULL);
	sz = (sz & 0xff) + ((sz >> 8) & 0xff) + (sz >> 16);
	insn_sizes1[sz1ndx++] = sz;
	TRACE2("isize=%d ", sz);
  return (sz);
}


/* Convert an instruction into a DATA atom including relocations,
   when necessary. */
dblock *eval_instruction(instruction *ip,section *sec,taddr pc)
{
  dblock *db = new_dblock();
  uint64_t postfix, postfix2;
  uint64_t insn;
  size_t sz;

	TRACE("+eval_instruction\n");
	postfix = 0;
	postfix2 = 0;
	sz = encode_rfPhoenix_operands(ip,sec,pc,&postfix,&postfix2,&insn,db);
	db->size = (sz & 0xff) + ((sz >> 8) & 0xff) + (sz >> 16);
	insn_sizes2[sz2ndx++] = db->size;
  if (db->size) {
    unsigned char *d = db->data = mymalloc(db->size);
    int i;

		if ((sz >> 8) & 0xff) {
	    d = setval(0,d,(sz >> 8) & 0xff,insn);
	    d = setval(0,d,sz & 0xff,postfix);
	    insn_count++;
	  }
	  else {
    	d = setval(0,d,sz & 0xff,insn);
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
