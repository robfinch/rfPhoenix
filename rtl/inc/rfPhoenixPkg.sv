package rfPhoenixPkg;

`undef IS_SIM
//`define IS_SIM	1

`define TRUE	1
`define FALSE	0
`define VAL		1
`define INV		0

// Comment out to remove the sigmoid approximate function
//`define SIGMOID	1

parameter TRUE = `TRUE;
parameter FALSE = `FALSE;
parameter VAL = `VAL;
parameter INV = `INV;

`define SUPPORT_16BIT_OPS		1
`define SUPPORT_128BIT_OPS	1
`define NLANES	8
`define NTHREADS	4
`define NREGS		64

`define L1CacheLines	512
`define L1CacheLineSize		512

`define L1ICacheLineSize	584
`define L1ICacheLines	512
`define L1ICacheWays 4

`define L1DCacheWays 4

parameter  NLANES = `NLANES;
// The following thread count carefully choosen.
// It cannot be over 13 as that makes the vector register file too big for
// synthesis to handle.
parameter NTHREADS = `NTHREADS;
parameter NREGS = `NREGS;

parameter pL1CacheLines = `L1CacheLines;
parameter pL1LineSize = `L1CacheLineSize;
parameter pL1ICacheLines = `L1CacheLines;
// The following arrived at as 512+32 bits for word at end of cache line, plus
// 40 bits for a possible constant postfix
parameter pL1ICacheLineSize = `L1ICacheLineSize;
parameter pL1Imsb = $clog2(`L1ICacheLines-1)-1+6;
parameter pL1ICacheWays = `L1ICacheWays;
parameter pL1DCacheWays = `L1DCacheWays;
parameter TidMSB = $clog2(`NTHREADS)-1;

parameter RAS_DEPTH	= 4;

typedef enum logic [5:0] {
	OP_BRK			= 6'h00,
	OP_PFX			= 6'h01,
	OP_R2				= 6'h02,
	OP_ADDI			= 6'h04,
	OP_SUBFI		= 6'h05,
	OP_MULI			= 6'h06,
	OP_CSR			= 6'h07,
	OP_ANDI			= 6'h08,
	OP_ORI			= 6'h09,
	OP_XORI			= 6'h0A,
	OP_NOP			= 6'h0B,
	OP_CMPI			= 6'h0D,
	OP_CMP_EQI	= 6'h0E,
	OP_CMP_NEI	= 6'h0F,
	OP_CMP_LTI	= 6'h10,
	OP_CMP_GEI	= 6'h11,
	OP_CMP_LEI	= 6'h12,
	OP_CMP_GTI	= 6'h13,
	OP_CMP_LTUI	= 6'h14,
	OP_CMP_GEUI	= 6'h15,
	OP_CMP_LEUI	= 6'h16,
	OP_CMP_GTUI	= 6'h17,
	OP_CALL			= 6'h18,
	OP_BSR			= 6'h19,
	OP_RET			= 6'h1A,
	OP_Bcc			= 6'h1C,
	OP_FBcc			= 6'h1D,
	OP_FCMP_EQI	= 6'h1E,
	OP_FCMP_NEI	= 6'h1F,
	OP_FMA16		= 6'h20,
	OP_FMS16		= 6'h21,
	OP_FNMA16		= 6'h22,
	OP_FNMS16		= 6'h23,
	OP_FCMP_LTI	= 6'h24,
	OP_FCMP_GEI	= 6'h25,
	OP_FCMP_LEI	= 6'h26,
	OP_FCMP_GTI	= 6'h27,
	OP_FMA128		= 6'h28,
	OP_FMS128		= 6'h29,
	OP_FNMA128	= 6'h2A,
	OP_FNMS128	= 6'h2B,
	OP_FMA 			= 6'h2C,
	OP_FMS 			= 6'h2D,
	OP_FNMA			= 6'h2E,
	OP_FNMS 		= 6'h2F,
	OP_LDB			= 6'h30,
	OP_LDBU			= 6'h31,
	OP_LDW			= 6'h32,
	OP_LDWU			= 6'h33,
	OP_LDT			= 6'h34,
	OP_LDC			= 6'h35,
	OP_LDSR			= 6'h36,
	OP_STB			= 6'h38,
	OP_STW			= 6'h39,
	OP_STT			= 6'h3A,
	OP_STC			= 6'h3B,
	OP_STCR			= 6'h3D
} opcode_t;

typedef enum logic [2:0] {
	BLT		= 3'd0,
	BGE		= 3'd1,
	BLTU	= 3'd2,
	BGEU	= 3'd3,
	BBS		= 3'd5,
	BEQ		= 3'd6,
	BNE		= 3'd7
	/*
	FBEQ	= 3'd0,
	FBNE	= 3'd1,
	FBLT	= 3'd2,
	FBLE	= 3'd3,
	FBGT	= 3'd4,
	FBGE	= 3'd5
	*/
} branch_cnd_t;

// R2 ops
typedef enum logic [5:0] {
	OP_R1			= 6'h01,
	OP_VSHUF		= 6'h02,
	OP_VEX			= 6'h03,
	OP_ADD			= 6'h04,
	OP_SUB			= 6'h05,
	OP_MUL			= 6'h06,
	OP_TLBRW		= 6'h07,
	OP_AND			= 6'h08,
	OP_OR			= 6'h09,
	OP_XOR			= 6'h0A,
	OP_VEINS		= 6'h0C,
	OP_CMP			= 6'h0D,
	OP_CMP_EQ	= 6'h0E,
	OP_CMP_NE	= 6'h0F,
	OP_CMP_LT	= 6'h10,
	OP_CMP_GE	= 6'h11,
	OP_CMP_LE	= 6'h12,
	OP_CMP_GT	= 6'h13,
	OP_CMP_LTU	= 6'h14,
	OP_CMP_GEU	= 6'h15,
	OP_CMP_LEU	= 6'h16,
	OP_CMP_GTU	= 6'h17,
	OP_SLLI		= 6'h18,
	OP_SRLI		= 6'h19,
	OP_SRAI		= 6'h1A,
	OP_SLL			= 6'h1B,
	OP_SRL			= 6'h1C,
	OP_SRA			= 6'h1D,
	OP_FCMP_EQ	= 6'h1E,
	OP_FCMP_NE	= 6'h1F,
	OP_VSLLVI	= 6'h20,
	OP_VSRLVI	= 6'h21,
	OP_VSLLV		= 6'h22,
	OP_VSRLV		= 6'h23,
	OP_FCMP_LT	= 6'h24,
	OP_FCMP_GE	= 6'h25,
	OP_FCMP_LE	= 6'h26,
	OP_FCMP_GT	= 6'h27,
	OP_FADD128	= 6'h28,
	OP_FSUB128	= 6'h29,
	OP_SHPTENDX	= 6'h2A,
	OP_FADD		= 6'h2C,
	OP_FSUB		= 6'h2D,
	OP_REMASK		= 6'h2E,
//	OP_FMUL		= 6'h2E,
	OP_PUSHQ		= 6'h2F,
	OP_LDBX		= 6'h30,
	OP_LDBUX		= 6'h31,
	OP_LDWX		= 6'h32,
	OP_LDWUX		= 6'h33,
	OP_LDTX		= 6'h34,
	OP_LDCX		= 6'h35,
	OP_LDSRX		= 6'h36,
	OP_STBX		= 6'h38,
	OP_STWX		= 6'h39,
	OP_STTX		= 6'h3A,
	OP_STCX		= 6'h3B,
	OP_STCRX		= 6'h3D
} r2func_t;

// R1 ops
typedef enum logic [5:0] {
	OP_CNTLZ		= 6'h00,
	OP_CNTPOP		= 6'h02,
	OP_PTGHASH	= 6'h07,
	OP_PEEKQ		= 6'h08,
	OP_POPQ			= 6'h09,
	OP_STATQ		= 6'h0B,
	OP_RESETQ 	= 6'h0C,
	OP_RTI			= 6'h19,
	OP_REX			= 6'h1A,
	OP_FFINITE 	= 6'h20,
	OP_FNEG			= 6'h23,
	OP_FRSQRTE	= 6'h24,
	OP_FRES			= 6'h25,
	OP_FSIGMOID	= 6'h26,
	OP_I2F			= 6'h28,
	OP_F2I			= 6'h29,
	OP_FABS			= 6'h2A,
	OP_FNABS		= 6'h2B,
	OP_FCLASS		= 6'h2C,
	OP_FMAN			= 6'h2D,
	OP_FSIGN		= 6'h2E,
	OP_FTRUNC		= 6'h2F
} r1func_t;

typedef enum logic [1:0] {
	PRC16 = 2'd0,
	PRC32 = 2'd1,
	PRC128 = 2'd2
} prec_t;

parameter NOP_INSN	= {34'd0,OP_NOP};

typedef enum logic [3:0] {
	MR_NOP = 4'd0,
	MR_LOAD = 4'd1,
	MR_LOADZ = 4'd2,
	MR_STORE = 4'd3,
	MR_TLBRD = 4'd4,
	MR_TLBRW = 4'd5,
	MR_TLB = 4'd6,
	MR_LEA = 4'd7,
	MR_MOVLD = 4'd8,
	MR_MOVST = 4'd9,
	MR_RGN = 4'd10,
	MR_ICACHE_LOAD = 4'd11,
	MR_PTG = 4'd12,
	MR_CACHE = 4'd13
} memop_t;

parameter CSR_IE		= 16'h?004;
parameter CSR_CAUSE	= 16'h?006;
parameter CSR_SEMA	= 16'h?00C;
parameter CSR_PTBR	= 16'h1003;
parameter CSR_HMASK	= 16'h1005;
parameter CSR_FSTAT	= 16'h?014;
parameter CSR_ASID	= 16'h101F;
parameter CSR_KEYS	= 16'b00010000001000??;
parameter CSR_KEYTBL= 16'h1024;
parameter CSR_SCRATCH=16'h?041;
parameter CSR_MCR0	= 16'h3000;
parameter CSR_MHARTID = 16'h3001;
parameter CSR_TICK	= 16'h3002;
parameter CSR_MBADADDR	= 16'h3007;
parameter CSR_MTVEC = 16'b00110000001100??;
parameter CSR_MDBAD	= 16'b00110000000110??;
parameter CSR_MDBAM	= 16'b00110000000111??;
parameter CSR_MDBCR	= 16'h3020;
parameter CSR_MDBSR	= 16'h3021;
parameter CSR_MPLSTACK	= 16'h303F;
parameter CSR_MPMSTACK	= 16'h3040;
parameter CSR_MSTUFF0	= 16'h3042;
parameter CSR_MSTUFF1	= 16'h3043;
parameter CSR_USTATUS	= 16'h0044;
parameter CSR_SSTATUS	= 16'h1044;
parameter CSR_HSTATUS	= 16'h2044;
parameter CSR_MSTATUS	= 16'h3044;
parameter CSR_MVSTEP= 16'h3046;
parameter CSR_MVTMP	= 16'h3047;
parameter CSR_MEIP	=	16'h3048;
parameter CSR_MECS	= 16'h3049;
parameter CSR_MPCS	= 16'h304A;
parameter CSR_UCA		=	16'b00000001000?????;
parameter CSR_SCA		=	16'b00010001000?????;
parameter CSR_HCA		=	16'b00100001000?????;
parameter CSR_MCA		=	16'b00110001000?????;
parameter CSR_MSEL	= 16'b0011010000100???;
parameter CSR_MTCBPTR=16'h3050;
parameter CSR_MGDT	= 16'h3051;
parameter CSR_MLDT	= 16'h3052;
parameter CSR_MTCB	= 16'h3054;
parameter CSR_MBVEC	= 16'b0011000001011???;
parameter CSR_MSP		= 16'h3060;
parameter CSR_TIME	= 16'h?FE0;
parameter CSR_MTIME	= 16'h3FE0;
parameter CSR_MTIMECMP	= 16'h3FE1;

typedef enum logic [11:0] {
	FLT_NONE	= 12'h000,
	FLT_TLBMISS = 12'h04,
	FLT_DCM		= 12'h005,
	FLT_SSM		= 12'h020,
	FLT_DBG		= 12'h021,
	FLT_IADR	= 12'h022,
	FLT_CHK		= 12'h027,
	FLT_DBZ		= 12'h028,
	FLT_OFL		= 12'h029,
	FLT_ALN		= 12'h030,
	FLT_KEY		= 12'h031,
	FLT_WRV		= 12'h032,
	FLT_RDV		= 12'h033,
	FLT_SGB		= 12'h034,
	FLT_PRIV	= 12'h035,
	FLT_WD		= 12'h036,
	FLT_UNIMP	= 12'h037,
	FLT_CPF		= 12'h039,
	FLT_DPF		= 12'h03A,
	FLT_LVL		= 12'h03B,
	FLT_PMA		= 12'h03D,
	FLT_BRK		= 12'h03F,
	FLT_PFX		= 12'h0C8,
	FLT_TMR		= 12'h0E2,
	FLT_RTI		= 12'h0ED,
	FLT_IRQ		= 12'h8EE,
	FLT_NMI		= 12'h8FE
} cause_code_t;

typedef enum logic [2:0] {
	nul = 3'd0,
	byt = 3'd1,
	wyde = 3'd2,
	tetra = 3'd3,
	octa = 3'd4,
	vect = 3'd5
} memsz_t;

typedef logic [TidMSB:0] Tid;
typedef logic [11:0] order_tag_t;
typedef logic [9:0] ASID;
typedef logic [31:0] Address;
typedef logic [31:0] VirtualAddress;
typedef logic [31:0] PhysicalAddress;
typedef logic [31:0] code_address_t;
typedef logic [31:0] Value;
typedef logic [31:0] value_t;
typedef logic [63:0] DoubleValue;
typedef logic [15:0] half_value_t;
typedef logic [127:0] quad_value_t;
typedef half_value_t [NLANES*2-1:0] vector_half_value_t;
typedef quad_value_t [NLANES/4-1:0] vector_quad_value_t;
typedef Value [NLANES-1:0] VecValue;
typedef logic [5:0] Func;
typedef logic [127:0] regs_bitmap_t;

typedef struct packed
{
	logic vec;					// 1=vector register
	logic [5:0] num;
} regspec_t;

typedef struct packed
{
	logic [7:0] pl;			// privilege level
	logic [5:0] resv3;
	logic mprv;					// memory access priv indicator	
	logic [4:0] resv2;
	logic [1:0] om;			// operating mode
	logic trace_en;			// instruction trace enable
	logic ssm;					// single step mode
	logic [2:0] ipl;		// interrupt privilege level
	logic die;					// debug interrupt enable
	logic mie;					// machine interrupt enable
	logic hie;					// hypervisor interrupt enable
	logic sie;					// supervisor interrupt enable
	logic uie;					// user interrupt enable
} status_reg_t;				// 32 bits

// Instruction types, makes decoding easier

typedef struct packed
{
	logic [15:0] pad2;
	logic [15:0] imm;
	logic [1:0] pad;
	opcode_t opcode;
} Postfix;

typedef struct packed
{
	logic [33:0] payload;
	opcode_t opcode;
} anyinst_t;

typedef struct packed
{
	logic [2:0] rm;
	regspec_t Rc;
	regspec_t Rb;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} f3inst_t;

typedef struct packed
{
	logic [2:0] rm;
	logic resv;
	r2func_t func;
	regspec_t Rb;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} r2inst_t;

typedef struct packed
{
	logic [2:0] rm;
	logic resv;
	r2func_t func;
	logic Tb;
	r1func_t func1;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} r1inst_t;

typedef struct packed
{
	logic resv;
	logic [15:0] imm;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} imminst_t;

typedef struct packed
{
	logic resv;
	logic [1:0] func;
	logic [13:0] imm;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} csrinst_t;

typedef struct packed
{
	logic resv;
	logic [15:0] disp;
	logic [2:0] Rm;
	regspec_t Ra;
	regspec_t Rt;
	opcode_t opcode;
} lsinst_t;

typedef struct packed
{
	logic resv;
	logic [15:0] disp;
	branch_cnd_t cnd;	
	regspec_t	Ra;
	regspec_t Rb;
	opcode_t opcode;
} brinst_t;

typedef struct packed
{
	logic [31:0] target;
	logic [1:0] Rt;
	opcode_t opcode;
} callinst_t;

typedef struct packed
{
	logic [33:0] cnst;
	opcode_t opcode;
} pfxinst_t;

typedef union packed
{
	f3inst_t 	f3;
	r2inst_t	r2;
	r1inst_t	r1;
	brinst_t	br;
	callinst_t	call;
	callinst_t	jmp;
	imminst_t	imm;
	imminst_t	ri;
	csrinst_t	csr;
	lsinst_t	ls;
	r2inst_t	lsn;
	pfxinst_t	pfx;
	anyinst_t any;
} instruction_t;

typedef struct packed
{
	Tid thread;
	logic v;
	order_tag_t tag;
	code_address_t ip;
	instruction_t insn;
	Postfix pfx;
	Postfix pfx2;
	cause_code_t cause;
	logic [2:0] sp_sel;
} InstructionFetchbuf;

typedef struct packed
{
	logic v;
	regspec_t Ra;
	regspec_t Rb;
	regspec_t Rc;
	regspec_t Rm;
	regspec_t Rt;
	logic Ta;
	logic Tb;
	logic Tt;
	logic hasRa;
	logic hasRb;
	logic hasRc;
	logic hasRm;
	logic hasRt;
	logic Rtsrc;	// Rt is a source register
	logic [31:0] imm;
	prec_t prc;
	logic rfwr;
	logic vrfwr;
	logic csr;
	logic csrrd;
	logic csrrw;
	logic csrrs;
	logic csrrc;
	logic is_vector;
	logic multicycle;
	logic mem;
	logic loadr;
	logic loadn;
	logic load;
	logic loadu;
	logic ldsr;
	logic storer;
	logic storen;
	logic store;
	logic stcr;
	logic need_steps;
	logic compress;
	memsz_t memsz;
	logic br;						// conditional branch
	logic cjb;					// call, jmp, or bra
	logic brk;
	logic irq;
	logic rti;
	logic flt;
	logic rex;
	logic pfx;
	logic popq;
} DecodeBus;

typedef struct packed
{
	logic v;
	logic regfetched;
	logic out;
	logic agen;
	logic executed;
	logic memory;
	logic imiss;
	Tid thread;
	InstructionFetchbuf ifb;
	DecodeBus	dec;
	logic [3:0] count;
	logic [3:0] step;
	logic [2:0] retry;		// retry count
	cause_code_t cause;
	Address badAddr;
	VecValue a;
	VecValue b;
	VecValue c;
	VecValue t;
	Value mask;
	VecValue res;
} pipeline_reg_t;

typedef struct packed {
	logic [4:0] imiss;
	logic sleep;
	code_address_t ip;				// current instruction pointer
	code_address_t miss_ip;	// I$ miss address
} ThreadInfo_t;

typedef struct packed {
	logic loaded;						// 1=loaded internally
	logic stored;						// 1=stored externally
	code_address_t ip;					// return address
	Address sp;							// Stack pointer location
} return_stack_t;

// No unsigned codes!
parameter MR_LDB	= 4'd0;
parameter MR_LDW	= 4'd1;
parameter MR_LDT	= 4'd2;
parameter MR_LDO	= 4'd3;
parameter MR_LDR	= 4'd4;
parameter MR_LDOB	= 4'd5;
parameter MR_LDOO = 4'd6;
parameter MR_LDH	= 4'd7;
parameter MR_LDHP = 4'd8;
parameter MR_LDV	= 4'd9;
parameter MR_LDG	= 4'd10;
parameter MR_LDPTG = 4'd0;
parameter MR_STPTG = 4'd1;
parameter MR_RAS 	= 4'd12;
parameter MR_STB	= 4'd0;
parameter MR_STW	= 4'd1;
parameter MR_STT	= 4'd2;
parameter MR_STO	= 4'd3;
parameter MR_STC	= 4'd4;
parameter MR_STOO	= 4'd5;
parameter MR_STH	= 4'd7;
parameter MR_STHP	= 4'd8;
parameter MR_STPTR	= 4'd9;

// All the fields in this structure are *output* back to the system.
typedef struct packed
{
	logic [7:0] tid;		// tran id
	order_tag_t tag;
	Tid thread;
	logic [1:0] omode;	// operating mode
	code_address_t ip;			// Debugging aid
	logic [5:0] step;		// vector step number
	logic [5:0] count;	// vector operation count
	logic wr;
	memop_t func;				// operation to perform
	logic [3:0] func2;	// more resolution to function
	logic load;					// needed to place results
	logic store;
	logic need_steps;
	logic v;
	logic empty;
	cause_code_t cause;
	logic [127:0] sel;
	ASID asid;
	Address adr;
	code_address_t vcadr;		// victim cache address
	logic [1023:0] res;		// stores unaligned data as well
	logic dchit;
	logic cmt;
	memsz_t sz;					// indicates size of data
	logic [1:0] hit;
	logic [1:0] mod;		// line modified indicators
	logic [3:0] acr;		// acr bits from TLB lookup
	logic tlb_access;
	logic ptgram_en;
	logic rgn_en;
	logic pmtram_ena;
	logic wr_tgt;
	regspec_t tgt;				// target register
} MemoryArg_t;		//

// The full pipeline structure is not needed for writeback. The writeback fifos
// can be made smaller using a smaller structure.
// Ah, but it appears that writeback needs some of the instruction buffer.
// To support a few instructions like RTI and REX.
/*
typedef struct packed
{
	logic v;
	order_tag_t tag;
	cause_code_t cause;		// cause code
	code_address_t ip;		// address of instruction
	Address adr;					// bad load/store address
	logic [5:0] step;			// vector step number
	logic [1023:0] res;		// instruction results
	logic wr_tgt;					// target register needs updating
	regspec_t tgt;				// target register
} writeback_info_t;
*/

const code_address_t RSTIP	= 32'hFFFD0000;

endpackage
