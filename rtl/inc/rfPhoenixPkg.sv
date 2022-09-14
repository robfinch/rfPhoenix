package rfPhoenixPkg;

`undef IS_SIM
`define IS_SIM	1

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

`define NLANES	12
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
	BRK			= 6'h00,
	PFX			= 6'h01,
	R2			= 6'h02,
	ADDI		= 6'h04,
	SUBFI		= 6'h05,
	MULI		= 6'h06,
	CSR			= 6'h07,
	ANDI		= 6'h08,
	ORI			= 6'h09,
	XORI		= 6'h0A,
	NOP			= 6'h0B,
	CMPI		= 6'h0D,
	CMP_EQI	= 6'h0E,
	CMP_NEI	= 6'h0F,
	CMP_LTI	= 6'h10,
	CMP_GEI	= 6'h11,
	CMP_LEI	= 6'h12,
	CMP_GTI	= 6'h13,
	CMP_LTUI	= 6'h14,
	CMP_GEUI	= 6'h15,
	CMP_LEUI	= 6'h16,
	CMP_GTUI	= 6'h17,
	CALL		= 6'h18,
	BSR			= 6'h19,
	RET			= 6'h1A,
	Bcc			= 6'h1C,
	FBcc		= 6'h1D,
	FCMP_EQI	= 6'h1E,
	FCMP_NEI	= 6'h1F,
	FCMP_LTI	= 6'h24,
	FCMP_GEI	= 6'h25,
	FCMP_LEI	= 6'h26,
	FCMP_GTI	= 6'h27,
	FMA 		= 6'h2C,
	FMS 		= 6'h2D,
	FNMA		= 6'h2E,
	FNMS 		= 6'h2F,
	LDB			= 6'h30,
	LDBU		= 6'h31,
	LDW			= 6'h32,
	LDWU		= 6'h33,
	LDT			= 6'h34,
	LDSR		= 6'h36,
	STB			= 6'h38,
	STW			= 6'h39,
	STT			= 6'h3A,
	STCR		= 6'h3D
} Opcode;

// R2 ops
typedef enum logic [5:0] {
	R1			= 6'h01,
	VSHUF		= 6'h02,
	VEX			= 6'h03,
	ADD			= 6'h04,
	SUB			= 6'h05,
	MUL			= 6'h06,
	TLBRW		= 6'h07,
	AND			= 6'h08,
	OR			= 6'h09,
	XOR			= 6'h0A,
	VEINS		= 6'h0C,
	CMP			= 6'h0D,
	CMP_EQ	= 6'h0E,
	CMP_NE	= 6'h0F,
	CMP_LT	= 6'h10,
	CMP_GE	= 6'h11,
	CMP_LE	= 6'h12,
	CMP_GT	= 6'h13,
	CMP_LTU	= 6'h14,
	CMP_GEU	= 6'h15,
	CMP_LEU	= 6'h16,
	CMP_GTU	= 6'h17,
	SLLI		= 6'h18,
	SRLI		= 6'h19,
	SRAI		= 6'h1A,
	SLL			= 6'h1B,
	SRL			= 6'h1C,
	SRA			= 6'h1D,
	FCMP_EQ	= 6'h1E,
	FCMP_NE	= 6'h1F,
	VSLLVI	= 6'h20,
	VSRLVI	= 6'h21,
	VSLLV		= 6'h22,
	VSRLV		= 6'h23,
	FCMP_LT	= 6'h24,
	FCMP_GE	= 6'h25,
	FCMP_LE	= 6'h26,
	FCMP_GT	= 6'h27,
	SHPTENDX	= 6'h28,
	PUSHQ		= 6'h29,
	FADD		= 6'h2C,
	FSUB		= 6'h2D,
	FMUL		= 6'h2E,
	LDBX		= 6'h30,
	LDBUX		= 6'h31,
	LDWX		= 6'h32,
	LDWUX		= 6'h33,
	LDTX		= 6'h34,
	LDSRX		= 6'h36,
	STBX		= 6'h38,
	STWX		= 6'h39,
	STTX		= 6'h3A,
	STCRX		= 6'h3D
} R2Func;

// R1 ops
typedef enum logic [5:0] {
	CNTLZ		= 6'h00,
	CNTPOP	= 6'h02,
	PTGHASH	= 6'h07,
	PEEKQ		= 6'h08,
	POPQ		= 6'h09,
	STATQ		= 6'h0B,
	RESETQ 	= 6'h0C,
	RTI			= 6'h19,
	REX			= 6'h1A,
	FFINITE = 6'h20,
	FNEG		= 6'h23,
	FRSQRTE	= 6'h24,
	FRES		= 6'h25,
	FSIGMOID= 6'h26,
	I2F			= 6'h28,
	F2I			= 6'h29,
	FABS		= 6'h2A,
	FNABS		= 6'h2B,
	FCLASS	= 6'h2C,
	FMAN		= 6'h2D,
	FSIGN		= 6'h2E,
	FTRUNC	= 6'h2F
} R1Func;

parameter NOP_INSN	= {34'd0,NOP};

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
} CauseCode;

typedef enum logic [2:0] {
	nul = 3'd0,
	byt = 3'd1,
	wyde = 3'd2,
	tetra = 3'd3,
	octa = 3'd4,
	vect = 3'd5
} memsz_t;

typedef logic [TidMSB:0] Tid;
typedef logic [9:0] ASID;
typedef logic [31:0] Address;
typedef logic [31:0] VirtualAddress;
typedef logic [31:0] PhysicalAddress;
typedef logic [31:0] CodeAddress;
typedef logic [31:0] Value;
typedef logic [63:0] DoubleValue;
typedef Value [NLANES-1:0] VecValue;
typedef logic [5:0] Func;
typedef logic [127:0] regs_bitmap_t;

typedef struct packed
{
	logic vec;					// 1=vector register
	logic [5:0] num;
} Regspec;

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

// Instruction types

typedef struct packed
{
	logic [15:0] pad2;
	logic [15:0] imm;
	logic [1:0] pad;
	Opcode opcode;
} Postfix;

typedef struct packed
{
	logic [33:0] payload;
	Opcode opcode;
} anyinst;

typedef struct packed
{
	logic m;
	logic [1:0] rm;
	Regspec Rc;
	Regspec Rb;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} f3inst;

typedef struct packed
{
	logic m;
	logic [2:0] pad;
	R2Func func;
	Regspec Rb;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} r2inst;

typedef struct packed
{
	logic m;
	logic [2:0] pad;
	R2Func func;
	logic Tb;
	R1Func func1;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} r1inst;

typedef struct packed
{
	logic m;
	logic [15:0] imm;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} imminst;

typedef struct packed
{
	logic m;
	logic [1:0] func;
	logic [13:0] imm;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} csrinst;

typedef struct packed
{
	logic m;
	logic [15:0] disp;
	logic [2:0] Rm;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} lsinst;

typedef struct packed
{
	logic resv;
	logic [15:0] disp;
	logic [2:0] cnd;	
	Regspec	Ra;
	Regspec Rb;
	Opcode opcode;
} brinst;

typedef struct packed
{
	logic [31:0] target;
	logic [1:0] Rt;
	Opcode opcode;
} callinst;

typedef struct packed
{
	logic [31:0] cnst;
	logic [1:0] sh;
	Opcode opcode;
} pfxinst;

typedef union packed
{
	f3inst 	f3;
	r2inst	r2;
	r1inst	r1;
	brinst	br;
	callinst	call;
	callinst	jmp;
	imminst	imm;
	imminst	ri;
	csrinst	csr;
	lsinst	ls;
	r2inst	lsn;
	pfxinst	pfx;
	anyinst any;
} Instruction;

typedef struct packed
{
	Tid thread;
	logic v;
	CodeAddress ip;
	Instruction insn;
	Postfix pfx;
	CauseCode cause;
	logic [2:0] sp_sel;
} InstructionFetchbuf;

typedef struct packed
{
	logic v;
	Regspec Ra;
	Regspec Rb;
	Regspec Rc;
	Regspec Rm;
	Regspec Rt;
	logic Ta;
	logic Tb;
	logic Tt;
	logic hasRa;
	logic hasRb;
	logic hasRc;
	logic hasRm;
	logic hasRt;
	logic Rtsrc;	// Rt is a source register
	Value imm;
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
	logic decoded;
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
	CauseCode cause;
	Address badAddr;
	VecValue a;
	VecValue b;
	VecValue c;
	VecValue t;
	Value mask;
	VecValue res;
} ExecuteBuffer;

typedef struct packed {
	logic [4:0] imiss;
	logic sleep;
	CodeAddress ip;				// current instruction pointer
	CodeAddress miss_ip;	// I$ miss address
} ThreadInfo_t;

typedef struct packed {
	logic loaded;						// 1=loaded internally
	logic stored;						// 1=stored externally
	CodeAddress ip;					// return address
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
	Tid thread;
	logic [1:0] omode;	// operating mode
	CodeAddress ip;			// Debugging aid
	logic [5:0] step;		// vector step number
	logic [5:0] count;	// vector operation count
	logic wr;
	memop_t func;				// operation to perform
	logic [3:0] func2;	// more resolution to function
	logic v;
	logic empty;
	CauseCode cause;
	logic [127:0] sel;
	ASID asid;
	Address adr;
	CodeAddress vcadr;		// victim cache address
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
	Regspec tgt;				// target register
} MemoryArg_t;		//

const CodeAddress RSTIP	= 32'hFFFD0000;

endpackage
