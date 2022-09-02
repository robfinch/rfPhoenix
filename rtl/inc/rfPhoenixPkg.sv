package rfPhoenixPkg;

parameter NLANES = 16;
// The following thread count carefully choosen.
// It cannot be over 13 as that makes the vector register file too big for
// synthesis to handle.
// It also needs to be greater than the latency of the icache read (5).
parameter NTHREADS = 6;
parameter NREGS = 64;
parameter REB_ENTRIES = 4;

parameter RSTIP	= 32'hFFFD0000;

parameter BRK			= 6'h00;
parameter PFX			= 6'h01;
parameter R2			= 6'h02;
parameter ADDI		= 6'h04;
parameter SUBFI		= 6'h05;
parameter MULI		= 6'h06;
parameter CSR			= 6'h07;
parameter ANDI		= 6'h08;
parameter ORI			= 6'h09;
parameter XORI		= 6'h0A;
parameter NOP			= 6'h0B;
parameter CMPI		= 6'h0D;
parameter CMP_EQI	= 6'h0E;
parameter CMP_NEI	= 6'h0F;
parameter CMP_LTI	= 6'h10;
parameter CMP_GEI	= 6'h11;
parameter CMP_LEI	= 6'h12;
parameter CMP_GTI	= 6'h13;
parameter CMP_LTUI	= 6'h14;
parameter CMP_GEUI	= 6'h15;
parameter CMP_LEUI	= 6'h16;
parameter CMP_GTUI	= 6'h17;
parameter CALLA		= 6'h18;
parameter CALLR		= 6'h19;
parameter JMP			= 6'h1A;
parameter BRA			= 6'h1B;
parameter Bcc			= 6'h1C;
parameter FBcc		= 6'h1D;
parameter FCMP_EQI	= 6'h1E;
parameter FCMP_NEI	= 6'h1F;
parameter FCMP_LTI	= 6'h24;
parameter FCMP_GEI	= 6'h25;
parameter FCMP_LEI	= 6'h26;
parameter FCMP_GTI	= 6'h27;
parameter FMA 		= 6'h2C;
parameter FMS 		= 6'h2D;
parameter FNMA		= 6'h2E;
parameter FNMS 		= 6'h2F;
parameter LDB			= 6'h30;
parameter LDBU		= 6'h31;
parameter LDW			= 6'h32;
parameter LDWU		= 6'h33;
parameter LDT			= 6'h34;
parameter LDR			= 6'h35;
parameter STB			= 6'h38;
parameter STW			= 6'h39;
parameter STT			= 6'h3A;
parameter STC			= 6'h3B;

// R2 ops
parameter R1			= 6'h01;
parameter VSHUF		= 6'h02;
parameter VEX			= 6'h03;
parameter ADD			= 6'h04;
parameter SUB			= 6'h05;
parameter TLBRW		= 6'h07;
parameter AND			= 6'h08;
parameter OR			= 6'h09;
parameter XOR			= 6'h0A;
parameter VEINS		= 6'h0C;
parameter CMP			= 6'h0D;
parameter CMP_EQ	= 6'h0E;
parameter CMP_NE	= 6'h0F;
parameter CMP_LT	= 6'h10;
parameter CMP_GE	= 6'h11;
parameter CMP_LE	= 6'h12;
parameter CMP_GT	= 6'h13;
parameter CMP_LTU	= 6'h14;
parameter CMP_GEU	= 6'h15;
parameter CMP_LEU	= 6'h16;
parameter CMP_GTU	= 6'h17;
parameter SLLI		= 6'h18;
parameter SRLI		= 6'h19;
parameter SRAI		= 6'h1A;
parameter SLL			= 6'h1B;
parameter SRL			= 6'h1C;
parameter SRA			= 6'h1D;
parameter FCMP_EQ	= 6'h1E;
parameter FCMP_NE	= 6'h1F;
parameter FCMP_LT	= 6'h24;
parameter FCMP_GE	= 6'h25;
parameter FCMP_LE	= 6'h26;
parameter FCMP_GT	= 6'h27;
parameter VSLLVI	= 6'h20;
parameter VSRLVI	= 6'h21;
parameter VSLLV		= 6'h22;
parameter VSRLV		= 6'h23;
parameter SHPTENDX	= 6'h28;
parameter FADD		= 6'h2C;
parameter FSUB		= 6'h2D;
parameter FMUL		= 6'h2E;
parameter LDBX		= 6'h30;
parameter LDBUX		= 6'h31;
parameter LDWX		= 6'h32;
parameter LDWUX		= 6'h33;
parameter LDTX		= 6'h34;
parameter LDRX		= 6'h35;
parameter STBX		= 6'h38;
parameter STWX		= 6'h39;
parameter STTX		= 6'h3A;
parameter STCX		= 6'h3B;

// R1 ops
parameter CNTLZ		= 6'h00;
parameter CNTPOP	= 6'h02;
parameter PTGHASH	= 6'h07;
parameter RTI			= 6'h19;
parameter REX			= 6'h1A;
parameter FFINITE = 6'h20;
parameter FNEG		= 6'h23;
parameter FRSQRTE	= 6'h24;
parameter FRES		= 6'h25;
parameter FSIGMOID= 6'h26;
parameter I2F			= 6'h28;
parameter F2I			= 6'h29;
parameter FABS		= 6'h2A;
parameter FNABS		= 6'h2B;
parameter FCLASS	= 6'h2C;
parameter FMAN		= 6'h2D;
parameter FSIGN		= 6'h2E;
parameter FTRUNC	= 6'h2F;

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
parameter CSR_MPLSTACK	= 16'h303F;
parameter CSR_MPMSTACK	= 16'h3040;
parameter CSR_MSTUFF0	= 16'h3042;
parameter CSR_MSTUFF1	= 16'h3043;
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

parameter FLT_NONE	= 8'h00;
parameter FLT_TLBMISS = 8'h04;
parameter FLT_DCM		= 8'h05;
parameter FLT_IADR	= 8'h22;
parameter FLT_CHK		= 8'h27;
parameter FLT_DBZ		= 8'h28;
parameter FLT_OFL		= 8'h29;
parameter FLT_ALN		= 8'h30;
parameter FLT_KEY		= 8'h31;
parameter FLT_WRV		= 8'h32;
parameter FLT_RDV		= 8'h33;
parameter FLT_SGB		= 8'h34;
parameter FLT_PRIV	= 8'h35;
parameter FLT_WD		= 8'h36;
parameter FLT_UNIMP	= 8'h37;
parameter FLT_CPF		= 8'h39;
parameter FLT_DPF		= 8'h3A;
parameter FLT_LVL		= 8'h3B;
parameter FLT_PMA		= 8'h3D;
parameter FLT_BRK		= 8'h3F;
parameter FLT_PFX		= 8'hC8;
parameter FLT_TMR		= 8'hE2;
parameter FLT_RTI		= 8'hED;
parameter FLT_IRQ		= 8'hEE;
parameter FLT_NMI		= 8'hFE;

parameter pL1CacheLines = 64;
parameter pL1LineSize = 512;
parameter pL1ICacheLines = 512;
parameter pL1ICacheLineSize = 640;
localparam pL1Imsb = $clog2(pL1ICacheLines-1)-1+6;

parameter nul = 3'd0;
parameter byt = 3'd1;
parameter wyde = 3'd2;
parameter tetra = 3'd3;
parameter octa = 3'd4;
parameter vect = 3'd5;

typedef logic [11:0] CauseCode;
typedef logic [3:0] Tid;
typedef logic [9:0] ASID;
typedef logic [31:0] Address;
typedef logic [31:0] VirtualAddress;
typedef logic [31:0] PhysicalAddress;
typedef logic [31:0] CodeAddress;
typedef logic [31:0] Value;
typedef logic [63:0] DoubleValue;
typedef Value [15:0] VecValue;
typedef logic [5:0] Opcode;
typedef logic [5:0] Func;

typedef struct packed
{
	logic vec;
	logic [5:0] num;
} Regspec;

// Instruction types

typedef struct packed
{
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
	logic [2:0] mask;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} f3inst;

typedef struct packed
{
	logic m;
	logic [2:0] pad;
	Func func;
	Regspec Rb;
	logic [2:0] mask;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} r2inst;

typedef struct packed
{
	logic m;
	logic [15:0] imm;
	logic [2:0] mask;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} imminst;

typedef struct packed
{
	logic m;
	logic [1:0] func;
	logic [13:0] imm;
	logic [2:0] mask;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} csrinst;

typedef struct packed
{
	logic m;
	logic [15:0] disp;
	logic [2:0] mask;
	Regspec Ra;
	Regspec Rt;
	Opcode opcode;
} lsinst;

typedef struct packed
{
	logic [16:0] disp;
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
	Tid thread;
	Regspec Ra;
	Regspec Rb;
	Regspec Rc;
	Regspec Rm;
	Regspec Rt;
	logic Tt;
	logic hasRa;
	logic hasRb;
	logic hasRc;
	logic hasRm;
	logic hasRt;
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
	logic ldr;
	logic storer;
	logic storen;
	logic store;
	logic stc;
	logic [2:0] memsz;
	logic br;						// conditional branch
	logic cjb;					// call, jmp, or bra
	logic brk;
	logic irq;
	logic rti;
	logic flt;
	logic pfx;
} DecodeBus;

typedef struct packed
{
	logic v;
	logic decoded;
	logic regfetched;
	logic out;
	logic executed;
	logic memory;
	logic imiss;
	Tid thread;
	InstructionFetchbuf ifb;
	DecodeBus	dec;
	logic [3:0] count;
	logic [3:0] step;
	CauseCode cause;
	Address badAddr;
	VecValue a;
	VecValue b;
	VecValue c;
	VecValue t;
	Value mask;
	VecValue res;
} ExecuteBuffer;

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
parameter MR_LDDESC = 4'd12;
parameter MR_STB	= 4'd0;
parameter MR_STW	= 4'd1;
parameter MR_STT	= 4'd2;
parameter MR_STO	= 4'd3;
parameter MR_STC	= 4'd4;
parameter MR_STOO	= 4'd5;
parameter MR_STH	= 4'd7;
parameter MR_STHP	= 4'd8;
parameter MR_STPTR	= 4'd9;

typedef struct packed
{
	logic [7:0] tid;		// tran id
	logic [3:0] thread;	// 
	logic [1:0] omode;	// operating mode
	CodeAddress ip;			// Debubgging aid
	logic [5:0] step;		// vector operation step
	logic [5:0] count;	// vector operation count
	logic wr;
	memop_t func;				// operation to perform
	logic [3:0] func2;	// more resolution to function
	logic v;
	ASID asid;
	Address adr;
	CodeAddress vcadr;		// victim cache address
	logic [639:0] dat;	// 512+128 for icache line
	logic [3:0] sz;		// indicates size of data
	logic [63:0] sel;
	logic [3:0] acr;		// acr bits from TLB lookup
	Regspec tgt;				// target register
} MemoryRequest;

// All the fields in this structure are *output* back to the system.
typedef struct packed
{
	logic [7:0] tid;		// tran id
	logic [3:0] thread;
	logic [1:0] omode;	// operating mode
	CodeAddress ip;			// Debugging aid
	logic [5:0] step;
	logic wr;
	memop_t func;				// operation to perform
	logic [3:0] func2;	// more resolution to function
	logic v;
	logic empty;
	CauseCode cause;
	logic [63:0] sel;
	ASID asid;
	Address badAddr;
	CodeAddress vcadr;		// victim cache address
	logic [1023:0] res;
	Value dat;
	logic dchit;
	logic cmt;
	logic [3:0] sz;		// indicates size of data
	logic [3:0] acr;		// acr bits from TLB lookup
	logic tlb_access;
	logic ptgram_en;
	logic rgn_en;
	logic pmtram_ena;
	Regspec tgt;				// target register
} MemoryResponse;		//

endpackage
