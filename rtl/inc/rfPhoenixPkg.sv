package rfPhoenixPkg;

parameter NLANES = 16;
// The following thread count carefully choosen.
// It cannot be over 13 as that makes the vector register file too big for
// synthesis to handle.
// It also needs to be greater than the number of reorder entries (12).
parameter NTHREADS = 16;
parameter NREGS = 64;
parameter REB_ENTRIES = 12;

parameter RSTIP	= 32'hFFFD0000;

parameter BRK			= 6'h00;
parameter PFX			= 6'h01;
parameter R2			= 6'h02;
parameter ADDI		= 6'h04;
parameter SUBFI		= 6'h05;
parameter MULI		= 6'h06;
parameter ANDI		= 6'h08;
parameter ORI			= 6'h09;
parameter XORI		= 6'h0A;
parameter NOP			= 6'h0B;
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
parameter FCMP_EQ	= 6'h1E;
parameter FCMP_NE	= 6'h1F;
parameter FCMP_LT	= 6'h24;
parameter FCMP_GE	= 6'h25;
parameter FCMP_LE	= 6'h26;
parameter FCMP_GT	= 6'h27;
parameter FMA 		= 6'h2C;
parameter FMS 		= 6'h2D;
parameter FNMA		= 6'h2E;
parameter FNMS 		= 6'h2F;
parameter LDB			= 6'h30;
parameter LDBU		= 6'h31;
parameter LDW			= 6'h32;
parameter LDWU		= 6'h33;
parameter LDT			= 6'h34;
parameter LDX			= 6'h37;
parameter STB			= 6'h38;
parameter STW			= 6'h39;
parameter STT			= 6'h3A;
parameter STX			= 6'h3F;

// R2 ops
parameter ADD			= 6'h04;
parameter SUB			= 6'h05;
parameter AND			= 6'h08;
parameter OR			= 6'h09;
parameter XOR			= 6'h0A;

parameter NOP_INSN	= {34'd0,NOP};

parameter MR_LOAD = 4'd0;
parameter MR_STORE = 4'd1;
parameter MR_TLB = 4'd2;
parameter MR_CACHE = 4'd3;
parameter LEA2 = 4'd4;
//parameter RTS2 = 3'd5;
parameter M_JALI	= 4'd5;
parameter M_CALL	= 4'd6;
parameter MR_LOADZ = 4'd7;		// unsigned load
parameter MR_MFSEL = 4'd8;
parameter MR_MTSEL = 4'd9;
parameter MR_MOVLD = 4'd10;
parameter MR_MOVST = 4'd11;
parameter MR_RGN = 4'd12;
parameter MR_PTG = 4'd15;

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
parameter CSR_MTVEC = 16'b0011000000110???;
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
parameter FLT_IADR	= 8'h22;
parameter FLT_CHK		= 8'h27;
parameter FLT_DBZ		= 8'h28;
parameter FLT_OFL		= 8'h29;
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

typedef logic [11:0] CauseCode;
typedef logic [3:0] Tid;
typedef logic [31:0] Address;
typedef logic [31:0] VirtualAddress;
typedef logic [31:0] PhysicalAddress;
typedef logic [31:0] CodeAddress;
typedef logic [31:0] Value;
typedef Value [15:0] VecValue;
typedef logic [5:0] Opcode;
typedef logic [5:0] Func;
typedef logic [5:0] Regspec;

// Instruction types

typedef struct packed
{
	logic [33:0] payload;
	Opcode opcode;
} anyinst;

typedef struct packed
{
	logic m;
	logic [1:0] rm;
	logic Tc;
	Regspec Rc;
	logic Tb;
	Regspec Rb;
	logic [2:0] mask;
	logic Ta;
	Regspec Ra;
	logic Tt;
	Regspec Rt;
	Opcode opcode;
} f3inst;

typedef struct packed
{
	logic m;
	logic [2:0] pad;
	Func func;
	logic Tb;
	Regspec Rb;
	logic [2:0] mask;
	logic Ta;
	Regspec Ra;
	logic Tt;
	Regspec Rt;
	Opcode opcode;
} r2inst;

typedef struct packed
{
	logic m;
	logic [15:0] imm;
	logic [2:0] mask;
	logic Ta;
	Regspec Ra;
	logic Tt;
	Regspec Rt;
	Opcode opcode;
} imminst;

typedef struct packed
{
	logic m;
	logic [15:0] disp;
	logic [2:0] mask;
	logic Ta;
	Regspec Ra;
	logic Tt;
	Regspec Rt;
	Opcode opcode;
} lsinst;

typedef struct packed
{
	logic [16:0] disp;
	logic [2:0] cnd;	
	logic		Ta;
	Regspec	Ra;
	logic		Tb;
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
	lsinst	ls;
	r2inst	lsn;
	pfxinst	pfx;
	anyinst any;
} Instruction;

typedef struct packed
{
	Tid thread;
	Regspec Ra;
	logic Ta;
	Regspec Rb;
	logic Tb;
	Regspec Rc;
	logic Tc;
	Regspec Rt;
	logic Tt;
	Value imm;
	logic rfwr;
	logic vrfwr;
	logic is_vector;
	logic multicycle;
} sDecodeBus;

typedef struct packed
{
	logic v;
	logic decoded;
	logic out;
	logic executed;
	Tid thread;
	CodeAddress ip;
	Instruction ir;
	sDecodeBus	dec;
	VecValue a;
	VecValue b;
	VecValue c;
	VecValue t;
	Value mask;
	VecValue res;
} sReorderEntry;

// No unsigned codes!
parameter MR_LDB	= 4'd0;
parameter MR_LDW	= 4'd1;
parameter MR_LDT	= 4'd2;
parameter MR_LDO	= 4'd3;
parameter MR_LDOR	= 4'd4;
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
parameter MR_STOC	= 4'd4;
parameter MR_STOO	= 4'd5;
parameter MR_STH	= 4'd7;
parameter MR_STHP	= 4'd8;
parameter MR_STPTR	= 4'd9;

typedef struct packed
{
	logic [7:0] tid;		// tran id
	CodeAddress ip;			// Debubgging aid
	logic [5:0] step;		// vector operation step
	logic [5:0] count;	// vector operation count
	logic wr;
	logic [3:0] func;		// function to perform
	logic [3:0] func2;	// more resolution to function
	Address adr;
	logic [511:0] dat;
	logic [3:0] sz;		// indicates size of data
} sMemoryRequest;	// 385

// All the fields in this structure are *output* back to the system.
typedef struct packed
{
	logic [7:0] tid;		// tran id
	CodeAddress ip;			// Debugging aid
	logic [5:0] step;
	logic wr;
	logic [3:0] func;		// function to perform
	logic [3:0] func2;	// more resolution to function
	logic v;
	logic empty;
	CauseCode cause;
	Address badAddr;
	VecValue res;
	logic cmt;
} sMemoryResponse;	// 614

endpackage
