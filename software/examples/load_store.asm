# load store test rfPhoenix asm

	.bss
_bss_a:
	.space	10

	.data
_data_a:
	.space	10

#	.org	0xFFFFFFFFFFFD0000
	.text
	.align	1
start:
	LDI		r2,0x1234
	STORE	r2,0xFFFC0000
	NOP
	LOAD	r3,0xFFFC0000
	CMP		r4,r2,r3
	LDI		r5,0x5678
	STORE	r5,0xFFFC0004
	NOP
	LOAD	r6,0xFFFC0004
	LDI		r1,0
	LDI		r2,16
loop1:
	STORE	r1,0xFFFC0000[r1]
	ADD		r1,r1,1
	ADD		r2,r2,-1
	BNE		r2,r0,loop1
	LDI		r1,0x0
	LDI		r2,16
loop2:
	LOAD	r3,0xFFFC0000[r1]
	ADD		r1,r1,1
	CMP		r4,r3,r1
	AND		r4,r4,1
	BEQ		r4,r0,mismatch
	ADD		r2,r2,-1
	BNE		r2,r0,loop2
mismatch:
	BRA		mismatch

	.balign	0x100,0x0B

