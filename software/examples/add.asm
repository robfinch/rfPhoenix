# add test rfPhoenix asm

	.bss
	.space	10

	.data
	.space	10

#	.org	0xFFFFFFFFFFFD0000
	.text
	.align	1
start:
	LDI		r2,0x1234
	ADD		r1,r2,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	ADD		r1,r1,1
	LDI		r2,10
loop1:
	ADD		r1,r1,1
	ADD		r2,r2,-1
	BNE		r2,r0,loop1
mismatch:
	BRA		mismatch

	.balign	0x100,0x0B

