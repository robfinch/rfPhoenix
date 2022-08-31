# Fibonacci calculator Thor2021 asm
# r1 in the end will hold the Nth fibonacci number

	.bss
_bss_a:
	.space	10

	.data
_data_a:
	.space	10

#	.org	0xFFFFFFFFFFFE0000
	.text
	.align	1
start:
	AND		r0,r0,0				# set r0 to zero
	CSRRD	r2,r0,0x3001	# get the thread number
	AND	r2,r2,3					# 0 to 3
	BNE	r2,r0,stall			# Allow only thread 0 to work

	LDI	r2,0xFD
	LDI	r2,0x01		# x = 1
	STT	r2,0xFFFC0000

	LDI	r3,0x10		# calculates 16th fibonacci number (13 = D in hex) (CHANGE HERE IF YOU WANT TO CALCULATE ANOTHER NUMBER)
	OR	r1,r3,r0	# transfer y register to accumulator
	ADD	r3,r3,-3	# handles the algorithm iteration counting

	LDI	r1,2		# a = 2
	STT	r1,0xFFFC0004		# stores a

floop: 
	LDT	r2,0xFFFC0004		# x = a
	ADD	r1,r1,r2	# a += x
	STT	r1,0xFFFC0004		# stores a
	STT	r2,0xFFFC0000		# stores x
	ADD	r3,r3,-1	# y -= 1
  BNE r3,r0,floop	# jumps back to loop if Z bit != 0 (y's decremention isn't zero yet)
  NOP
  NOP
  NOP
  NOP
  NOP
	NOP  
stall:
	BRA	stall
