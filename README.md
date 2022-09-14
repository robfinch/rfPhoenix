# rfPhoenix - *** Under Construction ***

## History
This project taken up in earnest beginning August 24, 2022 after studying the Nyuzi core by Jeff Bush.

## Features
- 32 bit data path
- 8 vector lanes (parameterized), with lane masking capability
- 4 threads of execution (parameterized)
- 64 registers per thread
- unified floating-point and integer register file
- fixed 40 bit instructions
- fused floating-point multiply and add instruction

## Noted characteristics
This core does not use any results forwarding. Instead results forwarding is avoided by using multiple threads of execution.

## Constrast to Nyuzi
The author wanted to apply a slightly different ISA to a Nyuzi style pipeline. This meant a large number of changes to the instruction decoder and some changes to the front end to support immediate postfixes. To date the author has decided to roll his own core rather than modify the Nyuzi.
### Register Read Ports
The rfPhoenix supports operations using requiring three register read ports whereas Nyuzi supports only two. Supporting three ports adds pressure to increase the size of instructions, consequently rfPhoenix has 40-bit instructions compared to Nyuzi's 32.
### Number of Register File Entries
rfPhoenix has a 64-entry register file. The register spec needs an extra bit per register specified then. This also increases the pressure to use a wider instruction format. There is minimal benefit to scalar operations for increasing the register file to 64 entries instead of 32. There may be somewhat more benefit for vector operations.
### Branch Instructions
The rfPhoenix uses fused compare-and-branch instructions whereas the Nyuzi uses a branch zero-non zero approach. The fused compare and branch helps make up some of the loss of code density for a wider instruction since it eliminates compares from the instruction stream.
### Immediates
rfPhoenix uses postfix immediate extensions to extend immediate constants. Nyuzi uses separate load hi instructions and additional registers. rfPhoenix's postfixes help increase code density and do not require using additional registers. The postfix immediate can also be applied to a wide range of instructions. The instruction and postfix are read as a single unit. 

## rfPhoenix Details

### Cache misses
Instruction cache misses do not stall other threads from proceeding. Only the thread that missed stalls until the cache is loaded.

### Pipelining
Floating point operations are now fully pipelined. The FMA requires eight clock cycles to perform.
Some out-of-order execution of instructions is supported. Instructions for a particular thread will be performed in-order. However between threads instructions may execute out of order with respect to the order they were queued.

## Register file
The register file contains 1024 32-bit registers to support 64 registers for 16 threads. Since there are that many vector register which are 512-bits in size,
the register file is quite large. It consumes about 100 block rams. The author wanted a design supporting more than the usual 32 registers. The author would
have preferred 128 registers but there is no easy way to encode that many in the ISA and still keep instructions a reasonable size.

The author would say the registers are general purpose in nature but the typical ABI specifies specific register usages. The first 32 registers are general purpose,
the remaining registers have an assortment of uses including vector mask registers, alternate stack pointers, and link registers.
These other registers are implemented as part of the general register file so that existing instructions may be applied to them.
In a pinch vector mask registers may be used as general purpose registers. There are also eight registers with unassigned usage. They may be used for floating-point for instance.

Vector mask registers are special in that they may be used to mask vector operations and are specified with a four-bit field in the instruction. 

## Instruction Set

### ALU
Basic ALU and shift operations are supported. Integer operations have register-register and register-immediate forms. 

### Floating-point
Fused multiply and add instructions make up most of the floating-point capabilities. Internal to the core, there are no independant add, subtract or multiply operations. FADD, FSUB, and FMUL use the FMA component with values set appropriately.
There is no hardware divide operation. There is a reciprocal estimate function accurate to about eight bits. There is also a reciprocal square-root estimate function.

### Immediates
16-bit immediate values are supported directly in instructions. 32-bit immediates are created using an immediate postfix instruction. The postfix instruction may be applied to extend the addressing range of load and store operations.

### Register Specs
Register specifications are six bit with an additional bit indicating the use of a vector or scaler register.

### Branches
Because the instruction format is wide enough to support it, branches use a compare-and-branch approach. Two registers are compared then a branch takes place according to a selected condition. Comparing branches eliminate compares from the instruction stream. It is really a fused instruction operation.

### Loads and Stores
Gathering loads and scattering stores are supported. Gather and scatter operations support only a tetra-byte size. Other load and store operations support byte, wyde and tetra sizes. Loads may automatically sign or zero extend to the machine's width.

Memory operations make use of a 10-bit address space identifier, ASID. Each thread may have its own ASID. 

# Software
Still in the early stages of development there is little software for the core. There is a version of the vasm assembler that supports some of the instruction set, but it is still under construction.

