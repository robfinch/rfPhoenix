# rfPhoenix - *** Under Construction ***

## History
This project taken up in earnest beginning August 24, 2022 after studying the Nyuzi core by Jeff Bush.

## Features
- 32 bit data path
- 16 vector lanes, with lane masking capability
- 16 threads of execution
- 64 registers per thread
- unified floating-point and integer register file
- fixed 40 bit instructions
- fused floating-point multiply and add instruction
- 12 entry instruction reordering buffer

## Noted characteristics
This core does not use any results forwarding. Instead results forwarding is avoided by using multiple threads of execution.

### Cache misses
Instruction cache misses do not stall other threads from proceeding. Only the thread that missed stalls until the cache is loaded.

### Out-of-Order Execution
Instructions may execute out-of-order due to the use of a reordering buffer. Instructions ready to execute will execute while longer running operations are taking place.
For instance, an integer add operation which takes only one clock cycle, may execute while a floating-point addition is taking place. However, the operations must be
for different threads of execution. There may be multiple outstanding load and store operations taking place. A load or store operation gets placed in a memory queue.

### Pipelining
Floating point operations are not fully pipelined. The FMA requires eight clock cycles to perform. Rather than fully pipeline the FMA operation the hope is that
multiple FMA units may be used provided there is enough resources to support them in the FPGA.
Some out-of-order execution of instructions is supported. Instructions for a particular thread will be performed in-order. However between threads instructions may execute
out of order with respect to the order they were queued.

## Register file
The register file contains 1024 32-bit registers to support 64 registers for 16 threads. Since there are that many vector register which are 512-bits in size,
the register file is quite large. It consumes about 100 block rams. The author wanted a design supporting more than the usual 32 registers. The author would
have preferred 128 registers but there is no easy way to encode that many in the ISA and still keep instructions a reasonable size.

The author would say the registers are general purpose in nature but the typical ABI specifies specific register usages. The first 32 registers are general purpose,
the remaining registers have an assortment of uses including vector mask registers, alternate stack pointers, link registers, and exception instruction pointer registers.
These other registers are implemented as part of the general register file so that existing instructions may be applied to them.
In a pinch vector mask registers may be used as general purpose registers. There are also eight registers with unassigned usage. They may be used for floating-point for
instance.

Vector mask registers are special in that they may be used to mask vector operations and are specified with a three-bit field in the instruction. 

## Instruction Set

### ALU
Basic ALU and shift operations are supported. Integer operations have register-register and register-immediate forms. 

### Floating-point
Fused multiply and add instructions make up most of the floating-point capabilities. There are no independant add, subtract or multiply operations.
The FMA instructions may be used for ordinary add/subtract operations by setting one of the multipiers to the value 1.0. Plain old multiply may be performed by setting
the value to add to 0.0. There is no hardware divide operation.

### Immediates
16-bit immediate values are supported directly in instructions. 32-bit immediates are created using an immediate prefix instruction. The prefix instruction may be applied
to extend the addressing range of load and store operations.

### Register Specs
Register specifications are six bit with an additional bit indicating the use of a vector or scaler register.

### Branches
Because the instruction format is wide enough to support it, branches use a compare-and-branch approach. Two registers are compared then a branch takes place according to a
selected condition. Comparing branches eliminate a lot of compares from the instruction stream. It is really a fused instruction operation.

### Loads and Stores
Gathering loads and scattering stores are supported. Gather and scatter operations support only a tetra-byte size. Other load and store operations support byte, wyde and tetra 
sizes. Loads may automatically sign or zero extend to the machine's width.


