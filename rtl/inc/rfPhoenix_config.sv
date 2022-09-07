/**** under construction ****/

package rfPhoenix_config;

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
// It also needs to be greater than the latency of the icache read (5).
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
parameter TidMSB = $ceil($log2(`NTHREADS))-1;

endpackage
