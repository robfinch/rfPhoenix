// ============================================================================
//        __
//   \\__/ o\    (C) 2015-2022  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
// BSD 3-Clause License
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// ============================================================================
//
package wishbone_pkg;

typedef logic [31:0] wb_address_t;
typedef logic [5:0] wb_burst_len_t;		// number of beats in a burst -1
typedef logic [3:0] wb_channel_t;			// channel for devices like system cache
typedef logic [7:0] wb_tranid_t;			// transaction id
typedef logic [7:0] wb_priv_level_t;	// 0=all access,
typedef logic [3:0] wb_priority_t;		// network transaction priority, higher is better

typedef enum logic [2:0] {
	CLASSIC = 3'b000,
	FIXED = 3'b001,					// constant data address
	INCR = 3'b010,					// incrementing data address
	EOB = 3'b111						// end of data burst
} wb_cycle_type_t;

typedef enum logic [2:0] {
	DATA = 3'b000,
	STACK = 3'b110,
	CODE = 3'b111
} wb_segment_t;

typedef enum logic [2:0] {
	LINEAR = 3'b000,
	WRAP4 = 3'b001,
	WRAP8 = 3'b010,
	WRAP16 = 3'b011,
	WRAP32 = 3'b100,
	WRAP64 = 3'b101,
	WRAP128 = 3'b110
} wb_burst_type_t;

// number of byte transferred in a beat
typedef enum logic [2:0] {
	B8 = 3'd0,
	B16 = 3'd1,
	B32 = 3'd2,
	B64 = 3'd3,
	B128 = 3'd4
} wb_size_t;

typedef enum logic [1:0] {
	OKAY = 2'b00,				// no error
	DECERR = 2'd01,			// decode error
	PROTERR = 2'b10,		// security violation
	ERR = 2'b11					// general error
} wb_error_t;

typedef enum logic [3:0] {
	NC_NB = 4'd0,										// Non-cacheable, non-bufferable
	NON_CACHEABLE = 4'd1,
	CACHEABLE_NB = 4'd2,						// Cacheable, non-bufferable
	CACHEABLE = 4'd3,								// Cacheable, bufferable
	WT_NO_ALLOCATE = 4'd8,					// Write Through
	WT_READ_ALLOCATE = 4'd9,
	WT_WRITE_ALLOCATE = 4'd10,
	WT_READWRITE_ALLOCATE = 4'd11,
	WB_NO_ALLOCATE = 4'd12,					// Write Back
	WB_READ_ALLOCATE = 4'd13,
	WB_WRITE_ALLOCATE = 4'd14,
	WB_READWRITE_ALLOCATE = 4'd15
} wb_cache_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Read requests
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request8_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [1:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request16_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [3:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request32_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [7:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request64_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [15:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request128_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [31:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request256_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	wb_address_t adr;			// address
	logic [63:0] sel;			// byte lane select
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic sr;							// set reservation
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_read_request512_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Write requests
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [7:0] dat;			// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request8_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [1:0] sel;			// byte lane selects
	logic [15:0] dat;			// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request16_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [3:0] sel;			// byte lane selects
	logic [31:0] dat;			// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request32_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [7:0] sel;			// byte lane selects
	logic [63:0] dat;			// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request64_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [15:0] sel;			// byte lane selects
	logic [127:0] dat;		// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request128_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [31:0] sel;			// byte lane selects
	logic [255:0] dat;		// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request256_t;

typedef struct packed {
	wb_burst_type_t bte;	// burst type extension
	wb_cycle_type_t cti;	// cycle type indicator
	wb_burst_len_t blen;	// length of burst-1
	wb_segment_t seg;			// segment
	logic cyc;						// valid cycle
	logic stb;						// data strobe
	logic we;							// write enable
	wb_address_t adr;			// address
	logic [63:0] sel;			// byte lane selects
	logic [511:0] dat;		// data
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic csr;						// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;		// privilege level
	wb_priority_t pri;		// transaction priority
	wb_cache_t cache;			// cache and buffer properties
} wb_write_request512_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Read responses
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [7:0] dat;			// data
} wb_read_response8_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [15:0] dat;			// data
} wb_read_response16_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [31:0] dat;			// data
} wb_read_response32_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [31:0] dat;			// data
} wb_response32_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [63:0] dat;			// data
} wb_read_response64_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [127:0] dat;		// data
} wb_read_response128_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [127:0] dat;		// data
} wb_response128_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [255:0] dat;		// data
} wb_read_response256_t;

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [511:0] dat;		// data
} wb_read_response512_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Write responses
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Sometimes write cycles can expect data responses back too. This is typically
// just a single bit. For instance, the reservation status. Write responses all
// have a common structure.

typedef struct packed {
	wb_channel_t cid;			// channel id
	wb_tranid_t tid;			// transaction id
	logic stall;					// stall pipeline
	logic next;						// advance to next transaction
	logic ack;						// response acknowledge
	logic rty;						// retry
	logic err;						// error
	wb_priority_t pri;		// response priority
	logic [7:0] dat;			// data
} wb_write_response_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// read/write requests
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed
{
	wb_read_request8_t read;
	wb_write_request8_t write;
} wb_readwrite_request8_t;

typedef struct packed
{
	wb_read_request16_t read;
	wb_write_request16_t write;
} wb_readwrite_request16_t;

typedef struct packed
{
	wb_read_request32_t read;
	wb_write_request32_t write;
} wb_readwrite_request32_t;

typedef struct packed
{
	wb_read_request64_t read;
	wb_write_request64_t write;
} wb_readwrite_request64_t;

typedef struct packed
{
	wb_read_request128_t read;
	wb_write_request128_t write;
} wb_readwrite_request128_t;

typedef struct packed
{
	wb_read_request256_t read;
	wb_write_request256_t write;
} wb_readwrite_request256_t;

typedef struct packed
{
	wb_read_request512_t read;
	wb_write_request512_t write;
} wb_readwrite_request512_t;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// read / write responses
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

typedef struct packed
{
	wb_read_response8_t read;
	wb_write_response_t write;
} wb_readwrite_response8_t;

typedef struct packed
{
	wb_read_response16_t read;
	wb_write_response_t write;
} wb_readwrite_response16_t;

typedef struct packed
{
	wb_read_response32_t read;
	wb_write_response_t write;
} wb_readwrite_response32_t;

typedef struct packed
{
	wb_read_response64_t read;
	wb_write_response_t write;
} wb_readwrite_response64_t;

typedef struct packed
{
	wb_read_response128_t read;
	wb_write_response_t write;
} wb_readwrite_response128_t;

typedef struct packed
{
	wb_read_response256_t read;
	wb_write_response_t write;
} wb_readwrite_response256_t;

typedef struct packed
{
	wb_read_response512_t read;
	wb_write_response_t write;
} wb_readwrite_response512_t;

endpackage

interface wb_request_i #(int WID);
	wb_burst_type_t bte;		// burst type extension
	wb_cycle_type_t cti;		// cycle type indicator
	wb_burst_len_t blen;		// length of burst-1
	wb_segment_t seg;				// segment
	logic cyc;							// valid cycle
	logic stb;							// data strobe
	logic we;								// write enable
	wb_address_t adr;				// address
	logic [WID/8-1:0] sel;	// byte lane selects
	logic [WID-1:0] dat;		// data
	wb_channel_t cid;				// channel id
	wb_tranid_t tid;				// transaction id
	logic csr;							// set or clear reservation we:1=clear 0=set
	wb_priv_level_t pl;			// privilege level
	wb_priority_t pri;			// transaction priority
	wb_cache_t cache;				// cache and buffer properties
endinterface

interface wb_response_i #(int WID);
	wb_channel_t cid;				// channel id
	wb_tranid_t tid;				// transaction id
	logic stall;						// stall pipeline
	logic next;							// advance to next transaction
	logic ack;							// response acknowledge
	logic rty;							// retry
	logic err;							// error
	wb_priority_t pri;			// response priority
	logic [WID-1:0] dat;		// data
endinterface
