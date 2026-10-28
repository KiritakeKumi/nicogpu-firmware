// nicoGPU CP firmware -- internal state shared by ngfw_core.c and ngfw_pkt.c.
// Not installed; not part of the firmware's public surface.
#ifndef NGFW_INTERNAL_H
#define NGFW_INTERNAL_H

#include <stdint.h>

#include "ngfw.h"
#include "ngfw_regs.h"

// Payload dwords buffered for one packet.  Every packet with a fixed or
// bounded COUNT needs at most 27 (SET_USER_REGS: CTRL + 26 values); NOP and
// WRITE_DATA payloads (COUNT up to 65535) are skipped / streamed instead.
#define NGFW_PKT_BUF_DW 27

// Shared staging buffer (dwords) for COPY/FILL/WRITE_DATA streaming.  64
// dwords = 256 B keeps the DMEM budget small while still exercising the
// chunked paths in the tests.
#define NGFW_STAGING_DW 64

typedef enum {
  NGFW_OK = 0,
  NGFW_BLOCKED = 1,  // WAIT_MEM condition not (yet) satisfied
  NGFW_ERROR = 2,
} ngfw_res;

// Dwords per speculative stream read (header + payload of the next packets in one bus access run).  Reads never
// go past the IB end or, for IBs, the 4 KiB page of the first dword (see ngfw_core.c); 8 covers every fixed-size
// packet up to COPY / WAIT_MEM, longer packets fetch their remainder on demand.
#ifndef NGFW_PKT_SPEC_DW
#define NGFW_PKT_SPEC_DW 8u
#endif

// Read-ahead window of one stream (ring or IB): the dwords last read speculatively, keyed by their position in the
// stream (ring: free-running dword counter; IB: dword offset), so the next packet header and payload that fall into
// the window are not fetched a second time.  Dropped whenever the stream can change under it (new INDIRECT, reset,
// ring register writes).
typedef struct ngfw_win {
  uint32_t pos;  // stream position of dw[0]
  uint32_t n;    // valid dwords (0 = empty)
  uint32_t dw[NGFW_PKT_SPEC_DW];
} ngfw_win;

// One packet under execution.  `stream` is NGFW_STREAM_RING or NGFW_STREAM_IB
// (bit-mask form); raw[0] is dw0 (the header); `count` is N (payload dwords).
// raw[1 + i] holds payload dword i+1; the first `have` dwords of raw[] are valid
// (header + whatever the speculative prefetch brought in), the rest is fetched on
// demand via ngfw_pkt_fetch.  raw[] is NOT zero-filled: handlers only read
// payload dwords up to `count`, which exec fetches first.
typedef struct ngfw_pkt {
  uint32_t stream;
  unsigned vmid;
  uint32_t count;
  uint32_t have;
  uint32_t raw[1 + NGFW_PKT_BUF_DW];
} ngfw_pkt;

// Per-IB compute state; reset at every INDIRECT.
typedef struct ngfw_compute_state {
  int shader_valid;
  uint64_t code_va, code_base;
  uint32_t vregs, shared_bytes, scratch_bytes;
  uint64_t scratch_va;
  uint32_t wg[3];
  uint32_t uregs[NGFW_USER_UREGS];
  int pred_enabled, pred_pass;
} ngfw_compute_state;

// Current INDIRECT (streamed, never buffered whole: an IB may be 2^20 dwords
// and the firmware has tens of KB of DMEM; allows this).
typedef struct ngfw_ib_state {
  int active;
  unsigned vmid;
  uint32_t ring_len;  // dwords of the INDIRECT packet in the ring
  uint32_t pos;       // dword offset of the next packet header inside the IB
  uint32_t size;      // IB_SIZE in dwords
  uint64_t va;        // IB base VA (translated in `vmid`)
  ngfw_win win;       // read-ahead window
} ngfw_ib_state;

// Command-stream packet table (mirror of kTable in the functional model).
typedef struct ngfw_pkt_info {
  const char *name;
  uint8_t opcode;
  uint16_t count_min, count_max;
  uint8_t streams;   // NGFW_STREAM_* mask
  uint8_t predicable;
  uint8_t implemented;  // 0: reserved / draft (not executed) -> UNSUPPORTED
} ngfw_pkt_info;

const ngfw_pkt_info *ngfw_packet_info(uint8_t opcode);

// The whole firmware is one static context (no dynamic memory).
typedef struct ngfw_ctx {
  const ngfw_plat *plat;

  // registers (kernel-visible state)
  uint64_t ring_base;
  uint32_t size_log2;    // raw register value; validated in service()
  uint32_t ring_mask;    // (1 << size_log2) - 1
  uint64_t fw_base;
  uint32_t fw_size;
  uint64_t rptr_wb;      // bit0 = enable (write-back address)
  int enabled, halt_req, core_reset, fw_ready;

  // execution state
  uint32_t rptr, wptr;
  int waiting, stopped;
  ngfw_ib_state ib;
  ngfw_win ring_win;  // ring read-ahead window
  ngfw_compute_state cs;

  // error latch (first error wins; cleared by CP_CTRL.RESET)
  ngfw_err_code err;
  const char *err_text;
  uint32_t err_rptr, err_info, err_ib_off;
  uint8_t cur_op;
  unsigned cur_vmid;
  int cur_in_ib;

  // CP_FW_STATUS.ABORT_TIMEOUT (sticky until CORE_RESET; CP_CTRL.RESET keeps it) and how many
  // dispatch aborts reported WDIST_STATUS.ABORT_TIMEOUT since ngfw_init
  int abort_timeout;
  uint64_t abort_timeouts;

  // counters
  uint64_t dispatches, ticks, pred_skipped;
  uint64_t stat_cs_invocations, stat_occlusion;
  ngfw_cache_stats cache;
  uint32_t heartbeat;

  // staging (COPY/FILL/WRITE_DATA streaming)
  uint32_t staging[NGFW_STAGING_DW];

  // last CP_STATUS value published through plat->reg_write32
  uint32_t published_status;
  uint64_t published_ticks;
  uint32_t published_heartbeat;
  // set while ngfw_repoll() runs a speculative attempt: TIMESTAMP/HEARTBEAT are
  // published only after it decided whether the attempt counts
  int hold_aux;
} ngfw_ctx;

// Provided by ngfw_core.c.
extern ngfw_ctx ngfw;

// Latch an error (first wins) and raise IRQ_CP_ERROR; publishes CP_ERROR*.
void ngfw_fail(ngfw_ctx *c, ngfw_err_code e, const char *text);
// a dispatch abort ended with WDIST_STATUS.ABORT_TIMEOUT.  Not a command-stream error (the
// pending CP_CTRL.RESET would clear CP_ERROR right away): latches CP_FW_STATUS.ABORT_TIMEOUT, publishes it
// and logs NGFW_LOG_ABORT_TIMEOUT; no interrupt (the kernel is inside its reset ladder and reads the bit there).
void ngfw_note_abort_timeout(ngfw_ctx *c);
// fail(BAD_PACKET) returning NGFW_ERROR, for packet checks.
ngfw_res ngfw_bad(ngfw_ctx *c, const char *text);
// fail(MEM_FAULT) returning NGFW_ERROR, for CP-side access faults.
ngfw_res ngfw_memfault(ngfw_ctx *c, const char *what);

// VMID memory access wrappers; on fault they latch MEM_FAULT and return 1.
// `what` is the pre-formatted error text (e.g. "FENCE fault").
int ngfw_mem_read(ngfw_ctx *c, unsigned vmid, uint64_t va, void *dst, uint32_t len,
                  const char *what);
int ngfw_mem_write(ngfw_ctx *c, unsigned vmid, uint64_t va, const void *src, uint32_t len,
                   const char *what);

// Fetch `ndw` payload dwords of the current packet, starting at payload
// index `first` (0 = dw1), into `out`; dwords already in p->raw[] are copied,
// the rest is read from the ring / IB (contiguous runs, one bus access run each).
// Ring fetches cannot fault; IB fetch faults latch MEM_FAULT attributed to the
// INDIRECT packet (the model prefetches the whole IB, so its fetch faults
// surface there too).  Returns 0 on success.
int ngfw_pkt_fetch(ngfw_ctx *c, ngfw_pkt *p, uint32_t first, uint32_t ndw, uint32_t *out);

// Execute one parsed packet (ngfw_pkt.c): the generic checks
// in order, then the per-packet handler.
ngfw_res ngfw_pkt_exec(ngfw_ctx *c, ngfw_pkt *p);

// DISPATCH/DISPATCH_INDIRECT execution (shared); mirrors Cp::dispatch().
ngfw_res ngfw_do_dispatch(ngfw_ctx *c, unsigned vmid, uint32_t gx, uint32_t gy, uint32_t gz);

// Compute-state reset (IB start, RESET, CORE_RESET): defaults wg={1,1,1},
// pred_pass=1 -- matching the model's ComputeState{}.
void ngfw_cs_reset(ngfw_ctx *c);

// Freestanding string/memory helpers (ngfw_util.c).
void *ngfw_memcpy(void *dst, const void *src, uint32_t n);
void *ngfw_memset(void *dst, uint8_t v, uint32_t n);

#endif  // NGFW_INTERNAL_H
