// nicoGPU CP firmware -- core: context, register block, ring/IB event loop.
//
// Observable semantics are a line-by-line port of the functional model
// (the golden reference for command stream v0.2): the same check order, the same error latch, the same
// RPTR/WPTR rules and the same CP_CTRL state machine.  Differences are
// deliberate and documented:
//   * the IB is streamed (an 8-dword read-ahead window, never past the IB end
//     or the header's 4 KiB page) instead of prefetched whole (BRAM budget;
//     both are allowed) -- IB fetch faults therefore latch with
//     INDIRECT attribution, exactly like the model's prefetch fault;
//   * engine work (DISPATCH) is submitted synchronously through the platform
//     (the serial model is one legal implementation).
#include "ngfw_internal.h"

// ---------------------------------------------------------------------------
// Packet table (mirror of the functional model's table, checked against the
// command-stream spec v0.3 by the golden-vector tests).  Indexed by
// opcode: unlisted opcodes are zero entries (name == 0 -> unknown opcode).
#define R NGFW_STREAM_RING
#define I NGFW_STREAM_IB
#define RI (NGFW_STREAM_RING | NGFW_STREAM_IB)
#define PKT(op, name, cmin, cmax, streams, pred, impl)   [op] = {name, op, cmin, cmax, streams, pred, impl}
static const ngfw_pkt_info k_table[] = {
    PKT(NGFW_OP_NOP, "NOP", 0, (uint16_t)NGFW_COUNT_MAX, RI, 0, 1),
    PKT(NGFW_OP_INDIRECT, "INDIRECT", 4, 4, R, 0, 1),
    PKT(NGFW_OP_FENCE, "FENCE", 5, 5, RI, 0, 1),
    PKT(NGFW_OP_TIMESTAMP, "TIMESTAMP", 2, 2, RI, 0, 1),
    PKT(NGFW_OP_COPY, "COPY", 7, 7, RI, 0, 1),
    PKT(NGFW_OP_DISPATCH, "DISPATCH", 3, 3, I, 1, 1),
    PKT(NGFW_OP_SET_SHADER, "SET_SHADER", 11, 11, I, 0, 1),
    PKT(NGFW_OP_SET_USER_REGS, "SET_USER_REGS", 2, 1u + NGFW_USER_UREGS, I, 0, 1),
    PKT(NGFW_OP_DISPATCH_INDIRECT, "DISPATCH_INDIRECT", 2, 2, I, 1, 1),
    PKT(NGFW_OP_WAIT_MEM, "WAIT_MEM", 7, 7, RI, 0, 1),
    PKT(NGFW_OP_CACHE, "CACHE", 1, 1, RI, 0, 1),
    PKT(NGFW_OP_FILL, "FILL", 5, 5, RI, 1, 1),
    PKT(NGFW_OP_QUERY, "QUERY", 3, 3, I, 0, 1),
    PKT(NGFW_OP_SET_PREDICATE, "SET_PREDICATE", 3, 3, I, 0, 1),
    PKT(NGFW_OP_WRITE_DATA, "WRITE_DATA", 3, (uint16_t)NGFW_COUNT_MAX, RI, 0, 1),
    PKT(NGFW_OP_DRAW, "DRAW", 4, 4, I, 1, 0),
    PKT(NGFW_OP_DRAW_INDEXED, "DRAW_INDEXED", 5, 5, I, 1, 0),
    PKT(NGFW_OP_DRAW_INDIRECT, "DRAW_INDIRECT", 6, 6, I, 1, 0),
    PKT(NGFW_OP_DRAW_INDEXED_INDIRECT, "DRAW_INDEXED_INDIRECT", 6, 6, I, 1, 0),
    // v0.3 draft graphics packets: table entries only, executing them is UNSUPPORTED.
    PKT(NGFW_OP_SET_GEOM_LAUNCH, "SET_GEOM_LAUNCH", 3, 3, I, 0, 0),
    PKT(NGFW_OP_SET_FRAG_SHADER, "SET_FRAG_SHADER", 8, 8, I, 0, 0),
    PKT(NGFW_OP_SET_FRAG_USER_REGS, "SET_FRAG_USER_REGS", 2, 27, I, 0, 0),
    PKT(NGFW_OP_SET_RASTER, "SET_RASTER", 5, 5, I, 0, 0),
    PKT(NGFW_OP_SET_ZS, "SET_ZS", 6, 6, I, 0, 0),
    PKT(NGFW_OP_SET_VIEWPORTS, "SET_VIEWPORTS", 5, 65, I, 0, 0),
    PKT(NGFW_OP_SET_FRAG_COUNTERS, "SET_FRAG_COUNTERS", 4, 4, I, 0, 0),
    PKT(NGFW_OP_BEGIN_PASS, "BEGIN_PASS", 7, 7, I, 0, 0),
    PKT(NGFW_OP_END_PASS, "END_PASS", 1, 1, I, 0, 0),
    PKT(NGFW_OP_PASS_BARRIER, "PASS_BARRIER", 1, 1, I, 0, 0),
};
#undef PKT
#undef R
#undef I
#undef RI

const ngfw_pkt_info *ngfw_packet_info(uint8_t opcode) {
  if (opcode >= sizeof(k_table) / sizeof(k_table[0])) return 0;
  return k_table[opcode].name ? &k_table[opcode] : 0;
}

// The one and only firmware context.
ngfw_ctx ngfw;

// ---------------------------------------------------------------------------
// Helpers
static void publish(ngfw_ctx *c, uint32_t off, uint32_t val) {
  if (c->plat->reg_write32) c->plat->reg_write32(off, val);
}

static void plat_log(ngfw_ctx *c, const char *tag, uint32_t code) {
  if (c->plat->log) c->plat->log(tag, code);
}

void ngfw_cs_reset(ngfw_ctx *c) {
  ngfw_memset(&c->cs, 0, sizeof(c->cs));
  c->cs.wg[0] = 1;
  c->cs.wg[1] = 1;
  c->cs.wg[2] = 1;
  c->cs.pred_pass = 1;
}

static void reset_state(ngfw_ctx *c) {
  // Mirror of Cp::reset(): error latch cleared, in-flight IB dropped, compute
  // state reset, RPTR := WPTR.  Does not clear stopped_
  // (that follows !HALT_REQ in the CP_CTRL handler), counters or cache stats.
  c->err = NGFW_ERR_NONE;
  c->err_text = 0;
  c->err_rptr = 0;
  c->err_info = 0;
  c->err_ib_off = 0;
  c->rptr = c->wptr;
  c->ib.active = 0;
  c->ib.win.n = 0;
  c->ring_win.n = 0;  // the host re-fills the ring after a reset
  ngfw_cs_reset(c);
  c->waiting = 0;
  publish(c, NGFW_REG_CP_ERROR, 0);
  publish(c, NGFW_REG_CP_RING_RPTR, c->rptr);
}

void ngfw_fail(ngfw_ctx *c, ngfw_err_code e, const char *text) {
  if (c->err != NGFW_ERR_NONE) return;  // first error wins
  c->err = e;
  c->err_text = text;
  c->err_rptr = c->rptr;
  c->err_info = (uint32_t)c->cur_op | ((c->cur_vmid & 0xfu) << 8) |
                (c->cur_in_ib ? NGFW_CP_ERROR_INFO_IN_IB : 0u);
  c->err_ib_off = c->cur_in_ib ? c->ib.pos : 0;
  c->waiting = 0;
  publish(c, NGFW_REG_CP_ERROR, (uint32_t)e);
  publish(c, NGFW_REG_CP_ERROR_RPTR, c->err_rptr);
  publish(c, NGFW_REG_CP_ERROR_INFO, c->err_info);
  publish(c, NGFW_REG_CP_ERROR_IB_OFFSET, c->err_ib_off);
  c->plat->irq_raise(NGFW_IRQ_CP_ERROR);
  plat_log(c, text, NGFW_LOG_HALT);
}

static uint32_t fw_status(const ngfw_ctx *c) {
  if (!c->fw_ready) return 0;
  return NGFW_CP_FW_STATUS_READY | (c->abort_timeout ? NGFW_CP_FW_STATUS_ABORT_TIMEOUT : 0u);
}

void ngfw_note_abort_timeout(ngfw_ctx *c) {
  ++c->abort_timeouts;
  if (!c->abort_timeout) {
    c->abort_timeout = 1;
    publish(c, NGFW_REG_CP_FW_STATUS, fw_status(c));
  }
  plat_log(c, "abort-timeout", NGFW_LOG_ABORT_TIMEOUT);
}

// Failure outside a packet handler: the error is attributed to `op` (0: none) at `vmid`, in the IB or the ring.
static void fail_at(ngfw_ctx *c, uint8_t op, unsigned vmid, int in_ib, ngfw_err_code e,
                    const char *text) {
  c->cur_op = op;
  c->cur_vmid = vmid;
  c->cur_in_ib = in_ib;
  ngfw_fail(c, e, text);
}

ngfw_res ngfw_bad(ngfw_ctx *c, const char *text) {
  ngfw_fail(c, NGFW_ERR_BAD_PACKET, text);
  return NGFW_ERROR;
}

ngfw_res ngfw_memfault(ngfw_ctx *c, const char *what) {
  ngfw_fail(c, NGFW_ERR_MEM_FAULT, what);
  return NGFW_ERROR;
}

// ---------------------------------------------------------------------------
// Memory / ring access
// Read `n` dwords of the ring starting at position `ptr` (a free-running dword counter; packets may wrap around
// the ring end) into `dst`.  Physical reads: a contiguous run is one mem_read (split only at the
// wrap).  Ring reads never fail the CP: a run the bus refuses is re-read dword by dword, and a dword that still
// faults reads as 0 -- exactly what the old one-phys_read32-per-dword loop did.
static void ring_read(ngfw_ctx *c, uint32_t ptr, uint32_t *dst, uint32_t n) {
  while (n != 0) {
    const uint32_t idx = ptr & c->ring_mask;
    uint32_t run = c->ring_mask + 1u - idx;  // dwords to the ring end
    if (run > n) run = n;
    const uint64_t pa = c->ring_base + 4ull * idx;
    if (run == 1 || !c->plat->mem_read || c->plat->mem_read(0, pa, dst, 4u * run) != NGFW_MEM_OK)
      for (uint32_t i = 0; i < run; ++i) dst[i] = c->plat->phys_read32(pa + 4ull * i);
    ptr += run;
    dst += run;
    n -= run;
  }
}

// Length of a speculative read of at most `limit` dwords, capped at `cap`, starting at a dword whose address
// parity is `odd`.  When the cap (not `limit`) cuts the read it is shortened to end on an even dword, so the bus
// does not finish with a lone 4-byte access.
static uint32_t spec_len(uint32_t limit, uint32_t cap, uint32_t odd) {
  if (limit <= cap) return limit;
  return (cap > 1 && ((odd + cap) & 1u)) ? cap - 1u : cap;
}

// Copy the part of window `w` from stream position `pos` onwards into `raw`; returns the dwords copied (0: miss).
static uint32_t win_take(const ngfw_win *w, uint32_t pos, uint32_t *raw) {
  const uint32_t off = pos - w->pos;  // wraps for pos < w->pos: a miss
  if (off >= w->n) return 0;
  const uint32_t m = w->n - off;
  for (uint32_t i = 0; i < m; ++i) raw[i] = w->dw[off + i];
  return m;
}

int ngfw_mem_read(ngfw_ctx *c, unsigned vmid, uint64_t va, void *dst, uint32_t len,
                  const char *what) {
  if (!c->plat->mem_read || c->plat->mem_read(vmid, va, dst, len) != NGFW_MEM_OK) {
    ngfw_memfault(c, what);
    return 1;
  }
  return 0;
}

int ngfw_mem_write(ngfw_ctx *c, unsigned vmid, uint64_t va, const void *src, uint32_t len,
                   const char *what) {
  if (!c->plat->mem_write || c->plat->mem_write(vmid, va, src, len) != NGFW_MEM_OK) {
    ngfw_memfault(c, what);
    return 1;
  }
  return 0;
}

int ngfw_pkt_fetch(ngfw_ctx *c, ngfw_pkt *p, uint32_t first, uint32_t ndw, uint32_t *out) {
  // Payload dword i is raw[1 + i]; the prefix the speculative prefetch already brought in is just copied.
  uint32_t done = 0;
  if (first + 1u < p->have) {
    done = p->have - 1u - first;
    if (done > ndw) done = ndw;
    if (out != &p->raw[1u + first])
      for (uint32_t i = 0; i < done; ++i) out[i] = p->raw[1u + first + i];
  }
  if (done == ndw) return 0;
  const uint32_t at = first + done;  // first payload dword still to be read
  if (p->stream == NGFW_STREAM_RING) {
    ring_read(c, c->rptr + 1u + at, out + done, ndw - done);
    return 0;
  }
  // IB payload fetch.  A fault here is attributed to the INDIRECT packet
  // itself: the model prefetches the whole IB before executing anything, so
  // its fetch faults latch at the same point with the same CP_ERROR_INFO.
  if (!c->plat->mem_read ||
      c->plat->mem_read(p->vmid, c->ib.va + 4ull * (c->ib.pos + 1u + at), out + done,
                        4u * (ndw - done)) != NGFW_MEM_OK) {
    fail_at(c, NGFW_OP_INDIRECT, 0, 0, NGFW_ERR_MEM_FAULT, "command buffer fetch fault");
    return 1;
  }
  return 0;
}

// RPTR write-back to host memory (posted write through the physical path).
// A fault here is ignored, matching the model (its PhysMem write cannot
// fail); the kernel would only hit it by misconfiguring CP_RPTR_WB.
static void write_rptr_wb(ngfw_ctx *c) {
  if (c->rptr_wb & NGFW_CP_RPTR_WB_ENABLE) {
    const uint64_t pa = c->rptr_wb & ~(uint64_t)3;
    if (c->plat->mem_write) (void)c->plat->mem_write(0, pa, &c->rptr, 4);
  }
}

static void publish_rptr(ngfw_ctx *c) {
  publish(c, NGFW_REG_CP_RING_RPTR, c->rptr);
  write_rptr_wb(c);
}

// ---------------------------------------------------------------------------
// Status
static int halted(const ngfw_ctx *c) { return c->err != NGFW_ERR_NONE; }

uint32_t ngfw_status(void) {
  ngfw_ctx *c = &ngfw;
  const int idle = halted(c) || (c->rptr == c->wptr && !c->ib.active && !c->waiting);
  return (idle ? NGFW_CP_STATUS_IDLE : 0u) | (halted(c) ? NGFW_CP_STATUS_HALTED : 0u) |
         (c->waiting ? NGFW_CP_STATUS_WAITING : 0u) | (c->stopped ? NGFW_CP_STATUS_STOPPED : 0u) |
         (c->fw_ready ? NGFW_CP_STATUS_FW_READY : 0u);
}

static void publish_status(ngfw_ctx *c) {
  const uint32_t s = ngfw_status();
  if (s != c->published_status) {
    publish(c, NGFW_REG_CP_STATUS, s);
    c->published_status = s;
  }
  // Timestamp / heartbeat feed for the hardware register block.
  if (c->plat->publish_aux && !c->hold_aux &&
      (c->ticks != c->published_ticks || c->heartbeat != c->published_heartbeat)) {
    c->plat->publish_aux(c->ticks, c->heartbeat);
    c->published_ticks = c->ticks;
    c->published_heartbeat = c->heartbeat;
  }
}

// ---------------------------------------------------------------------------
// IB execution
static ngfw_res run_ib(ngfw_ctx *c) {
  while (c->ib.pos < c->ib.size) {
    ngfw_pkt pkt;
    // Header plus a speculative run of payload (and following packets') dwords in one read.  The packet size is
    // only known after the header, and the old code never touched anything past the packet it was executing, so
    // the run is limited to what cannot fault once the header read succeeds: it stops at the end of the IB the
    // host declared and at the end of the 4 KiB page the header is in (translation, permissions and the VRAM
    // interleave are all 4 KiB granular, and that page is proven accessible by the header's own bytes).  A
    // failing read therefore failed inside the header's own access (the bus stops at the first faulting chunk and
    // the chunk holding the header is the first): same faulting address and same failure record (INDIRECT, VMID
    // 0, not in IB, MEM_FAULT) as the old separate header read.  The dwords read ahead stay in the window, so a
    // following header that falls inside it costs no access (it lies in a page that was read successfully).
    const uint64_t va = c->ib.va + 4ull * c->ib.pos;
    uint32_t limit = (4096u - (uint32_t)(va & 0xfffu)) >> 2;  // dwords to the end of the header's page
    if (limit > c->ib.size - c->ib.pos) limit = c->ib.size - c->ib.pos;
    uint32_t k = win_take(&c->ib.win, c->ib.pos, pkt.raw);
    if (k > limit) k = limit;
    if (k == 0 || (k == 1 && limit > 1)) {  // miss, or only the header is left in the window: refill
      const uint32_t want = spec_len(limit, NGFW_PKT_SPEC_DW, (uint32_t)(va >> 2) & 1u);
      if (!c->plat->mem_read ||
          c->plat->mem_read(c->ib.vmid, va, c->ib.win.dw, 4u * want) != NGFW_MEM_OK) {
        c->ib.win.n = 0;
        fail_at(c, NGFW_OP_INDIRECT, 0, 0, NGFW_ERR_MEM_FAULT, "command buffer fetch fault");
        return NGFW_ERROR;
      }
      c->ib.win.pos = c->ib.pos;
      c->ib.win.n = want;
      k = win_take(&c->ib.win, c->ib.pos, pkt.raw);
    }
    const uint32_t hdr = pkt.raw[0];
    const uint32_t n = hdr >> NGFW_HDR_COUNT_SHIFT;
    if (n >= c->ib.size - c->ib.pos) {  // step 0
      fail_at(c, (uint8_t)(hdr & NGFW_HDR_OPCODE_MASK), c->ib.vmid, 1, NGFW_ERR_BAD_PACKET,
              "packet runs past the end of the command buffer");
      return NGFW_ERROR;
    }
    pkt.stream = NGFW_STREAM_IB;
    pkt.vmid = c->ib.vmid;
    pkt.count = n;
    pkt.have = k;
    const ngfw_res r = ngfw_pkt_exec(c, &pkt);
    if (r != NGFW_OK) return r;
    c->ib.pos += n + 1;
  }
  return NGFW_OK;
}

void ngfw_tick(void) {
  ngfw_ctx *c = &ngfw;
  c->heartbeat++;
  publish_status(c);
}

// ---------------------------------------------------------------------------
// Event loop (one drain attempt; mirrors Cp::process())
ngfw_svc_status ngfw_service(void) {
  ngfw_ctx *c = &ngfw;
  ngfw_svc_status st = NGFW_SVC_IDLE;

  c->heartbeat++;
  if (halted(c)) {
    st = NGFW_SVC_ERROR;
    goto done;
  }
  if (!c->enabled || c->core_reset || !c->fw_ready) {
    st = NGFW_SVC_OFF;
    goto done;
  }
  if (c->size_log2 < 10 || c->size_log2 > 20 || (c->ring_base & 0xfffull) != 0) {
    fail_at(c, 0, 0, 0, NGFW_ERR_RING_CONFIG, "ring base/size invalid");
    st = NGFW_SVC_ERROR;
    goto done;
  }

  for (;;) {
    if (c->ib.active) {
      const ngfw_res r = run_ib(c);
      if (r != NGFW_OK) {
        st = (r == NGFW_BLOCKED) ? NGFW_SVC_WAITING : NGFW_SVC_ERROR;
        goto done;
      }
      // The whole IB has been issued: only now does RPTR move past the
      // INDIRECT.
      c->rptr += c->ib.ring_len;
      c->ib.active = 0;
      publish_rptr(c);
      continue;
    }
    // Command-buffer boundary: the only point where HALT_REQ takes effect.
    if (c->halt_req) {
      if (!c->stopped) {
        c->stopped = 1;
        c->plat->irq_raise(NGFW_IRQ_CP_STOPPED);
        plat_log(c, "stopped", NGFW_LOG_STOPPED);
      }
      st = NGFW_SVC_STOPPED;
      goto done;
    }
    if (c->rptr == c->wptr) {
      st = NGFW_SVC_IDLE;
      goto done;
    }
    const uint32_t avail = c->wptr - c->rptr;
    if (avail > (1u << c->size_log2)) {
      fail_at(c, 0, 0, 0, NGFW_ERR_RING_CONFIG, "wptr ahead of rptr by more than the ring size");
      st = NGFW_SVC_ERROR;
      goto done;
    }
    // Header plus a speculative run of dwords, kept in the ring window; everything below WPTR is ring memory the
    // host has finished writing and ring reads cannot fault the CP, so only `avail` bounds the run.
    ngfw_pkt pkt;
    uint32_t k = win_take(&c->ring_win, c->rptr, pkt.raw);
    if (k > avail) k = avail;
    if (k == 0 || (k == 1 && avail > 1)) {  // miss, or only the header is left in the window: refill
      const uint32_t want = spec_len(avail, NGFW_PKT_SPEC_DW, c->rptr & 1u);
      ring_read(c, c->rptr, c->ring_win.dw, want);
      c->ring_win.pos = c->rptr;
      c->ring_win.n = want;
      k = win_take(&c->ring_win, c->rptr, pkt.raw);
    }
    const uint32_t hdr = pkt.raw[0];
    const uint32_t n = hdr >> NGFW_HDR_COUNT_SHIFT;
    if (n >= avail) {  // incomplete packet: wait for the next doorbell (7.2)
      st = NGFW_SVC_WAIT_RING;
      goto done;
    }
    pkt.stream = NGFW_STREAM_RING;
    pkt.vmid = 0;  // the ring executes at VMID 0 (physical)
    pkt.count = n;
    pkt.have = k;
    const ngfw_res r = ngfw_pkt_exec(c, &pkt);
    if (r != NGFW_OK) {
      st = (r == NGFW_BLOCKED) ? NGFW_SVC_WAITING : NGFW_SVC_ERROR;
      goto done;
    }
    if (c->ib.active) continue;  // RPTR advances when the IB finishes
    c->rptr += n + 1;
    publish_rptr(c);
  }

done:
  publish_status(c);
  return st;
}

// ---------------------------------------------------------------------------
// Lifecycle
void ngfw_init(const ngfw_plat *plat) {
  ngfw_ctx *c = &ngfw;
  ngfw_memset(c, 0, sizeof(*c));
  c->plat = plat;
  c->core_reset = 1;  // power-on: Ibex held in reset, firmware not READY
}

void ngfw_doorbell(uint32_t wptr) {
  ngfw.wptr = wptr;
  (void)ngfw_service();
}

// Background re-poll of a blocked WAIT_MEM.  The model only re-polls
// on host events, and every attempt counts as a packet in the functional
// timestamp; a hardware CP polls continuously, so re-polls that find the wait
// still blocked are not packets: the counters are restored.  Event-driven
// re-polls (ngfw_doorbell / ngfw_mmio_write) still count, as in the model.
// TIMESTAMP_LO/HI are held back while the attempt runs: publishing the
// attempt's +1 and then the restored value would make the host-visible counter
// step back (monotonic; blocked re-polls do not count).
ngfw_svc_status ngfw_repoll(void) {
  ngfw_ctx *c = &ngfw;
  const uint64_t ticks = c->ticks;
  const uint32_t rptr = c->rptr;
  c->hold_aux = 1;
  const ngfw_svc_status st = ngfw_service();
  c->hold_aux = 0;
  if (st == NGFW_SVC_WAITING && c->rptr == rptr) c->ticks = ticks;
  publish_status(c);
  return st;
}

void ngfw_main_loop(void) {
  for (;;) {
    // Poll the doorbell latch (push mode: ngfw_doorbell() from the IRQ
    // handler reaches the same code).  One read is atomic on the target.
    if (ngfw.plat->doorbell_wptr) {
      const uint32_t w = ngfw.plat->doorbell_wptr();
      if (w != ngfw.wptr) ngfw_doorbell(w);
    }
    const ngfw_svc_status st = ngfw_service();
    // WAIT_MEM must keep re-polling memory, so only sleep when not waiting.
    if (st != NGFW_SVC_WAITING && ngfw.plat->wait_event) ngfw.plat->wait_event();
  }
}

// ---------------------------------------------------------------------------
// Register block (BAR0 CP page, offsets from ngfw_regs.h)
uint32_t ngfw_mmio_read(uint32_t off) {
  ngfw_ctx *c = &ngfw;
  switch (off) {
    case NGFW_REG_CP_RING_BASE_LO: return (uint32_t)c->ring_base;
    case NGFW_REG_CP_RING_BASE_HI: return (uint32_t)(c->ring_base >> 32);
    case NGFW_REG_CP_RING_SIZE_LOG2: return c->size_log2;
    case NGFW_REG_CP_RING_RPTR: return c->rptr;
    case NGFW_REG_CP_RING_WPTR: return c->wptr;
    case NGFW_REG_CP_CTRL:
      return (c->enabled ? NGFW_CP_CTRL_ENABLE : 0u) | (c->halt_req ? NGFW_CP_CTRL_HALT_REQ : 0u) |
             (c->core_reset ? NGFW_CP_CTRL_CORE_RESET : 0u);
    case NGFW_REG_CP_STATUS: return ngfw_status();
    case NGFW_REG_CP_ERROR: return (uint32_t)c->err;
    case NGFW_REG_CP_ERROR_RPTR: return c->err_rptr;
    case NGFW_REG_CP_ERROR_INFO: return c->err_info;
    case NGFW_REG_CP_ERROR_IB_OFFSET: return c->err_ib_off;
    case NGFW_REG_CP_HEARTBEAT: return c->heartbeat;
    case NGFW_REG_CP_CMD_VERSION:
      return c->fw_ready ? (NGFW_CMD_VERSION_MAJOR << 16) | NGFW_CMD_VERSION_MINOR : 0u;
    case NGFW_REG_CP_FW_VERSION:
      return c->fw_ready
                 ? ((NGFW_FW_VERSION_MAJOR << 24) | (NGFW_FW_VERSION_MINOR << 16) |
                    NGFW_FW_VERSION_PATCH)
                 : 0u;
    case NGFW_REG_CP_FW_STATUS: return fw_status(c);
    case NGFW_REG_CP_FW_BASE_LO: return (uint32_t)c->fw_base;
    case NGFW_REG_CP_FW_BASE_HI: return (uint32_t)(c->fw_base >> 32);
    case NGFW_REG_CP_FW_SIZE: return c->fw_size;
    case NGFW_REG_CP_RPTR_WB_LO: return (uint32_t)c->rptr_wb;
    case NGFW_REG_CP_RPTR_WB_HI: return (uint32_t)(c->rptr_wb >> 32);
    default: return 0;
  }
}

// Replace one 32-bit half of a 64-bit register pair.
static uint64_t set_lo(uint64_t v, uint32_t lo) { return (v & ~(uint64_t)0xffffffff) | lo; }
static uint64_t set_hi(uint64_t v, uint32_t hi) { return (v & (uint64_t)0xffffffff) | ((uint64_t)hi << 32); }

void ngfw_mmio_write(uint32_t off, uint32_t val) {
  ngfw_ctx *c = &ngfw;
  switch (off) {
    case NGFW_REG_CP_RING_BASE_LO: c->ring_base = set_lo(c->ring_base, val); c->ring_win.n = 0; break;
    case NGFW_REG_CP_RING_BASE_HI: c->ring_base = set_hi(c->ring_base, val); c->ring_win.n = 0; break;
    case NGFW_REG_CP_RING_SIZE_LOG2:
      c->ring_win.n = 0;
      // Full 32-bit register, reads back what was written (same as the
      // model).  service() rejects values outside 10..20 with
      // RING_CONFIG; the mask is only meaningful once validated, and the
      // shift is kept defined for out-of-range values.
      c->size_log2 = val;
      c->ring_mask = (val < 32u) ? ((1u << val) - 1u) : 0u;
      break;
    case NGFW_REG_CP_FW_BASE_LO: c->fw_base = set_lo(c->fw_base, val); break;
    case NGFW_REG_CP_FW_BASE_HI: c->fw_base = set_hi(c->fw_base, val); break;
    case NGFW_REG_CP_FW_SIZE: c->fw_size = val; break;
    case NGFW_REG_CP_RPTR_WB_LO: c->rptr_wb = set_lo(c->rptr_wb, val & ~2u); break;
    case NGFW_REG_CP_RPTR_WB_HI: c->rptr_wb = set_hi(c->rptr_wb, val); break;
    case NGFW_REG_CP_CTRL: {
      const int core_reset = (val & NGFW_CP_CTRL_CORE_RESET) != 0;
      c->enabled = (val & NGFW_CP_CTRL_ENABLE) != 0;
      c->halt_req = (val & NGFW_CP_CTRL_HALT_REQ) != 0;
      if (core_reset && !c->core_reset) {
        // Firmware stops; everything in flight is dropped (the hardware clears CP_FW_STATUS,
        // ABORT_TIMEOUT included: the restarted firmware starts without it).
        c->fw_ready = 0;
        c->abort_timeout = 0;
        publish(c, NGFW_REG_CP_FW_STATUS, 0);
        reset_state(c);
      } else if (!core_reset && c->core_reset) {
        // Boot done: report versions, set READY, raise IRQ_CP_FW (7.4 step 3).
        c->fw_ready = 1;
        publish(c, NGFW_REG_CP_FW_VERSION, ngfw_mmio_read(NGFW_REG_CP_FW_VERSION));
        publish(c, NGFW_REG_CP_CMD_VERSION, ngfw_mmio_read(NGFW_REG_CP_CMD_VERSION));
        publish(c, NGFW_REG_CP_FW_STATUS, fw_status(c));
        c->plat->irq_raise(NGFW_IRQ_CP_FW);
        plat_log(c, "boot-ready", NGFW_LOG_BOOT_READY);
      }
      c->core_reset = core_reset;
      if (val & NGFW_CP_CTRL_RESET) {
        reset_state(c);
        plat_log(c, "reset", NGFW_LOG_RESET);
      }
      if (!c->halt_req) c->stopped = 0;
      (void)ngfw_service();
      break;
    }
    default: break;  // read-only / unknown offsets are ignored
  }
}

// ---------------------------------------------------------------------------
// Observability
uint64_t ngfw_ticks(void) { return ngfw.ticks; }
uint32_t ngfw_rptr(void) { return ngfw.rptr; }
uint32_t ngfw_wptr(void) { return ngfw.wptr; }
int ngfw_halted(void) { return ngfw.err != NGFW_ERR_NONE; }
int ngfw_waiting(void) { return ngfw.waiting; }
int ngfw_stopped(void) { return ngfw.stopped; }
int ngfw_idle(void) { return (ngfw_status() & NGFW_CP_STATUS_IDLE) != 0; }
ngfw_err_code ngfw_error(void) { return ngfw.err; }
uint32_t ngfw_error_rptr(void) { return ngfw.err_rptr; }
uint32_t ngfw_error_info(void) { return ngfw.err_info; }
uint32_t ngfw_error_ib_offset(void) { return ngfw.err_ib_off; }
uint64_t ngfw_dispatches(void) { return ngfw.dispatches; }
uint64_t ngfw_abort_timeouts(void) { return ngfw.abort_timeouts; }
uint64_t ngfw_pred_skipped(void) { return ngfw.pred_skipped; }
const ngfw_cache_stats *ngfw_cache(void) { return &ngfw.cache; }
