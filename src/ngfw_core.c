// nicoGPU CP firmware -- core: context, register block, ring/IB event loop.
//
// Observable semantics are a line-by-line port of the functional model
// (the golden reference for command stream v0.2): the same check order, the same error latch, the same
// RPTR/WPTR rules and the same CP_CTRL state machine.  Differences are
// deliberate and documented:
//   * the IB is streamed dword-by-dword instead of prefetched (BRAM budget;
//     both are allowed) -- IB fetch faults therefore latch with
//     INDIRECT attribution, exactly like the model's prefetch fault;
//   * engine work (DISPATCH) is submitted synchronously through the platform
//     (the serial model is one legal implementation).
#include "ngfw_internal.h"

// ---------------------------------------------------------------------------
// Packet table (mirror of the functional model's table, checked against the
// command-stream spec v0.2 by the golden-vector tests).
#define R NGFW_STREAM_RING
#define I NGFW_STREAM_IB
#define RI (NGFW_STREAM_RING | NGFW_STREAM_IB)
static const ngfw_pkt_info k_table[] = {
    {"NOP", NGFW_OP_NOP, 0, (uint16_t)NGFW_COUNT_MAX, RI, 0, 1},
    {"INDIRECT", NGFW_OP_INDIRECT, 4, 4, R, 0, 1},
    {"FENCE", NGFW_OP_FENCE, 5, 5, RI, 0, 1},
    {"TIMESTAMP", NGFW_OP_TIMESTAMP, 2, 2, RI, 0, 1},
    {"COPY", NGFW_OP_COPY, 7, 7, RI, 0, 1},
    {"DISPATCH", NGFW_OP_DISPATCH, 3, 3, I, 1, 1},
    {"SET_SHADER", NGFW_OP_SET_SHADER, 11, 11, I, 0, 1},
    {"SET_USER_REGS", NGFW_OP_SET_USER_REGS, 2, 1u + NGFW_USER_UREGS, I, 0, 1},
    {"DISPATCH_INDIRECT", NGFW_OP_DISPATCH_INDIRECT, 2, 2, I, 1, 1},
    {"WAIT_MEM", NGFW_OP_WAIT_MEM, 7, 7, RI, 0, 1},
    {"CACHE", NGFW_OP_CACHE, 1, 1, RI, 0, 1},
    {"FILL", NGFW_OP_FILL, 5, 5, RI, 1, 1},
    {"QUERY", NGFW_OP_QUERY, 3, 3, I, 0, 1},
    {"SET_PREDICATE", NGFW_OP_SET_PREDICATE, 3, 3, I, 0, 1},
    {"WRITE_DATA", NGFW_OP_WRITE_DATA, 3, (uint16_t)NGFW_COUNT_MAX, RI, 0, 1},
    {"DRAW", NGFW_OP_DRAW, 4, 4, I, 1, 0},
    {"DRAW_INDEXED", NGFW_OP_DRAW_INDEXED, 5, 5, I, 1, 0},
    {"DRAW_INDIRECT", NGFW_OP_DRAW_INDIRECT, 6, 6, I, 1, 0},
    {"DRAW_INDEXED_INDIRECT", NGFW_OP_DRAW_INDEXED_INDIRECT, 6, 6, I, 1, 0},
};
#undef R
#undef I
#undef RI

const ngfw_pkt_info *ngfw_packet_info(uint8_t opcode) {
  for (unsigned i = 0; i < sizeof(k_table) / sizeof(k_table[0]); ++i)
    if (k_table[i].opcode == opcode) return &k_table[i];
  return 0;
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
  // state reset, RPTR:= WPTR. Does not clear stopped_
  // (that follows !HALT_REQ in the CP_CTRL handler), counters or cache stats.
  c->err = NGFW_ERR_NONE;
  c->err_text = 0;
  c->err_rptr = 0;
  c->err_info = 0;
  c->err_ib_off = 0;
  c->rptr = c->wptr;
  c->ib.active = 0;
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
uint32_t ngfw_ring_dw(ngfw_ctx *c, uint32_t ptr) {
  // Packets may wrap around the ring end.
  return c->plat->phys_read32(c->ring_base + 4ull * (ptr & c->ring_mask));
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
  if (p->stream == NGFW_STREAM_RING) {
    for (uint32_t i = 0; i < ndw; ++i) out[i] = ngfw_ring_dw(c, c->rptr + 1u + first + i);
    return 0;
  }
  // IB payload fetch.  A fault here is attributed to the INDIRECT packet
  // itself: the model prefetches the whole IB before executing anything, so
  // its fetch faults latch at the same point with the same CP_ERROR_INFO.
  if (!c->plat->mem_read ||
      c->plat->mem_read(p->vmid, c->ib.va + 4ull * (c->ib.pos + 1u + first), out, 4u * ndw) !=
          NGFW_MEM_OK) {
    c->cur_op = NGFW_OP_INDIRECT;
    c->cur_vmid = 0;
    c->cur_in_ib = 0;
    ngfw_fail(c, NGFW_ERR_MEM_FAULT, "command buffer fetch fault");
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
  if (c->plat->publish_aux &&
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
    uint32_t hdr;
    if (!c->plat->mem_read ||
        c->plat->mem_read(c->ib.vmid, c->ib.va + 4ull * c->ib.pos, &hdr, 4) != NGFW_MEM_OK) {
      c->cur_op = NGFW_OP_INDIRECT;
      c->cur_vmid = 0;
      c->cur_in_ib = 0;
      ngfw_fail(c, NGFW_ERR_MEM_FAULT, "command buffer fetch fault");
      return NGFW_ERROR;
    }
    const uint32_t n = hdr >> NGFW_HDR_COUNT_SHIFT;
    if (n >= c->ib.size - c->ib.pos) { // step 0
      c->cur_op = (uint8_t)(hdr & NGFW_HDR_OPCODE_MASK);
      c->cur_vmid = c->ib.vmid;
      c->cur_in_ib = 1;
      ngfw_fail(c, NGFW_ERR_BAD_PACKET, "packet runs past the end of the command buffer");
      return NGFW_ERROR;
    }
    ngfw_pkt pkt;
    pkt.stream = NGFW_STREAM_IB;
    pkt.vmid = c->ib.vmid;
    pkt.hdr = hdr;
    pkt.count = n;
    ngfw_memset(pkt.buf, 0, sizeof(pkt.buf));
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
    c->cur_op = 0;
    c->cur_vmid = 0;
    c->cur_in_ib = 0;
    ngfw_fail(c, NGFW_ERR_RING_CONFIG, "ring base/size invalid");
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
      c->cur_op = 0;
      c->cur_vmid = 0;
      c->cur_in_ib = 0;
      ngfw_fail(c, NGFW_ERR_RING_CONFIG, "wptr ahead of rptr by more than the ring size");
      st = NGFW_SVC_ERROR;
      goto done;
    }
    const uint32_t hdr = ngfw_ring_dw(c, c->rptr);
    const uint32_t n = hdr >> NGFW_HDR_COUNT_SHIFT;
    if (n >= avail) {  // incomplete packet: wait for the next doorbell (7.2)
      st = NGFW_SVC_WAIT_RING;
      goto done;
    }
    ngfw_pkt pkt;
    pkt.stream = NGFW_STREAM_RING;
    pkt.vmid = 0; // the ring executes at VMID 0 (physical)
    pkt.hdr = hdr;
    pkt.count = n;
    ngfw_memset(pkt.buf, 0, sizeof(pkt.buf));
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

// Background re-poll of a blocked WAIT_MEM. The model only re-polls
// on host events, and every attempt counts as a packet in the functional
// timestamp; a hardware CP polls continuously, so re-polls that find the wait
// still blocked are not packets: the counters are restored.  Event-driven
// re-polls (ngfw_doorbell / ngfw_mmio_write) still count, as in the model.
ngfw_svc_status ngfw_repoll(void) {
  ngfw_ctx *c = &ngfw;
  const uint64_t ticks = c->ticks;
  const uint32_t rptr = c->rptr;
  const ngfw_svc_status st = ngfw_service();
  if (st == NGFW_SVC_WAITING && c->rptr == rptr) {
    c->ticks = ticks;
    publish_status(c);
  }
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

void ngfw_mmio_write(uint32_t off, uint32_t val) {
  ngfw_ctx *c = &ngfw;
  switch (off) {
    case NGFW_REG_CP_RING_BASE_LO:
      c->ring_base = (c->ring_base & ~(uint64_t)0xffffffff) | val;
      break;
    case NGFW_REG_CP_RING_BASE_HI:
      c->ring_base = (c->ring_base & (uint64_t)0xffffffff) | ((uint64_t)val << 32);
      break;
    case NGFW_REG_CP_RING_SIZE_LOG2:
      // Full 32-bit register, reads back what was written (same as the
      // model).  service() rejects values outside 10..20 with
      // RING_CONFIG; the mask is only meaningful once validated, and the
      // shift is kept defined for out-of-range values.
      c->size_log2 = val;
      c->ring_mask = (val < 32u) ? ((1u << val) - 1u) : 0u;
      break;
    case NGFW_REG_CP_FW_BASE_LO:
      c->fw_base = (c->fw_base & ~(uint64_t)0xffffffff) | val;
      break;
    case NGFW_REG_CP_FW_BASE_HI:
      c->fw_base = (c->fw_base & (uint64_t)0xffffffff) | ((uint64_t)val << 32);
      break;
    case NGFW_REG_CP_FW_SIZE: c->fw_size = val; break;
    case NGFW_REG_CP_RPTR_WB_LO:
      c->rptr_wb = (c->rptr_wb & ~(uint64_t)0xffffffff) | (val & ~2u);
      break;
    case NGFW_REG_CP_RPTR_WB_HI:
      c->rptr_wb = (c->rptr_wb & (uint64_t)0xffffffff) | ((uint64_t)val << 32);
      break;
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
