// nicoGPU CP firmware -- packet parsing and execution.
//
// Line-by-line port of Cp::exec_packet / Cp::exec_indirect / Cp::dispatch /
// Cp::begin_ib_state of the functional model.  Check order, error codes and
// observable effects (memory writes, IRQ bits, counters, RPTR rules) match
// the model; the golden-vector tests pin them.
#include "ngfw_internal.h"

// Payload dword i (1..count); the payload is pre-fetched into p->raw.
#define DW(i) (p->raw[(i)])

// Bytes until the next 4 KiB granule boundary.  The VRAM interleave
// hands every 4 KiB granule to a different DDR
// channel (C1/C2/C1/C4), so engine transfers must not blindly cross one.
static uint32_t granule_left(uint64_t addr) {
  return 4096u - (uint32_t)(addr & 0xfffu);
}

// Compute-state reset + mandatory implicit invalidate at every INDIRECT
// (clause C-INV 3.4).
static void begin_ib_state(ngfw_ctx *c) {
  ngfw_cs_reset(c);
  ++c->cache.ib_starts;
  ++c->cache.inv_l1;
  ++c->cache.inv_k;
  ++c->cache.inv_i;
  if (c->plat->sp_inv) c->plat->sp_inv(NGFW_CACHE_INV_L1 | NGFW_CACHE_INV_K | NGFW_CACHE_INV_I);  // S5b: C-INV is real
}

// Cp::exec_indirect.  The IB itself is streamed later (run_ib in
// ngfw_core.c), never buffered whole: IB_SIZE may be 2^20 dwords and the
// firmware DMEM budget is tens of KB.
static ngfw_res exec_indirect(ngfw_ctx *c, ngfw_pkt *p) {
  if ((DW(1) & 3u) != 0 || (DW(2) & ~NGFW_VA_HI_MASK) != 0 ||
      (DW(4) & ~NGFW_IB_CTRL_VMID_MASK) != 0)
    return ngfw_bad(c, "INDIRECT reserved bits");
  const unsigned vmid = DW(4) & NGFW_IB_CTRL_VMID_MASK;
  if (vmid == 0) return ngfw_bad(c, "INDIRECT with VMID 0");  // 1.6.2
  const uint32_t size = DW(3);
  if (size > NGFW_MAX_IB_DWORDS) return ngfw_bad(c, "command buffer is too large");
  c->ib.active = 1;
  c->ib.vmid = vmid;
  c->ib.ring_len = p->count + 1;
  c->ib.pos = 0;
  c->ib.size = size;
  c->ib.va = ngfw_lohi(DW(1), DW(2));
  c->ib.win.n = 0;
  begin_ib_state(c);
  return NGFW_OK;
}

// Cp::dispatch: shared by DISPATCH and DISPATCH_INDIRECT.
ngfw_res ngfw_do_dispatch(ngfw_ctx *c, unsigned vmid, uint32_t gx, uint32_t gy, uint32_t gz) {
  if (!c->cs.shader_valid) {
    ngfw_fail(c, NGFW_ERR_NO_SHADER_STATE, "DISPATCH without SET_SHADER in this command buffer");
    return NGFW_ERROR;
  }
  if (gx == 0 || gy == 0 || gz == 0) return NGFW_OK;  // empty dispatch is legal
  ngfw_dispatch_req di;
  ngfw_memset(&di, 0, sizeof(di));
  di.vmid = vmid;
  di.code_va = c->cs.code_va;
  di.code_base = c->cs.code_base;
  di.scratch_va = c->cs.scratch_va;
  di.scratch_bytes = c->cs.scratch_bytes;
  di.grid[0] = gx;
  di.grid[1] = gy;
  di.grid[2] = gz;
  di.wg[0] = c->cs.wg[0];
  di.wg[1] = c->cs.wg[1];
  di.wg[2] = c->cs.wg[2];
  di.shared_bytes = c->cs.shared_bytes;
  di.num_vregs = c->cs.vregs;
  for (unsigned i = 0; i < NGFW_USER_UREGS; ++i) di.user_uregs[i] = c->cs.uregs[i];
  ++c->dispatches;
  uint64_t instructions = 0;
  const int st = c->plat->dispatch_req ? c->plat->dispatch_req(&di, &instructions) : -1;
  c->ticks += instructions;  // counted for trapped dispatches too (model)
  if (st == NGFW_DISPATCH_ABORT) return NGFW_BLOCKED;  // CP_CTRL.RESET pending: the event loop resets next (as COPY)
  if (st == NGFW_DISPATCH_ABORT_TIMEOUT) { // the same, but the SP fabric did not drain in time
    ngfw_note_abort_timeout(c);
    return NGFW_BLOCKED;
  }
  if (st != 0) {
    ngfw_fail(c, NGFW_ERR_SHADER_FAULT, "shader trap/fault during dispatch");
    return NGFW_ERROR;
  }
  c->stat_cs_invocations +=
      (uint64_t)gx * gy * gz * ((uint64_t)c->cs.wg[0] * c->cs.wg[1] * c->cs.wg[2]);
  return NGFW_OK;
}

ngfw_res ngfw_pkt_exec(ngfw_ctx *c, ngfw_pkt *p) {
  const uint32_t hdr = p->raw[0];
  const uint8_t op = (uint8_t)(hdr & NGFW_HDR_OPCODE_MASK);
  const uint32_t n = p->count;
  const unsigned vmid = p->vmid;
  const int in_ib = p->stream == NGFW_STREAM_IB;

  c->cur_op = op;
  c->cur_vmid = vmid;
  c->cur_in_ib = in_ib;
  ++c->ticks;  // the functional timestamp advances per packet (model) ...
  ++c->heartbeat;

  // Generic checks, in order.
  const ngfw_pkt_info *pi = ngfw_packet_info(op);
  if (pi == 0) {
    ngfw_fail(c, NGFW_ERR_BAD_OPCODE, "unknown opcode");
    return NGFW_ERROR;
  }
  if ((hdr & NGFW_HDR_RSVD_MASK) != 0) return ngfw_bad(c, "reserved header bits set");
  if ((pi->streams & p->stream) == 0) {
    ngfw_fail(c, NGFW_ERR_NOT_PERMITTED, pi->name);
    return NGFW_ERROR;
  }
  if (n < pi->count_min || n > pi->count_max) return ngfw_bad(c, pi->name);
  if ((hdr & NGFW_HDR_PRED) != 0 && !pi->predicable)
    return ngfw_bad(c, "PRED set on a non-predicable packet");
  if (!pi->implemented) {
    ngfw_fail(c, NGFW_ERR_UNSUPPORTED, pi->name);
    return NGFW_ERROR;
  }
  if ((hdr & NGFW_HDR_PRED) != 0 && c->cs.pred_enabled && !c->cs.pred_pass) {
    ++c->pred_skipped;
    return NGFW_OK;  // skipped: no payload fetch, no payload validation (1.4.7)
  }

  // Fetch the payload the handler reads.  NOP payloads are ignored and
  // WRITE_DATA streams its tail, so neither needs the fixed buffer.
  if (op != NGFW_OP_NOP && op != NGFW_OP_WRITE_DATA) {
    const uint32_t nb = n < NGFW_PKT_BUF_DW ? n : NGFW_PKT_BUF_DW;
    if (ngfw_pkt_fetch(c, p, 0, nb, &p->raw[1]) != 0) return NGFW_ERROR;
  } else if (op == NGFW_OP_WRITE_DATA) {
    if (ngfw_pkt_fetch(c, p, 0, 2, &p->raw[1]) != 0) return NGFW_ERROR;
  }

  switch (op) {
    case NGFW_OP_NOP: return NGFW_OK;  // payload (if any) fully ignored

    case NGFW_OP_INDIRECT: return exec_indirect(c, p);

    case NGFW_OP_FENCE: {
      const uint32_t flags = DW(5);
      if ((flags & NGFW_FENCE_FLAGS_RSVD) != 0) return ngfw_bad(c, "FENCE reserved flags");
      if (in_ib && (flags & NGFW_FENCE_FLAG_IRQ) != 0) {
        ngfw_fail(c, NGFW_ERR_NOT_PERMITTED, "FENCE.IRQ only allowed in the ring");
        return NGFW_ERROR;
      }
      const uint64_t addr = ngfw_lohi(DW(1), DW(2));
      const uint32_t bytes = (flags & NGFW_FENCE_FLAG_BIT64) ? 8u : 4u;
      if ((addr & (bytes - 1u)) != 0) return ngfw_bad(c, "FENCE address misaligned");
      if (c->plat->wait_idle) c->plat->wait_idle();  // end-of-pipe (3.1)
      if ((flags & NGFW_FENCE_FLAG_L2_WB) != 0) ++c->cache.wb_l2;
      uint32_t w[2];
      w[0] = DW(3);
      w[1] = DW(4);
      if (ngfw_mem_write(c, vmid, addr, w, bytes, "FENCE fault") != 0) return NGFW_ERROR;
      if ((flags & NGFW_FENCE_FLAG_IRQ) != 0) c->plat->irq_raise(NGFW_IRQ_FENCE);
      return NGFW_OK;
    }

    case NGFW_OP_TIMESTAMP: {
      const uint64_t addr = ngfw_lohi(DW(1), DW(2));
      if (addr % 8u != 0) return ngfw_bad(c, "TIMESTAMP address misaligned");
      if (c->plat->wait_idle) c->plat->wait_idle();  // end-of-pipe (4.1)
      uint64_t ts = c->ticks;
      if (ngfw_mem_write(c, vmid, addr, &ts, 8, "TIMESTAMP fault") != 0) return NGFW_ERROR;
      return NGFW_OK;
    }

    case NGFW_OP_COPY: {
      const uint32_t flags = DW(7);
      if ((flags & ~NGFW_COPY_MODE_MASK) != 0) return ngfw_bad(c, "COPY reserved flags");
      const uint32_t mode = flags & NGFW_COPY_MODE_MASK;
      if (mode == 3) return ngfw_bad(c, "COPY reserved mode");
      if (mode != NGFW_COPY_MODE_LINEAR) {
        ngfw_fail(c, NGFW_ERR_UNSUPPORTED, "tiled COPY (t64k) is reserved in v0.1");
        return NGFW_ERROR;
      }
      uint64_t src = ngfw_lohi(DW(1), DW(2));
      uint64_t dst = ngfw_lohi(DW(3), DW(4));
      uint64_t size = ngfw_lohi(DW(5), DW(6));
      if (size != 0 && c->plat->copy_req) { // dedicated Copy engine
        const int m = c->plat->copy_req(vmid, src, dst, size);
        if (m == NGFW_MEM_ABORT) return NGFW_BLOCKED;  // CP_CTRL.RESET pending: the event loop resets next
        if (m != NGFW_MEM_OK) return ngfw_memfault(c, "COPY fault");
        return NGFW_OK;
      }
      while (size != 0) {
        // Chunk through the staging buffer, clipped at 4 KiB granule
        // boundaries of both ends (VRAM channel interleave, 5.1).
        uint32_t chunk = NGFW_STAGING_DW * 4u;
        if ((uint64_t)chunk > size) chunk = (uint32_t)size;
        if (chunk > granule_left(src)) chunk = granule_left(src);
        if (chunk > granule_left(dst)) chunk = granule_left(dst);
        if (ngfw_mem_read(c, vmid, src, c->staging, chunk, "COPY fault") != 0)
          return NGFW_ERROR;
        if (ngfw_mem_write(c, vmid, dst, c->staging, chunk, "COPY fault") != 0)
          return NGFW_ERROR;
        src += chunk;
        dst += chunk;
        size -= chunk;
      }
      return NGFW_OK;
    }

    case NGFW_OP_DISPATCH: return ngfw_do_dispatch(c, vmid, DW(1), DW(2), DW(3));

    case NGFW_OP_SET_SHADER: {
      if ((DW(1) & 0xfffu) != 0 || (DW(2) & ~NGFW_VA_HI_MASK) != 0)
        return ngfw_bad(c, "SET_SHADER CODE_BASE misaligned/reserved");
      if ((DW(3) & 0xfu) != 0) return ngfw_bad(c, "SET_SHADER ENTRY misaligned");
      if ((DW(4) & ~NGFW_RSRC_VREGS_MASK) != 0) return ngfw_bad(c, "SET_SHADER RSRC reserved bits");
      const uint32_t vregs = DW(4) & NGFW_RSRC_VREGS_MASK;
      if (vregs == 0 || vregs > NGFW_MAX_VREGS)
        return ngfw_bad(c, "SET_SHADER VREGS out of range");
      if (DW(5) > NGFW_MAX_SHARED_PER_WG) return ngfw_bad(c, "SET_SHADER SHARED_BYTES too large");
      if (DW(6) % 16u != 0) return ngfw_bad(c, "SET_SHADER SCRATCH_BYTES not a multiple of 16");
      if ((DW(7) & 0xffu) != 0 || (DW(8) & ~NGFW_VA_HI_MASK) != 0)
        return ngfw_bad(c, "SET_SHADER SCRATCH_VA misaligned/reserved");
      const uint64_t scratch_va = ngfw_lohi(DW(7), DW(8));
      if (DW(6) == 0 && scratch_va != 0)
        return ngfw_bad(c, "SET_SHADER SCRATCH_VA without SCRATCH_BYTES");
      uint64_t threads = 1;
      for (int i = 0; i < 3; ++i) {
        if (DW(9 + i) == 0) return ngfw_bad(c, "SET_SHADER workgroup dimension is zero");
        threads *= DW(9 + i);
      }
      if (threads > NGFW_MAX_WG_THREADS) return ngfw_bad(c, "SET_SHADER workgroup too large");
      c->cs.shader_valid = 1;
      c->cs.code_base = ngfw_lohi(DW(1), DW(2));
      c->cs.code_va = c->cs.code_base + DW(3);
      c->cs.vregs = vregs;
      c->cs.shared_bytes = DW(5);
      c->cs.scratch_bytes = DW(6);
      c->cs.scratch_va = scratch_va;
      c->cs.wg[0] = DW(9);
      c->cs.wg[1] = DW(10);
      c->cs.wg[2] = DW(11);
      return NGFW_OK;
    }

    case NGFW_OP_SET_USER_REGS: {
      if ((DW(1) & ~NGFW_USER_REGS_START_MASK) != 0)
        return ngfw_bad(c, "SET_USER_REGS reserved bits");
      const uint32_t start = DW(1) & NGFW_USER_REGS_START_MASK;
      const uint32_t k = p->count - 1;
      if (start + k > NGFW_USER_UREGS) return ngfw_bad(c, "SET_USER_REGS range beyond u25");
      for (uint32_t i = 0; i < k; ++i) c->cs.uregs[start + i] = DW(2 + i);
      return NGFW_OK;
    }

    case NGFW_OP_DISPATCH_INDIRECT: {
      const uint64_t args = ngfw_lohi(DW(1), DW(2));
      if (args % 4u != 0) return ngfw_bad(c, "DISPATCH_INDIRECT args misaligned");
      // Shader state is checked before the argument read.
      if (!c->cs.shader_valid) {
        ngfw_fail(c, NGFW_ERR_NO_SHADER_STATE,
                  "DISPATCH_INDIRECT without SET_SHADER in this command buffer");
        return NGFW_ERROR;
      }
      uint32_t g[3];
      if (ngfw_mem_read(c, vmid, args, g, sizeof(g), "DISPATCH_INDIRECT args fault") != 0)
        return NGFW_ERROR;
      return ngfw_do_dispatch(c, vmid, g[0], g[1], g[2]);
    }

    case NGFW_OP_WAIT_MEM: {
      const uint32_t ctrl = DW(7);
      if ((ctrl & ~(NGFW_WAIT_FUNC_MASK | NGFW_WAIT_BIT64)) != 0)
        return ngfw_bad(c, "WAIT_MEM reserved bits");
      const uint32_t func = ctrl & NGFW_WAIT_FUNC_MASK;
      if (func > NGFW_WAIT_LE) return ngfw_bad(c, "WAIT_MEM reserved FUNC");
      const int b64 = (ctrl & NGFW_WAIT_BIT64) != 0;
      const uint64_t addr = ngfw_lohi(DW(1), DW(2));
      if ((addr & (b64 ? 7u : 3u)) != 0) return ngfw_bad(c, "WAIT_MEM address misaligned");
      if (!b64 && (DW(4) != 0 || DW(6) != 0))
        return ngfw_bad(c, "WAIT_MEM REF_HI/MASK_HI must be 0 for 32-bit waits");
      const uint64_t ref = ngfw_lohi(DW(3), DW(4));
      const uint64_t mask = ngfw_lohi(DW(5), DW(6));
      uint64_t v = 0;
      if (ngfw_mem_read(c, vmid, addr, &v, b64 ? 8u : 4u, "WAIT_MEM fault") != 0)
        return NGFW_ERROR;
      v &= mask;
      int ok;
      switch (func) {
        case NGFW_WAIT_EQ: ok = v == ref; break;
        case NGFW_WAIT_NE: ok = v != ref; break;
        case NGFW_WAIT_GE: ok = v >= ref; break;
        case NGFW_WAIT_GT: ok = v > ref; break;
        case NGFW_WAIT_LT: ok = v < ref; break;
        default: ok = v <= ref; break;
      }
      c->waiting = !ok;
      return ok ? NGFW_OK : NGFW_BLOCKED;
    }

    case NGFW_OP_CACHE: {
      const uint32_t f = DW(1);
      if ((f & NGFW_CACHE_FLAGS_RSVD) != 0) return ngfw_bad(c, "CACHE reserved flags");
      // Order WAIT_IDLE -> WB_L2 -> INV_L2 -> INV_L1/K/I (3.3).  The phase-1
      // engines are synchronous, so the counters are what the host tests observe; on the target (S5b) the
      // INV_L1/K/I part also reaches the SP (plat->sp_inv).  WB_L2 / INV_L2 stay counters (the L2 is not wired to
      // the CP yet).
      if ((f & NGFW_CACHE_WAIT_IDLE) != 0) {
        if (c->plat->wait_idle) c->plat->wait_idle();
        ++c->cache.wait_idle;
      }
      if ((f & NGFW_CACHE_WB_L2) != 0) ++c->cache.wb_l2;
      if ((f & NGFW_CACHE_INV_L2) != 0) ++c->cache.inv_l2;
      if ((f & NGFW_CACHE_INV_L1) != 0) ++c->cache.inv_l1;
      if ((f & NGFW_CACHE_INV_K) != 0) ++c->cache.inv_k;
      if ((f & NGFW_CACHE_INV_I) != 0) ++c->cache.inv_i;
      if (c->plat->sp_inv)
        c->plat->sp_inv(f & (NGFW_CACHE_INV_L1 | NGFW_CACHE_INV_K | NGFW_CACHE_INV_I));
      return NGFW_OK;
    }

    case NGFW_OP_FILL: {
      uint64_t dst = ngfw_lohi(DW(1), DW(2));
      uint64_t size = ngfw_lohi(DW(3), DW(4));
      if (dst % 4u != 0 || size % 4u != 0)
        return ngfw_bad(c, "FILL address/size not a multiple of 4");
      if (size != 0 && c->plat->fill_req) { // dedicated Copy engine
        const int m = c->plat->fill_req(vmid, dst, size, DW(5));
        if (m == NGFW_MEM_ABORT) return NGFW_BLOCKED;  // as COPY
        if (m != NGFW_MEM_OK) return ngfw_memfault(c, "FILL fault");
        return NGFW_OK;
      }
      for (uint32_t i = 0; i < NGFW_STAGING_DW; ++i) c->staging[i] = DW(5);
      while (size != 0) {
        uint32_t chunk = NGFW_STAGING_DW * 4u;
        if ((uint64_t)chunk > size) chunk = (uint32_t)size;
        if (chunk > granule_left(dst)) chunk = granule_left(dst);
        if (ngfw_mem_write(c, vmid, dst, c->staging, chunk, "FILL fault") != 0)
          return NGFW_ERROR;
        dst += chunk;
        size -= chunk;
      }
      return NGFW_OK;
    }

    case NGFW_OP_QUERY: {
      const uint64_t addr = ngfw_lohi(DW(1), DW(2));
      if (addr % 8u != 0) return ngfw_bad(c, "QUERY address misaligned");
      if ((DW(3) & ~NGFW_QUERY_KIND_MASK) != 0) return ngfw_bad(c, "QUERY reserved bits");
      const uint32_t kind = DW(3) & NGFW_QUERY_KIND_MASK;
      if (c->plat->wait_idle) c->plat->wait_idle();  // end-of-pipe (4.2)
      if (kind == NGFW_QUERY_OCCLUSION) {
        if (ngfw_mem_write(c, vmid, addr, &c->stat_occlusion, 8, "QUERY fault") != 0)
          return NGFW_ERROR;
        return NGFW_OK;
      }
      if (kind == NGFW_QUERY_PIPELINE_STATS) {
        uint64_t slots[NGFW_PIPELINE_STATS_SLOTS];
        for (uint32_t i = 0; i < NGFW_PIPELINE_STATS_SLOTS; ++i) slots[i] = 0;
        slots[NGFW_PIPELINE_STAT_CS_INVOCATIONS] = c->stat_cs_invocations;
        if (ngfw_mem_write(c, vmid, addr, slots, (uint32_t)sizeof(slots), "QUERY fault") != 0)
          return NGFW_ERROR;
        return NGFW_OK;
      }
      if (kind == NGFW_QUERY_XFB) {
        ngfw_fail(c, NGFW_ERR_UNSUPPORTED, "QUERY XFB is reserved in v0.1");
        return NGFW_ERROR;
      }
      return ngfw_bad(c, "QUERY reserved KIND");
    }

    case NGFW_OP_SET_PREDICATE: {
      const uint32_t ctrl = DW(3);
      if ((ctrl & ~(NGFW_PRED_ENABLE | NGFW_PRED_INVERT)) != 0)
        return ngfw_bad(c, "SET_PREDICATE reserved bits");
      if ((ctrl & NGFW_PRED_ENABLE) == 0) {
        if (DW(1) != 0 || DW(2) != 0 || (ctrl & NGFW_PRED_INVERT) != 0)
          return ngfw_bad(c, "SET_PREDICATE disable with nonzero fields");
        c->cs.pred_enabled = 0;
        c->cs.pred_pass = 1;
        return NGFW_OK;
      }
      const uint64_t addr = ngfw_lohi(DW(1), DW(2));
      if (addr % 4u != 0) return ngfw_bad(c, "SET_PREDICATE address misaligned");
      uint32_t v = 0;
      if (ngfw_mem_read(c, vmid, addr, &v, 4, "SET_PREDICATE fault") != 0) return NGFW_ERROR;
      c->cs.pred_enabled = 1;
      c->cs.pred_pass = ((v != 0) != ((ctrl & NGFW_PRED_INVERT) != 0));
      return NGFW_OK;
    }

    case NGFW_OP_WRITE_DATA: {
      uint64_t addr = ngfw_lohi(DW(1), DW(2));
      if (addr % 4u != 0) return ngfw_bad(c, "WRITE_DATA address misaligned");
      uint32_t left = p->count - 2;  // data dwords (count >= 3)
      uint32_t done = 0;
      while (left != 0) {
        uint32_t chunk = left > NGFW_STAGING_DW ? NGFW_STAGING_DW : left;
        if (ngfw_pkt_fetch(c, p, 2 + done, chunk, c->staging) != 0) return NGFW_ERROR;
        if (ngfw_mem_write(c, vmid, addr, c->staging, chunk * 4u, "WRITE_DATA fault") != 0)
          return NGFW_ERROR;
        addr += 4ull * chunk;
        done += chunk;
        left -= chunk;
      }
      return NGFW_OK;
    }

    default: break;
  }
  ngfw_fail(c, NGFW_ERR_BAD_OPCODE, "unknown opcode");
  return NGFW_ERROR;
}
