// nicoGPU CP firmware -- Ibex (RV32) platform implementation.
//
// Every access goes through the port map documented in ngfw_plat.h /
// ngfw_ibex_port.h (PROVISIONAL, implemented by the CP RTL).  The file is
// plain freestanding C99 and compiles on the host too, so `run_host_tests.py`
// builds it with both host compilers to keep the cross build warning-clean.
#include <stdint.h>

#include "ngfw.h"
#include "ngfw_regs.h"

#include "ngfw_ibex_port.h"

static void mem_wait_done(void) {
  while ((mm32(port(P_MEM_STATUS)) & MEM_STATUS_BUSY) != 0) {
  }
}

static int mem_check_fault(void) {
  const uint32_t st = mm32(port(P_MEM_STATUS));
  if ((st & MEM_STATUS_FAULT) != 0) {
    mm32_write(port(P_MEM_STATUS), MEM_STATUS_FAULT);  // W1C
    return NGFW_MEM_FAULT;
  }
  return NGFW_MEM_OK;
}

// One bus access of 1/2/4/8 bytes; `data` is little-endian firmware memory.
static int mem_access(unsigned vmid, uint64_t va, void *data, uint32_t len, int is_write) {
  uint8_t *p = (uint8_t *)data;
  uint64_t addr = va;
  uint32_t left = len;
  while (left != 0) {
    uint32_t size_log2;
    if ((addr & 7u) == 0 && left >= 8u)
      size_log2 = 3;
    else if ((addr & 3u) == 0 && left >= 4u)
      size_log2 = 2;
    else if ((addr & 1u) == 0 && left >= 2u)
      size_log2 = 1;
    else
      size_log2 = 0;
    const uint32_t bytes = 1u << size_log2;
    // lo/hi u32 halves: variable shifts of a 64-bit value would pull in libgcc helpers.
    uint32_t v_lo = 0, v_hi = 0;
    if (is_write) {
      for (uint32_t i = 0; i < bytes; ++i) {
        if (i < 4u) v_lo |= (uint32_t)p[i] << (8u * i);
        else v_hi |= (uint32_t)p[i] << (8u * (i - 4u));
      }
    }
    mm32_write(port(P_MEM_ADDR_LO), (uint32_t)addr);
    mm32_write(port(P_MEM_ADDR_HI), (uint32_t)(addr >> 32));
    mm32_write(port(P_MEM_VMID), vmid);
    if (is_write) {
      mm32_write(port(P_MEM_DATA), v_lo);
      if (size_log2 == 3) mm32_write(port(P_MEM_DATA_HI), v_hi);
    }
    mm32_write(port(P_MEM_CTRL),
               MEM_CTRL_OP(is_write ? MEM_OP_WRITE : MEM_OP_READ, size_log2));
    mem_wait_done();
    if (mem_check_fault() != NGFW_MEM_OK) return NGFW_MEM_FAULT;
    if (!is_write) {
      const uint32_t w_lo = mm32(port(P_MEM_DATA));
      const uint32_t w_hi = (size_log2 == 3) ? mm32(port(P_MEM_DATA_HI)) : 0u;
      for (uint32_t i = 0; i < bytes; ++i)
        p[i] = (i < 4u) ? (uint8_t)(w_lo >> (8u * i)) : (uint8_t)(w_hi >> (8u * (i - 4u)));
    }
    addr += bytes;
    p += bytes;
    left -= bytes;
  }
  return NGFW_MEM_OK;
}

static uint32_t ibex_doorbell_wptr(void) { return mm32(port(P_DOORBELL)); }

static uint32_t ibex_phys_read32(uint64_t pa) {
  uint32_t v = 0;
  (void)mem_access(0, pa, &v, 4, 0);
  return v;
}

static int ibex_mem_read(unsigned vmid, uint64_t va, void *dst, uint32_t len) {
  return mem_access(vmid, va, dst, len, 0);
}

static int ibex_mem_write(unsigned vmid, uint64_t va, const void *src, uint32_t len) {
  return mem_access(vmid, va, (void *)src, len, 1);
}

static void ibex_wait_idle(void) {
  // bit0 SP idle | bit1 copy engine idle.
  while ((mm32(port(P_ENGINES_IDLE)) & 0x3u) != 0x3u) {
  }
}

// S5b: CACHE(INV_L1 / INV_K / INV_I) and the implicit invalidate of every INDIRECT reach the SP.
static void ibex_sp_inv(uint32_t flags) {
  uint32_t v = 0;
  if ((flags & NGFW_CACHE_INV_L1) != 0) v |= 1u;
  if ((flags & NGFW_CACHE_INV_K) != 0) v |= 2u;
  if ((flags & NGFW_CACHE_INV_I) != 0) v |= 4u;
  if (v != 0) mm32_write(port(P_SP_INV), v);
}

static int ibex_dispatch_req(const ngfw_dispatch_req *req, uint64_t *instructions) {
  // PROVISIONAL request format: 42 dword writes, fixed order, last = GO
  // (captures them; xg_cp_pkg::xg_cp_wd_req_t).
  uint32_t w[42];
  w[0] = (uint32_t)req->vmid | (req->num_vregs << 8);
  w[1] = req->shared_bytes;
  w[2] = (uint32_t)req->code_va;
  w[3] = (uint32_t)(req->code_va >> 32);
  w[4] = (uint32_t)req->code_base;
  w[5] = (uint32_t)(req->code_base >> 32);
  w[6] = (uint32_t)req->scratch_va;
  w[7] = (uint32_t)(req->scratch_va >> 32);
  w[8] = req->scratch_bytes;
  w[9] = req->grid[0];
  w[10] = req->grid[1];
  w[11] = req->grid[2];
  w[12] = req->wg[0];
  w[13] = req->wg[1];
  w[14] = req->wg[2];
  for (unsigned i = 0; i < NGFW_USER_UREGS; ++i) w[15 + i] = req->user_uregs[i];
  w[41] = WDIST_GO;
  for (unsigned i = 0; i < 42; ++i) mm32_write(port(P_WDIST_REQ), w[i]);
  // A dispatch can run for minutes: keep the heartbeat moving and take CP_CTRL.RESET (as copy_wait).  The reset
  // aborts the dispatch in the work distributor (SP datapath reset, Ibex untouched), waits until the SP has
  // drained, and leaves the event pending: the event loop performs the CP reset next.
  // the drain ends either because the SP's fabric slots came back or because the hardware watchdog
  // expired (WDIST_STATUS.ABORT_TIMEOUT, read in the same status word that shows the drain over; older RMs read 0).
  while ((mm32(port(P_WDIST_STATUS)) & WDIST_STATUS_BUSY) != 0) {
    if ((mm32(port(P_TICK)) & 1u) != 0) {
      mm32_write(port(P_TICK), 1u);
      ngfw_tick();
    }
    if ((mm32(port(P_EVENT)) & NGFW_EV_RESET) != 0) {
      uint32_t st;
      mm32_write(port(P_WDIST_CTRL), WDIST_CTRL_ABORT);
      do {
        st = mm32(port(P_WDIST_STATUS));
      } while ((st & (WDIST_STATUS_BUSY | WDIST_STATUS_ABORTING)) != 0);
      return (st & WDIST_STATUS_ABORT_TIMEOUT) != 0 ? NGFW_DISPATCH_ABORT_TIMEOUT : NGFW_DISPATCH_ABORT;
    }
  }
  if ((mm32(port(P_WDIST_STATUS)) & WDIST_STATUS_FAULT) != 0) return 1;
  *instructions = (uint64_t)mm32(port(P_WDIST_INSTR_LO)) |
                  ((uint64_t)mm32(port(P_WDIST_INSTR_HI)) << 32);
  return 0;
}

// A large COPY runs for seconds (one 8-byte access at a time over PCIe); the
// kernel's job timeout then writes CP_CTRL.RESET, which must not wait for the
// whole command.  The heartbeat keeps ticking meanwhile: the kernel only tries
// CP_CTRL.RESET (L1) when CP_HEARTBEAT moves, otherwise it declares the
// firmware dead and reloads it.
static int copy_wait(void) {
  while ((mm32(port(P_COPY_STATUS)) & COPY_STATUS_BUSY) != 0) {
    if ((mm32(port(P_TICK)) & 1u) != 0) {
      mm32_write(port(P_TICK), 1u);
      ngfw_tick();
    }
    if ((mm32(port(P_EVENT)) & NGFW_EV_RESET) != 0) {  // left pending: the event loop handles it
      mm32_write(port(P_COPY_CTRL), COPY_CTRL_ABORT);
      while ((mm32(port(P_COPY_STATUS)) & COPY_STATUS_BUSY) != 0) {
      }
      return NGFW_MEM_ABORT;
    }
  }
  if ((mm32(port(P_COPY_STATUS)) & COPY_STATUS_FAULT) != 0) {
    mm32_write(port(P_COPY_STATUS), COPY_STATUS_FAULT);  // W1C
    return NGFW_MEM_FAULT;
  }
  return NGFW_MEM_OK;
}

static void copy_program(unsigned vmid, uint64_t dst, uint64_t size) {
  mm32_write(port(P_COPY_DST_LO), (uint32_t)dst);
  mm32_write(port(P_COPY_DST_HI), (uint32_t)(dst >> 32));
  mm32_write(port(P_COPY_SIZE_LO), (uint32_t)size);
  mm32_write(port(P_COPY_SIZE_HI), (uint32_t)(size >> 32));
  mm32_write(port(P_COPY_VMID), vmid);
}

static int ibex_copy_req(unsigned vmid, uint64_t src, uint64_t dst, uint64_t size) {
  mm32_write(port(P_COPY_SRC_LO), (uint32_t)src);
  mm32_write(port(P_COPY_SRC_HI), (uint32_t)(src >> 32));
  copy_program(vmid, dst, size);
  mm32_write(port(P_COPY_CTRL), COPY_OP_COPY);
  return copy_wait();
}

static int ibex_fill_req(unsigned vmid, uint64_t dst, uint64_t size, uint32_t value) {
  mm32_write(port(P_COPY_FILL_VALUE), value);
  copy_program(vmid, dst, size);
  mm32_write(port(P_COPY_CTRL), COPY_OP_FILL);
  return copy_wait();
}

static void ibex_publish_aux(uint64_t ticks, uint32_t heartbeat) {
  mm32_write(regmirror(NGFW_REG_TIMESTAMP_LO), (uint32_t)ticks);
  mm32_write(regmirror(NGFW_REG_TIMESTAMP_HI), (uint32_t)(ticks >> 32));
  mm32_write(regmirror(NGFW_REG_CP_HEARTBEAT), heartbeat);
}

static void ibex_reg_write32(uint32_t off, uint32_t val) {
  mm32_write(regmirror(off), val);
}

static void ibex_irq_raise(uint32_t bits) { mm32_write(port(P_IRQ_SET), bits); }

static uint64_t ibex_timestamp64(void) {
  // HI-LO-HI retry, so a rollover between the two reads cannot tear.
  uint32_t hi = mm32(port(P_TIME_HI));
  for (;;) {
    const uint32_t lo = mm32(port(P_TIME_LO));
    const uint32_t hi2 = mm32(port(P_TIME_HI));
    if (hi == hi2) return ((uint64_t)hi << 32) | lo;
    hi = hi2;
  }
}

static void ibex_wait_event(void) {
#if defined(NGFW_IBEX)
  __asm__ volatile("wfi");
#elif defined(NGFW_BUS_HOOK)
  ngfw_bus_wfi();
#endif
}

static void ibex_log(const char *tag, uint32_t code) {
  (void)tag;  // the target only gets the numeric code
  mm32_write(port(P_DEBUG), code);
}

void ngfw_plat_ibex_init(ngfw_plat *plat) {
  plat->doorbell_wptr = ibex_doorbell_wptr;
  plat->phys_read32 = ibex_phys_read32;
  plat->mem_read = ibex_mem_read;
  plat->mem_write = ibex_mem_write;
  plat->dispatch_req = ibex_dispatch_req;
  plat->wait_idle = ibex_wait_idle;
  plat->reg_write32 = ibex_reg_write32;
  plat->irq_raise = ibex_irq_raise;
  plat->timestamp64 = ibex_timestamp64;
  plat->wait_event = ibex_wait_event;
  plat->log = ibex_log;
  plat->copy_req = ibex_copy_req;
  plat->fill_req = ibex_fill_req;
  plat->publish_aux = ibex_publish_aux;
  plat->sp_inv = ibex_sp_inv;
}
