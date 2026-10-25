// Host-side platform for the CP firmware golden-vector tests -- implementation.
#include "ngfw_hostplat.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The plat callbacks have no closure argument (they mirror hardware), so the
// harness is a singleton, exactly like the firmware context itself.
static ngfw_host *g_host;

ngfw_host *ngfw_host_current(void) { return g_host; }

static uint64_t window_offset(unsigned vmid, uint64_t va) {
  const uint64_t mask = (1ull << NGFW_TEST_WIN_LOG2) - 1ull;
  return (uint64_t)vmid * (1ull << NGFW_TEST_WIN_LOG2) + (va & mask);
}

static int in_fault(const ngfw_host *h, unsigned vmid, uint64_t va) {
  for (uint32_t i = 0; i < h->n_faults; ++i)
    if (h->faults[i].vmid == vmid && va >= h->faults[i].start && va < h->faults[i].end)
      return 1;
  return 0;
}

static void record_regwrite(ngfw_host *h, uint32_t off, uint32_t val) {
  if (h->n_regwrites < NGFW_TEST_MAX_REGWRITES) {
    h->regwrites[h->n_regwrites].off = off;
    h->regwrites[h->n_regwrites].val = val;
    ++h->n_regwrites;
  }
}

// --- ngfw_plat implementation -------------------------------------------------

static uint32_t host_doorbell_wptr(void) {
  return 0;  // push mode: scenarios call ngfw_doorbell() directly
}

static uint32_t host_phys_read32(uint64_t pa) {
  ngfw_host *h = g_host;
  uint32_t v = 0;
  memcpy(&v, h->mem + window_offset(0, pa), 4);
  return v;
}

static int host_mem_read(unsigned vmid, uint64_t va, void *dst, uint32_t len) {
  ngfw_host *h = g_host;
  if (in_fault(h, vmid, va)) return NGFW_MEM_FAULT;
  memcpy(dst, h->mem + window_offset(vmid, va), len);
  return NGFW_MEM_OK;
}

static int host_mem_write(unsigned vmid, uint64_t va, const void *src, uint32_t len) {
  ngfw_host *h = g_host;
  if (in_fault(h, vmid, va)) return NGFW_MEM_FAULT;
  memcpy(h->mem + window_offset(vmid, va), src, len);
  return NGFW_MEM_OK;
}

static int host_dispatch_req(const ngfw_dispatch_req *req, uint64_t *instructions) {
  ngfw_host *h = g_host;
  if (h->n_dispatch < NGFW_TEST_MAX_DISPATCH) {
    h->dispatch[h->n_dispatch] = *req;
    ++h->n_dispatch;
  }
  if (h->dispatch_abort != 0) {
    --h->dispatch_abort;
    return NGFW_DISPATCH_ABORT;  // nothing retired
  }
  if (h->dispatch_abort_tmo != 0) {
    --h->dispatch_abort_tmo;
    return NGFW_DISPATCH_ABORT_TIMEOUT;  // nothing retired either
  }
  *instructions = (uint64_t)req->grid[0] * req->grid[1] * req->grid[2] * h->instr_per_group;
  if (h->dispatch_fail != 0) {
    --h->dispatch_fail;
    return 1;  // shader trap -> firmware latches SHADER_FAULT
  }
  return 0;
}

static void host_wait_idle(void) { /* engines are synchronous in phase 1 */ }

static void host_reg_write32(uint32_t off, uint32_t val) { record_regwrite(g_host, off, val); }

static void host_irq_raise(uint32_t bits) { g_host->irq_bits |= bits; }

static uint64_t host_timestamp64(void) {
  static uint64_t fake_time;
  return ++fake_time;
}

static void host_log(const char *tag, uint32_t code) {
  printf("  [ngfw] %s (%u)\n", tag, (unsigned)code);
}

// --- harness lifecycle --------------------------------------------------------

void ngfw_host_reset(ngfw_host *h) {
  const uint64_t bytes = (uint64_t)NGFW_TEST_NVMID << NGFW_TEST_WIN_LOG2;
  memset(h->mem, 0, (size_t)bytes);
  h->irq_bits = 0;
  h->n_dispatch = 0;
  h->dispatch_fail = 0;
  h->dispatch_abort = 0;
  h->dispatch_abort_tmo = 0;
  h->instr_per_group = 1;
  h->n_faults = 0;
  h->n_regwrites = 0;
  h->ring_dw_written = 0;
}

void ngfw_host_init(ngfw_host *h) {
  if (h == NULL) {  // tear down the singleton's allocation
    if (g_host != NULL) {
      free(g_host->mem);
      g_host->mem = NULL;
      g_host = NULL;
    }
    return;
  }
  const uint64_t bytes = (uint64_t)NGFW_TEST_NVMID << NGFW_TEST_WIN_LOG2;
  h->mem = (uint8_t *)calloc(1, (size_t)bytes);
  if (h->mem == NULL) {
    fprintf(stderr, "ngfw_host_init: out of memory (%llu MiB)\n",
            (unsigned long long)(bytes >> 20));
    exit(2);
  }
  g_host = h;

  h->plat.doorbell_wptr = host_doorbell_wptr;
  h->plat.phys_read32 = host_phys_read32;
  h->plat.mem_read = host_mem_read;
  h->plat.mem_write = host_mem_write;
  h->plat.dispatch_req = host_dispatch_req;
  h->plat.wait_idle = host_wait_idle;
  h->plat.reg_write32 = host_reg_write32;
  h->plat.irq_raise = host_irq_raise;
  h->plat.timestamp64 = host_timestamp64;
  h->plat.wait_event = NULL;
  h->plat.log = host_log;
  ngfw_host_reset(h);
}

// --- host-side peek/poke ------------------------------------------------------

int ngfw_host_poke(ngfw_host *h, unsigned vmid, uint64_t va, const uint32_t *dws,
                   uint32_t ndw) {
  if (vmid >= NGFW_TEST_NVMID) return 1;
  memcpy(h->mem + window_offset(vmid, va), dws, (size_t)ndw * 4u);
  return 0;
}

int ngfw_host_peek(ngfw_host *h, unsigned vmid, uint64_t va, uint32_t *dws, uint32_t ndw) {
  if (vmid >= NGFW_TEST_NVMID) return 1;
  memcpy(dws, h->mem + window_offset(vmid, va), (size_t)ndw * 4u);
  return 0;
}

void ngfw_host_add_fault(ngfw_host *h, unsigned vmid, uint64_t start, uint64_t end) {
  if (h->n_faults < NGFW_TEST_MAX_FAULTS) {
    h->faults[h->n_faults].vmid = vmid;
    h->faults[h->n_faults].start = start;
    h->faults[h->n_faults].end = end;
    ++h->n_faults;
  }
}
