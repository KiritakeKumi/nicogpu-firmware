// Host-side platform for the CP firmware golden-vector tests.
//
// Simulates just enough of the device for the firmware core: per-VMID flat
// memory windows, fault-injection ranges, a work distributor that records
// ngfw_dispatch_req, IRQ accumulation and the register-write trace.  Host
// only: uses libc, unlike the firmware core.
#ifndef NGFW_HOSTPLAT_H
#define NGFW_HOSTPLAT_H

#include <stdint.h>

#include "ngfw.h"

// 16 VMIDs x 4 MiB windows; a VA is masked into its VMID's window.  The
// harness cannot model the full 48-bit space; golden vectors only use VAs
// inside one window (documented per vector).
#define NGFW_TEST_WIN_LOG2 22
#define NGFW_TEST_WIN_DW (1u << (NGFW_TEST_WIN_LOG2 - 2))
#define NGFW_TEST_NVMID 16

#define NGFW_TEST_MAX_DISPATCH 64
#define NGFW_TEST_MAX_FAULTS 8
#define NGFW_TEST_MAX_REGWRITES 512

typedef struct {
  unsigned vmid;
  uint64_t start, end;  // faulting VA range [start, end) in this VMID
} ngfw_test_fault_range;

typedef struct {
  uint32_t off, val;
} ngfw_test_regwrite;

typedef struct ngfw_host {
  ngfw_plat plat;
  uint8_t *mem;  // NGFW_TEST_NVMID windows, zero-initialised
  uint32_t irq_bits;
  ngfw_dispatch_req dispatch[NGFW_TEST_MAX_DISPATCH];
  uint32_t n_dispatch;
  uint32_t dispatch_fail;    // next N dispatch requests fail (SHADER_FAULT)
  uint32_t dispatch_abort;   // next N dispatch requests return NGFW_DISPATCH_ABORT (CP_CTRL.RESET mid-dispatch)
  uint32_t dispatch_abort_tmo; // next N return NGFW_DISPATCH_ABORT_TIMEOUT (the abort's fabric drain timed out)
  uint64_t instr_per_group;  // retired instructions reported per workgroup
  ngfw_test_fault_range faults[NGFW_TEST_MAX_FAULTS];
  uint32_t n_faults;
  ngfw_test_regwrite regwrites[NGFW_TEST_MAX_REGWRITES];
  uint32_t n_regwrites;
  // Ring contents written by the scenario (dword counter = default doorbell).
  uint32_t ring_dw_written;
} ngfw_host;

// Prepare the harness (allocates memory; call ngfw_host_init(NULL) to free).
void ngfw_host_init(ngfw_host *h);
// The singleton behind the platform callbacks (no closure args in the vtable).
ngfw_host *ngfw_host_current(void);
// Reset scenario state (memory zeroed, recorders cleared); keeps allocation.
void ngfw_host_reset(ngfw_host *h);

// Host-side memory peek/poke (bypasses the firmware), full 64-bit VAs.
int ngfw_host_poke(ngfw_host *h, unsigned vmid, uint64_t va, const uint32_t *dws,
                   uint32_t ndw);
int ngfw_host_peek(ngfw_host *h, unsigned vmid, uint64_t va, uint32_t *dws, uint32_t ndw);
// Record a faulting VA range [start, end) for `vmid`; clears on failure(=full).
void ngfw_host_add_fault(ngfw_host *h, unsigned vmid, uint64_t start, uint64_t end);

#endif  // NGFW_HOSTPLAT_H
