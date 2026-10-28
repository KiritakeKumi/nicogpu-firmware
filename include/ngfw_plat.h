// nicoGPU CP firmware -- platform abstraction layer.
//
// The firmware core (ngfw_core.c / ngfw_pkt.c) is pure C99 with no libc and
// no hardware knowledge.  Everything that touches the device goes through
// one `ngfw_plat` vtable with two implementations:
//
//   * host: backed by the golden-vector test
//     harness (flat memory windows, fault-injection regions, dispatch and
//     register-write recorders).  Used by the host unit tests.
//   * Ibex: volatile MMIO at the PROVISIONAL
//     port addresses below; compiled for the real RV32 target.
//
// Freestanding C99: implementations must not pull in libc or platform headers.
#ifndef NGFW_PLAT_H
#define NGFW_PLAT_H

#include <stdint.h>

#include "ngfw_cmd.h"

struct ngfw_dispatch_req;

// Return codes for memory accesses: 0 = ok, nonzero = MMU/physical fault.
#define NGFW_MEM_OK 0
#define NGFW_MEM_FAULT 1
// copy_req / fill_req only: the command was abandoned because CP_CTRL.RESET is
// pending (the event loop resets the CP next); not a fault, nothing reported.
#define NGFW_MEM_ABORT 2
// dispatch_req only: see below.
#define NGFW_DISPATCH_ABORT 2
// dispatch_req only (PROVISIONAL): aborted like NGFW_DISPATCH_ABORT, but the hardware reported that
// the SP's in-flight fabric requests did not drain within its watchdog (WDIST_STATUS.ABORT_TIMEOUT).
#define NGFW_DISPATCH_ABORT_TIMEOUT 3

typedef struct ngfw_plat {
  // -- ring / doorbell ------------------------------------------------------
  // Last DOORBELL0 value (monotonic dword counter).  Must be a single atomic
  // load on the target (the doorbell value only moves
  // forward); on Ibex one 32-bit register read is atomic.
  uint32_t (*doorbell_wptr)(void);
  // Physical (VMID 0 / bypass) 32-bit read, used for ring fetches.
  uint32_t (*phys_read32)(uint64_t pa);

  // -- memory through the MMU -----------------------------------------------
  // Read/write `len` bytes at VA `va` in execution VMID `vmid` (vmid 0 =
  // physical).  Little-endian byte order; the callee handles page splits,
  // the VRAM channel interleave (4 KiB granules on
  // C1/C2/C1/C4) and sparse pages.  Return NGFW_MEM_OK or NGFW_MEM_FAULT.
  // `dst`/`src` are firmware-local buffers of at least `len` bytes.
  int (*mem_read)(unsigned vmid, uint64_t va, void *dst, uint32_t len);
  int (*mem_write)(unsigned vmid, uint64_t va, const void *src, uint32_t len);

  // -- engines ---------------------------------------------------------------
  // Submit one compute dispatch to the work distributor and run it to
  // completion (synchronous semantics are legal; the
  // functional model is serial too).  On success return 0 and add the
  // retired instruction count to *instructions (the functional timestamp
  // source).  1 = shader trap/fault -> SHADER_FAULT.
  // NGFW_DISPATCH_ABORT: a CP_CTRL.RESET arrived while the dispatch ran, the
  // dispatch was aborted in the work distributor; not a fault, nothing is reported
  // and the event is left pending for the event loop (as NGFW_MEM_ABORT).
  // NGFW_DISPATCH_ABORT_TIMEOUT: the same abort, but the SP's fabric did
  // not drain within the hardware watchdog.  Not a normal abort: the core latches
  // CP_FW_STATUS.ABORT_TIMEOUT (sticky until CORE_RESET) so the kernel escalates to
  // a GPU-zone reset; the packet is otherwise handled as for NGFW_DISPATCH_ABORT.
  int (*dispatch_req)(const struct ngfw_dispatch_req *req, uint64_t *instructions);
  // Wait until every previously submitted DISPATCH/COPY/FILL has completed
  // and its writes reached the coherence point (end-of-pipe).  Called before
  // FENCE/TIMESTAMP/QUERY writes and CACHE(WAIT_IDLE); a no-op while all
  // engines are synchronous.
  void (*wait_idle)(void);

  // -- device registers / interrupts ------------------------------------------
  // Publish a firmware-owned BAR0 register value (offsets from ngfw_regs.h:
  // CP_RING_RPTR, CP_ERROR*, CP_FW_*, CP_STATUS, ...).  May be NULL when the
  // register block is served directly from firmware memory (host tests).
  void (*reg_write32)(uint32_t off, uint32_t val);
  // Raise IRQ_STATUS bits (and MSI if enabled) -- ngfw_regs.h NGFW_IRQ_*.
  void (*irq_raise)(uint32_t bits);

  // -- time / events / diagnostics --------------------------------------------
  // Free-running platform time, used only for pacing and the watchdog; NOT
  // the spec-visible timestamp (that is the CP tick counter, ngfw_ticks()).
  uint64_t (*timestamp64)(void);
  // Optional low-power wait until the next doorbell/timer event (Ibex: wfi).
  // NULL = spin.  Never called by the host tests.
  void (*wait_event)(void);
  // Diagnostics: fixed tag + numeric code (see NGFW_LOG_*).  NULL = silent.
  void (*log)(const char *tag, uint32_t code);

  // -- additions (all optional; NULL = old behaviour, so the host
  //    tests and the differential test are unaffected) ---------------
  // Dedicated Copy-engine requests.  `vmid` is the
  // execution VMID of both ends (0 = physical).  The engine copies/fills
  // `size` bytes (byte granularity for COPY, 4-byte multiple for FILL), the
  // call returns when it is done: NGFW_MEM_OK, or NGFW_MEM_FAULT when any
  // access faulted (the engine stops at the faulting access), NGFW_MEM_ABORT
  // when a pending CP_CTRL.RESET stopped it early.  NULL = the
  // firmware moves the data itself through mem_read/mem_write.
  int (*copy_req)(unsigned vmid, uint64_t src, uint64_t dst, uint64_t size);
  int (*fill_req)(unsigned vmid, uint64_t dst, uint64_t size, uint32_t value);
  // Called at the end of every ngfw_service() when the functional timestamp
  // or the heartbeat changed, so the hardware register block (which answers
  // the host) can serve TIMESTAMP_LO/HI and CP_HEARTBEAT.
  void (*publish_aux)(uint64_t ticks, uint32_t heartbeat);
  // S5b: invalidate the SP's caches.  `flags` = NGFW_CACHE_INV_L1 / _INV_K / _INV_I (ngfw_cmd.h; the
  // bit positions of CACHE.FLAGS).  Called by CACHE (after WAIT_IDLE / WB_L2 / INV_L2, in that order) and by the
  // implicit invalidate of every INDIRECT (C-INV); the hardware does the invalidate in the SP's next idle L1D cycle.
  // NULL = nothing to invalidate (host tests, functional model: the counters are the only observable).
  void (*sp_inv)(uint32_t flags);
} ngfw_plat;

// Log codes passed to ngfw_plat::log.
#define NGFW_LOG_BOOT_READY 1u // firmware handshake done
#define NGFW_LOG_HALT 2u        // error latch taken; code = NGFW_ERR_*
#define NGFW_LOG_RESET 3u       // CP_CTRL.RESET / CORE_RESET observed
#define NGFW_LOG_STOPPED 4u     // HALT_REQ honoured at a boundary
#define NGFW_LOG_ABORT_TIMEOUT 5u  // dispatch abort: fabric did not drain (CP_FW_STATUS.ABORT_TIMEOUT)

// ---------------------------------------------------------------------------
// Ibex target port map (offsets in ngfw_ibex_port.h).  Still PROVISIONAL
// as a whole.
//
// The CP sits inside the reconfigurable partition and reaches the rest of the
// device over a private 32-bit data bus (Ibex LSU port).  Two regions:
//
//   NGFW_IBEX_REG_BASE  (0x40000000)
//     Window onto the BAR0 CP page: the same offsets as ngfw_regs.h.
//     Reads return the value the host would read; writes of
//     firmware-owned registers are the "publish" path,
//     writes of other offsets are ignored.
//
//   NGFW_IBEX_PORT_BASE (0x40020000)
//     CP-private port, 32-bit registers (offsets in ngfw_ibex_port.h):
//       DOORBELL    R   last DOORBELL0 value (hardware WPTR latch)
//       IRQ_SET     W   OR bits into IRQ_STATUS
//       EVENT       RW  pending host events {DOORBELL,CTRL,RING,RESET};
//                       write 1 clears (the Ibex interrupt is their OR)
//       TIME_LO/HI  R   free-running platform time
//       ENGINES_IDLE R  bit0 SP idle, bit1 copy engine idle
//       MEM_*       RW  one 1/2/4/8-byte access through the MMU (or
//                       physical when MEM_VMID = 0)
//       WDIST_*     RW  work-distributor request port (42 dword sequence,
//                       see ngfw_plat_ibex.c), completion status and abort
//                       (WDIST_CTRL / WDIST_STATUS bits in ngfw_ibex_port.h)
//       COPY_*      RW  Copy engine (COPY/FILL), see ngfw_plat_ibex.c
//       SP_INV      W   S5b cache invalidate of the SP: bit0 L1D, bit1 K (= L1D), bit2 I$
//       DEBUG       W   log codes (NGFW_LOG_*)
// ---------------------------------------------------------------------------
#define NGFW_IBEX_REG_BASE 0x40000000u
#define NGFW_IBEX_PORT_BASE 0x40020000u

// Fills `plat` with the Ibex implementation (see the port map above).
// Compiled on all hosts as plain C, so the cross build stays warning-clean;
// only meaningful when linked into the RV32 firmware image.
void ngfw_plat_ibex_init(ngfw_plat *plat);

#endif  // NGFW_PLAT_H
