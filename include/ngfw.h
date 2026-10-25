// nicoGPU CP firmware -- public interface of the firmware core.
//
// The core implements the CP of command stream v0.2 (frozen) with the semantics of the functional
// model: ring + doorbell processing, every stable packet, the
// error latch, HALT_REQ, RPTR write-back and the firmware handshake.
//
// Lifecycle:
//   ngfw_init(&plat)   once at the firmware entry point (boot ROM jumped to
//                      the image entry; CORE_RESET may still be held);
//   ngfw_mmio_write    kernel writes into the BAR0 CP page (ring config,
//                      CP_CTRL, ...);
//   ngfw_doorbell      DOORBELL0 write;
//   ngfw_service       one event-loop turn: drain the ring until the CP is
//                      idle / waiting / stopped / halted;
//   ngfw_main_loop     the real firmware's for(;;) { ngfw_service(); wfi; }.
//
// No dynamic memory, no recursion, no libc: the whole core is a single
// static context (ngfw_internal.h) inside ngfw_core.c.
#ifndef NGFW_H
#define NGFW_H

#include <stdint.h>

#include "ngfw_cmd.h"
#include "ngfw_plat.h"

#ifdef __cplusplus
extern "C" {
#endif

// Everything a compute dispatch needs; filled by the CP from the current
// compute state (mirror of sp::DispatchInfo, minus what the SP fills itself).
typedef struct ngfw_dispatch_req {
  unsigned vmid;
  uint64_t code_va;                        // entry PC (absolute VA)
  uint64_t code_base;                      // CODE_BASE (PC-relative offsets)
  uint64_t scratch_va;                     // SET_SHADER SCRATCH_VA (0 = none)
  uint32_t scratch_bytes;                  // per lane
  uint32_t grid[3];                        // in workgroups
  uint32_t wg[3];                          // in threads
  uint32_t shared_bytes;
  uint32_t num_vregs;                      // per thread
  uint32_t user_uregs[NGFW_USER_UREGS];    // u0..u25 preloaded by the CP
} ngfw_dispatch_req;

// Cache maintenance performed (the functional model has no caches; the
// counters make the implicit C-INV clause and explicit CACHE observable).
typedef struct ngfw_cache_stats {
  uint64_t inv_l1, inv_k, inv_i, wb_l2, inv_l2, wait_idle;
  uint64_t ib_starts;  // INDIRECTs started (each implies INV_L1|INV_K|INV_I)
} ngfw_cache_stats;

// ngfw_service() outcome.
typedef enum {
  NGFW_SVC_IDLE = 0,     // RPTR == WPTR, nothing in flight
  NGFW_SVC_WAIT_RING = 1,  // header packet not fully written yet (7.2)
  NGFW_SVC_WAITING = 2,  // blocked in WAIT_MEM (re-call to re-poll)
  NGFW_SVC_STOPPED = 3,  // HALT_REQ honoured at a boundary
  NGFW_SVC_ERROR = 4,    // halted on a latched error
  NGFW_SVC_OFF = 5,      // !ENABLE / CORE_RESET / !FW_READY
} ngfw_svc_status;

// Firmware entry: bind the platform and set power-on state (CORE_RESET held,
// not READY) -- mirrors the Cp constructor of the functional model.
void ngfw_init(const ngfw_plat *plat);

// BAR0 CP-page register block (offsets from ngfw_regs.h, 0x0100..0x0154).
// CP_CTRL writes and doorbells can (re)start processing, exactly like the
// model: mmio_write(CP_CTRL) and ngfw_doorbell() both call ngfw_service().
//
// On the real device the BAR0 CP page is a
// HARDWARE register block; the host never goes through firmware.  Here
// ngfw_mmio_write() is the handler of a "host wrote this register" event
// (run from the Ibex interrupt with the value held in the block),
// ngfw_mmio_read() is the host-visible view of the block, and
// ngfw_doorbell() is the doorbell event.  The CORE_RESET transitions handled
// in ngfw_mmio_write() are host-test emulation only: hardware resets Ibex.
uint32_t ngfw_mmio_read(uint32_t off);
void ngfw_mmio_write(uint32_t off, uint32_t val);

// DOORBELL0: the new WPTR (monotonic dword counter).
void ngfw_doorbell(uint32_t wptr);

// One event-loop turn; safe to call from any wait state.
ngfw_svc_status ngfw_service(void);

// 1 ms heartbeat tick from the hardware timer (Ibex build): bumps CP_HEARTBEAT and publishes it, nothing else.
void ngfw_tick(void);

// Background re-poll of a blocked WAIT_MEM: like ngfw_service(), but a poll that
// finds the wait still blocked leaves the functional timestamp unchanged
// (hardware polls continuously; the model re-polls on events only).
ngfw_svc_status ngfw_repoll(void);

// Real firmware main loop: service + wait for the next event, forever.
void ngfw_main_loop(void);

// --- observability (tests, devcoredump helpers) ------------------------------
uint64_t ngfw_ticks(void);            // functional timestamp counter
uint32_t ngfw_rptr(void);
uint32_t ngfw_wptr(void);
int ngfw_halted(void);
int ngfw_waiting(void);
int ngfw_stopped(void);
int ngfw_idle(void);                  // CP_STATUS.IDLE
uint32_t ngfw_status(void);           // CP_STATUS value
ngfw_err_code ngfw_error(void);
uint32_t ngfw_error_rptr(void);
uint32_t ngfw_error_info(void);
uint32_t ngfw_error_ib_offset(void);
uint64_t ngfw_dispatches(void);
uint64_t ngfw_abort_timeouts(void); // dispatch aborts that reported ABORT_TIMEOUT
uint64_t ngfw_pred_skipped(void);
const ngfw_cache_stats *ngfw_cache(void);

#ifdef __cplusplus
}
#endif

#endif  // NGFW_H
