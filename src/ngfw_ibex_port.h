// nicoGPU CP firmware -- Ibex private port offsets and MMIO accessors.
//
// Single source for the firmware side; the RTL counterpart is the CP port package
// (kept in sync by a testbench that drives the real firmware through
// these addresses).  PROVISIONAL.
//
// NGFW_BUS_HOOK: the simulation build (xsim DPI behavioural CPU) replaces the
// volatile pointer access by two functions provided by the DPI layer, so the
// very same firmware sources run against the RTL.
#ifndef NGFW_IBEX_PORT_H
#define NGFW_IBEX_PORT_H

#include <stdint.h>

#include "ngfw_plat.h"

// Port register offsets (bytes from NGFW_IBEX_PORT_BASE).
#define P_DOORBELL 0x00u
#define P_IRQ_SET 0x04u
#define P_EVENT 0x08u
#define P_TIME_LO 0x0cu
#define P_TIME_HI 0x10u
#define P_ENGINES_IDLE 0x14u
#define P_MEM_ADDR_LO 0x20u
#define P_MEM_ADDR_HI 0x24u
#define P_MEM_VMID 0x28u
#define P_MEM_CTRL 0x2cu
#define P_MEM_STATUS 0x30u
#define P_MEM_DATA 0x34u
#define P_MEM_DATA_HI 0x38u
#define P_WDIST_REQ 0x40u
#define P_WDIST_STATUS 0x44u
#define P_WDIST_INSTR_LO 0x48u
#define P_WDIST_INSTR_HI 0x4cu
#define P_DEBUG 0x50u
#define P_WDIST_CTRL 0x54u
#define P_SP_INV 0x58u  // S5b: write bit0 INV_L1, bit1 INV_K, bit2 INV_I (xg_cp_port sp_inv_o pulses)
#define P_COPY_SRC_LO 0x60u
#define P_COPY_SRC_HI 0x64u
#define P_COPY_DST_LO 0x68u
#define P_COPY_DST_HI 0x6cu
#define P_COPY_SIZE_LO 0x70u
#define P_COPY_SIZE_HI 0x74u
#define P_COPY_VMID 0x78u
#define P_COPY_FILL_VALUE 0x7cu
#define P_COPY_CTRL 0x80u
#define P_COPY_STATUS 0x84u
#define P_TICK 0x90u // bit0: 1 ms tick pending (RO); write 1 clears. Wakes the Ibex through mip.MTIP

// EVENT bits (host writes that need the firmware).
#define NGFW_EV_DOORBELL 0x1u
#define NGFW_EV_CTRL 0x2u
#define NGFW_EV_RING 0x4u   // CP_RING_*, CP_FW_*, CP_RPTR_WB_* writes
#define NGFW_EV_RESET 0x8u  // CP_CTRL.RESET written 1

// MEM_CTRL encodings.
#define MEM_OP_READ 0u
#define MEM_OP_WRITE 1u
#define MEM_CTRL_OP(op, size_log2) ((op) | ((size_log2) << 2))
#define MEM_STATUS_BUSY 0x1u
#define MEM_STATUS_FAULT 0x2u

// COPY_CTRL: write starts the operation; ABORT stops a running one (in-flight
// accesses drain, then BUSY falls; no FAULT).
#define COPY_OP_COPY 1u
#define COPY_OP_FILL 2u
#define COPY_CTRL_ABORT 4u
#define COPY_STATUS_BUSY 0x1u
#define COPY_STATUS_FAULT 0x2u

#define WDIST_GO 0xffffffffu
// WDIST_STATUS: bit0 busy, bit1 fault, bit2 aborting (WDIST_CTRL.ABORT written; the SP is being reset and its fabric
// requests drain), bit3 aborted (the last dispatch was aborted; cleared by the next GO).  WDIST_CTRL: write bit0 = ABORT
// the running dispatch; the Ibex is not reset.
// bit4 abort_timeout (PROVISIONAL): set when the abort's fabric-drain watchdog (default 2^20 clk_ts, about
// 10.5 ms) expired with SP fabric slots still allocated; it is set no later than the cycle aborting falls, cleared by
// the next ABORT write and by CORE_RESET (a GO does not clear it). RMs without read 0. Software reads it
// only once, after its own abort has drained (busy and aborting both 0), and keeps its own copy.
#define WDIST_STATUS_BUSY 0x1u
#define WDIST_STATUS_FAULT 0x2u
#define WDIST_STATUS_ABORTING 0x4u
#define WDIST_STATUS_ABORTED 0x8u
#define WDIST_STATUS_ABORT_TIMEOUT 0x10u
#define WDIST_CTRL_ABORT 0x1u

#ifdef NGFW_BUS_HOOK
uint32_t ngfw_bus_rd32(uint32_t addr);
void ngfw_bus_wr32(uint32_t addr, uint32_t val);
void ngfw_bus_wfi(void);  // sleep until the CPU interrupt line is high
static inline uint32_t mm32(uint64_t addr) { return ngfw_bus_rd32((uint32_t)addr); }
static inline void mm32_write(uint64_t addr, uint32_t v) { ngfw_bus_wr32((uint32_t)addr, v); }
#else
static inline uint32_t mm32(uint64_t addr) { return *(volatile uint32_t *)(uintptr_t)addr; }
static inline void mm32_write(uint64_t addr, uint32_t v) {
  *(volatile uint32_t *)(uintptr_t)addr = v;
}
#endif

static inline uint32_t port(uint32_t off) { return NGFW_IBEX_PORT_BASE + off; }
static inline uint32_t regmirror(uint32_t off) { return NGFW_IBEX_REG_BASE + off; }

// Entry point of the real firmware image (ngfw_ibex_main.c): boot handshake
// and the event loop.  Never returns.
void ngfw_ibex_main(void);
// Event-loop quiet indicator for simulation: nonzero once the last loop
// iteration found no pending event and nothing left to do or poll.
extern volatile uint32_t ngfw_ibex_quiet;

#endif  // NGFW_IBEX_PORT_H
