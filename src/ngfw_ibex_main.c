// nicoGPU CP firmware -- Ibex entry point: boot handshake and event loop.
//
// Runs on the real core after the boot ROM jumped to the image, and -- built
// with -DNGFW_BUS_HOOK -- inside the xsim DPI behavioural CPU, where it is the
// very same code driving the RTL register block.
//
// The BAR0 CP page is a hardware register block. The host
// never talks to this code; it learns about host writes through the EVENT
// bits of the private port (the level behind the Ibex interrupt) and reads
// the *current* register values back from the register mirror, so the result
// depends only on the final register state (WPTR only moves forward).
#include "ngfw.h"
#include "ngfw_ibex_port.h"
#include "ngfw_regs.h"

volatile uint32_t ngfw_ibex_quiet;

static ngfw_plat g_plat;

static uint32_t reg_rd(uint32_t off) { return mm32(regmirror(off)); }

// Host-owned configuration registers that do not by themselves start work
// (the model's mmio_write32 only stores them).
static void apply_ring_regs(void) {
  static const uint32_t offs[] = {
      NGFW_REG_CP_RING_BASE_LO, NGFW_REG_CP_RING_BASE_HI, NGFW_REG_CP_RING_SIZE_LOG2,
      NGFW_REG_CP_FW_BASE_LO,   NGFW_REG_CP_FW_BASE_HI,   NGFW_REG_CP_FW_SIZE,
      NGFW_REG_CP_RPTR_WB_LO,   NGFW_REG_CP_RPTR_WB_HI,
  };
  for (unsigned i = 0; i < sizeof(offs) / sizeof(offs[0]); ++i)
    ngfw_mmio_write(offs[i], reg_rd(offs[i]));
}

// A doorbell event always re-runs the event loop, even when the value did not
// change (the model re-polls a blocked WAIT_MEM on every doorbell write).
static void sync_doorbell(int force) {
  const uint32_t w = mm32(port(P_DOORBELL));
  if (force || w != ngfw_wptr()) ngfw_doorbell(w);
}

static void handle_events(uint32_t ev) {
  if ((ev & NGFW_EV_RING) != 0) apply_ring_regs();
  if ((ev & (NGFW_EV_CTRL | NGFW_EV_RESET)) != 0) {
    // RESET reads back as 0 in the register block; the event carries it.
    uint32_t ctrl = reg_rd(NGFW_REG_CP_CTRL);
    if ((ev & NGFW_EV_RESET) != 0) ctrl |= NGFW_CP_CTRL_RESET;
    ngfw_mmio_write(NGFW_REG_CP_CTRL, ctrl);
  }
  sync_doorbell((ev & NGFW_EV_DOORBELL) != 0);
}

void ngfw_ibex_main(void) {
  ngfw_plat_ibex_init(&g_plat);
  ngfw_init(&g_plat);

  // Boot: the core was released from CORE_RESET, so the CTRL value written
  // here is the power-on -> running transition of the firmware context and
  // performs the 7.4 handshake (versions, READY, IRQ_CP_FW).
  mm32_write(port(P_EVENT), 0xfu);  // events raised while in reset are stale
  apply_ring_regs();
  ngfw_mmio_write(NGFW_REG_CP_CTRL, reg_rd(NGFW_REG_CP_CTRL) & ~NGFW_CP_CTRL_RESET);
  sync_doorbell(0);

  uint32_t stalled = 0;
  for (;;) {
    if ((mm32(port(P_TICK)) & 1u) != 0) {  // 1 ms hardware tick: CP_HEARTBEAT advances while the CP is idle (7.5)
      mm32_write(port(P_TICK), 1u);
      ngfw_tick();
    }
    const uint32_t ev = mm32(port(P_EVENT));
    if (ev != 0) {
      ngfw_ibex_quiet = 0;
      stalled = 0;
      mm32_write(port(P_EVENT), ev);  // W1C before reading the registers
      handle_events(ev);
    } else if (ngfw_waiting()) {
      // WAIT_MEM keeps re-polling memory (no event wakes it up).  The quiet
      // indicator rises once two polls in a row made no progress.
      const uint64_t t0 = ngfw_ticks();
      const uint32_t r0 = ngfw_rptr();
      (void)ngfw_repoll();
      if (ngfw_waiting() && ngfw_ticks() == t0 && ngfw_rptr() == r0) {
        if (stalled < 2) ++stalled;
        if (stalled >= 2) ngfw_ibex_quiet = 1;
      } else {
        stalled = 0;
        ngfw_ibex_quiet = 0;
      }
    } else {
      ngfw_ibex_quiet = 1;
      if (g_plat.wait_event) g_plat.wait_event();
    }
  }
}
