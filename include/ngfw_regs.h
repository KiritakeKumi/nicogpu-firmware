// nicoGPU CP firmware -- BAR0 register offsets used by the firmware.
//
// Values follow the CP block of the command-stream register map (v0.2,
// frozen).  Offsets are bytes from the BAR0 base; all
// registers are 32-bit little-endian.
//
// PROVISIONAL: how the kernel's BAR0 writes reach the firmware
// (hardware register block + write trap vs. firmware-served mailbox) is an
// integration decision.  The firmware keeps its own copy of this block
// (ngfw_mmio_read/ngfw_mmio_write) and publishes changes through
// ngfw_plat::reg_write32 at these same offsets.
#ifndef NGFW_REGS_H
#define NGFW_REGS_H

// Interrupts.
#define NGFW_REG_IRQ_STATUS 0x0040u
#define NGFW_REG_IRQ_ENABLE 0x0044u

// Timestamp (functional counter driven by CP packets/retired instructions;
// the CP block owns it).
#define NGFW_REG_TIMESTAMP_LO 0x0050u
#define NGFW_REG_TIMESTAMP_HI 0x0054u
#define NGFW_REG_TIMESTAMP_FREQ 0x0058u

// CP ring 0 / firmware block (0x0100..0x0154).
#define NGFW_REG_CP_RING_BASE_LO 0x0100u
#define NGFW_REG_CP_RING_BASE_HI 0x0104u
#define NGFW_REG_CP_RING_SIZE_LOG2 0x0108u
#define NGFW_REG_CP_RING_RPTR 0x010cu
#define NGFW_REG_CP_RING_WPTR 0x0110u
#define NGFW_REG_CP_CTRL 0x0114u
#define NGFW_REG_CP_STATUS 0x0118u
#define NGFW_REG_CP_ERROR 0x011cu
#define NGFW_REG_CP_ERROR_RPTR 0x0120u
#define NGFW_REG_CP_ERROR_INFO 0x0124u
#define NGFW_REG_CP_ERROR_IB_OFFSET 0x0128u
#define NGFW_REG_CP_HEARTBEAT 0x012cu
#define NGFW_REG_CP_CMD_VERSION 0x0130u
#define NGFW_REG_CP_FW_VERSION 0x0134u
#define NGFW_REG_CP_FW_STATUS 0x0138u
#define NGFW_REG_CP_FW_BASE_LO 0x0140u
#define NGFW_REG_CP_FW_BASE_HI 0x0144u
#define NGFW_REG_CP_FW_SIZE 0x0148u
#define NGFW_REG_CP_RPTR_WB_LO 0x0150u
#define NGFW_REG_CP_RPTR_WB_HI 0x0154u

// CP_CTRL bits (RESET self-clears, reads as 0; CORE_RESET reset value 1).
#define NGFW_CP_CTRL_ENABLE 0x1u
#define NGFW_CP_CTRL_RESET 0x2u
#define NGFW_CP_CTRL_HALT_REQ 0x4u
#define NGFW_CP_CTRL_CORE_RESET 0x8u

// CP_STATUS bits.
#define NGFW_CP_STATUS_IDLE 0x1u
#define NGFW_CP_STATUS_HALTED 0x2u
#define NGFW_CP_STATUS_WAITING 0x4u
#define NGFW_CP_STATUS_STOPPED 0x8u
#define NGFW_CP_STATUS_FW_READY 0x10u

// CP_ERROR_INFO bits.
#define NGFW_CP_ERROR_INFO_IN_IB 0x1000u

// CP_FW_STATUS bits.
#define NGFW_CP_FW_STATUS_READY 0x1u
#define NGFW_CP_FW_STATUS_PANIC 0x2u
// PROVISIONAL ("abort_timeout"): a dispatch aborted for CP_CTRL.RESET saw
// WDIST_STATUS.ABORT_TIMEOUT (the SP's fabric requests did not drain within the hardware watchdog).  Sticky:
// CP_CTRL.RESET keeps it, CORE_RESET clears it; the kernel answers with a GPU-zone reset.
#define NGFW_CP_FW_STATUS_ABORT_TIMEOUT 0x4u

// CP_RPTR_WB_LO bit 0 is the enable; address bits [31:2].
#define NGFW_CP_RPTR_WB_ENABLE 0x1u

// Doorbell page (own 4 KiB page; v0 implements ring 0 only).
#define NGFW_REG_DOORBELL0 0x1000u

#endif  // NGFW_REGS_H
