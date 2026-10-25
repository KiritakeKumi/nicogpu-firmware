// nicoGPU CP firmware -- command-stream constants, command stream v0.2 (frozen).
//
// Generated file -- DO NOT EDIT.
//
// Firmware-owned values not present in the YAML (FW version,
// MAX_SHARED_PER_WG, PIPELINE_STAT_CS_INVOCATIONS) are template values
// in the generator and marked PROVISIONAL.
//
// Freestanding C99: only <stdint.h>, no libc, no platform headers.
#ifndef NGFW_CMD_H
#define NGFW_CMD_H

#include <stdint.h>

// Versions reported through CP_CMD_VERSION / CP_FW_VERSION.
#define NGFW_CMD_VERSION_MAJOR 0u
#define NGFW_CMD_VERSION_MINOR 2u
#define NGFW_FW_VERSION_MAJOR 0u
#define NGFW_FW_VERSION_MINOR 1u
#define NGFW_FW_VERSION_PATCH 0u

// Opcodes.
enum {
  NGFW_OP_NOP = 0x00,
  NGFW_OP_INDIRECT = 0x01,
  NGFW_OP_FENCE = 0x02,
  NGFW_OP_TIMESTAMP = 0x03,
  NGFW_OP_COPY = 0x04,
  NGFW_OP_DISPATCH = 0x05,
  NGFW_OP_SET_SHADER = 0x06,
  NGFW_OP_SET_USER_REGS = 0x07,
  NGFW_OP_DISPATCH_INDIRECT = 0x08,
  NGFW_OP_WAIT_MEM = 0x09,
  NGFW_OP_CACHE = 0x0a,
  NGFW_OP_FILL = 0x0b,
  NGFW_OP_QUERY = 0x0c,
  NGFW_OP_SET_PREDICATE = 0x0d,
  NGFW_OP_WRITE_DATA = 0x0e,
  NGFW_OP_DRAW = 0x10,  // reserved (P2) -> UNSUPPORTED
  NGFW_OP_DRAW_INDEXED = 0x11,  // reserved (P2) -> UNSUPPORTED
  NGFW_OP_DRAW_INDIRECT = 0x12,  // reserved (P2) -> UNSUPPORTED
  NGFW_OP_DRAW_INDEXED_INDIRECT = 0x13,  // reserved (P2) -> UNSUPPORTED
  NGFW_OP_FENCE_WRITE = NGFW_OP_FENCE,
};

// Packet header.
#define NGFW_HDR_OPCODE_MASK 0x000000ffu
#define NGFW_HDR_PRED 0x00000100u
#define NGFW_HDR_RSVD_MASK 0x0000fe00u
#define NGFW_HDR_COUNT_SHIFT 16u
#define NGFW_COUNT_MAX 0xffffu

// Streams (bit masks, as in PacketInfo::streams).
#define NGFW_STREAM_RING 0x1u
#define NGFW_STREAM_IB 0x2u

// FENCE.FLAGS.
#define NGFW_FENCE_FLAG_BIT64 0x1u
#define NGFW_FENCE_FLAG_IRQ 0x2u
#define NGFW_FENCE_FLAG_L2_WB 0x4u
#define NGFW_FENCE_FLAGS_RSVD (~(0x1u | 0x2u | 0x4u))

// INDIRECT.
#define NGFW_IB_CTRL_VMID_MASK 0xfu
#define NGFW_VA_HI_MASK 0xffffu  // 48-bit VAs: *_HI carries [15:0] = VA[47:32]

// COPY.FLAGS.
#define NGFW_COPY_MODE_MASK 0x3u
#define NGFW_COPY_MODE_LINEAR 0u
#define NGFW_COPY_MODE_LINEAR_TO_T64K 1u
#define NGFW_COPY_MODE_T64K_TO_LINEAR 2u

// SET_SHADER.RSRC and limits.
#define NGFW_RSRC_VREGS_MASK 0xffu
#define NGFW_MAX_VREGS 128u
// PROVISIONAL: the device_info/max_shared_bytes source does not exist
// for the firmware yet; 64 KiB matches the functional model CoreConfig.
#define NGFW_MAX_SHARED_PER_WG (64u * 1024u)

// SET_USER_REGS.
#define NGFW_USER_REGS_START_MASK 0x1fu
#define NGFW_USER_UREGS 26u  // u0..u25 (ABI v0.1)

// WAIT_MEM.CTRL.
#define NGFW_WAIT_FUNC_MASK 0x7u
#define NGFW_WAIT_BIT64 0x8u
enum {
  NGFW_WAIT_EQ = 0,
  NGFW_WAIT_NE = 1,
  NGFW_WAIT_GE = 2,
  NGFW_WAIT_GT = 3,
  NGFW_WAIT_LT = 4,
  NGFW_WAIT_LE = 5,
};

// CACHE.FLAGS (execution order: WAIT_IDLE -> WB_L2 -> INV_L2 -> INV_L1/K/I).
#define NGFW_CACHE_WAIT_IDLE 0x01u
#define NGFW_CACHE_INV_L1 0x02u
#define NGFW_CACHE_INV_K 0x04u
#define NGFW_CACHE_INV_I 0x08u
#define NGFW_CACHE_WB_L2 0x10u
#define NGFW_CACHE_INV_L2 0x20u
#define NGFW_CACHE_FLAGS_RSVD (~(0x1u | 0x2u | 0x4u | 0x8u | 0x10u | 0x20u))

// QUERY.CTRL.
#define NGFW_QUERY_KIND_MASK 0xfu
#define NGFW_QUERY_OCCLUSION 0u
#define NGFW_QUERY_PIPELINE_STATS 1u
#define NGFW_QUERY_XFB 2u
#define NGFW_PIPELINE_STATS_SLOTS 11u
#define NGFW_PIPELINE_STAT_CS_INVOCATIONS 10u

// SET_PREDICATE.CTRL.
#define NGFW_PRED_ENABLE 0x1u
#define NGFW_PRED_INVERT 0x2u

// Limits (nicogpu_cmd.yaml `limits`).
#define NGFW_MAX_IB_DWORDS (1u << 20)   // max INDIRECT size in dwords
#define NGFW_MAX_WG_THREADS 1024u

// CP_ERROR codes.
typedef enum {
  NGFW_ERR_NONE = 0,
  NGFW_ERR_BAD_OPCODE = 1,
  NGFW_ERR_BAD_PACKET = 2,
  NGFW_ERR_MEM_FAULT = 3,
  NGFW_ERR_SHADER_FAULT = 4,
  NGFW_ERR_RING_CONFIG = 5,
  NGFW_ERR_NOT_PERMITTED = 6,
  NGFW_ERR_UNSUPPORTED = 7,
  NGFW_ERR_NO_SHADER_STATE = 8,
  NGFW_ERR_WATCHDOG = 9,
} ngfw_err_code;

// IRQ_STATUS bits (single MSI vector).
#define NGFW_IRQ_FENCE 0x1u
#define NGFW_IRQ_CP_ERROR 0x2u
#define NGFW_IRQ_MMU_FAULT 0x4u
#define NGFW_IRQ_CP_FW 0x8u
#define NGFW_IRQ_CP_STOPPED 0x10u

// Build a packet header dword (tests and diagnostics; the core parses only).
static inline uint32_t ngfw_hdr(uint32_t opcode, uint32_t count, unsigned pred) {
  return (opcode & NGFW_HDR_OPCODE_MASK) | (pred ? NGFW_HDR_PRED : 0u) |
         ((count & 0xffffu) << NGFW_HDR_COUNT_SHIFT);
}

// 64-bit helpers: little-endian dword pairs, low word first.
static inline uint64_t ngfw_lohi(uint32_t lo, uint32_t hi) {
  return ((uint64_t)hi << 32) | (uint64_t)lo;
}

#endif  // NGFW_CMD_H
