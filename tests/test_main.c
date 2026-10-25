// CP firmware golden-vector test runner (host only).
//
// Each vector file is a scenario written against
// the same register-level view the kernel driver has:
// ring writes + doorbells + CP register writes drive the firmware core, and
// `check` commands assert the produced memory writes, dispatch requests,
// register values, IRQ bits and the register-publish sequence.  The expected
// values were hand-derived from the functional model (the golden reference).
//
// Usage: ngfw_test [manifest]      (default: vectors/manifest.txt)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ngfw_hostplat.h"

static ngfw_host *H;
static uint64_t g_ring_base;
static uint32_t g_ring_log2, g_ring_mask;

static int g_checks, g_fails;
static const char *g_file;
static int g_line;

// ---------------------------------------------------------------------------
// Failure reporting
static void check_failed(const char *what, unsigned long long got, unsigned long long want) {
  ++g_fails;
  printf("%s:%d: CHECK %s: got %llu (0x%llx), want %llu (0x%llx)\n", g_file, g_line, what, got,
         got, want, want);
}

#define CHK_U64(what, got, want)                                    \
  do {                                                              \
    ++g_checks;                                                     \
    if ((unsigned long long)(got) != (unsigned long long)(want))    \
      check_failed(what, (unsigned long long)(got), (unsigned long long)(want)); \
  } while (0)

#define CHK_STR(what, cond)                                       \
  do {                                                            \
    ++g_checks;                                                   \
    if (!(cond)) {                                                \
      ++g_fails;                                                  \
      printf("%s:%d: CHECK %s failed\n", g_file, g_line, what);   \
    }                                                             \
  } while (0)

// ---------------------------------------------------------------------------
// Line tokenizer
#define MAX_TOK 96

static char *g_tok[MAX_TOK];
static int g_ntok;

static int tokenize(char *line) {
  g_ntok = 0;
  char *s = line;
  for (;;) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;
    if (*s == 0) break;
    if (g_ntok == MAX_TOK) break;
    g_tok[g_ntok++] = s;
    while (*s != 0 && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') ++s;
    if (*s != 0) {
      *s = 0;
      ++s;
    }
  }
  return g_ntok;
}

static int tok_u64(int idx, uint64_t *out) {
  if (idx >= g_ntok) return 0;
  char *end = 0;
  *out = strtoull(g_tok[idx], &end, 0);
  return end != 0 && *end == 0;
}

// ---------------------------------------------------------------------------
// Scenario helpers
static void ring_append(const uint32_t *dws, uint32_t n) {
  // Wrap by the ring mask, like the kernel does when filling the ring.
  for (uint32_t i = 0; i < n; ++i) {
    const uint64_t va = g_ring_base + 4ull * (H->ring_dw_written & g_ring_mask);
    (void)ngfw_host_poke(H, 0, va, &dws[i], 1);
    ++H->ring_dw_written;
  }
}

static void gen_dwords(uint32_t *out, uint32_t n, uint64_t first, uint64_t stride) {
  for (uint32_t i = 0; i < n; ++i) out[i] = (uint32_t)(first + stride * i);
}

static void cmd_init(void) {
  uint64_t log2 = 10, base = 0x100000000ull;  // C3 system region (model Rig)
  (void)tok_u64(1, &log2);
  (void)tok_u64(2, &base);
  g_ring_log2 = (uint32_t)log2;
  g_ring_mask = (1u << g_ring_log2) - 1u;
  g_ring_base = base;
  ngfw_host_reset(H);
  ngfw_init(&H->plat);
  ngfw_mmio_write(0x0100, (uint32_t)g_ring_base);
  ngfw_mmio_write(0x0104, (uint32_t)(g_ring_base >> 32));
  ngfw_mmio_write(0x0108, g_ring_log2);
}

// ---------------------------------------------------------------------------
// One line = one command; returns 0 to abort the file (parse error).
static int run_check(void);

static int run_command(void) {
  const char *cmd = g_tok[0];

  if (strcmp(cmd, "init") == 0) {
    cmd_init();
  } else if (strcmp(cmd, "ctrl") == 0) {
    uint64_t v = 0;
    if (!tok_u64(1, &v)) return 0;
    ngfw_mmio_write(0x0114, (uint32_t)v);
  } else if (strcmp(cmd, "regw") == 0) {
    uint64_t off = 0, v = 0;
    if (!tok_u64(1, &off) || !tok_u64(2, &v)) return 0;
    ngfw_mmio_write((uint32_t)off, (uint32_t)v);
  } else if (strcmp(cmd, "doorbell") == 0) {
    uint64_t v = H->ring_dw_written;
    if (g_ntok >= 2) {
      if (!tok_u64(1, &v)) return 0;
      // An explicit WPTR also moves the ring write position (the kernel
      // cannot doorbell beyond what it wrote; artificial WPTR jumps for
      // RING_CONFIG tests reset the writer accordingly).
      if (v > H->ring_dw_written) H->ring_dw_written = (uint32_t)v;
    }
    ngfw_doorbell((uint32_t)v);
  } else if (strcmp(cmd, "ring") == 0) {
    uint32_t dws[MAX_TOK - 1];
    const int n = g_ntok - 1;
    for (int i = 0; i < n; ++i) {
      uint64_t v = 0;
      if (!tok_u64(1 + i, &v)) return 0;
      dws[i] = (uint32_t)v;
    }
    ring_append(dws, (uint32_t)n);
  } else if (strcmp(cmd, "ringgen") == 0) {
    uint64_t n = 0, first = 0, stride = 1;
    if (!tok_u64(1, &n)) return 0;
    (void)tok_u64(2, &first);
    (void)tok_u64(3, &stride);
    uint32_t dws[MAX_TOK - 1];
    while (n != 0) {
      uint32_t chunk = n > MAX_TOK - 1 ? (uint32_t)(MAX_TOK - 1) : (uint32_t)n;
      gen_dwords(dws, chunk, first, stride);
      ring_append(dws, chunk);
      first += stride * chunk;
      n -= chunk;
    }
  } else if (strcmp(cmd, "ib") == 0) {
    uint64_t vmid = 0, va = 0;
    if (!tok_u64(1, &vmid) || !tok_u64(2, &va)) return 0;
    uint32_t dws[MAX_TOK - 3];
    const int n = g_ntok - 3;
    for (int i = 0; i < n; ++i) {
      uint64_t v = 0;
      if (!tok_u64(3 + i, &v)) return 0;
      dws[i] = (uint32_t)v;
    }
    (void)ngfw_host_poke(H, (unsigned)vmid, va, dws, (uint32_t)n);
  } else if (strcmp(cmd, "ibgen") == 0) {
    uint64_t vmid = 0, va = 0, n = 0, first = 0, stride = 1;
    if (!tok_u64(1, &vmid) || !tok_u64(2, &va) || !tok_u64(3, &n)) return 0;
    (void)tok_u64(4, &first);
    (void)tok_u64(5, &stride);
    uint32_t dws[MAX_TOK - 1];
    while (n != 0) {
      uint32_t chunk = n > MAX_TOK - 1 ? (uint32_t)(MAX_TOK - 1) : (uint32_t)n;
      gen_dwords(dws, chunk, first, stride);
      (void)ngfw_host_poke(H, (unsigned)vmid, va, dws, chunk);
      va += 4ull * chunk;
      first += stride * chunk;
      n -= chunk;
    }
  } else if (strcmp(cmd, "poke") == 0) {
    uint64_t vmid = 0, va = 0;
    if (!tok_u64(1, &vmid) || !tok_u64(2, &va)) return 0;
    uint32_t dws[MAX_TOK - 3];
    const int n = g_ntok - 3;
    for (int i = 0; i < n; ++i) {
      uint64_t v = 0;
      if (!tok_u64(3 + i, &v)) return 0;
      dws[i] = (uint32_t)v;
    }
    (void)ngfw_host_poke(H, (unsigned)vmid, va, dws, (uint32_t)n);
  } else if (strcmp(cmd, "poke64") == 0) {
    uint64_t vmid = 0, va = 0, lo = 0, hi = 0;
    if (!tok_u64(1, &vmid) || !tok_u64(2, &va) || !tok_u64(3, &lo) || !tok_u64(4, &hi)) return 0;
    const uint32_t dws[2] = {(uint32_t)lo, (uint32_t)hi};
    (void)ngfw_host_poke(H, (unsigned)vmid, va, dws, 2);
  } else if (strcmp(cmd, "fault") == 0) {
    uint64_t vmid = 0, start = 0, end = 0;
    if (!tok_u64(1, &vmid) || !tok_u64(2, &start) || !tok_u64(3, &end)) return 0;
    ngfw_host_add_fault(H, (unsigned)vmid, start, end);
  } else if (strcmp(cmd, "nofault") == 0) {
    H->n_faults = 0;
  } else if (strcmp(cmd, "dispfail") == 0) {
    uint64_t n = 0;
    if (!tok_u64(1, &n)) return 0;
    H->dispatch_fail = (uint32_t)n;
  } else if (strcmp(cmd, "dispabort") == 0) {
    uint64_t n = 0;
    if (!tok_u64(1, &n)) return 0;
    H->dispatch_abort = (uint32_t)n;
  } else if (strcmp(cmd, "dispaborttmo") == 0) {
    uint64_t n = 0;
    if (!tok_u64(1, &n)) return 0;
    H->dispatch_abort_tmo = (uint32_t)n;
  } else if (strcmp(cmd, "dispins") == 0) {
    uint64_t n = 1;
    if (!tok_u64(1, &n)) return 0;
    H->instr_per_group = n;
  } else if (strcmp(cmd, "service") == 0) {
    (void)ngfw_service();
  } else if (strcmp(cmd, "irqclear") == 0) {
    H->irq_bits = 0;
  } else if (strcmp(cmd, "regclear") == 0) {
    H->n_regwrites = 0;
  } else if (strcmp(cmd, "check") == 0) {
    return run_check();
  } else {
    printf("%s:%d: unknown command '%s'\n", g_file, g_line, cmd);
    ++g_fails;
    return 0;
  }
  return 1;
}

// `check ...` dispatch; 0 aborts the file.
static int run_check(void) {
  const char *what = g_tok[1];
  if (what == 0) return 0;

  if (strcmp(what, "mmio") == 0) {
    uint64_t off = 0, want = 0;
    if (!tok_u64(2, &off) || !tok_u64(3, &want)) return 0;
    CHK_U64("mmio", ngfw_mmio_read((uint32_t)off), want);
  } else if (strcmp(what, "rptr") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("rptr", ngfw_mmio_read(0x010c), want);
  } else if (strcmp(what, "wptr") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("wptr", ngfw_mmio_read(0x0110), want);
  } else if (strcmp(what, "idle") == 0) {
    CHK_STR("idle", (ngfw_mmio_read(0x0118) & 1u) != 0);
  } else if (strcmp(what, "busy") == 0) {
    CHK_STR("busy", (ngfw_mmio_read(0x0118) & 1u) == 0);
  } else if (strcmp(what, "error") == 0) {
    uint64_t code = 0, rptr = 0, info = 0, iboff = 0;
    if (!tok_u64(2, &code) || !tok_u64(3, &rptr) || !tok_u64(4, &info) || !tok_u64(5, &iboff))
      return 0;
    CHK_U64("CP_ERROR", ngfw_mmio_read(0x011c), code);
    CHK_U64("CP_ERROR_RPTR", ngfw_mmio_read(0x0120), rptr);
    CHK_U64("CP_ERROR_INFO", ngfw_mmio_read(0x0124), info);
    CHK_U64("CP_ERROR_IB_OFFSET", ngfw_mmio_read(0x0128), iboff);
  } else if (strcmp(what, "irq") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("irq", H->irq_bits, want);
  } else if (strcmp(what, "mem") == 0) {
    uint64_t vmid = 0, va = 0, want = 0;
    if (!tok_u64(2, &vmid) || !tok_u64(3, &va) || !tok_u64(4, &want)) return 0;
    uint32_t got = 0;
    (void)ngfw_host_peek(H, (unsigned)vmid, va, &got, 1);
    CHK_U64("mem", got, want);
  } else if (strcmp(what, "mem64") == 0) {
    uint64_t vmid = 0, va = 0, lo = 0, hi = 0;
    if (!tok_u64(2, &vmid) || !tok_u64(3, &va) || !tok_u64(4, &lo) || !tok_u64(5, &hi)) return 0;
    uint32_t got[2] = {0, 0};
    (void)ngfw_host_peek(H, (unsigned)vmid, va, got, 2);
    CHK_U64("mem64.lo", got[0], lo);
    CHK_U64("mem64.hi", got[1], hi);
  } else if (strcmp(what, "dispatch") == 0) {
    uint64_t idx = 0, f[11];
    if (!tok_u64(2, &idx)) return 0;
    for (int i = 0; i < 11; ++i) {
      if (!tok_u64(3 + i, &f[i])) return 0;
    }
    if (idx >= H->n_dispatch) {
      ++g_checks;
      ++g_fails;
      printf("%s:%d: CHECK dispatch: index %llu, only %u recorded\n", g_file, g_line, idx,
             H->n_dispatch);
      return 1;
    }
    const ngfw_dispatch_req *d = &H->dispatch[idx];
    CHK_U64("disp.vmid", d->vmid, f[0]);
    CHK_U64("disp.gx", d->grid[0], f[1]);
    CHK_U64("disp.gy", d->grid[1], f[2]);
    CHK_U64("disp.gz", d->grid[2], f[3]);
    CHK_U64("disp.wgx", d->wg[0], f[4]);
    CHK_U64("disp.wgy", d->wg[1], f[5]);
    CHK_U64("disp.wgz", d->wg[2], f[6]);
    CHK_U64("disp.vregs", d->num_vregs, f[7]);
    CHK_U64("disp.u0", d->user_uregs[0], f[8]);
    CHK_U64("disp.u1", d->user_uregs[1], f[9]);
    CHK_U64("disp.code_lo", (uint32_t)d->code_va, f[10]);
  } else if (strcmp(what, "ndispatch") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("ndispatch", H->n_dispatch, want);
  } else if (strcmp(what, "dispatches") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("dispatches", ngfw_dispatches(), want);
  } else if (strcmp(what, "aborttmo") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("abort_timeouts", ngfw_abort_timeouts(), want);
  } else if (strcmp(what, "skipped") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("pred_skipped", ngfw_pred_skipped(), want);
  } else if (strcmp(what, "cache") == 0) {
    uint64_t f[7];
    for (int i = 0; i < 7; ++i) {
      if (!tok_u64(2 + i, &f[i])) return 0;
    }
    const ngfw_cache_stats *cs = ngfw_cache();
    CHK_U64("cache.inv_l1", cs->inv_l1, f[0]);
    CHK_U64("cache.inv_k", cs->inv_k, f[1]);
    CHK_U64("cache.inv_i", cs->inv_i, f[2]);
    CHK_U64("cache.wb_l2", cs->wb_l2, f[3]);
    CHK_U64("cache.inv_l2", cs->inv_l2, f[4]);
    CHK_U64("cache.wait_idle", cs->wait_idle, f[5]);
    CHK_U64("cache.ib_starts", cs->ib_starts, f[6]);
  } else if (strcmp(what, "ticks") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("ticks", ngfw_ticks(), want);
  } else if (strcmp(what, "reg") == 0) {
    uint64_t idx = 0, off = 0, val = 0;
    if (!tok_u64(2, &idx) || !tok_u64(3, &off) || !tok_u64(4, &val)) return 0;
    if (idx >= H->n_regwrites) {
      ++g_checks;
      ++g_fails;
      printf("%s:%d: CHECK reg: index %llu, only %u recorded\n", g_file, g_line, idx,
             H->n_regwrites);
      return 1;
    }
    CHK_U64("reg.off", H->regwrites[idx].off, off);
    CHK_U64("reg.val", H->regwrites[idx].val, val);
  } else if (strcmp(what, "regn") == 0) {
    uint64_t want = 0;
    if (!tok_u64(2, &want)) return 0;
    CHK_U64("regn", H->n_regwrites, want);
  } else {
    printf("%s:%d: unknown check '%s'\n", g_file, g_line, what);
    ++g_fails;
    return 0;
  }
  return 1;
}

// ---------------------------------------------------------------------------
static int run_vector_file(const char *path) {
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    printf("cannot open %s\n", path);
    return -1;
  }
  g_file = path;
  char line[8192];
  int ok = 1;
  int file_checks = g_checks, file_fails = g_fails;
  g_line = 0;
  while (fgets(line, sizeof(line), f) != NULL) {
    ++g_line;
    char *hash = strchr(line, '#');
    if (hash != NULL) *hash = 0;  // strip comments
    hash = strchr(line, ';');
    if (hash != NULL) *hash = 0;
    if (tokenize(line) == 0) continue;
    if (run_command() == 0) {
      ok = 0;
      break;
    }
  }
  fclose(f);
  const int dchecks = g_checks - file_checks, dfails = g_fails - file_fails;
  printf("%-42s %s (%d checks%s)\n", path, (ok && dfails == 0) ? "PASS" : "FAIL", dchecks,
         dfails != 0 ? ", FAILURES" : "");
  return (ok && dfails == 0) ? 0 : 1;
}

int main(int argc, char **argv) {
  static ngfw_host host;
  H = &host;
  ngfw_host_init(H);

  const char *manifest = argc > 1 ? argv[1] : "vectors/manifest.txt";
  FILE *f = fopen(manifest, "r");
  if (f == NULL) {
    printf("cannot open manifest %s\n", manifest);
    return 2;
  }
  // Vector paths are relative to the manifest's directory.
  char dir[512];
  {
    const char *slash = strrchr(manifest, '/');
    const char *bslash = strrchr(manifest, '\\');
    const char *sep = slash != NULL && (bslash == NULL || slash > bslash) ? slash : bslash;
    if (sep != NULL) {
      size_t n = (size_t)(sep - manifest);
      if (n >= sizeof(dir)) n = sizeof(dir) - 1;
      memcpy(dir, manifest, n);
      dir[n] = 0;
    } else {
      strcpy(dir, ".");
    }
  }

  int failed_files = 0, nfiles = 0;
  char name[512];
  while (fgets(name, sizeof(name), f) != NULL) {
    char *s = name;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' ' || e[-1] == '\t')) --e;
    *e = 0;
    if (*s == 0 || *s == '#') continue;
    char path[1024];
    if (dir[0] != 0 && strcmp(dir, ".") != 0)
      snprintf(path, sizeof(path), "%s/%s", dir, s);
    else
      snprintf(path, sizeof(path), "%s", s);
    ++nfiles;
    if (run_vector_file(path) != 0) ++failed_files;
  }
  fclose(f);
  ngfw_host_init(NULL);

  printf("\n%d vector files, %d checks, %d failures, %d failing files\n", nfiles, g_checks,
         g_fails, failed_files);
  return (g_fails == 0 && failed_files == 0) ? 0 : 1;
}
