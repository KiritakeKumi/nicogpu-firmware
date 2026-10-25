// nicoGPU CP firmware -- freestanding memory helpers (no libc on the target).
#include "ngfw_internal.h"

void *ngfw_memcpy(void *dst, const void *src, uint32_t n) {
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;
  while (n--) *d++ = *s++;
  return dst;
}

void *ngfw_memset(void *dst, uint8_t v, uint32_t n) {
  uint8_t *d = (uint8_t *)dst;
  while (n--) *d++ = v;
  return dst;
}
