/* nicoGPU CP firmware -- Ibex link script (PROVISIONAL sizes).
 * IMEM 64 KiB and DMEM 32 KiB are placeholders;
 * ci-riscv.yml enforces the 64 KiB code budget on the objects.
 */
ENTRY(_start)
/* Ibex boot_addr_i = 0: vector table 0x00..0x7f, reset vector 0x80 */
MEMORY
{
  IMEM (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
  DMEM (rwx) : ORIGIN = 0x00100000, LENGTH = 32K
}
SECTIONS
{
  .text : { KEEP(*(.text.vectors)) . = 0x80; *(.text.start) *(.text*) *(.rodata*) } > IMEM
  .bss  : { __bss_start = .; *(.sbss*) *(.bss*) *(COMMON) __bss_end = .; } > DMEM
  .data : { *(.data*) *(.sdata*) } > DMEM
  ASSERT(SIZEOF(.data) == 0, "firmware must not have initialised writable data (crt0 does not copy .data)")
  __stack_top = ORIGIN(DMEM) + LENGTH(DMEM);
}
