/* nicoGPU CP firmware -- Ibex RV32 start-up (PROVISIONAL, not built in CI).
 *
 * Entry from the boot ROM / reset vector.  Sets up the stack, clears .bss,
 * enables the machine external interrupt line in `mie` (so `wfi` wakes on a
 * host event) while leaving mstatus.MIE clear: the firmware polls the EVENT
 * register after every wake-up instead of taking traps.
 * The firmware has no initialised writable data (all state is zero-initialised
 * .bss), so there is no .data copy loop; link.ld asserts that.
 */
/* Ibex (boot_addr_i = 0): mtvec = 0x1 (vectored), exceptions at 0x00, interrupt
 * cause N at 4*N, reset vector at 0x80 (link.ld places .text.start there).  The
 * firmware polls and never enables mstatus.MIE; every vector parks the core. */
    .section .text.vectors, "ax"
    .option push
    .option norvc
    .globl _vectors
_vectors:
    .rept 32
    j       _trap
    .endr
    .option pop

    .section .text.start, "ax"
    .globl _start
_start:
    la      sp, __stack_top
    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b
2:  li      t0, 0x880            /* mie.MEIE (external) | mie.MTIE (1 ms tick, wakes wfi; mstatus.MIE stays 0) */
    csrw    mie, t0
    call    ngfw_ibex_main
3:  wfi
    j       3b
_trap:
4:  j       4b
