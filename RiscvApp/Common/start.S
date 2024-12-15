// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#define ENABLE_CSR_MSCRATCH
#define ENABLE_RVTST
#define ENABLE_MULTST
#define ENABLE_STATS

// #ifndef ENABLE_QREGS
// #  undef ENABLE_RVTST
// #endif

// Only save registers in IRQ wrapper that are to be saved by the caller in
// the RISC-V ABI, with the excpetion of the stack pointer. The IRQ handler
// will save the rest if necessary. I.e. skip x3, x4, x8, x9, and x18-x27.
#undef ENABLE_FASTIRQ

#include "custom_ops.S"

	.section .text

reset_vec:
	// no more than 16 bytes here !
	j start

scratch_space:
  j  .  // irq_regs base
  j  .  // os irq entry
  j  .  //


/* Interrupt handler
 **********************************/

.balign 16
irq_vec:
	nop
	nop
	nop
	nop
	/* save registers */
#ifdef ENABLE_CSR_MSCRATCH
	// Store x1 to memory 0x04
	sw x1,  4(x0)
#else
	// Store ra directly in application stack
	addi sp, sp, -4
	sw x1, 0*4(sp)
#endif

	// ra = &irq_regs
	lui x1, %hi(irq_regs)
	addi x1, x1, %lo(irq_regs)

	// Store sp
	sw sp, 0*4(x1)

#ifdef ENABLE_FASTIRQ
	sw x5,   5*4(x1)
	sw x6,   6*4(x1)
	sw x7,   7*4(x1)
	sw x10, 10*4(x1)
	sw x11, 11*4(x1)
	sw x12, 12*4(x1)
	sw x13, 13*4(x1)
	sw x14, 14*4(x1)
	sw x15, 15*4(x1)
	sw x16, 16*4(x1)
	sw x17, 17*4(x1)
	sw x28, 28*4(x1)
	sw x29, 29*4(x1)
	sw x30, 30*4(x1)
	sw x31, 31*4(x1)
#else
	sw x3,   3*4(x1)
	sw x4,   4*4(x1)
	sw x5,   5*4(x1)
	sw x6,   6*4(x1)
	sw x7,   7*4(x1)
	sw x8,   8*4(x1)
	sw x9,   9*4(x1)
	sw x10, 10*4(x1)
	sw x11, 11*4(x1)
	sw x12, 12*4(x1)
	sw x13, 13*4(x1)
	sw x14, 14*4(x1)
	sw x15, 15*4(x1)
	sw x16, 16*4(x1)
	sw x17, 17*4(x1)
	sw x18, 18*4(x1)
	sw x19, 19*4(x1)
	sw x20, 20*4(x1)
	sw x21, 21*4(x1)
	sw x22, 22*4(x1)
	sw x23, 23*4(x1)
	sw x24, 24*4(x1)
	sw x25, 25*4(x1)
	sw x26, 26*4(x1)
	sw x27, 27*4(x1)
	sw x28, 28*4(x1)
	sw x29, 29*4(x1)
	sw x30, 30*4(x1)
	sw x31, 31*4(x1)
#endif

	/* call interrupt handler C function */
	lui sp, %hi(irq_stack)
	addi sp, sp, %lo(irq_stack)

  csrr ra, mcause
  srli ra, ra, 31
  bnez ra, not_a_trap

  // li   a1, 0xffff1000
  // sw   a0, (a1)

	// Is it a trap?
	csrr ra, csr_custom_trap
	andi ra, ra, 1		// mask mtrap bit
	beqz ra, not_a_trap


trap_handler:
	// call trap handler
	csrr a0, mepc
	csrr a1, mcause
	csrr a2, mtval

	jal ra, trap

	// if return value is not zero, consider trap correctly handled
	bnez a0, trap_handled

	// stop CPU
	ebreak

trap_handled:
	// ok, trap handled, clear mtrap and return

	// mtrap_prev = 0
	csrci csr_custom_trap, 2

	// check whether trapped instruction is a compressed one
	csrr x1, mepc
	lh x2, 0(ra)
	andi x2, x2, 3		// mask op[1:0]
	addi x3, zero, 3
	bne x2, x3, not_compressed
	addi x1, x1, 2

not_compressed:
	addi x1, x1, 2

	// x1 contains pointer to the next instruction
	csrw mepc, x1

	// Proceed to recalling registers
	j irq_vec_end

not_a_trap:

	// It's a normal IRQ
	// arg0 = mip & mie
	csrr a0, mip
	csrr ra, mie
	and a0, a0, ra

	// arg1 = custom_csr_irq_pend
	csrr a1, csr_custom_irq_pend

	// call to C function
	jal ra, irq

	beqz  a0, irq_vec_end

	lui x1, %hi(irq_regs)
	addi x1, x1, %lo(irq_regs)
	lw x3,   3*4(x1)
	lw x4,   4*4(x1)
	lw x5,   5*4(x1)
	lw x6,   6*4(x1)
	lw x7,   7*4(x1)
	lw x8,   8*4(x1)
	lw x9,   9*4(x1)
	lw x10, 10*4(x1)
	lw x11, 11*4(x1)
	lw x12, 12*4(x1)
	lw x13, 13*4(x1)
	lw x14, 14*4(x1)
	lw x15, 15*4(x1)
	lw x16, 16*4(x1)
	lw x17, 17*4(x1)
	lw x18, 18*4(x1)
	lw x19, 19*4(x1)
	lw x20, 20*4(x1)
	lw x21, 21*4(x1)
	lw x22, 22*4(x1)
	lw x23, 23*4(x1)
	lw x24, 24*4(x1)
	lw x25, 25*4(x1)
	lw x26, 26*4(x1)
	lw x27, 27*4(x1)
	lw x28, 28*4(x1)
	lw x29, 29*4(x1)
	lw x30, 30*4(x1)
	lw x31, 31*4(x1)

	// Restore sp
	lw sp, 0*4(x1)
	lw x1, 4(x0)

  // jump to OS mtvec
  jalr x0, -4(x0)

	j .

irq_vec_end:
  /* restore registers */
	lui x1, %hi(irq_regs)
	addi x1, x1, %lo(irq_regs)

#ifdef ENABLE_FASTIRQ
	lw x5,   5*4(x1)
	lw x6,   6*4(x1)
	lw x7,   7*4(x1)
	lw x10, 10*4(x1)
	lw x11, 11*4(x1)
	lw x12, 12*4(x1)
	lw x13, 13*4(x1)
	lw x14, 14*4(x1)
	lw x15, 15*4(x1)
	lw x16, 16*4(x1)
	lw x17, 17*4(x1)
	lw x28, 28*4(x1)
	lw x29, 29*4(x1)
	lw x30, 30*4(x1)
	lw x31, 31*4(x1)
#else
	lw x3,   3*4(x1)
	lw x4,   4*4(x1)
	lw x5,   5*4(x1)
	lw x6,   6*4(x1)
	lw x7,   7*4(x1)
	lw x8,   8*4(x1)
	lw x9,   9*4(x1)
	lw x10, 10*4(x1)
	lw x11, 11*4(x1)
	lw x12, 12*4(x1)
	lw x13, 13*4(x1)
	lw x14, 14*4(x1)
	lw x15, 15*4(x1)
	lw x16, 16*4(x1)
	lw x17, 17*4(x1)
	lw x18, 18*4(x1)
	lw x19, 19*4(x1)
	lw x20, 20*4(x1)
	lw x21, 21*4(x1)
	lw x22, 22*4(x1)
	lw x23, 23*4(x1)
	lw x24, 24*4(x1)
	lw x25, 25*4(x1)
	lw x26, 26*4(x1)
	lw x27, 27*4(x1)
	lw x28, 28*4(x1)
	lw x29, 29*4(x1)
	lw x30, 30*4(x1)
	lw x31, 31*4(x1)
#endif

	// Restore sp
	lw sp, 0*4(x1)

#ifdef ENABLE_CSR_MSCRATCH
	// Restore x1 from memory 0x04
	lw x1, 4(x0)
#else
	// Restore ra from application stack
	lw x1, 0*4(sp)
	addi sp, sp, 4
#endif


	mret
