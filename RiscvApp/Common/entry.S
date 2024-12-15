// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include "custom_ops.S"

.section .text
	.global irq
	.global trap
	.global multest
	.global hard_mul
	.global hard_mulh
	.global hard_mulhsu
	.global hard_mulhu
	.global hard_div
	.global hard_divu
	.global hard_rem
	.global hard_remu
	.global stats
	.global clear_bits_mip
	.global clear_bits_mie
	.global clear_bits_custom_irq_pend
	.global irq_regs
	.global irq_stack
	.global setup_timer
	.global start

/* Main program
 **********************************/

start:

  /* Disable global interrupt. */
	csrci mstatus, 8

	/* zero-initialize all registers */
	addi x1, zero, 0
	addi x2, zero, 0
	addi x3, zero, 0
	addi x4, zero, 0
	addi x5, zero, 0
	addi x6, zero, 0
	addi x7, zero, 0
	addi x8, zero, 0
	addi x9, zero, 0
	addi x10, zero, 0
	addi x11, zero, 0
	addi x12, zero, 0
	addi x13, zero, 0
	addi x14, zero, 0
	addi x15, zero, 0
	addi x16, zero, 0
	addi x17, zero, 0
	addi x18, zero, 0
	addi x19, zero, 0
	addi x20, zero, 0
	addi x21, zero, 0
	addi x22, zero, 0
	addi x23, zero, 0
	addi x24, zero, 0
	addi x25, zero, 0
	addi x26, zero, 0
	addi x27, zero, 0
	addi x28, zero, 0
	addi x29, zero, 0
	addi x30, zero, 0
	addi x31, zero, 0

	/* set stack pointer */
	lui sp,(32*1024)>>12

	/* set gp and tp */
	lui  gp, %hi(0xdeadbeef)
	addi gp, gp, %lo(0xdeadbeef)
	addi tp, gp, 0

	/* global enable IRQ handling */
	csrw mstatus, 8		// mie = 1

  /* call main C code */
	jal ra,main

	/* trap */
	ebreak

/* Hard mul functions for multest.c
 **********************************/

hard_mul:
	mul a0, a0, a1
	ret

hard_mulh:
	mulh a0, a0, a1
	ret

hard_mulhsu:
	mulhsu a0, a0, a1
	ret

hard_mulhu:
	mulhu a0, a0, a1
	ret

hard_div:
	div a0, a0, a1
	ret

hard_divu:
	divu a0, a0, a1
	ret

hard_rem:
	rem a0, a0, a1
	ret

hard_remu:
	remu a0, a0, a1
	ret

csr_rd:
	remu a0, a0, a1
	ret

// routines to access CSR from C code
clear_bits_mip:
	csrc mip, a0
	ret

clear_bits_mie:
	csrc mie, a0
	ret

clear_bits_custom_irq_pend:
	csrc csr_custom_irq_pend, a0
	ret

setup_timer:
	// a0 contains number of clocks

	// Disable timer interrupt
	addi t0, zero, 0x80
	csrc mie, t0

	// Prepare MTIME_BASE_ADDR in t2
	lui t2, %hi(MTIME_BASE_ADDR)
	addi t2, t2, %lo(MTIME_BASE_ADDR)

	// read mtime into t1:t0
atomic_read_mtime:
	lw t1, 4(t2)
	lw t0, 0(t2)
	lw t3, 4(t2)
	bne t1, t3, atomic_read_mtime

	// Calculate mtimecmp = t1:t0 + a0
	add t0, t0, a0
	sltu t3, t0, a0
	add t1, t1, t3

	// Store mtimecmp
	li t3, -1
	sw t3, 8(t2) # No smaller than old value.
	sw t1, 12(t2) # No smaller than new value.
	sw t0, 8(t2) # New value.

	// Reenable timer interrupt
	addi t0, zero, 0x80
	csrs mie, t0

	ret

# #ifndef ENABLE_QREGS
.balign 0x200
# #endif
irq_regs:
	// registers are saved to this memory region during interrupt handling
	// the program counter is saved as register 0
	.fill 32,  4

	// stack for the interrupt handler
	.fill 256, 4
irq_stack:
