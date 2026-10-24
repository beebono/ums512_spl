/*
 * cfw EL3 monitor: header, secondary reset entry, vectors, NS entry.
 */
#include "monitor.h"

/* x0..x30 frame on the EL3 stack, 16-byte aligned */
#define FRAME_SIZE	(32 * 8)

/* \reg = top of this core's EL3 stack; the core index must be < MON_NCPUS */
.macro mon_stack_top reg, tmp
	mrs	\tmp, mpidr_el1
	ubfx	\tmp, \tmp, #8, #8
	add	\tmp, \tmp, #1
	adr	\reg, mon_stacks
	add	\reg, \reg, \tmp, lsl #MON_STACK_SHIFT
.endm

	.section .text.head, "ax"
	.globl	mon_head
mon_head:
	b	mon_warm_entry
	.word	MON_MAGIC
	.word	MON_VERSION
	.word	__image_end - mon_head
	.quad	mon_init
	.quad	mon_vectors
	.quad	mon_stacks + (1 << MON_STACK_SHIFT)

	.text
/*
 * Secondary reset entry (RVBARADDRn = MON_BASE >> 2): EL3, MMU and caches
 * off, SP and much of the system state UNKNOWN.
 */
mon_warm_entry:
	ldr	x0, =0x30c50830			/* SCTLR_EL3 RES1, MMU/caches off, LE */
	msr	sctlr_el3, x0
	isb
	adr	x0, mon_vectors
	msr	vbar_el3, x0
	isb
	mrs	x0, mpidr_el1
	ubfx	x0, x0, #8, #8
	cmp	x0, #MON_NCPUS
	b.hs	mon_park
	mon_stack_top x1, x2
	mov	sp, x1
	bl	mon_warm			/* (core), does not return */

mon_park:
	wfi
	b	mon_park

/*
 * mon_enter_ns(entry, context): eret to non-secure EL2h at entry with
 * x0 = context and this core's EL3 stack reset for the next SMC.
 */
	.globl	mon_enter_ns
mon_enter_ns:
	msr	elr_el3, x0
	mov	x0, #0x3c9			/* EL2h, DAIF masked */
	msr	spsr_el3, x0
	mon_stack_top x2, x3
	mov	sp, x2
	mov	x0, x1
	mov	x1, xzr
	mov	x2, xzr
	mov	x3, xzr
	isb
	eret

/* Forward an exception to the same slot of the SPL's EL3 vectors, which
 * record it in uboot_log (or handle the EL2 stub's smc #1 checkpoint).
 * Clobbers x9, as the SPL's own entry clobbers x9..x11. */
.macro mon_forward idx
	ldr	x9, mon_spl_vbar
	cbz	x9, mon_park
	add	x9, x9, #((\idx) * 0x80)
	br	x9
.endm

.macro mon_fwd_vector idx
	.align	7
	mon_forward \idx
.endm

/* Reload x0..x30 from the frame and drop it. */
.macro mon_restore
	ldp	x0, x1, [sp, #16 * 0]
	ldp	x2, x3, [sp, #16 * 1]
	ldp	x4, x5, [sp, #16 * 2]
	ldp	x6, x7, [sp, #16 * 3]
	ldp	x8, x9, [sp, #16 * 4]
	ldp	x10, x11, [sp, #16 * 5]
	ldp	x12, x13, [sp, #16 * 6]
	ldp	x14, x15, [sp, #16 * 7]
	ldp	x16, x17, [sp, #16 * 8]
	ldp	x18, x19, [sp, #16 * 9]
	ldp	x20, x21, [sp, #16 * 10]
	ldp	x22, x23, [sp, #16 * 11]
	ldp	x24, x25, [sp, #16 * 12]
	ldp	x26, x27, [sp, #16 * 13]
	ldp	x28, x29, [sp, #16 * 14]
	ldr	x30, [sp, #16 * 15]
	add	sp, sp, #FRAME_SIZE
.endm

	.align	11
mon_vectors:
	mon_fwd_vector 0		/* current EL, SP_EL0 */
	mon_fwd_vector 1
	mon_fwd_vector 2
	mon_fwd_vector 3
	mon_fwd_vector 4		/* current EL, SP_ELx: a monitor bug */
	mon_fwd_vector 5
	mon_fwd_vector 6
	mon_fwd_vector 7
	.align	7			/* lower EL, AArch64: sync */
	b	mon_sync_lower
	mon_fwd_vector 9
	mon_fwd_vector 10
	mon_fwd_vector 11
	mon_fwd_vector 12		/* lower EL, AArch32 */
	mon_fwd_vector 13
	mon_fwd_vector 14
	mon_fwd_vector 15

mon_sync_lower:
	sub	sp, sp, #FRAME_SIZE
	stp	x0, x1, [sp, #16 * 0]
	stp	x2, x3, [sp, #16 * 1]
	stp	x4, x5, [sp, #16 * 2]
	stp	x6, x7, [sp, #16 * 3]
	stp	x8, x9, [sp, #16 * 4]
	stp	x10, x11, [sp, #16 * 5]
	stp	x12, x13, [sp, #16 * 6]
	stp	x14, x15, [sp, #16 * 7]
	stp	x16, x17, [sp, #16 * 8]
	stp	x18, x19, [sp, #16 * 9]
	stp	x20, x21, [sp, #16 * 10]
	stp	x22, x23, [sp, #16 * 11]
	stp	x24, x25, [sp, #16 * 12]
	stp	x26, x27, [sp, #16 * 13]
	stp	x28, x29, [sp, #16 * 14]
	str	x30, [sp, #16 * 15]

	/* SMC64 (EC 0x17) or SMC32 (EC 0x13) with imm16 0 is PSCI; smc #1/#2
	 * from the EL2 stub and anything else goes to the SPL. */
	mrs	x9, esr_el3
	lsr	x10, x9, #26
	cmp	x10, #0x17
	b.eq	1f
	cmp	x10, #0x13
	b.ne	2f
1:	tst	x9, #0xffff
	b.ne	2f

	mov	x0, sp
	bl	mon_smc
	mon_restore
	eret

2:	mon_restore
	mon_forward 8
