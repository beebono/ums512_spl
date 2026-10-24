/*
 * EL3 -> EL2 handoff into an arm64 Linux Image (CFW boot path).
 *
 * x0 = kernel entry, x1 = dtb, x2/x3 = the EL3 monitor's VBAR_EL3/SP_EL3 to
 * leave behind (x2 = 0: keep the SPL's vectors and stack). The kernel gets
 * x0 = dtb, x1..x3 = 0, MMU and caches off (the SPL never enables them), EL2h
 * with DAIF masked.
 */
#include <config.h>
#include <linux/linkage.h>

#ifdef CONFIG_SPL_BOOT_LINUX

/* Reserved cfw-el2-page (ums512-rg-rotate-cfw.dts): EL2 vectors + stub at
 * the start, then the EL2 record; linux_boot.c keeps its canary above. */
#define CFW_EL2_PAGE		0x80002000
#define CFW_EL2_RECORD_OFF	0xe00

#define ICC_SRE_EL3	S3_6_C12_C12_5

.macro cfw_magic reg, a, b, c, d
	movz	\reg, #\a
	movk	\reg, #\b, lsl #16
	movk	\reg, #\c, lsl #32
	movk	\reg, #\d, lsl #48
.endm

ENTRY(spl_enter_linux_el2)
	mov	x19, x0
	mov	x20, x1
	mov	x21, x2
	mov	x22, x3

	/* Catch faults in the setup below too (e.g. an UNDEFINED system register
	 * on this SoC), not only after the eret. */
	adr	x2, cfw_el3_vectors
	msr	vbar_el3, x2
	isb

	/* GICv3 system-register interface at EL3, lower ELs allowed to use theirs
	 * (Enable). ICC_SRE_EL2 is UNDEFINED from here on this SoC (traps EC 0x0);
	 * the kernel sets it from EL2 itself. ACTLR_EL3/EL2 are left at reset, as
	 * u-boot's armv8_switch_to_el2 does. */
	mov	x2, #0xf			/* SRE | DFB | DIB | Enable */
	msr	ICC_SRE_EL3, x2
	isb

	msr	mdcr_el3, xzr			/* no debug/PMU traps to EL3 */
	msr	cptr_el3, xzr			/* no FP/SIMD traps */
	msr	cntvoff_el2, xzr

	ldr	x2, =0x30c50830			/* SCTLR_EL2 RES1, MMU/caches off */
	msr	sctlr_el2, x2

	/* NS | RES1[5:4] | HCE | RW: next EL is non-secure AArch64 EL2. SMC is
	 * left enabled so EL2 diagnostics can trap back into cfw_el3_vectors. */
	mov	x2, #0x531
	msr	scr_el3, x2

	/* EL2 exceptions before the stub runs land in IRAM and smc up to EL3. */
	adr	x2, cfw_el2_smc_vectors
	msr	vbar_el2, x2

	/* Enter through the EL2 stub, copied by linux_boot.c to CFW_EL2_PAGE. */
	ic	iallu
	dsb	sy
	isb
	adr	x2, cfw_el2_entry
	adr	x3, cfw_el2_blob
	sub	x2, x2, x3
	ldr	x3, =CFW_EL2_PAGE
	add	x2, x2, x3

	mov	x3, #0x3c9			/* EL2h, DAIF masked */
	msr	spsr_el3, x3
	msr	elr_el3, x2

	/* From here on EL3 belongs to the monitor (PSCI); it forwards anything
	 * else back to cfw_el3_vectors. */
	cbz	x21, 1f
	msr	vbar_el3, x21
	mov	sp, x22
1:	isb
	eret				/* x19 = kernel, x20 = dtb carry over */
ENDPROC(spl_enter_linux_el2)

/* Every EL3 vector: index via x10, the trapped x0..x5 stashed, then
 * vector/ESR/ELR/FAR/SPSR into C. Clobbers x9/x10 of the trapped context. */
.macro cfw_vector idx
	.align	7
	mov	x10, #\idx
	b	cfw_el3_trap_entry
.endm

	.pushsection .vectors, "ax"
	.align	11
	.globl	cfw_el3_vectors
cfw_el3_vectors:
	cfw_vector 0
	cfw_vector 1
	cfw_vector 2
	cfw_vector 3
	cfw_vector 4
	cfw_vector 5
	cfw_vector 6
	cfw_vector 7
	cfw_vector 8
	cfw_vector 9
	cfw_vector 10
	cfw_vector 11
	cfw_vector 12
	cfw_vector 13
	cfw_vector 14
	cfw_vector 15
	.popsection

/*
 * EL2 stub, run from DRAM (copied to CFW_EL2_PAGE) in non-secure EL2 right
 * after the eret, so it is fully PC-relative. It marks the EL2 record
 * "EL2ENTRY", points VBAR_EL2 at its own vectors, and enters the kernel. Any
 * EL2 exception before the kernel installs its own vectors overwrites the
 * record with "EL2TRAP!" and the syndrome, then parks. The SPL reports the
 * record after a reset that keeps DRAM.
 */
.macro cfw_el2_vector idx
	.align	7
	mov	x0, #\idx
	b	cfw_el2_trap
.endm


	.pushsection .vectors, "ax"
	.align	11
	.globl	cfw_el2_blob
cfw_el2_blob:
	cfw_el2_vector 0
	cfw_el2_vector 1
	cfw_el2_vector 2
	cfw_el2_vector 3
	cfw_el2_vector 4
	cfw_el2_vector 5
	cfw_el2_vector 6
	cfw_el2_vector 7
	cfw_el2_vector 8
	cfw_el2_vector 9
	cfw_el2_vector 10
	cfw_el2_vector 11
	cfw_el2_vector 12
	cfw_el2_vector 13
	cfw_el2_vector 14
	cfw_el2_vector 15

cfw_el2_entry:
	smc	#1			/* checkpoint: reached the DRAM stub at EL2 */
	adr	x2, cfw_el2_blob
	add	x2, x2, #CFW_EL2_RECORD_OFF
	cfw_magic x3, 0x4c45, 0x4532, 0x544e, 0x5952	/* "EL2ENTRY" */
	str	x3, [x2]
	dsb	sy
	adr	x3, cfw_el2_blob
	msr	vbar_el2, x3
	isb
	mov	x0, x20
	mov	x1, xzr
	mov	x2, xzr
	mov	x3, xzr
	br	x19

cfw_el2_trap:
	adr	x2, cfw_el2_blob
	add	x2, x2, #CFW_EL2_RECORD_OFF
	cfw_magic x3, 0x4c45, 0x5432, 0x4152, 0x2150	/* "EL2TRAP!" */
	stp	x3, x0, [x2]
	mrs	x3, esr_el2
	mrs	x4, elr_el2
	stp	x3, x4, [x2, #16]
	mrs	x3, far_el2
	mrs	x4, spsr_el2
	stp	x3, x4, [x2, #32]
	dsb	sy
1:	wfe
	b	1b
	.globl	cfw_el2_blob_end
cfw_el2_blob_end:
	.popsection

/* EL2 vectors in IRAM: report any EL2 exception to EL3 as smc #2 with
 * x0 = vector, x1 = ESR_EL2, x2 = ELR_EL2, x3 = FAR_EL2. */
.macro cfw_el2_smc_vector idx
	.align	7
	mov	x0, #\idx
	b	cfw_el2_smc_report
.endm

	.pushsection .vectors, "ax"
	.align	11
cfw_el2_smc_vectors:
	cfw_el2_smc_vector 0
	cfw_el2_smc_vector 1
	cfw_el2_smc_vector 2
	cfw_el2_smc_vector 3
	cfw_el2_smc_vector 4
	cfw_el2_smc_vector 5
	cfw_el2_smc_vector 6
	cfw_el2_smc_vector 7
	cfw_el2_smc_vector 8
	cfw_el2_smc_vector 9
	cfw_el2_smc_vector 10
	cfw_el2_smc_vector 11
	cfw_el2_smc_vector 12
	cfw_el2_smc_vector 13
	cfw_el2_smc_vector 14
	cfw_el2_smc_vector 15
cfw_el2_smc_report:
	mrs	x1, esr_el2
	mrs	x2, elr_el2
	mrs	x3, far_el2
	smc	#2
	b	.
	.popsection

cfw_el3_trap_entry:
	/* smc #1 from the EL2 stub is a checkpoint, not a failure: mark the
	 * reserved page "EL2SMC1!" (reported in uboot_log sector 3, +0x180 on
	 * the next boot) and return to the stub. Uses only x9/x11. */
	mrs	x9, esr_el3
	ldr	x11, =0x5e000001		/* EC 0x17 SMC64, IL, imm 1 */
	cmp	x9, x11
	b.ne	1f
	ldr	x9, =(CFW_EL2_PAGE + 0xf80)
	cfw_magic x11, 0x4c45, 0x5332, 0x434d, 0x2131	/* "EL2SMC1!" */
	str	x11, [x9]
	dsb	sy
	eret
1:
	ldr	x9, =0x808e0000			/* CFW_EL3_SAVED_REGS */
	stp	x0, x1, [x9]
	stp	x2, x3, [x9, #16]
	stp	x4, x5, [x9, #32]
	mov	x0, x10				/* vector index, from the stub */
	ldr	x1, =0x808f0000			/* SPL stack, below the kernel */
	mov	sp, x1
	mrs	x1, esr_el3
	mrs	x2, elr_el3
	mrs	x3, far_el3
	mrs	x4, spsr_el3
	bl	cfw_el3_trap
	b	.

#endif /* CONFIG_SPL_BOOT_LINUX */
