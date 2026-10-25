/*
 * cfw EL3 monitor: PSCI 1.0 subset and the ums512 core power sequence.
 *
 * Implemented: PSCI_VERSION, PSCI_FEATURES, CPU_ON, AFFINITY_INFO,
 * MIGRATE_INFO_TYPE and CPU_OFF. CPU_OFF powers the core down through the
 * PMU; CPU_ON powers it up again and it comes back through its reset vector
 * (mon_warm_entry). Everything else returns NOT_SUPPORTED: SYSTEM_RESET/OFF
 * then fall through to the sc27xx handlers, and with no CPU_SUSPEND
 * cpuidle-psci stays out.
 *
 * The register sequences are those of the stock SML (Unisoc TF-A v1.4,
 * plat_sprd_psci_pm_ops), decoded from the v1.44 PAC.
 */
#include "monitor.h"
#include "pmu_seq.h"
#include "csp_ops.h"

#define REG32(a)		(*(volatile uint32_t *)(uintptr_t)(a))
#define REG64(a)		(*(volatile uint64_t *)(uintptr_t)(a))

/* PMU_APB, with its write-1-to-set and write-1-to-clear aliases */
#define PMU_BASE		0x327e0000UL
#define PMU_SET(r)		(PMU_BASE + 0x1000 + (r))
#define PMU_CLR(r)		(PMU_BASE + 0x2000 + (r))
#define PMU_PD_APCPU_TOP_CFG	0x000
#define PMU_APCPU_CORE_FORCE_STOP 0x3d4
#define PMU_APCPU_SOFT_INT_GEN	0x454
#define PMU_APCPU_CSYSPWRUP_WAKEUP_EN 0x4f8
/* One bit per core: set for a suspend, clear for an off (stock 0x1338 vs
 * 0x1604), so presumably whether an interrupt may wake the core. */
#define PMU_APCPU_CORE_WAKEUP_EN 0x174

#define PD_DBG_SHUTDOWN_EN	(1u << 28)
#define PD_SEL			(1u << 27)	/* PD_APCPU_TOP_CFG only */
#define PD_FORCE_SHUTDOWN	(1u << 25)
#define PD_AUTO_SHUTDOWN_EN	(1u << 24)
#define TOP_FORCE_DEEP_STOP	(1u << 8)	/* APCPU_CORE_FORCE_STOP */
#define PWR_STATE_OFF		7

/* Per core, in core (MPIDR Aff1) order; the layout is irregular. */
static const uint16_t pd_cfg[MON_NCPUS] = {
	0x004, 0x008, 0x00c, 0x7d8, 0x7dc, 0x7e0, 0x814, 0x370
};
static const uint16_t dslp_ena[MON_NCPUS] = {
	0x348, 0x34c, 0x350, 0x808, 0x80c, 0x810, 0x800, 0x1e8
};
static const uint16_t pwr_state_reg[MON_NCPUS] = {
	0x378, 0x378, 0x378, 0x820, 0x820, 0x820, 0x820, 0x37c
};
static const uint8_t pwr_state_shift[MON_NCPUS] = {
	8, 16, 24, 24, 16, 8, 0, 0
};
/* Stock core off/suspend also set bit 0 here; name unknown. */
static const uint16_t pd_misc[MON_NCPUS] = {
	0x1bc, 0x1c0, 0x1c4, 0x7e8, 0x7ec, 0x7f0, 0x7f4, 0x1d8
};
/* The byte stock polls before the final WFI of an off or suspend, until
 * (v & ~4) == 2 or v == 0xf. */
static const uint16_t pd_status_reg[MON_NCPUS] = {
	0x3d8, 0x3d8, 0x3d8, 0x840, 0x840, 0x840, 0x840, 0x3dc
};
static const uint8_t pd_status_shift[MON_NCPUS] = {
	0, 8, 16, 0, 8, 16, 24, 0
};

/* Reset vector per core, in words (the roc1 u-boot writes addr >> 2 too). */
#define AON_SEC_RVBARADDR(n)	(0x32880040UL + 4 * (n))

#define GICR_BASE		0x12040000UL
#define GICR_STRIDE		0x20000UL
#define GICR_FRAMES		8
#define GICR_TYPER		0x0008
#define GICR_WAKER		0x0014
#define GICR_PWRR		0x0024
#define GICR_PWRR_RDPD		(1u << 0)
#define GICR_PWRR_RDGPD		(1u << 2)
#define GICR_PWRR_RDGPO		(1u << 3)
#define GICR_WAKER_PSLEEP	(1u << 1)
#define GICR_WAKER_CASLEEP	(1u << 2)
#define GICR_IGROUPR0		(0x10000 + 0x0080)
#define GICR_ISPENDR0		(0x10000 + 0x0200)
#define GICR_IGRPMODR0		(0x10000 + 0x0d00)

#define PSCI_VERSION		0x84000000
#define PSCI_CPU_OFF		0x84000002
#define PSCI_CPU_ON_32		0x84000003
#define PSCI_CPU_ON_64		0xc4000003
#define PSCI_AFFINITY_INFO_32	0x84000004
#define PSCI_AFFINITY_INFO_64	0xc4000004
#define PSCI_MIGRATE_INFO_TYPE	0x84000006
#define PSCI_FEATURES		0x8400000a

#define PSCI_SUCCESS		0
#define PSCI_NOT_SUPPORTED	-1
#define PSCI_INVALID_PARAMS	-2
#define PSCI_ALREADY_ON		-4
#define PSCI_ON_PENDING		-5
#define PSCI_INTERNAL_FAILURE	-6

#define TICKS_PER_MS		(MON_CNTFRQ / 1000)

#define CPUPWRCTLR_CORE_PWRDN_EN	1u
/* Stock on_finish writes 4 << 4 here (WFI_RET_CTRL: dynamic retention
 * after a while in WFI). Left off for now: it was blamed for idle cores
 * dropping off, which turned out to be the missing PMU power sequencing
 * (pmu_seq.h); retention itself is untested here, and stock also sets up
 * cluster power control (plat setup 0x8870) that this monitor doesn't. */
#define CPUPWRCTLR_WFI_RET_OFF		0u

struct mon_regs {
	uint64_t x[31];
	uint64_t pad;
};

uint64_t mon_spl_vbar;		/* SPL's VBAR_EL3, for forwarded exceptions */
uint8_t mon_stacks[MON_NCPUS << MON_STACK_SHIFT] __attribute__((aligned(16)));
static struct mon_status status __attribute__((section(".status")));
static volatile struct mon_status *const st = &status;
static struct mon_status status_prev __attribute__((section(".status_prev")));
/* what each pmu_seq register held before mon_init overwrote it (reset or
 * SPL value), in table order, for devmem at MON_BASE + 0xf400 */
static uint32_t pmu_seq_before[64] __attribute__((section(".status_seq")));
/* likewise for csp_ops, at MON_BASE + 0xf500 */
static uint32_t csp_before[96] __attribute__((section(".status_csp")));

static inline void dsb(void)
{
	asm volatile("dsb sy" ::: "memory");
}

static inline uint64_t ticks(void)
{
	uint64_t t;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (t));
	return t;
}

static inline unsigned int my_core(void)
{
	uint64_t mpidr;

	asm volatile("mrs %0, mpidr_el1" : "=r" (mpidr));
	return (mpidr >> 8) & 0xff;
}

/* PSCI target_cpu -> core index, or -1: only Aff1 may be set. */
static int target_core(uint64_t target)
{
	if (target & ~0xff00ULL || (target >> 8) >= MON_NCPUS)
		return -1;
	return target >> 8;
}

static uint32_t core_pwr_state(unsigned int core)
{
	return (REG32(PMU_BASE + pwr_state_reg[core]) >> pwr_state_shift[core]) & 0xff;
}

static uint32_t core_pd_status(unsigned int core)
{
	return (REG32(PMU_BASE + pd_status_reg[core]) >> pd_status_shift[core]) & 0xff;
}

static inline void write_cpupwrctlr(uint64_t v)
{
	asm volatile("msr S3_0_C15_C2_7, %0\n" "isb" :: "r" (v));
}

/* The per-core EL3 state Linux's boot CPU got from the SPL. */
static void el3_setup(void)
{
	asm volatile(
		"msr	scr_el3, %0\n"		/* NS | RES1 | HCE | RW, SMC on */
		"msr	S3_6_C12_C12_5, %1\n"	/* ICC_SRE_EL3: SRE | DFB | DIB | Enable */
		"isb\n"
		"msr	mdcr_el3, xzr\n"
		"msr	cptr_el3, xzr\n"
		"msr	cntfrq_el0, %2\n"
		:: "r" (0x531UL), "r" (0xfUL), "r" ((uint64_t)MON_CNTFRQ));
}

/* EL2 as Linux expects it at a PSCI entry point: MMU off. Also needed on a
 * CPU_ON after CPU_OFF, where EL2 still holds the old kernel's state. */
static void el2_reset(void)
{
	asm volatile(
		"msr	sctlr_el2, %0\n"
		"msr	cntvoff_el2, xzr\n"
		"isb\n"
		:: "r" (0x30c50830UL));
}

static int wait32(unsigned long reg, uint32_t mask, uint32_t want)
{
	uint64_t t0 = ticks();

	while ((REG32(reg) & mask) != want)
		if (ticks() - t0 > 10 * TICKS_PER_MS)
			return -1;
	return 0;
}

/*
 * Power up and wake this core's GIC-600 redistributor and put its SGIs/PPIs
 * in Group 1 Non-secure, as the SPL's cfw_gic_init() does for CPU0. Linux
 * cannot do any of this from the non-secure side.
 */
/* This core's redistributor frame, matched on GICR_TYPER's affinity. */
static unsigned long gic_redist(unsigned int core, uint32_t *flags)
{
	uint64_t mpidr, aff;
	unsigned int i;

	asm volatile("mrs %0, mpidr_el1" : "=r" (mpidr));
	aff = ((mpidr >> 32) & 0xff) << 24 | (mpidr & 0xffffff);
	for (i = 0; i < GICR_FRAMES; i++)
		if ((REG64(GICR_BASE + i * GICR_STRIDE + GICR_TYPER) >> 32) == aff)
			return GICR_BASE + i * GICR_STRIDE;
	*flags |= MON_F_GIC_NOMATCH;
	return GICR_BASE + core * GICR_STRIDE;
}

static uint32_t gic_redist_init(unsigned int core)
{
	uint32_t flags = 0, pwrr;
	unsigned long rd = gic_redist(core, &flags);
	uint64_t t0;

	/* Clear RDPD once no group power transition is in flight (TF-A
	 * gic600_pwr_on). */
	t0 = ticks();
	for (;;) {
		pwrr = REG32(rd + GICR_PWRR);
		if (!!(pwrr & GICR_PWRR_RDGPD) == !!(pwrr & GICR_PWRR_RDGPO)) {
			REG32(rd + GICR_PWRR) = 0;
			if (!(REG32(rd + GICR_PWRR) & GICR_PWRR_RDPD))
				break;
		}
		if (ticks() - t0 > 10 * TICKS_PER_MS)
			return flags | MON_F_GIC_TIMEOUT;
	}

	REG32(rd + GICR_WAKER) &= ~GICR_WAKER_PSLEEP;
	if (wait32(rd + GICR_WAKER, GICR_WAKER_CASLEEP, 0))
		flags |= MON_F_GIC_TIMEOUT;
	REG32(rd + GICR_IGROUPR0) = 0xffffffff;
	REG32(rd + GICR_IGRPMODR0) = 0;
	return flags;
}

/*
 * TF-A gicv3_cpuif_disable, as stock calls it before a core goes down:
 * no bypass, both groups off at the CPU interface, then the redistributor
 * told the core is asleep.
 */
static uint32_t gic_cpuif_disable(unsigned int core)
{
	uint32_t flags = 0;
	unsigned long rd = gic_redist(core, &flags);
	uint64_t v;

	asm volatile("mrs %0, S3_6_C12_C12_5" : "=r" (v));	/* ICC_SRE_EL3 */
	asm volatile("msr S3_6_C12_C12_5, %0" :: "r" (v | 0x6));	/* DIB | DFB */
	asm volatile("mrs %0, S3_0_C12_C12_6" : "=r" (v));	/* ICC_IGRPEN0_EL1 */
	asm volatile("msr S3_0_C12_C12_6, %0" :: "r" (v & ~1UL));
	asm volatile("mrs %0, S3_6_C12_C12_7" : "=r" (v));	/* ICC_IGRPEN1_EL3 */
	asm volatile("msr S3_6_C12_C12_7, %0\n" "isb" :: "r" (v & ~3UL));

	REG32(rd + GICR_WAKER) |= GICR_WAKER_PSLEEP;
	if (wait32(rd + GICR_WAKER, GICR_WAKER_CASLEEP, GICR_WAKER_CASLEEP))
		flags |= MON_F_GIC_TIMEOUT;
	return flags;
}

/*
 * The stock core power-down (0x1338 for an off, 0x1604 for a suspend, which
 * differ only in the wake-up bit), then the core's own power-down request.
 * Returns once the PMU reports it ready; the caller's WFI powers it off.
 */
static void core_power_down(unsigned int core, int wake_on_irq)
{
	volatile struct mon_cpu *c = &st->cpu[core];
	uint64_t t0;
	uint32_t v;

	c->off_step = MON_OFF_STEP_GIC_OFF;
	c->flags |= gic_cpuif_disable(core);
	/* stock trusts the DynamIQ hardware flush on power-down; a set/way
	 * clean of L1/L2 costs microseconds and doesn't depend on it */
	mon_flush_private_dcache();
	c->off_step = MON_OFF_STEP_FLUSHED;
	dsb();

	REG32(PMU_SET(dslp_ena[core])) = 1;
	if (wake_on_irq)
		REG32(PMU_SET(PMU_APCPU_CORE_WAKEUP_EN)) = 1u << core;
	else
		REG32(PMU_CLR(PMU_APCPU_CORE_WAKEUP_EN)) = 1u << core;
	REG32(PMU_SET(pd_misc[core])) = 1;
	REG32(PMU_CLR(pd_cfg[core])) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(pd_cfg[core])) = PD_AUTO_SHUTDOWN_EN;
	REG32(PMU_CLR(pd_cfg[core])) = PD_SEL;
	dsb();

	asm volatile("msr dbgprcr_el1, xzr");
	write_cpupwrctlr(CPUPWRCTLR_CORE_PWRDN_EN);
	c->off_step = MON_OFF_STEP_PMU_ARMED;
	dsb();

	/* stock waits forever here; give up after 100 ms and note it, so a
	 * wrong guess shows on the status page instead of a silent hang */
	t0 = ticks();
	for (;;) {
		v = core_pd_status(core);
		if ((v & ~4u) == 2 || v == 0xf)
			break;
		if (ticks() - t0 > 100 * TICKS_PER_MS) {
			c->flags |= MON_F_PD_TIMEOUT;
			c->pd_timeouts++;
			break;
		}
	}
	t0 = ticks() - t0;
	if (t0 > c->pd_wait_max)
		c->pd_wait_max = (uint32_t)t0;
	c->pd_seen = v;
	c->off_step = MON_OFF_STEP_POLLED;
	dsb();
}

/* Power this core down for good: the end of CPU_OFF, and of a spurious
 * wake. Only a CPU_ON brings it back. */
static void __attribute__((noreturn)) core_off_wfi(unsigned int core)
{
	volatile struct mon_cpu *c = &st->cpu[core];

	core_power_down(core, 0);
	c->state = MON_OFF;
	c->off_step = MON_OFF_STEP_WFI;
	dsb();
	for (;;) {
		asm volatile("wfi");
		/* the PMU should have cut the power by now */
		c->wfi_wakes++;
		dsb();
	}
}

/* First C on a secondary after reset (entry.S), on its own EL3 stack. */
void __attribute__((noreturn)) mon_warm(unsigned int core)
{
	volatile struct mon_cpu *c = &st->cpu[core];

	c->warm++;
	dsb();
	el3_setup();

	/*
	 * Powered up without a CPU_ON for it: the PMU woke an off core, for
	 * reasons not yet known (nothing pending in its SGIs/PPIs). It must
	 * not run the last CPU_ON's entry point again. Powering it straight
	 * back down left it unable to power up on the next CPU_ON, so it
	 * parks here instead, powered, and CPU_ON releases it.
	 */
	if (c->state != MON_ON_PENDING) {
		uint32_t f = 0;

		st->spurious_wakes++;
		st->spurious_core = core;
		st->spurious_pend = REG32(gic_redist(core, &f) + GICR_ISPENDR0);
		st->spurious_mode = core_pd_status(core);
		st->spurious_wake_en = REG32(PMU_BASE + PMU_APCPU_CORE_WAKEUP_EN);
		/* not a power-down candidate while it waits */
		REG32(PMU_CLR(dslp_ena[core])) = 1;
		c->flags |= MON_F_PARKED;
		dsb();
		while (c->state != MON_ON_PENDING)
			asm volatile("wfe");
		c->flags &= ~MON_F_PARKED;
		dsb();
	}

	/* CPU_ON's own writes, for a core that took a CPU_ON while bouncing
	 * off a spurious wake (and so skipped them), then stock on_finish:
	 * auto shutdown stays armed; no WFI retention (see above) */
	REG32(PMU_CLR(dslp_ena[core])) = 1;
	REG32(PMU_CLR(pd_cfg[core])) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(pd_cfg[core])) = PD_AUTO_SHUTDOWN_EN;
	write_cpupwrctlr(CPUPWRCTLR_WFI_RET_OFF);
	c->flags |= gic_redist_init(core);
	el2_reset();
	c->state = MON_ON;
	dsb();
	mon_enter_ns(c->entry, c->context);
}

static int64_t psci_cpu_on(uint64_t target, uint64_t entry, uint64_t context)
{
	int core = target_core(target);
	volatile struct mon_cpu *c;
	uint64_t t0;

	if (core < 0)
		return PSCI_INVALID_PARAMS;
	c = &st->cpu[core];
	c->on_calls++;
	if (c->state == MON_ON)
		return c->on_ret = PSCI_ALREADY_ON;
	if (c->state == MON_ON_PENDING)
		return c->on_ret = PSCI_ON_PENDING;

	c->entry = entry;
	c->context = context;
	dsb();
	c->state = MON_ON_PENDING;
	dsb();

	/* parked after a spurious wake: it is powered and waiting */
	if (c->flags & MON_F_PARKED) {
		st->spurious_cpu_on++;
		asm volatile("sev");
		return c->on_ret = PSCI_SUCCESS;
	}

	/* Stock pwr_domain_on: wait for the core to be fully off, then
	 * deep-sleep disable, release FORCE_SHUTDOWN, AUTO_SHUTDOWN_EN on. */
	c->pwr_seen = core_pwr_state(core);
	t0 = ticks();
	while (core_pwr_state(core) != PWR_STATE_OFF) {
		/* a core that woke spuriously just as this came in can take
		 * the request directly (mon_warm sees ON_PENDING, or wakes
		 * from its park on this sev): then it is on */
		asm volatile("sev");
		if (c->state == MON_ON) {
			st->spurious_cpu_on++;
			return c->on_ret = PSCI_SUCCESS;
		}
		if (ticks() - t0 > 10 * TICKS_PER_MS) {
			c->state = MON_OFF;
			return c->on_ret = PSCI_INTERNAL_FAILURE;
		}
	}
	REG32(AON_SEC_RVBARADDR(core)) = (uint32_t)(MON_BASE >> 2);
	REG32(PMU_CLR(dslp_ena[core])) = 1;
	REG32(PMU_CLR(pd_cfg[core])) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(pd_cfg[core])) = PD_AUTO_SHUTDOWN_EN;
	dsb();
	return c->on_ret = PSCI_SUCCESS;
}

/* The core powers off in the WFI; CPU_ON brings it back through its reset
 * vector. core_power_down() cleans the core's own caches first. */
static void __attribute__((noreturn)) psci_cpu_off(void)
{
	unsigned int core = my_core();
	volatile struct mon_cpu *c = &st->cpu[core];

	c->off_calls++;
	c->off_step = MON_OFF_STEP_ENTERED;
	dsb();
	core_off_wfi(core);
}

static int64_t psci_affinity_info(uint64_t target, uint64_t level)
{
	int core = target_core(target);

	if (core < 0 || level)
		return PSCI_INVALID_PARAMS;
	switch (st->cpu[core].state) {
	case MON_ON:
		return 0;
	case MON_ON_PENDING:
		return 2;
	default:
		/* off once the PMU says so (not when CPU_OFF started), or
		 * parked after a spurious wake */
		if (st->cpu[core].flags & MON_F_PARKED)
			return 1;
		return core_pwr_state(core) == PWR_STATE_OFF ? 1 : 0;
	}
}

static int64_t psci_features(uint32_t fid)
{
	switch (fid) {
	case PSCI_VERSION:
	case PSCI_CPU_OFF:
	case PSCI_CPU_ON_32:
	case PSCI_CPU_ON_64:
	case PSCI_AFFINITY_INFO_32:
	case PSCI_AFFINITY_INFO_64:
	case PSCI_MIGRATE_INFO_TYPE:
	case PSCI_FEATURES:
		return PSCI_SUCCESS;
	default:
		return PSCI_NOT_SUPPORTED;
	}
}

/* SMC #0 from the non-secure side (entry.S); result in x0. */
void mon_smc(struct mon_regs *r)
{
	uint32_t fid = (uint32_t)r->x[0];
	unsigned int me = my_core();
	int64_t ret;

	st->smc_calls++;
	st->last_fid = fid;
	if (me < MON_NCPUS)
		st->in_smc[me] = (uint8_t)((fid & 0xff) + 1);
	dsb();

	switch (fid) {
	case PSCI_VERSION:
		ret = 0x10000;		/* 1.0 */
		break;
	case PSCI_CPU_ON_32:
		ret = psci_cpu_on((uint32_t)r->x[1], (uint32_t)r->x[2], (uint32_t)r->x[3]);
		break;
	case PSCI_CPU_ON_64:
		ret = psci_cpu_on(r->x[1], r->x[2], r->x[3]);
		break;
	case PSCI_AFFINITY_INFO_32:
		ret = psci_affinity_info((uint32_t)r->x[1], (uint32_t)r->x[2]);
		break;
	case PSCI_AFFINITY_INFO_64:
		ret = psci_affinity_info(r->x[1], r->x[2]);
		break;
	case PSCI_CPU_OFF:
		psci_cpu_off();
	case PSCI_MIGRATE_INFO_TYPE:
		ret = 2;		/* no trusted OS: migration not required */
		break;
	case PSCI_FEATURES:
		ret = psci_features((uint32_t)r->x[1]);
		break;
	default:
		ret = PSCI_NOT_SUPPORTED;
		break;
	}
	r->x[0] = (uint64_t)ret;
	if (me < MON_NCPUS)
		st->in_smc[me] = 0;
	dsb();
}

/*
 * Called by the SPL at EL3 on CPU0, on the SPL's stack, just before the jump
 * to Linux. Also does the stock BL31's one-time core power setup (plat setup
 * at +0x1a88), minus its RVBAR writes, which happen per core in CPU_ON.
 */
int mon_init(uint64_t spl_vbar)
{
	extern uint64_t __bss_start[], __bss_end[];
	volatile uint64_t *p;
	unsigned int i;

	for (p = __bss_start; p < __bss_end; p++)
		*p = 0;
	/* a panic or watchdog reset keeps DRAM: keep what the last boot's
	 * monitor saw, for devmem at MON_BASE + MON_PREV_OFF */
	{
		volatile uint64_t *d = (volatile uint64_t *)&status_prev;

		for (p = (volatile uint64_t *)st; p < (volatile uint64_t *)(st + 1); p++)
			*d++ = (st->magic == 0x31304e4f4d574643ULL) ? *p : 0;
	}
	for (p = (volatile uint64_t *)st; p < (volatile uint64_t *)(st + 1); p++)
		*p = 0;
	mon_spl_vbar = spl_vbar;

	/* u-boot's PMU power sequencing, before any core is powered up,
	 * except the big cluster's DCDC_CPU1 domain (PD_APCPU_CPU1_CFG1/2)
	 * and its RAM/DCDC delays (PD_APCPU_TOP_CFG3): u-boot's values zero
	 * the DCDC power-on delay (0x818: 0x0100000f against a reset value of
	 * 0x02108080), and CPU6, the first big core up, then never started.
	 * Those keep their reset values. */
	for (i = 0; i < sizeof(pmu_seq) / sizeof(pmu_seq[0]); i++) {
		volatile uint32_t *r = (volatile uint32_t *)(uintptr_t)pmu_seq[i].reg;

		if (i < 64)
			((volatile uint32_t *)pmu_seq_before)[i] = *r;
		if (pmu_seq[i].reg == PMU_BASE + 0x818 ||
		    pmu_seq[i].reg == PMU_BASE + 0x81c ||
		    pmu_seq[i].reg == PMU_BASE + 0x374)
			continue;
		*r = pmu_seq[i].val;
	}
	dsb();

	/* and the rest of u-boot's CSP_Init, in its order */
	for (i = 0; i < sizeof(csp_ops) / sizeof(csp_ops[0]); i++) {
		volatile uint32_t *r = (volatile uint32_t *)(uintptr_t)csp_ops[i].reg;
		uint32_t v = *r;

		if (i < 96)
			((volatile uint32_t *)csp_before)[i] = v;
		*r = (v & csp_ops[i].and_mask) | csp_ops[i].or_mask;
	}
	dsb();

	/* The SPL leaves CPU0 at 0x1800000 (spl_start.S); every core must
	 * agree, and stock uses 26 MHz. */
	asm volatile("msr cntfrq_el0, %0" :: "r" ((uint64_t)MON_CNTFRQ));

	for (i = 0; i < MON_NCPUS; i++)
		REG32(PMU_CLR(pd_cfg[i])) = PD_DBG_SHUTDOWN_EN;
	REG32(PMU_CLR(PMU_APCPU_SOFT_INT_GEN)) = 0xff;
	REG32(PMU_CLR(PMU_APCPU_CORE_FORCE_STOP)) = 0xff | TOP_FORCE_DEEP_STOP;
	REG32(PMU_SET(PMU_APCPU_CSYSPWRUP_WAKEUP_EN)) = 0xff;
	REG32(PMU_CLR(PMU_PD_APCPU_TOP_CFG)) = PD_DBG_SHUTDOWN_EN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_SEL;
	dsb();

	st->cpu[my_core()].state = MON_ON;
	st->magic = 0x31304e4f4d574643ULL;	/* "CFWMON01" */
	dsb();
	return 0;
}
