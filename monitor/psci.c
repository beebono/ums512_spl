/*
 * cfw EL3 monitor: PSCI 1.0 subset and the ums512 core power sequence.
 *
 * Implemented: PSCI_VERSION, PSCI_FEATURES, CPU_ON, AFFINITY_INFO,
 * MIGRATE_INFO_TYPE and CPU_OFF. CPU_OFF parks the core at EL3 (still
 * powered and coherent) rather than powering it down; CPU_ON wakes it again.
 * Everything else returns NOT_SUPPORTED: SYSTEM_RESET/OFF then fall through to
 * the sc27xx handlers, and with no CPU_SUSPEND cpuidle-psci stays out.
 *
 * The register sequences are those of the stock SML (Unisoc TF-A v1.4,
 * plat_sprd_psci_pm_ops), decoded from the v1.44 PAC.
 */
#include "monitor.h"

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

struct mon_regs {
	uint64_t x[31];
	uint64_t pad;
};

uint64_t mon_spl_vbar;		/* SPL's VBAR_EL3, for forwarded exceptions */
uint8_t mon_stacks[MON_NCPUS << MON_STACK_SHIFT] __attribute__((aligned(16)));
static struct mon_status status __attribute__((section(".status")));
static volatile struct mon_status *const st = &status;

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
static uint32_t gic_redist_init(unsigned int core)
{
	uint64_t mpidr, aff;
	unsigned long rd = 0;
	uint32_t flags = 0, pwrr;
	uint64_t t0;
	unsigned int i;

	asm volatile("mrs %0, mpidr_el1" : "=r" (mpidr));
	aff = ((mpidr >> 32) & 0xff) << 24 | (mpidr & 0xffffff);
	for (i = 0; i < GICR_FRAMES; i++)
		if ((REG64(GICR_BASE + i * GICR_STRIDE + GICR_TYPER) >> 32) == aff)
			rd = GICR_BASE + i * GICR_STRIDE;
	if (!rd) {
		rd = GICR_BASE + core * GICR_STRIDE;
		flags |= MON_F_GIC_NOMATCH;
	}

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

/* First C on a secondary after reset (entry.S), on its own EL3 stack. */
void __attribute__((noreturn)) mon_warm(unsigned int core)
{
	volatile struct mon_cpu *c = &st->cpu[core];

	c->warm++;
	dsb();
	el3_setup();
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

	if (c->flags & MON_F_PARKED) {
		asm volatile("sev");
		return c->on_ret = PSCI_SUCCESS;
	}

	/* Stock pwr_domain_on: wait for the core to be fully off, then
	 * deep-sleep disable, release FORCE_SHUTDOWN, AUTO_SHUTDOWN_EN on. */
	c->pwr_seen = core_pwr_state(core);
	t0 = ticks();
	while (core_pwr_state(core) != PWR_STATE_OFF) {
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

static void __attribute__((noreturn)) psci_cpu_off(void)
{
	volatile struct mon_cpu *c = &st->cpu[my_core()];

	c->flags |= MON_F_PARKED;
	dsb();
	c->state = MON_OFF;
	dsb();
	while (c->state != MON_ON_PENDING)
		asm volatile("wfe");
	c->flags &= ~MON_F_PARKED;
	el2_reset();
	c->state = MON_ON;
	dsb();
	mon_enter_ns(c->entry, c->context);
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
		return 1;
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
	int64_t ret;

	st->smc_calls++;
	st->last_fid = fid;

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
	for (p = (volatile uint64_t *)st; p < (volatile uint64_t *)(st + 1); p++)
		*p = 0;
	mon_spl_vbar = spl_vbar;

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
