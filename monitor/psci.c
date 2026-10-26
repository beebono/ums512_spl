/*
 * cfw EL3 monitor: PSCI 1.0 subset and the ums512 core power sequence.
 *
 * Implemented: PSCI_VERSION, PSCI_FEATURES, CPU_ON, AFFINITY_INFO,
 * MIGRATE_INFO_TYPE, CPU_OFF, CPU_SUSPEND (standby, and core power-down)
 * and SYSTEM_SUSPEND. CPU_OFF powers the core down through the PMU; CPU_ON
 * powers it up again and it comes back through its reset vector
 * (mon_warm_entry). A power-down CPU_SUSPEND is the same sequence with the
 * core's wake-up enable set: an interrupt for it powers it back up, and it
 * resumes at the entry point Linux gave. SYSTEM_SUSPEND, with every other
 * core off, takes the cluster and AP_SYS down with it and lets the PMU put
 * the SoC in deep sleep; it wakes through an IRAM trampoline. Everything
 * else returns NOT_SUPPORTED: SYSTEM_RESET/OFF then fall through to the
 * sc27xx handlers.
 *
 * The register sequences are those of the stock SML (Unisoc TF-A v1.4,
 * plat_sprd_psci_pm_ops), decoded from the v1.44 PAC.
 */
#include "monitor.h"
#include "pmu_seq.h"
#include "csp_ops.h"
#include "pmic_seq.h"

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

/*
 * System suspend. The cluster level (stock 0x1010 / 0xfcc) and the system
 * level (0xf04 / 0xea8, 0x8e8 / 0x8d4, 0x8fc) of the stock BL31's
 * pwr_domain_suspend and suspend_finish.
 */
#define PMU_LTEPLL_REL_CFG	0x0a0
#define PMU_PD_AP_SYS_CFG	0x01c
#define PMU_AP_DSLP_ENA		0x208
#define PMU_APCPU_TOP_DSLP_ENA	0x214
#define PMU_PUB_ACC_RDY		0x250
#define PMU_GIC_CFG		0x3a8
#define AON_APB_BASE		0x327d0000UL
#define AON_APB_SET(r)		(AON_APB_BASE + 0x1000 + (r))
#define AON_APB_CLR(r)		(AON_APB_BASE + 0x2000 + (r))
#define AON_APB_EB1		0x004
#define AON_APB_EB1_AP_SYSCNT	(1u << 17)	/* what stock enables before restarting the counter */
#define AON_APB_EB2		0x008
#define AON_APB_EB2_SLEEP_OFF	0x01c00000u	/* bits 22-24, off for the sleep */
#define AON_SEC_BASE		0x32880000UL
#define AON_SEC_SET(r)		(AON_SEC_BASE + 0x1000 + (r))
#define AON_SEC_CLR(r)		(AON_SEC_BASE + 0x2000 + (r))
#define AON_SEC_APCPU_APB_EB	0x024
/* The memory-mapped system counter (APCPU_TS0); it loses power in deep sleep */
#define SYSCNT_BASE		0x32610000UL
#define SYSCNT_CNTCR		0x00
#define SYSCNT_CNTCV_LO		0x08
#define SYSCNT_CNTCV_HI		0x0c
#define SYSCNT_CNTFID0		0x20
/* The wake trampoline's home in IRAM (RVBARADDR 2), as stock. Free once the
 * SPL is done with its early stack (0x3000 down) and Linux runs. */
#define IRAM_TRAMP		0x8UL

/*
 * The AP_SYS registers that lose their contents when AP_SYS powers down,
 * the stock BL31's three save tables (+0xcee8, +0xd1e0, +0xd098): AP_AHB,
 * AP_APB and AP_CLK_CORE. Saved before the sleep, restored in this order.
 */
static const uint32_t ap_sys_regs[] = {
	0x20100000, 0x2010000c, 0x20100010, 0x20100014, 0x20100018, 0x20100020,
	0x20100024, 0x20100028, 0x2010002c, 0x20100040, 0x2010004c, 0x20100050,
	0x201000a8, 0x20100054, 0x20100058, 0x2010005c, 0x20100060, 0x20100064,
	0x20100068, 0x2010006c, 0x20100070, 0x20100074, 0x20100078, 0x2010007c,
	0x20100088, 0x2010008c, 0x20100090, 0x20100094, 0x20100098, 0x2010009c,
	0x201000a0, 0x201000a4, 0x201000ac, 0x201000b0, 0x201000b8, 0x201000bc,
	0x201000d4, 0x201000d8, 0x20100100, 0x20100104, 0x20100108, 0x2010010c,
	0x20100110, 0x20100114, 0x20100118, 0x2010011c, 0x20100120, 0x20100124,
	0x20103014, 0x2010301c, 0x20103034, 0x20103084, 0x20103090, 0x20103094,

	0x71000000, 0x71000004, 0x71000008,

	0x20200020, 0x20200024, 0x20200028, 0x2020002c, 0x20200030, 0x20200034,
	0x20200038, 0x2020003c, 0x20200040, 0x20200044, 0x20200048, 0x2020004c,
	0x20200050, 0x20200054, 0x20200058, 0x2020005c, 0x20200060, 0x20200064,
	0x20200068, 0x2020006c, 0x20200070, 0x20200074, 0x20200078, 0x2020007c,
	0x20200080, 0x20200084, 0x20200088, 0x2020008c, 0x20200090, 0x20200094,
	0x20200098, 0x2020009c, 0x202000a0, 0x202000a4, 0x202000a8, 0x202000ac,
	0x202000b0, 0x202000b4, 0x202000b8, 0x202000bc, 0x202000c0,
};
#define N_AP_SYS_REGS	(sizeof(ap_sys_regs) / sizeof(ap_sys_regs[0]))
#define AP_SYS_AP_AHB_EB	0	/* index of 0x20100000 */
#define AP_SYS_AP_APB_EB	54	/* index of 0x71000000 */
static uint32_t ap_sys_saved[N_AP_SYS_REGS];

/*
 * The ADI bus to the SC2730 PMIC, 15-bit address mode (as the SPL sets it
 * up): a write is a store into the slave window once the FIFO has room; a
 * read posts the address and polls for the data, whose top half echoes the
 * address >> 2.
 */
#define ADI_BASE		0x32100000UL
#define ADI_RD_CMD		(ADI_BASE + 0x28)
#define ADI_RD_DATA		(ADI_BASE + 0x2c)
#define ADI_RD_BUSY		(1u << 31)
#define ADI_FIFO_STS		(ADI_BASE + 0x30)
#define ADI_FIFO_FULL		(1u << 11)
#define ADI_FIFO_EMPTY		(1u << 10)
#define ADI_SLAVE		0x32120000UL

/* the PMIC registers u-boot's pmic_lowpower_config() set, as mon_init found
 * them, at MON_BASE + 0xf800 */
static struct {
	uint32_t chip_id;	/* SC2730 CHIP_ID_LOW */
	uint32_t errors;	/* ADI timeouts or read-back mismatches */
	uint16_t before[64];
} pmic_status __attribute__((section(".status_pmic")));

/* GICv3 distributor, secure view */
#define GICD_BASE		0x12000000UL
#define GICD_CTLR		0x0000
#define GICD_TYPER		0x0004
#define GICD_IGROUPR		0x0080
#define GICD_ISENABLER		0x0100
#define GICD_ISPENDR		0x0200
#define GICD_IPRIORITYR		0x0400
#define GICD_ICFGR		0x0c00
#define GICD_IGRPMODR		0x0d00
#define GICD_NSACR		0x0e00
#define GICD_IROUTER		0x6000
#define GICD_CTLR_ENABLES	0x7u		/* Grp0, Grp1NS, Grp1S */
#define GICD_CTLR_RWP		(1u << 31)
#define GIC_MAX_LINES		1024
/* SGI/PPI frame of a redistributor */
#define GICR_ISENABLER0		(0x10000 + 0x0100)
#define GICR_IPRIORITYR0	(0x10000 + 0x0400)
#define GICR_ICFGR0		(0x10000 + 0x0c00)
#define GICR_NSACR		(0x10000 + 0x0e00)

/* AP sleep setup the stock plat setup does once (0xf6c, 0x1a64) */
#define PMU_SLEEP_CTRL		0x0cc
#define PMU_AP_FORCE_DEEP_SLEEP	(1u << 16)
#define PMU_LIGHT_SLEEP_ENABLE	0x230
#define PMU_AP_LSLP_ENA		(1u << 0)
#define PMU_AP_SMART_LSLP_ENA	(1u << 16)
#define PMU_DOZE_SLEEP_ENABLE	0x238
#define PMU_AP_DOZE_ENA		(1u << 1)
#define PMU_GICDISABLE		(1u << 0)	/* in PMU_GIC_CFG */
#define AP_APB_BASE		0x71000000UL
#define AP_APB_CLR(r)		(AP_APB_BASE + 0x2000 + (r))
#define AP_APB_SIM0_EB		(1u << 0)	/* in AP_APB_EB, 0x000 */
#define AON_APB_EB2_AP_SIM_AON_TOP (1u << 3)

/*
 * The AP INTCs' enables (INTC0-5 + 0x8): which interrupts the PMU may wake
 * AP_SYS from deep sleep for, SPI numbered. Stock's list (0x81c), which
 * covers ours: the AP GPIOs (61: the hall switch, the buttons), the EICs
 * (72), the PMIC behind the ADI (173: power key, RTC), the mailbox (82), USB
 * (69) and the watchdog (79).
 */
static const uint8_t wake_spis[] = {
	180, 188, 173, 104, 94, 93, 92, 91, 90, 89, 83, 82, 80, 79, 78, 72,
	71, 70, 69, 67, 61,
};
#define AP_INTC_BASE(n)		(0x32310000UL + 0x10000 * (n))
#define AP_INTC_STS		0x0	/* masked status */
#define AP_INTC_RAW		0x4
#define AP_INTC_EN(n)		(AP_INTC_BASE(n) + 0x8)
static uint32_t intc_stock_en[6];

/*
 * Clocks the BootROM and SPL leave on that nothing in Linux owns (the
 * clock driver marks them CLK_IGNORE_UNUSED, so they stay on for good), and
 * that keep AP_SYS from deep sleep: CoreSight, the CPU debug port and the
 * bus monitors, and the two UARTs, which the CFW doesn't use. Gated for the
 * sleep only, like stock's APCPU_CSSYS_EB: the bits that were on come back
 * on in the resume.
 */
#define AON_APB_EB0		0x000
#define AON_EB0_SLEEP_OFF	(0xfu << 14)	/* APCPU_DAP, CSSYS, CSSYS_APB, CSSYS_PUB */
#define AON_EB1_SLEEP_OFF	((1u << 16) | (1u << 18))	/* DJTAG_CTRL, APB_BUSMON */
#define AP_AHB_BASE		0x20100000UL
#define AP_AHB_EB_SLEEP_OFF	(1u << 8)	/* BUSMON_CLOCK */
#define AP_APB_EB_SLEEP_OFF	((1u << 14) | (1u << 15))	/* UART0, UART1 */
static uint32_t sleep_off_eb0, sleep_off_eb1;

/* What mon_sys.snap records at SYSTEM_SUSPEND entry, at 0xf750 */
static const uint32_t sys_snap_regs[16] = {
	0x327d0000,	/* AON_APB_EB0 (GPU_EB b11, MM_EB b9) */
	0x327d0004,	/* AON_APB_EB1 (OTG_UTMI_EB) */
	0x327d0008,	/* AON_APB_EB2 */
	0x20100000,	/* AP_AHB_EB (DISPC_EB) */
	0x71000000,	/* AP_APB_EB */
	0x327e00bc,	/* PMU PWR_STATUS0_DBG */
	0x327e00c0,	/* PWR_STATUS1_DBG */
	0x327e00c4,	/* PWR_STATUS2_DBG */
	0x327e010c,	/* PWR_STATUS3_DBG (GPU_TOP, MM) */
	0x327e00b8,	/* PWR_STATUS4_DBG */
	0x327e00d4,	/* SLEEP_STATUS */
	0x327e00cc,	/* SLEEP_CTRL */
	0x327e0084,	/* XTL0_REL_CFG */
	0x327e07b8,	/* DDR_SLP_CTRL_STATE */
	0x327e0378,	/* APCU_PWR_STATE0 */
	0x327e0820,	/* APCU_PWR_STATE1 */
};
#define AP_INTCS		6

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
#define PSCI_CPU_SUSPEND_32	0x84000001
#define PSCI_CPU_SUSPEND_64	0xc4000001
#define PSCI_SYSTEM_SUSPEND_32	0x8400000e
#define PSCI_SYSTEM_SUSPEND_64	0xc400000e

/* power_state, original format: StateID [15:0], type [16], level [25:24] */
#define PSTATE_TYPE_PWRDN	(1u << 16)
#define PSTATE_LEVEL(p)		(((p) >> 24) & 3)

#define PSCI_SUCCESS		0
#define PSCI_NOT_SUPPORTED	-1
#define PSCI_INVALID_PARAMS	-2
#define PSCI_DENIED		-3
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
static struct mon_susp susp_stats __attribute__((section(".status_susp")));
static volatile struct mon_susp *const ss = &susp_stats;
/* likewise for csp_ops, at MON_BASE + 0xf500 */
static uint32_t csp_before[96] __attribute__((section(".status_csp")));
static struct mon_sys sys_stats __attribute__((section(".status_sys")));
static volatile struct mon_sys *const sy = &sys_stats;
static struct mon_sys sys_prev __attribute__((section(".status_sys_prev")));

/* GIC state across a system suspend: the distributor's SPIs, and the SGI/PPI
 * frame of the core that suspends (the others are off, and Linux sets theirs
 * up again as it brings them back) */
static struct {
	uint32_t igroupr[GIC_MAX_LINES / 32];
	uint32_t isenabler[GIC_MAX_LINES / 32];
	uint32_t ispendr[GIC_MAX_LINES / 32];
	uint32_t igrpmodr[GIC_MAX_LINES / 32];
	uint32_t ipriorityr[GIC_MAX_LINES / 4];
	uint32_t icfgr[GIC_MAX_LINES / 16];
	uint32_t nsacr[GIC_MAX_LINES / 16];
	uint32_t irouter[GIC_MAX_LINES][2];
	uint32_t r_isenabler0;
	uint32_t r_ipriorityr[8];
	uint32_t r_icfgr[2];
	uint32_t r_nsacr;
} gic_ctx;

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

static void gicd_wait_rwp(void)
{
	wait32(GICD_BASE + GICD_CTLR, GICD_CTLR_RWP, 0);
}

/* The distributor's SPI state and this core's SGIs/PPIs (TF-A's
 * gicv3_distif_save / gicv3_rdistif_save, cut to what Linux uses: no LPIs,
 * no active state). */
static void gic_save(unsigned int core)
{
	uint32_t lines, i, f = 0;
	unsigned long rd = gic_redist(core, &f);

	lines = ((REG32(GICD_BASE + GICD_TYPER) & 0x1f) + 1) * 32;
	if (lines > GIC_MAX_LINES)
		lines = GIC_MAX_LINES;
	sy->gicd_lines = lines;
	sy->gicd_ctlr = REG32(GICD_BASE + GICD_CTLR);

	for (i = 1; i < lines / 32; i++) {
		gic_ctx.igroupr[i] = REG32(GICD_BASE + GICD_IGROUPR + 4 * i);
		gic_ctx.isenabler[i] = REG32(GICD_BASE + GICD_ISENABLER + 4 * i);
		gic_ctx.ispendr[i] = REG32(GICD_BASE + GICD_ISPENDR + 4 * i);
		gic_ctx.igrpmodr[i] = REG32(GICD_BASE + GICD_IGRPMODR + 4 * i);
	}
	for (i = 32 / 4; i < lines / 4; i++)
		gic_ctx.ipriorityr[i] = REG32(GICD_BASE + GICD_IPRIORITYR + 4 * i);
	for (i = 32 / 16; i < lines / 16; i++) {
		gic_ctx.icfgr[i] = REG32(GICD_BASE + GICD_ICFGR + 4 * i);
		gic_ctx.nsacr[i] = REG32(GICD_BASE + GICD_NSACR + 4 * i);
	}
	for (i = 32; i < lines; i++) {
		gic_ctx.irouter[i][0] = REG32(GICD_BASE + GICD_IROUTER + 8 * i);
		gic_ctx.irouter[i][1] = REG32(GICD_BASE + GICD_IROUTER + 8 * i + 4);
	}

	gic_ctx.r_isenabler0 = REG32(rd + GICR_ISENABLER0);
	for (i = 0; i < 8; i++)
		gic_ctx.r_ipriorityr[i] = REG32(rd + GICR_IPRIORITYR0 + 4 * i);
	gic_ctx.r_icfgr[0] = REG32(rd + GICR_ICFGR0);
	gic_ctx.r_icfgr[1] = REG32(rd + GICR_ICFGR0 + 4);
	gic_ctx.r_nsacr = REG32(rd + GICR_NSACR);
}

/* Distributor first, with affinity routing on and the groups off until the
 * end (TF-A gicv3_distif_init_restore), then this core's redistributor,
 * which the caller has powered up and woken. Harmless if nothing was lost. */
static void gic_restore(unsigned int core)
{
	uint32_t lines = sy->gicd_lines, i, f = 0;
	unsigned long rd = gic_redist(core, &f);

	REG32(GICD_BASE + GICD_CTLR) = sy->gicd_ctlr & ~GICD_CTLR_ENABLES;
	gicd_wait_rwp();
	for (i = 1; i < lines / 32; i++) {
		REG32(GICD_BASE + GICD_IGROUPR + 4 * i) = gic_ctx.igroupr[i];
		REG32(GICD_BASE + GICD_IGRPMODR + 4 * i) = gic_ctx.igrpmodr[i];
	}
	for (i = 32 / 4; i < lines / 4; i++)
		REG32(GICD_BASE + GICD_IPRIORITYR + 4 * i) = gic_ctx.ipriorityr[i];
	for (i = 32 / 16; i < lines / 16; i++) {
		REG32(GICD_BASE + GICD_ICFGR + 4 * i) = gic_ctx.icfgr[i];
		REG32(GICD_BASE + GICD_NSACR + 4 * i) = gic_ctx.nsacr[i];
	}
	for (i = 32; i < lines; i++) {
		REG32(GICD_BASE + GICD_IROUTER + 8 * i) = gic_ctx.irouter[i][0];
		REG32(GICD_BASE + GICD_IROUTER + 8 * i + 4) = gic_ctx.irouter[i][1];
	}
	for (i = 1; i < lines / 32; i++) {
		REG32(GICD_BASE + GICD_ISPENDR + 4 * i) = gic_ctx.ispendr[i];
		REG32(GICD_BASE + GICD_ISENABLER + 4 * i) = gic_ctx.isenabler[i];
	}
	REG32(GICD_BASE + GICD_CTLR) = sy->gicd_ctlr;
	gicd_wait_rwp();

	for (i = 0; i < 8; i++)
		REG32(rd + GICR_IPRIORITYR0 + 4 * i) = gic_ctx.r_ipriorityr[i];
	REG32(rd + GICR_ICFGR0) = gic_ctx.r_icfgr[0];
	REG32(rd + GICR_ICFGR0 + 4) = gic_ctx.r_icfgr[1];
	REG32(rd + GICR_NSACR) = gic_ctx.r_nsacr;
	REG32(rd + GICR_ISENABLER0) = gic_ctx.r_isenabler0;
}

/* Cluster power-down (stock 0x1010, then CLUSTERPWRDN_EL1 = 0) */
static void cluster_down(void)
{
	REG32(PMU_SET(PMU_APCPU_TOP_DSLP_ENA)) = 1;
	REG32(PMU_CLR(PMU_PD_APCPU_TOP_CFG)) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_AUTO_SHUTDOWN_EN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_SEL;
	asm volatile("msr S3_0_C15_C3_6, xzr\n" "isb");
}

/* ... and back (0xfcc; CLUSTERPWRCTLR = 0xf0, CLUSTERPWRDN = 2) */
static void cluster_up(void)
{
	REG32(PMU_CLR(PMU_APCPU_TOP_DSLP_ENA)) = 1;
	REG32(PMU_CLR(PMU_PD_APCPU_TOP_CFG)) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_AUTO_SHUTDOWN_EN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_SEL;
	asm volatile(
		"msr	S3_0_C15_C3_5, %0\n"
		"isb\n"
		"msr	S3_0_C15_C3_6, %1\n"
		"isb\n"
		:: "r" (0xf0UL), "r" (2UL));
}

/* AP_SYS deep sleep enabled, its clocks parked (stock 0xf04, 0x8e8) */
static void system_down(void)
{
	REG32(AON_SEC_CLR(AON_SEC_APCPU_APB_EB)) = 1;
	REG32(PMU_SET(PMU_LTEPLL_REL_CFG)) = 1;
	REG32(PMU_CLR(PMU_LTEPLL_REL_CFG)) = 0x10;
	REG32(PMU_SET(PMU_GIC_CFG)) = 1;
	REG32(PMU_SET(PMU_AP_DSLP_ENA)) = 1;
	REG32(PMU_CLR(PMU_PD_AP_SYS_CFG)) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(PMU_PD_AP_SYS_CFG)) = PD_AUTO_SHUTDOWN_EN;
	REG32(AON_APB_CLR(AON_APB_EB2)) = AON_APB_EB2_SLEEP_OFF;
}

/* ... and back (0xea8, 0x8d4), without the register tables or the counter */
static void system_up(void)
{
	REG32(PMU_CLR(PMU_GIC_CFG)) = 1;
	REG32(PMU_SET(PMU_LTEPLL_REL_CFG)) = 0x10;
	REG32(PMU_CLR(PMU_AP_DSLP_ENA)) = 1;
	REG32(PMU_CLR(PMU_PD_AP_SYS_CFG)) = PD_FORCE_SHUTDOWN;
	REG32(PMU_SET(PMU_PD_AP_SYS_CFG)) = PD_AUTO_SHUTDOWN_EN;
	REG32(AON_SEC_SET(AON_SEC_APCPU_APB_EB)) = 1;
}

/* After the AP_SYS tables are saved, so their restore turns the AP_AHB
 * and AP_APB bits back on by itself */
static void sleep_clocks_off(void)
{
	sleep_off_eb0 = REG32(AON_APB_BASE + AON_APB_EB0) & AON_EB0_SLEEP_OFF;
	sleep_off_eb1 = REG32(AON_APB_BASE + AON_APB_EB1) & AON_EB1_SLEEP_OFF;
	REG32(AON_APB_CLR(AON_APB_EB0)) = AON_EB0_SLEEP_OFF;
	REG32(AON_APB_CLR(AON_APB_EB1)) = AON_EB1_SLEEP_OFF;
	REG32(AP_AHB_BASE + 0x2000) = AP_AHB_EB_SLEEP_OFF;
	REG32(AP_APB_CLR(0x000)) = AP_APB_EB_SLEEP_OFF;
}

/*
 * Arm only the wake sources Linux armed. By now it has masked every
 * interrupt that isn't a wake source at the GIC, but the INTCs know nothing
 * of that: a masked line that stays asserted (USB with the cable pulled)
 * never wakes the core, yet the PMU counts it as a wake request and won't
 * let AP_SYS sleep. INTC bit n is SPI n, GICD_ISENABLER word n / 32 + 1.
 */
static void intc_arm_for_sleep(void)
{
	unsigned int i;

	for (i = 0; i < AP_INTCS; i++) {
		sy->intc_armed[i] = intc_stock_en[i] & gic_ctx.isenabler[i + 1];
		REG32(AP_INTC_EN(i)) = sy->intc_armed[i];
	}
	dsb();
	for (i = 0; i < AP_INTCS; i++) {
		sy->intc_blocking[i] = REG32(AP_INTC_BASE(i) + AP_INTC_STS);
		sy->intc_raw[i] = REG32(AP_INTC_BASE(i) + AP_INTC_RAW);
	}
}

static void intc_restore(void)
{
	unsigned int i;

	for (i = 0; i < AP_INTCS; i++)
		REG32(AP_INTC_EN(i)) = intc_stock_en[i];
}

static void sleep_clocks_on(void)
{
	REG32(AON_APB_SET(AON_APB_EB0)) = sleep_off_eb0;
	REG32(AON_APB_SET(AON_APB_EB1)) = sleep_off_eb1;
}

/*
 * The stock core power-down (0x1338 for an off, 0x1604 for a suspend, which
 * differ only in the wake-up bit), then the core's own power-down request,
 * then for level 1 the cluster's and for level 2 AP_SYS's as well. Returns
 * once the PMU reports the core ready; the caller's WFI powers it off.
 */
static void core_power_down(unsigned int core, int wake_on_irq, int level)
{
	volatile struct mon_cpu *c = &st->cpu[core];
	uint64_t t0;
	uint32_t v;

	c->off_step = MON_OFF_STEP_GIC_OFF;
	c->flags |= gic_cpuif_disable(core);
	/* stock trusts the DynamIQ hardware flush on power-down; a set/way
	 * clean of L1/L2 costs microseconds and doesn't depend on it. The
	 * cluster going down takes L3 too, and DRAM goes to self-refresh. */
	if (level)
		mon_flush_dcache_all();
	else
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
	if (level >= 1)
		cluster_down();
	if (level >= 2)
		system_down();
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

	core_power_down(core, 0, 0);
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

static void __attribute__((noreturn)) mon_resume(unsigned int core);

/*
 * Back from a system suspend, through the IRAM trampoline: AP_SYS and the
 * cluster as stock's suspend_finish leaves them (0xea8, the tables, 0x8d4,
 * 0x8fc, 0xfcc, CLUSTERPWRCTLR, the GIC). The counter restarts where it
 * stopped rather than at stock's 0: Linux's clocksource is not nonstop
 * (arm,no-tick-in-suspend) and takes the time asleep from the RTC.
 */
static void system_resume(unsigned int core)
{
	uint32_t i;

	sy->resumed++;
	sy->pub_acc_rdy = REG32(PMU_BASE + PMU_PUB_ACC_RDY);
	for (i = 0; i < AP_INTCS; i++)
		sy->intc_wake[i] = REG32(AP_INTC_BASE(i) + AP_INTC_STS);
	intc_restore();
	system_up();
	for (i = 0; i < N_AP_SYS_REGS; i++)
		REG32(ap_sys_regs[i]) = ap_sys_saved[i];
	REG32(AON_APB_SET(AON_APB_EB2)) = AON_APB_EB2_SLEEP_OFF;
	sleep_clocks_on();

	REG32(AON_APB_SET(AON_APB_EB1)) = AON_APB_EB1_AP_SYSCNT;
	sy->cnt_wake = (uint64_t)REG32(SYSCNT_BASE + SYSCNT_CNTCV_HI) << 32 |
		       REG32(SYSCNT_BASE + SYSCNT_CNTCV_LO);
	REG32(SYSCNT_BASE + SYSCNT_CNTCR) = 0;
	REG32(SYSCNT_BASE + SYSCNT_CNTFID0) = MON_CNTFRQ;
	REG32(SYSCNT_BASE + SYSCNT_CNTCV_LO) = (uint32_t)sy->cnt_suspend;
	REG32(SYSCNT_BASE + SYSCNT_CNTCV_HI) = (uint32_t)(sy->cnt_suspend >> 32);
	REG32(SYSCNT_BASE + SYSCNT_CNTCR) = 1;

	cluster_up();
	sy->gicd_ctlr_wake = REG32(GICD_BASE + GICD_CTLR);
	st->cpu[core].flags |= gic_redist_init(core);
	gic_restore(core);
	sy->in_progress = 0;
	dsb();
}

/* First C on a secondary after reset (entry.S), on its own EL3 stack. */
void __attribute__((noreturn)) mon_warm(unsigned int core)
{
	volatile struct mon_cpu *c = &st->cpu[core];

	c->warm++;
	dsb();
	el3_setup();

	if (c->state == MON_SUSPENDED) {
		ss->resumed[core]++;
		if (sy->in_progress)
			system_resume(core);
	}

	/*
	 * Powered up with neither a CPU_ON nor a suspend for it: the PMU woke
	 * an off core (seen before the PMU power sequencing was applied). It
	 * must not run the last CPU_ON's entry point again. Powering it
	 * straight back down left it unable to power up on the next CPU_ON,
	 * so it parks here instead, powered, and CPU_ON releases it.
	 */
	if (c->state != MON_ON_PENDING && c->state != MON_SUSPENDED) {
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
	mon_resume(core);
}

/*
 * Into Linux at this core's entry point with the core as a fresh CPU_ON
 * would leave it: after a CPU_ON, a power-down suspend (both through the
 * reset vector) or a suspend whose WFI returned with the power still on.
 */
static void __attribute__((noreturn)) mon_resume(unsigned int core)
{
	volatile struct mon_cpu *c = &st->cpu[core];

	/* the power-down set the core's interrupt disable; a power cycle
	 * clears it, a WFI that returned early doesn't */
	REG32(PMU_CLR(pd_misc[core])) = 1;
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
	/* a suspend never returns through mon_smc: its marker ends here */
	st->in_smc[core] = 0;
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
	if (c->state == MON_ON || c->state == MON_SUSPENDED)
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

/*
 * CPU_SUSPEND. Standby is a WFI. A core power-down (level 0) runs the
 * CPU_OFF sequence with the wake-up enable set; an interrupt for the core
 * powers it back up through the reset vector into mon_warm. If the WFI
 * returns with the power still on (something was already pending), resume
 * the same way, so Linux sees one path. Cluster and system levels are
 * SYSTEM_SUSPEND's.
 */
static int64_t psci_cpu_suspend(uint32_t pstate, uint64_t entry, uint64_t context)
{
	unsigned int core = my_core();
	volatile struct mon_cpu *c = &st->cpu[core];

	if (!(pstate & PSTATE_TYPE_PWRDN)) {
		ss->standby[core]++;
		dsb();
		asm volatile("wfi");
		return PSCI_SUCCESS;
	}
	if (PSTATE_LEVEL(pstate) != 0)
		return PSCI_INVALID_PARAMS;

	ss->calls[core]++;
	c->entry = entry;
	c->context = context;
	REG32(AON_SEC_RVBARADDR(core)) = (uint32_t)(MON_BASE >> 2);
	c->state = MON_SUSPENDED;
	dsb();
	core_power_down(core, 1, 0);
	asm volatile("wfi");

	ss->early[core]++;
	dsb();
	el3_setup();
	mon_resume(core);
}

/*
 * SYSTEM_SUSPEND, the last core up (Linux has CPU_OFF'd the others): the
 * core, the cluster and AP_SYS all power down, and the PMU takes the SoC
 * into deep sleep once the other subsystems agree. A wake-up powers the
 * core up at the IRAM trampoline, which waits for DRAM and comes back
 * through mon_warm -> system_resume -> mon_resume into Linux's entry point.
 */
static int64_t psci_system_suspend(uint64_t entry, uint64_t context)
{
	unsigned int core = my_core(), i;
	volatile struct mon_cpu *c = &st->cpu[core];

	sy->calls++;
	for (i = 0; i < 16; i++)
		sy->snap[i] = REG32(sys_snap_regs[i]);
	for (i = 0; i < MON_NCPUS; i++) {
		if (i == core)
			continue;
		if (st->cpu[i].state != MON_OFF || (st->cpu[i].flags & MON_F_PARKED) ||
		    core_pwr_state(i) != PWR_STATE_OFF) {
			sy->denied++;
			sy->denied_core = i;
			return PSCI_DENIED;
		}
	}

	c->entry = entry;
	c->context = context;
	gic_save(core);
	intc_arm_for_sleep();
	for (i = 0; i < N_AP_SYS_REGS; i++)
		ap_sys_saved[i] = REG32(ap_sys_regs[i]);
	sleep_clocks_off();
	sy->cnt_suspend = ticks();
	for (i = 0; i < (uint32_t)(mon_tramp_end - mon_tramp); i++)
		REG32(IRAM_TRAMP + 4 * i) = mon_tramp[i];
	REG32(AON_SEC_RVBARADDR(core)) = (uint32_t)(IRAM_TRAMP >> 2);
	c->state = MON_SUSPENDED;
	sy->in_progress = 1;
	dsb();
	core_power_down(core, 1, 2);
	asm volatile("wfi");

	/* an interrupt was already pending: nothing powered down, and nothing
	 * lost its state, so only the PMU and clock setup is undone */
	sy->early++;
	intc_restore();
	system_up();
	REG32(AON_APB_SET(AON_APB_EB2)) = AON_APB_EB2_SLEEP_OFF;
	sleep_clocks_on();
	REG32(AP_AHB_BASE + 0x1000) = ap_sys_saved[AP_SYS_AP_AHB_EB] & AP_AHB_EB_SLEEP_OFF;
	REG32(AP_APB_BASE + 0x1000) = ap_sys_saved[AP_SYS_AP_APB_EB] & AP_APB_EB_SLEEP_OFF;
	cluster_up();
	sy->in_progress = 0;
	dsb();
	el3_setup();
	mon_resume(core);
}

static int64_t psci_affinity_info(uint64_t target, uint64_t level)
{
	int core = target_core(target);

	if (core < 0 || level)
		return PSCI_INVALID_PARAMS;
	switch (st->cpu[core].state) {
	case MON_ON:
	case MON_SUSPENDED:
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
	case PSCI_SYSTEM_SUSPEND_32:
	case PSCI_SYSTEM_SUSPEND_64:
		return PSCI_SUCCESS;
	case PSCI_CPU_SUSPEND_32:
	case PSCI_CPU_SUSPEND_64:
		return 0;	/* original power_state format, platform coordinated */
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
	case PSCI_CPU_SUSPEND_32:
		ret = psci_cpu_suspend((uint32_t)r->x[1], (uint32_t)r->x[2], (uint32_t)r->x[3]);
		break;
	case PSCI_CPU_SUSPEND_64:
		ret = psci_cpu_suspend((uint32_t)r->x[1], r->x[2], r->x[3]);
		break;
	case PSCI_SYSTEM_SUSPEND_32:
		ret = psci_system_suspend((uint32_t)r->x[1], (uint32_t)r->x[2]);
		break;
	case PSCI_SYSTEM_SUSPEND_64:
		ret = psci_system_suspend(r->x[1], r->x[2]);
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

static int adi_read(uint32_t off)
{
	unsigned long a = ADI_SLAVE + off;
	uint64_t t0 = ticks();
	uint32_t v;

	REG32(ADI_RD_CMD) = (uint32_t)a;
	do {
		v = REG32(ADI_RD_DATA);
		if (ticks() - t0 > TICKS_PER_MS)
			return -1;
	} while (v & ADI_RD_BUSY);
	if (((v >> 16) & 0xffff) != ((a & 0xffff) >> 2))
		return -1;
	return v & 0xffff;
}

static int adi_write(uint32_t off, uint16_t val)
{
	if (wait32(ADI_FIFO_STS, ADI_FIFO_FULL, 0))
		return -1;
	REG32(ADI_SLAVE + off) = val;
	return wait32(ADI_FIFO_STS, ADI_FIFO_EMPTY, ADI_FIFO_EMPTY);
}

/* u-boot's pmic_lowpower_config(), which our boot chain otherwise skips */
static void pmic_lowpower_init(void)
{
	unsigned int i;
	int v, chip;

	chip = adi_read(PMIC_CHIP_ID_LOW);
	pmic_status.chip_id = (uint32_t)chip;
	for (i = 0; i < sizeof(pmic_seq) / sizeof(pmic_seq[0]); i++) {
		uint16_t val = pmic_seq[i].val;
		uint64_t t0;

		v = adi_read(pmic_seq[i].off);
		if (i < 64)
			pmic_status.before[i] = (uint16_t)v;
		if (v < 0)
			pmic_status.errors++;

		switch (pmic_seq[i].op) {
		case PMIC_GET_PROT:
			t0 = ticks();
			while (!(adi_read(pmic_seq[i].off) & 0x8000))
				if (ticks() - t0 > 10 * TICKS_PER_MS) {
					pmic_status.errors++;
					break;
				}
			continue;
		case PMIC_OR:
			if (v < 0)
				continue;
			val |= (uint16_t)v;
			break;
		default:
			if (pmic_seq[i].off == PMIC_SLP_DCDC_LP_CTRL &&
			    chip >= 0xb000)
				val = PMIC_SLP_DCDC_LP_CTRL_B;
			break;
		}
		if (adi_write(pmic_seq[i].off, val))
			pmic_status.errors++;
	}
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
		d = (volatile uint64_t *)&sys_prev;
		for (p = (volatile uint64_t *)sy; p < (volatile uint64_t *)(sy + 1); p++)
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

	/* u-boot runs pmic_lowpower_config() just before CSP_Init */
	for (p = (volatile uint64_t *)&pmic_status;
	     p < (volatile uint64_t *)(&pmic_status + 1); p++)
		*p = 0;
	pmic_lowpower_init();

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

	/* every core's reset vector here, core 0 included: CPU_ON only sets
	 * the ones it starts, and core 0 can now power down in idle */
	for (i = 0; i < MON_NCPUS; i++)
		REG32(AON_SEC_RVBARADDR(i)) = (uint32_t)(MON_BASE >> 2);
	for (p = (volatile uint64_t *)ss; p < (volatile uint64_t *)(ss + 1); p++)
		*p = 0;
	for (p = (volatile uint64_t *)sy; p < (volatile uint64_t *)(sy + 1); p++)
		*p = 0;

	for (i = 0; i < MON_NCPUS; i++)
		REG32(PMU_CLR(pd_cfg[i])) = PD_DBG_SHUTDOWN_EN;
	REG32(PMU_CLR(PMU_APCPU_SOFT_INT_GEN)) = 0xff;
	REG32(PMU_CLR(PMU_APCPU_CORE_FORCE_STOP)) = 0xff | TOP_FORCE_DEEP_STOP;
	REG32(PMU_SET(PMU_APCPU_CSYSPWRUP_WAKEUP_EN)) = 0xff;
	REG32(PMU_CLR(PMU_PD_APCPU_TOP_CFG)) = PD_DBG_SHUTDOWN_EN;
	REG32(PMU_SET(PMU_PD_APCPU_TOP_CFG)) = PD_SEL;

	/* The rest of stock's plat setup: the DSU's cluster power control
	 * (0x8870 with 0xf: all L3 portions powered, memory retention on a
	 * cluster power-down), AP light sleep on, doze and forced deep sleep
	 * off, the GIC's sleep disable released (0xf6c), the SIM clocks off
	 * (0x1a64), then the AP INTC wake sources (0x81c). Without the INTC
	 * enables nothing could wake AP_SYS from deep sleep. */
	asm volatile(
		"msr	S3_0_C15_C3_5, %0\n"
		"isb\n"
		"msr	S3_0_C15_C3_6, %1\n"
		"isb\n"
		:: "r" (0xf0UL), "r" (2UL));
	REG32(PMU_CLR(PMU_GIC_CFG)) = PMU_GICDISABLE;
	REG32(PMU_CLR(PMU_SLEEP_CTRL)) = PMU_AP_FORCE_DEEP_SLEEP;
	REG32(PMU_SET(PMU_LIGHT_SLEEP_ENABLE)) = PMU_AP_SMART_LSLP_ENA;
	REG32(PMU_SET(PMU_LIGHT_SLEEP_ENABLE)) = PMU_AP_LSLP_ENA;
	REG32(PMU_CLR(PMU_DOZE_SLEEP_ENABLE)) = PMU_AP_DOZE_ENA;
	REG32(AP_APB_CLR(0x000)) = AP_APB_SIM0_EB;
	REG32(AON_APB_CLR(AON_APB_EB2)) = AON_APB_EB2_AP_SIM_AON_TOP;
	{
		uint32_t en[AP_INTCS] = { 0 };

		for (i = 0; i < sizeof(wake_spis); i++)
			en[wake_spis[i] / 32] |= 1u << (wake_spis[i] % 32);
		for (i = 0; i < AP_INTCS; i++) {
			sy->intc_en_before[i] = REG32(AP_INTC_EN(i));
			intc_stock_en[i] = en[i];
			REG32(AP_INTC_EN(i)) = en[i];
		}
	}
	dsb();

	st->cpu[my_core()].state = MON_ON;
	st->magic = 0x31304e4f4d574643ULL;	/* "CFWMON01" */
	dsb();
	return 0;
}
