/*
 * cfw EL3 monitor: the minimal PSCI firmware for the standalone CFW boot path.
 *
 * The SPL loads monitor.bin from the eMMC "boot" partition to MON_BASE
 * (reserved no-map in ums512-rg-rotate-cfw.dts), calls mon_init() through the
 * header below at EL3, and erets into Linux with VBAR_EL3 and SP_EL3 pointing
 * here. Linux then reaches the monitor only through SMC #0 (PSCI). Any other
 * EL3 exception, including the EL2 stub's smc #1/#2 checkpoints, is forwarded
 * to the SPL's own vectors in IRAM, which stay resident and secure.
 *
 * Everything runs with the MMU and caches off, so all data accesses are
 * Device-nGnRnE: no unaligned accesses, no exclusives.
 */
#ifndef CFW_MONITOR_H
#define CFW_MONITOR_H

/* Must match CFW_MON_BASE in linux_boot.c, monitor.lds and the cfw dts. */
#define MON_BASE		0x80100000
#define MON_SIZE		0x10000
#define MON_STATUS_OFF		0xf000		/* status page, for devmem */
#define MON_PREV_OFF		0xe000		/* the previous boot's status page, if DRAM kept it */

#define MON_MAGIC		0x4d574643	/* "CFWM" */
#define MON_VERSION		1

#define MON_NCPUS		8
#define MON_STACK_SHIFT		10		/* 1 KiB EL3 stack per core */

/* Stock BL31 writes this on every core (plat setup and pwr_domain_on_finish). */
#define MON_CNTFRQ		26000000

#ifndef __ASSEMBLER__
#include <stdint.h>

/* At MON_BASE. The SPL checks magic, then uses the absolute addresses. */
struct mon_hdr {
	uint32_t branch;	/* b mon_warm_entry: RVBARADDRn points here */
	uint32_t magic;
	uint32_t version;
	uint32_t size;		/* loaded bytes */
	uint64_t init;		/* int mon_init(uint64_t spl_vbar), 0 = ok */
	uint64_t vectors;	/* VBAR_EL3 for the handoff */
	uint64_t stack0;	/* SP_EL3 for the handoff (CPU0's stack top) */
};

enum { MON_OFF, MON_ON_PENDING, MON_ON, MON_SUSPENDED };

#define MON_F_PD_TIMEOUT	(1u << 0)	/* CPU_OFF: the PMU never reported the core ready to power down */
#define MON_F_GIC_TIMEOUT	(1u << 1)	/* redistributor power-up or wake timed out */
#define MON_F_GIC_NOMATCH	(1u << 2)	/* no GICR frame matched; used frame == core */
#define MON_F_PARKED		(1u << 3)	/* woke with no CPU_ON: parked at EL3, powered */

/* Status page at MON_BASE + MON_STATUS_OFF: header, then cpu[n] at 0x40 * (n + 1). */
struct mon_cpu {
	uint32_t state;		/* MON_OFF / MON_ON_PENDING / MON_ON */
	uint32_t flags;		/* MON_F_* */
	uint32_t on_calls;	/* CPU_ON requests naming this core */
	uint32_t warm;		/* times this core reached mon_warm() */
	uint32_t pwr_seen;	/* PMU power state field when CPU_ON came in */
	int32_t on_ret;		/* last CPU_ON result */
	uint64_t entry;		/* CPU_ON entry point and context id */
	uint64_t context;
	uint32_t off_calls;	/* CPU_OFF calls on this core */
	uint32_t pd_seen;	/* PMU power-down status byte when CPU_OFF went to WFI */
	uint32_t pd_wait_max;	/* longest wait for that byte, in 26 MHz ticks */
	uint32_t pd_timeouts;	/* waits that gave up (MON_F_PD_TIMEOUT) */
	uint32_t off_step;	/* how far the last CPU_OFF got (MON_OFF_STEP_*) */
	uint32_t wfi_wakes;	/* times CPU_OFF's final WFI returned */
};

enum {
	MON_OFF_STEP_ENTERED = 1,
	MON_OFF_STEP_GIC_OFF,
	MON_OFF_STEP_FLUSHED,
	MON_OFF_STEP_PMU_ARMED,
	MON_OFF_STEP_POLLED,
	MON_OFF_STEP_WFI,
};

struct mon_status {
	uint64_t magic;		/* "CFWMON01" once mon_init() ran */
	uint32_t smc_calls;	/* PSCI calls seen */
	uint32_t last_fid;
	/* a core that came out of reset with no CPU_ON for it, and went
	 * straight back down: how often, the last one, and its GIC pending
	 * SGIs/PPIs (GICR_ISPENDR0) as it woke */
	uint32_t spurious_wakes;
	uint32_t spurious_core;
	uint32_t spurious_pend;
	uint32_t spurious_cpu_on;	/* CPU_ON calls that found such a core already up */
	uint32_t spurious_mode;		/* its APCU_MODE_STATE byte as it woke */
	uint32_t spurious_wake_en;	/* APCPU_CORE_WAKEUP_EN then */
	/* per core, (PSCI function id & 0xff) + 1 while it is inside an
	 * SMC, 0 otherwise: a core stuck at EL3 shows here */
	uint8_t in_smc[MON_NCPUS];
	uint64_t pad[2];
	struct mon_cpu cpu[MON_NCPUS];
};

_Static_assert(sizeof(struct mon_cpu) == 0x40, "status page layout");
_Static_assert(sizeof(struct mon_status) == 0x40 * (MON_NCPUS + 1), "status page layout");

/* CPU_SUSPEND counters, per core, at MON_BASE + 0xf300 */
struct mon_susp {
	uint32_t calls[MON_NCPUS];	/* power-down CPU_SUSPEND calls */
	uint32_t early[MON_NCPUS];	/* of those, WFI returned with the power on */
	uint32_t standby[MON_NCPUS];	/* standby (WFI only) calls */
	uint32_t resumed[MON_NCPUS];	/* came back through the reset vector */
};

void mon_enter_ns(uint64_t entry, uint64_t context) __attribute__((noreturn));
void mon_flush_private_dcache(void);
#endif

#endif /* CFW_MONITOR_H */
