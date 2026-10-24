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

enum { MON_OFF, MON_ON_PENDING, MON_ON };

#define MON_F_PARKED		(1u << 0)	/* in CPU_OFF's park loop, still powered */
#define MON_F_GIC_TIMEOUT	(1u << 1)	/* redistributor power-up or wake timed out */
#define MON_F_GIC_NOMATCH	(1u << 2)	/* no GICR frame matched; used frame == core */

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
	uint64_t pad[3];
};

struct mon_status {
	uint64_t magic;		/* "CFWMON01" once mon_init() ran */
	uint32_t smc_calls;	/* PSCI calls seen */
	uint32_t last_fid;
	uint64_t pad[6];
	struct mon_cpu cpu[MON_NCPUS];
};

_Static_assert(sizeof(struct mon_cpu) == 0x40, "status page layout");
_Static_assert(sizeof(struct mon_status) == 0x40 * (MON_NCPUS + 1), "status page layout");

void mon_enter_ns(uint64_t entry, uint64_t context) __attribute__((noreturn));
#endif

#endif /* CFW_MONITOR_H */
