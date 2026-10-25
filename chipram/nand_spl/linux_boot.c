/*
 * CFW boot path: SPL -> Linux, no SML/TOS/u-boot.
 *
 * The eMMC "boot" partition is raw: an arm64 Image (initramfs built in) at
 * offset 0 and a flattened device tree at CFW_DTB_OFFSET. Both carry their own
 * size (Image header image_size, FDT totalsize), so no DHTB wrapper is needed.
 *
 * With no secure world, the SPL takes over the little EL3 setup Linux expects
 * from firmware (GICv3 groups/redistributor wake, ICC_SRE, SCR) and drops to
 * EL2 on CPU0. The secondaries are powered on later by the EL3 monitor
 * (ext/boot/spl/monitor), which the SPL loads and leaves behind for PSCI.
 */
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <card_sdio.h>
#include <part.h>
#include <security/trustzone/trustzone.h>
#include <security/sprd_ce_ctrl.h>
#include <adi.h>

#ifdef CONFIG_SPL_BOOT_LINUX
#if defined(CONFIG_SPL_VIBRATE_MARKERS) || defined(CONFIG_SPL_VIBRATE_BOOT)
extern void spl_buzz(int count);
#ifdef CONFIG_SPL_VIBRATE_MARKERS
#define CFW_BUZZ(n)	spl_buzz(n)
#else
#define CFW_BUZZ(n)	do { } while (0)
#endif
#endif

extern void spl_enter_linux_el2(unsigned long entry, unsigned long dtb,
				unsigned long mon_vbar, unsigned long mon_sp);
extern uint8_t cfw_el3_vectors[];
extern void udelay(uint32 us);
extern int read_common_partition(block_dev_desc_t *dev, uchar *partition_name,
				 uint32_t offsetsector, uint32_t size, uint8_t *buf);

#define CFW_PART		"boot"
#define CFW_DTB_OFFSET		(48 << 20)
/* mk_cfw_boot.py trailer in the sector before the dtb: "CFWS", Image length,
 * monitor offset and length (0 = none) */
#define CFW_TRAILER_OFFSET	(CFW_DTB_OFFSET - 512)
#define CFW_TRAILER_MAGIC	0x53574643
/* Where the SPL normally lands u-boot (0x9EFFFE00 + DHTB header): known-good
 * for running code after the handoff, and 2 MiB aligned. The cfw dts frees
 * sysdump-uboot here; the 96 MiB window stays below widevine's old base. */
#define CFW_KERNEL_BASE		0x9f000000UL
#define CFW_KERNEL_LIMIT	0xa5000000UL
/* Between the low reservations (ddrbist/sysdumpinfo/cfw-el2-page, < 0x80003000)
 * and the SPL stack. */
#define CFW_DTB_ADDR		0x80600000UL
#define CFW_DTB_MAX		0x100000UL
#define CFW_CHUNK_BLKS		2048	/* 1 MiB per CMD18, the size u-boot loads used */

/* EL3 monitor (monitor/monitor.h): loaded here, reserved no-map in the dts.
 * The status page at +0xf000 is past CFW_MON_MAX. */
#define CFW_MON_BASE		0x80100000UL
#define CFW_MON_MAX		0xf000
#define CFW_MON_MAGIC		0x4d574643	/* "CFWM" */

struct cfw_mon_hdr {
	uint32_t branch, magic, version, size;
	uint64_t init, vectors, stack0;
};

#define ARM64_IMAGE_MAGIC	0x644d5241	/* "ARM\x64" */
#define FDT_MAGIC		0xd00dfeed

#define CFW_APCPU_TS0		0x32610000UL	/* SPRD_APCPUTS0_PHYS */

#define GICD_BASE		0x12000000UL
#define GICR_BASE		0x12040000UL
#define GICR_STRIDE		0x20000UL
#define GICD_CTLR		0x0000
#define GICD_TYPER		0x0004
#define GICD_IGROUPR		0x0080
#define GICD_IGRPMODR		0x0d00
#define GICD_CTLR_ARE_S		(1u << 4)
#define GICD_CTLR_ARE_NS	(1u << 5)
#define GICD_CTLR_RWP		(1u << 31)
#define GICR_TYPER		0x0008
#define GICR_TYPER_LAST		(1u << 4)
#define GICR_FRAMES		8		/* 0x100000 GICR region / stride */
#define GICR_WAKER		0x0014
#define GICR_PWRR		0x0024
#define GICR_PWRR_RDPD		(1u << 0)
#define GICR_PWRR_RDGPD		(1u << 2)
#define GICR_PWRR_RDGPO		(1u << 3)
#define GICR_WAKER_PSLEEP	(1u << 1)
#define GICR_WAKER_CASLEEP	(1u << 2)
#define GICR_SGI		0x10000
#define GICR_IGROUPR0		(GICR_SGI + 0x0080)
#define GICR_IGRPMODR0		(GICR_SGI + 0x0d00)

struct arm64_image_header {
	uint32_t code0, code1;
	uint64_t text_offset;
	uint64_t image_size;
	uint64_t flags;
	uint64_t res2, res3, res4;
	uint32_t magic;
	uint32_t res5;
};

static uint32_t be32(const void *p)
{
	const uint8_t *b = p;

	return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
}

static void __attribute__((noreturn)) cfw_fail(int code)
{
	/* 8 pulses = CFW failure marker, then the stage code. */
	for (;;) {
		CFW_BUZZ(8);
		CFW_BUZZ(code);
	}
}

/* Read len bytes at byte offset off (sector aligned) of the boot partition. */
static int cfw_read(block_dev_desc_t *dev, uint32_t off, uint32_t len, uint8_t *buf)
{
	uint32_t sect = off / dev->blksz;
	uint32_t left = (len + dev->blksz - 1) / dev->blksz;

	while (left) {
		uint32_t n = left > CFW_CHUNK_BLKS ? CFW_CHUNK_BLKS : left;

		if (read_common_partition(dev, (uchar *)CFW_PART, sect,
					  n * dev->blksz, buf))
			return -1;
		sect += n;
		left -= n;
		buf += n * dev->blksz;
	}
	return 0;
}

/*
 * EL3 exception sink, installed by spl_enter_linux_el2(). With no secure
 * world, anything that traps to EL3 after the handoff would otherwise land in
 * whatever VBAR_EL3 the BootROM left, which restarts the boot. Record the
 * syndrome in the first sector of uboot_log, then 8 + 9 pulses forever.
 */
struct cfw_trap_record {
	uint32_t magic;		/* "CFWX" */
	uint32_t vector;	/* offset into the vector table / 0x80 */
	uint64_t esr, elr, far, spsr, scr;
	uint64_t x[6];		/* trapped context's x0..x5 (linux_entry.S) */
};

#define CFW_EL3_SAVED_REGS	0x808e0000UL	/* linux_entry.S stashes x0..x5 here */
/* Diagnostic sector buffers live in DRAM scratch (between the dtb and the SPL
 * stack) rather than BSS, to keep the SPL's IRAM footprint where it was. */
#define CFW_SCRATCH(n)		((uint8_t *)(0x80800000UL + (n) * 0x1000))

void cfw_el3_trap(uint64_t vector, uint64_t esr, uint64_t elr, uint64_t far,
		  uint64_t spsr)
{
	uint8_t *sect = CFW_SCRATCH(1);
	struct cfw_trap_record *r = (struct cfw_trap_record *)sect;
	block_dev_desc_t *dev;
	disk_partition_t info;

	r->magic = 0x58574643;
	r->vector = vector;
	r->esr = esr;
	r->elr = elr;
	r->far = far;
	r->spsr = spsr;
	asm volatile("mrs %0, scr_el3" : "=r" (r->scr));
	for (int i = 0; i < 6; i++)
		r->x[i] = ((volatile uint64_t *)CFW_EL3_SAVED_REGS)[i];

	if (TRUE == Emmc_Init()) {
		spl_set_boot_dev(NULL);
		dev = get_dev();
#ifdef CONFIG_SPL_EMMC_TRACE
		if (dev && !get_partition_info_by_name(dev, (uchar *)"uboot_log", &info))
			Emmc_Write(PARTITION_USER, (uint32_t)info.start, 1, sect);
#endif
	}
	cfw_fail(9);
}

/*
 * Board setup u-boot's board_init() did that Linux drivers rely on but do not
 * do themselves (their bindings take no clocks). Add to this as the kernel
 * finds more: each entry names the u-boot function it replaces.
 */
#define CFW_AON_APB_EB1		0x327d0004UL
#define CFW_AON_APB_EB2		0x327d0008UL
#define CFW_AON_APB_RTC_EB0	0x327d0018UL
/* sc2730 GLB over ADI (0x32120000 + 0x1800, as spl_buzz in mcu.c) */
#define CFW_ANA_GLB		(0x32120000 + 0x1800)
#define CFW_ANA_MODULE_EN0	(CFW_ANA_GLB + 0x0008)	/* ANA_REG_GLB_ARM_MODULE_EN */
#define CFW_ANA_RTC_CLK_EN0	(CFW_ANA_GLB + 0x0010)	/* ANA_REG_GLB_RTC_CLK_EN */

struct cfw_pin {
	uint32_t reg;
	uint32_t val;
};

#include "cfw_pinmap.h"

#define CFW_PIN_BASE		0x32450000UL
#define CFW_ADI_BASE		0x32120000

static void cfw_board_init(void)
{
	unsigned int i;

	/* pin_init(): the board pad mux/pull table. Linux only touches pads its
	 * DT names, so i2c0-3, the amp and charger pads etc. rely on this. */
	for (i = 0; i < ARRAY_SIZE(cfw_pinmap); i++)
		REG32(CFW_PIN_BASE + (cfw_pinmap[i] >> 7) * 4) =
			cfw_pinvals[cfw_pinmap[i] & 0x7f];
	for (i = 0; i < ARRAY_SIZE(cfw_adie_pinmap); i++)
		sci_adi_write(CFW_ADI_BASE + cfw_adie_pinmap[i].reg,
			      cfw_adie_pinmap[i].val, 0);

	/* u-boot's dispc div_to_clk(): drop the DSI analog isolation
	 * (ANALOG_MIPI_DSI_4LANE_DSI_ISO_SW, resets to 0x5). */
	REG32(0x323a0034UL) = 0;

	/* sprd_gpio_init(): AP GPIO controller gate */
	REG32(CFW_AON_APB_EB1) |= 1u << 1;		/* GPIO_EB */

	/* sprd_eic_init(): AON EIC gate + RTC clocks, PMIC-side EIC */
	REG32(CFW_AON_APB_EB2) |= 1u << 10;		/* EIC_EB */
	REG32(CFW_AON_APB_RTC_EB0) |= 3u << 5;		/* EIC_RTC_EB, EIC_RTCDV5_EB */
	sci_adi_write(CFW_ANA_MODULE_EN0, 1u << 3, 0);	/* BIT_EIC_EN */
	sci_adi_write(CFW_ANA_RTC_CLK_EN0, 1u << 3, 0);	/* BIT_RTC_EIC_EN */
}

/*
 * DRAM (PUB) memory firewall. The SPL runs secure at EL3; everything after the
 * eret is non-secure. The stock path relies on SML/firewall_config_pre having
 * programmed these segments, and the block is not reset by a warm reset, so
 * the cfw path must state the policy itself. Each segment matches
 * [first, last] in 4 KiB pages from 0x80000000; first > last disables it.
 */
#define CFW_PUB_FW_BASE		0x3280b000UL	/* SPRD_SEC_TZPC_PHYS + 0xb000 */
#define CFW_PUB_FW_SEG(n)	(CFW_PUB_FW_BASE + 0x1000 + (n) * 0x80)
#define CFW_PUB_FW_FIRST	0x00
#define CFW_PUB_FW_LAST		0x04
#define CFW_PUB_FW_MASKS	0x08	/* rd_sec[4], rd_nsec[4], wr_sec[4], wr_nsec[4] */
#define CFW_DRAM_PAGES		(0xc0000000UL >> 12)	/* 3 GiB */

/* Raw firewall state as found, before this boot changes it: uboot_log
 * sectors 4 (PUB fw base registers) and 5..6 (segments 0..7). */
static void cfw_dump_fw(block_dev_desc_t *dev)
{
	uint32_t *buf = (uint32_t *)CFW_SCRATCH(0);
	disk_partition_t info;
	unsigned int i;

	for (i = 0; i < 128; i++)
		buf[i] = REG32(CFW_PUB_FW_BASE + i * 4);
	for (i = 0; i < 256; i++)
		buf[128 + i] = REG32(CFW_PUB_FW_SEG(0) + i * 4);
#ifdef CONFIG_SPL_EMMC_TRACE
	if (!get_partition_info_by_name(dev, (uchar *)"uboot_log", &info))
		Emmc_Write(PARTITION_USER, (uint32_t)info.start + 4, 3, (uint8_t *)buf);
#endif
}

/* Segment 0 = all of DRAM, every master, secure and non-secure; 1..6 off.
 * Segment 7 (above-DRAM catch-all) is left as it is. */
static void cfw_open_dram(void)
{
	unsigned int n, i;

	REG32(CFW_PUB_FW_SEG(0) + CFW_PUB_FW_FIRST) = 0xffffffff;
	for (i = 0; i < 16; i++)
		REG32(CFW_PUB_FW_SEG(0) + CFW_PUB_FW_MASKS + i * 4) = 0xffffffff;
	REG32(CFW_PUB_FW_SEG(0) + CFW_PUB_FW_LAST) = CFW_DRAM_PAGES - 1;
	REG32(CFW_PUB_FW_SEG(0) + CFW_PUB_FW_FIRST) = 0;

	for (n = 1; n < 7; n++) {
		REG32(CFW_PUB_FW_SEG(n) + CFW_PUB_FW_FIRST) = 0xffffffff;
		REG32(CFW_PUB_FW_SEG(n) + CFW_PUB_FW_LAST) = 0;
	}
}

/* Route every interrupt to Group 1 Non-secure and wake CPU0's redistributor,
 * which is what Linux's GICv3 driver assumes firmware has done. Every wait is
 * bounded: a stuck GIC fails with 8 + 7 instead of hanging silently. */
static void cfw_gic_wait(unsigned long reg, uint32_t mask, uint32_t want)
{
	int t;

	for (t = 0; t < 100000; t++) {
		if ((REG32(reg) & mask) == want)
			return;
		udelay(1);
	}
	cfw_fail(7);
}

static void cfw_gic_init(void)
{
	unsigned long rd = GICR_BASE;
	uint64_t mpidr, aff;
	uint32_t i, nregs, pwrr;
	int t;

	REG32(GICD_BASE + GICD_CTLR) = GICD_CTLR_ARE_S | GICD_CTLR_ARE_NS;
	cfw_gic_wait(GICD_BASE + GICD_CTLR, GICD_CTLR_RWP, 0);

	nregs = (REG32(GICD_BASE + GICD_TYPER) & 0x1f) + 1;
	for (i = 1; i < nregs; i++) {
		REG32(GICD_BASE + GICD_IGROUPR + i * 4) = 0xffffffff;
		REG32(GICD_BASE + GICD_IGRPMODR + i * 4) = 0;
	}

	asm volatile("mrs %0, mpidr_el1" : "=r" (mpidr));
	aff = ((mpidr >> 32) & 0xff) << 24 | (mpidr & 0xffffff);
	for (i = 0; i < GICR_FRAMES; i++, rd += GICR_STRIDE) {
		uint64_t typer = *(volatile uint64_t *)(rd + GICR_TYPER);

		if ((typer >> 32) == aff)
			break;
		if (typer & GICR_TYPER_LAST)
			cfw_fail(7);
	}
	if (i == GICR_FRAMES)
		cfw_fail(7);

	/* GIC-600: the redistributor is powered down until EL3 clears
	 * GICR_PWRR.RDPD, after any group power transition has settled (as TF-A's
	 * gic600_pwr_on). Reserved, RAZ/WI, on a GIC-500. */
	for (t = 0; t < 100000; t++) {
		pwrr = REG32(rd + GICR_PWRR);
		if (!!(pwrr & GICR_PWRR_RDGPD) == !!(pwrr & GICR_PWRR_RDGPO)) {
			REG32(rd + GICR_PWRR) = 0;
			if (!(REG32(rd + GICR_PWRR) & GICR_PWRR_RDPD))
				break;
		}
		udelay(1);
	}
	if (t == 100000)
		cfw_fail(7);

	REG32(rd + GICR_WAKER) &= ~GICR_WAKER_PSLEEP;
	cfw_gic_wait(rd + GICR_WAKER, GICR_WAKER_CASLEEP, 0);
	REG32(rd + GICR_IGROUPR0) = 0xffffffff;
	REG32(rd + GICR_IGRPMODR0) = 0;
}

/*
 * Rescue the previous kernel's log. Across a reset that keeps DRAM, the old
 * kernel is still at CFW_KERNEL_BASE with its printk buffer in its BSS. Find
 * the first boot line as printed (the format string in the Image itself reads
 * "0x%010lx", so it cannot match), and copy a window from just before it to
 * uboot_log: a cfw_log_record in sector 1, the raw RAM from sector 8.
 */
/*
 * Progress probe in the reserved cfw-el2-page, for a reset that keeps
 * DRAM. Before the jump the SPL records where the kernel is and fills the
 * part of image_size past the Image file (BSS, then the early page tables)
 * with CFW_FILL. On the next boot, the 16 KiB chunks the old kernel wrote are
 * mapped into uboot_log sector 2: the idmap tables are built with the MMU
 * off, BSS is cleared only after it is on. 6 pulses = DRAM survived.
 */
struct cfw_canary {
	uint64_t magic;		/* CFW_CANARY */
	uint64_t kernel;
	uint64_t fill_start, fill_end;
};

struct cfw_probe_record {
	uint32_t magic;		/* "CFWB" */
	uint32_t chunks;	/* entries used in map[] */
	uint64_t kernel, fill_start, fill_end;
	/* per CFW_PROBE_CHUNK: 0 = untouched, 1 = partly written, 2 = fully */
	uint8_t map[512 - 32];
};

/* Page layout (linux_entry.S): EL2 stub at 0x000, EL2 record at 0xe00,
 * canary at 0xf00. */
#define CFW_EL2_PAGE		0x80002000UL
#define CFW_EL2_RECORD		(CFW_EL2_PAGE + 0xe00)
#define CFW_CANARY_ADDR		(CFW_EL2_PAGE + 0xf00)
#define CFW_CANARY		0x5952414e41435746ULL	/* "FWCANARY" */
#define CFW_FILL		0xa5c3a5c3a5c3a5c3ULL
#define CFW_PROBE_CHUNK		0x4000

static void cfw_probe_old_kernel(block_dev_desc_t *dev)
{
	uint8_t *sect = CFW_SCRATCH(2);
	struct cfw_canary *c = (struct cfw_canary *)CFW_CANARY_ADDR;
	struct cfw_probe_record *r = (struct cfw_probe_record *)sect;
	disk_partition_t info;
	unsigned long p, q;
	uint32_t n;

	r->magic = 0x42574643;
	r->kernel = c->kernel;
	r->fill_start = c->fill_start;
	r->fill_end = c->fill_end;
	for (n = 0, p = c->fill_start; p < c->fill_end && n < sizeof(r->map);
	     n++, p += CFW_PROBE_CHUNK) {
		unsigned long end = p + CFW_PROBE_CHUNK < c->fill_end ?
				    p + CFW_PROBE_CHUNK : c->fill_end;
		uint32_t kept = 0, words = 0;

		for (q = p; q < end; q += 8, words++)
			if (*(volatile uint64_t *)q == CFW_FILL)
				kept++;
		r->map[n] = kept == words ? 0 : kept ? 1 : 2;
	}
	r->chunks = n;

#ifdef CONFIG_SPL_EMMC_TRACE
	if (!get_partition_info_by_name(dev, (uchar *)"uboot_log", &info)) {
		Emmc_Write(PARTITION_USER, (uint32_t)info.start + 2, 1, sect);
		/* EL2 stub record, verbatim (sector 3) */
		Emmc_Write(PARTITION_USER, (uint32_t)info.start + 3, 1,
			   (uint8_t *)CFW_EL2_RECORD);
	}
#endif
}

/* ramoops (ums512-rg-rotate.dts: 0xfff80000, 256 KiB) keeps a complete console
 * copy; save it raw after the rescued log window, at sector 8 + 2048. */
#define CFW_RAMOOPS_ADDR	0xfff80000UL
#define CFW_RAMOOPS_SIZE	0x40000
#define CFW_RAMOOPS_SECT	(8 + 2048)

static void cfw_save_ramoops(block_dev_desc_t *dev)
{
	disk_partition_t info;
	unsigned long p;

#ifdef CONFIG_SPL_EMMC_TRACE
	if (get_partition_info_by_name(dev, (uchar *)"uboot_log", &info))
		return;
	for (p = 0; p < CFW_RAMOOPS_SIZE; p += CFW_CHUNK_BLKS * 512)
		if (FALSE == Emmc_Write(PARTITION_USER,
					(uint32_t)info.start + CFW_RAMOOPS_SECT + p / 512,
					CFW_CHUNK_BLKS, (uint8_t *)(CFW_RAMOOPS_ADDR + p)))
			return;
#endif
}

static void cfw_arm_probe(unsigned long kernel, uint64_t image_size,
			  uint32_t file_len)
{
	struct cfw_canary *c = (struct cfw_canary *)CFW_CANARY_ADDR;
	unsigned long p;

	c->kernel = kernel;
	c->fill_start = (kernel + file_len + 7) & ~7UL;
	c->fill_end = (kernel + image_size) & ~7UL;
	for (p = c->fill_start; p < c->fill_end; p += 8)
		*(volatile uint64_t *)p = CFW_FILL;
	c->magic = CFW_CANARY;
}

extern uint8_t cfw_el2_blob[], cfw_el2_blob_end[];

/* Install the EL2 entry stub and clear its record (see linux_entry.S). */
static void cfw_install_el2_stub(void)
{
	unsigned long i;

	/* EL2 record and the EL3 checkpoint marker (+0x180); the canary at
	 * +0x100 is armed separately. */
	for (i = 0; i < 0x100; i += 8)
		*(volatile uint64_t *)(CFW_EL2_RECORD + i) = 0;
	*(volatile uint64_t *)(CFW_EL2_RECORD + 0x180) = 0;
	for (i = 0; i < (unsigned long)(cfw_el2_blob_end - cfw_el2_blob); i += 4)
		*(volatile uint32_t *)(CFW_EL2_PAGE + i) = *(uint32_t *)(cfw_el2_blob + i);
}

/* Load the monitor the trailer names; NULL if there is none or it is bad. */
static struct cfw_mon_hdr *cfw_load_monitor(block_dev_desc_t *dev,
					    uint32_t off, uint32_t len)
{
	struct cfw_mon_hdr *h = (struct cfw_mon_hdr *)CFW_MON_BASE;

	if (!off || !len || len > CFW_MON_MAX ||
	    cfw_read(dev, off, len, (uint8_t *)h) || h->magic != CFW_MON_MAGIC)
		return NULL;
	return h;
}

#define CFW_LOG_NEEDLE		"Booting Linux on physical CPU 0x00"
#define CFW_LOG_LEAD		0x10000		/* bytes kept before the match */
#define CFW_LOG_LEN		0x100000
#define CFW_LOG_SECT		8

struct cfw_log_record {
	uint32_t magic;		/* "CFWL" */
	uint32_t len;
	uint64_t addr;		/* physical address of the window */
	uint64_t match;		/* physical address of the needle */
};

static int cfw_match(const uint8_t *p, const char *s)
{
	while (*s)
		if (*p++ != (uint8_t)*s++)
			return 0;
	return 1;
}

static void cfw_save_old_log(block_dev_desc_t *dev)
{
	uint8_t *sect = CFW_SCRATCH(3);
	struct arm64_image_header *old = (struct arm64_image_header *)CFW_KERNEL_BASE;
	struct cfw_log_record *r = (struct cfw_log_record *)sect;
	unsigned long base, end, p, win;
	disk_partition_t info;

	if (old->magic != ARM64_IMAGE_MAGIC || !old->image_size ||
	    CFW_KERNEL_BASE + old->text_offset + old->image_size > CFW_KERNEL_LIMIT)
		return;
	base = CFW_KERNEL_BASE + old->text_offset;
	end = base + old->image_size - sizeof(CFW_LOG_NEEDLE);

	/* Word loads, byte compares only where a 'B' is in the word. */
	for (p = base; p < end; p += 8) {
		uint64_t w = *(volatile uint64_t *)p ^ 0x4242424242424242ULL;
		int i;

		if (!((w - 0x0101010101010101ULL) & ~w & 0x8080808080808080ULL))
			continue;
		for (i = 0; i < 8; i++)
			if (cfw_match((const uint8_t *)p + i, CFW_LOG_NEEDLE))
				goto found;
	}
	return;

found:
#ifdef CONFIG_SPL_EMMC_TRACE
	win = (p - CFW_LOG_LEAD > base ? p - CFW_LOG_LEAD : base) & ~0x1ffUL;
	if (get_partition_info_by_name(dev, (uchar *)"uboot_log", &info))
		return;

	r->magic = 0x4c574643;
	r->len = CFW_LOG_LEN;
	r->addr = win;
	r->match = p;
	if (FALSE == Emmc_Write(PARTITION_USER, (uint32_t)info.start + 1, 1, sect))
		return;
	for (p = 0; p < CFW_LOG_LEN; p += CFW_CHUNK_BLKS * 512)
		if (FALSE == Emmc_Write(PARTITION_USER,
					(uint32_t)info.start + CFW_LOG_SECT + p / 512,
					CFW_CHUNK_BLKS, (uint8_t *)(win + p)))
			return;
	CFW_BUZZ(3);	/* previous kernel log saved */
#endif
}

void nand_boot(void)
{
	struct arm64_image_header *hdr;
	struct cfw_mon_hdr *mon = NULL;
	block_dev_desc_t *dev;
	unsigned long kernel, mon_vbar = 0, mon_sp = 0;
	uint8_t fdt_hdr[512];
	uint32_t dtb_size, file_len, *trailer = (uint32_t *)fdt_hdr;

#ifdef CONFIG_SPL_VIBRATE_BOOT
	spl_buzz(1);
#endif

	sprd_firewall_usb_clk_enable();

	if (TRUE != Emmc_Init())
		cfw_fail(1);
	spl_set_boot_dev(NULL);
	dev = get_dev();
	if (!dev)
		cfw_fail(1);

	if (*(volatile uint64_t *)CFW_CANARY_ADDR == CFW_CANARY) {
		cfw_probe_old_kernel(dev);
		cfw_save_ramoops(dev);
	}
	*(volatile uint64_t *)CFW_CANARY_ADDR = 0;
	cfw_save_old_log(dev);	/* before the new kernel lands on top of it */
	cfw_dump_fw(dev);

	/* Kernel header first: it gives the size to read and validates the
	 * partition. Read it straight into place, then the rest over it. */
	hdr = (struct arm64_image_header *)CFW_KERNEL_BASE;
	if (cfw_read(dev, 0, dev->blksz, (uint8_t *)hdr) ||
	    hdr->magic != ARM64_IMAGE_MAGIC)
		cfw_fail(2);
	kernel = CFW_KERNEL_BASE + hdr->text_offset;
	if (!hdr->image_size ||
	    kernel + hdr->image_size > CFW_KERNEL_LIMIT)
		cfw_fail(3);
	if (hdr->text_offset) {
		/* Legacy text_offset kernel: reload at the offset address. */
		if (cfw_read(dev, 0, dev->blksz, (uint8_t *)kernel))
			cfw_fail(2);
		hdr = (struct arm64_image_header *)kernel;
	}
	/* image_size includes BSS, so this over-reads a little; harmless. */
	if (cfw_read(dev, 0, (uint32_t)hdr->image_size, (uint8_t *)kernel))
		cfw_fail(4);

	if (cfw_read(dev, CFW_DTB_OFFSET, sizeof(fdt_hdr), fdt_hdr) ||
	    be32(fdt_hdr) != FDT_MAGIC)
		cfw_fail(5);
	dtb_size = be32(fdt_hdr + 4);
	if (dtb_size > CFW_DTB_MAX ||
	    cfw_read(dev, CFW_DTB_OFFSET, dtb_size, (uint8_t *)CFW_DTB_ADDR))
		cfw_fail(6);

	/* Image file length, for the progress probe (without the trailer the
	 * probe region is empty), and the monitor. */
	file_len = (uint32_t)hdr->image_size;
	if (!cfw_read(dev, CFW_TRAILER_OFFSET, sizeof(fdt_hdr), fdt_hdr) &&
	    trailer[0] == CFW_TRAILER_MAGIC) {
		if (trailer[1] < hdr->image_size)
			file_len = trailer[1];
		mon = cfw_load_monitor(dev, trailer[2], trailer[3]);
	}

	disable_ahb_ce_eb();
	disable_ahb_ce_efs_eb();
	Emmc_DisSdClk();

	/* Stock pre-jump firewall step (eMMC/SDIO/USB DMA masters non-secure,
	 * IRAM locked to secure), then open DRAM to the non-secure kernel. */
	sprd_firewall_config();
	cfw_open_dram();

	/* Feed the timestamp clock to the arch timer, as u-boot's timer_init()
	 * does: without it CNTPCT never advances and Linux hangs in its first
	 * delay, before any console. */
	REG32(CFW_APCPU_TS0) = 1;
	cfw_board_init();

	cfw_gic_init();
	cfw_install_el2_stub();
	cfw_arm_probe(kernel, hdr->image_size, file_len);

	/* Without a monitor Linux has no PSCI: a psci node in the dtb then
	 * traps into cfw_el3_trap (8 + 9 pulses). */
	if (mon && !((int (*)(unsigned long))mon->init)((unsigned long)cfw_el3_vectors)) {
		mon_vbar = mon->vectors;
		mon_sp = mon->stack0;
	}
	spl_enter_linux_el2(kernel, CFW_DTB_ADDR, mon_vbar, mon_sp);
}

#endif /* CONFIG_SPL_BOOT_LINUX */
