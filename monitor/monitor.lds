/* cfw EL3 monitor, linked to run at MON_BASE (monitor.h). */
OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(mon_head)

MON_BASE = 0x80100000;

SECTIONS
{
	. = MON_BASE;
	.text : {
		KEEP(*(.text.head))
		*(.text*)
	}
	.rodata : ALIGN(8) { *(.rodata*) }
	.data : ALIGN(8) { *(.data*) }
	__image_end = .;

	.bss (NOLOAD) : ALIGN(16) {
		__bss_start = .;
		*(.bss* COMMON)
		. = ALIGN(16);
		__bss_end = .;
	}
	ASSERT(. <= MON_BASE + 0xe000, "monitor overlaps the previous status page")

	.status_prev MON_BASE + 0xe000 (NOLOAD) : { *(.status_prev) }

	.status MON_BASE + 0xf000 (NOLOAD) : { *(.status) }
	.status_seq MON_BASE + 0xf400 (NOLOAD) : { *(.status_seq) }
	.status_csp MON_BASE + 0xf500 (NOLOAD) : { *(.status_csp) }
	ASSERT(. <= MON_BASE + 0x10000, "status page past the reservation")

	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
