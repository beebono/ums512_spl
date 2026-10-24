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
	ASSERT(. <= MON_BASE + 0xf000, "monitor overlaps its status page")

	.status MON_BASE + 0xf000 (NOLOAD) : { *(.status) }
	ASSERT(. <= MON_BASE + 0x10000, "status page past the reservation")

	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
