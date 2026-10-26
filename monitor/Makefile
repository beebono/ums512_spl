# cfw EL3 monitor. tools/build_cfw_boot.sh builds it into the boot partition:
#   make -C ext/boot/spl/monitor O=<out dir>
CROSS_COMPILE ?= aarch64-linux-gnu-
O ?= .

CC := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy

# MMU off at EL3 means Device memory: strict alignment, and no FP/SIMD so
# the non-secure FP state never needs saving.
CFLAGS := -std=gnu11 -O2 -Wall -Wextra -ffreestanding -fno-builtin \
	-fno-tree-loop-distribute-patterns -fno-pic -fno-pie \
	-fno-stack-protector -fno-asynchronous-unwind-tables \
	-mgeneral-regs-only -mstrict-align
LDFLAGS := -nostdlib -static -no-pie -Wl,--build-id=none -Wl,--no-warn-rwx-segments -Wl,-T,monitor.lds

SRCS := entry.S psci.c

$(O)/monitor.bin: $(O)/monitor.elf
	$(OBJCOPY) -O binary $< $@

$(O)/monitor.elf: $(SRCS) monitor.h pmu_seq.h csp_ops.h pmic_seq.h monitor.lds
	@mkdir -p $(O)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(O)/monitor.elf $(O)/monitor.bin

.PHONY: clean
