#!/usr/bin/env python3
"""Pack the raw eMMC "boot" partition the cfw SPL reads (linux_boot.c).

  offset 0       arm64 Image (initramfs built in)
  48 MiB - 512   trailer: b"CFWS", then u32 LE Image file length, monitor
                 offset, monitor length (0 = no monitor)
  offset 48 MiB  flattened device tree
  offset 49 MiB  EL3 monitor (ext/boot/spl/monitor), optional

usage: mk_cfw_boot.py Image board.dtb boot.img [monitor.bin]
"""
import struct
import sys

DTB_OFFSET = 48 << 20
PART_SIZE = 64 << 20
KERNEL_LIMIT = 0xa5000000 - 0x9f000000  # must match CFW_KERNEL_BASE/LIMIT
DTB_MAX = 1 << 20
MON_OFFSET = DTB_OFFSET + DTB_MAX
MON_MAX = 0xf000  # must match CFW_MON_MAX

if len(sys.argv) not in (4, 5):
    sys.exit(__doc__.strip().splitlines()[-1])
image, dtb, out = sys.argv[1:4]
k = open(image, "rb").read()
d = open(dtb, "rb").read()
m = open(sys.argv[4], "rb").read() if len(sys.argv) == 5 else b""

if len(k) < 64 or k[56:60] != b"ARM\x64":
    sys.exit("error: %s is not an arm64 Image" % image)
text_offset, image_size = struct.unpack_from("<QQ", k, 8)
if text_offset + image_size > KERNEL_LIMIT:
    sys.exit("error: kernel image_size 0x%x exceeds the SPL load window" % image_size)
if len(k) > DTB_OFFSET - 512:
    sys.exit("error: Image (%d bytes) overlaps the dtb offset" % len(k))
if d[:4] != b"\xd0\x0d\xfe\xed" or len(d) > DTB_MAX:
    sys.exit("error: %s is not a dtb under 1 MiB" % dtb)
if m and (m[4:8] != b"CFWM" or len(m) > MON_MAX):
    sys.exit("error: %s is not a cfw monitor under 0x%x bytes" % (sys.argv[4], MON_MAX))

buf = bytearray(PART_SIZE)
buf[:len(k)] = k
buf[DTB_OFFSET - 512:DTB_OFFSET - 496] = b"CFWS" + struct.pack(
    "<III", len(k), MON_OFFSET if m else 0, len(m))
buf[DTB_OFFSET:DTB_OFFSET + len(d)] = d
buf[MON_OFFSET:MON_OFFSET + len(m)] = m
open(out, "wb").write(buf)
print("%s: Image %d bytes (image_size 0x%x), dtb %d bytes, monitor %d bytes"
      % (out, len(k), image_size, len(d), len(m)))
