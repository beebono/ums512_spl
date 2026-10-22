#!/usr/bin/env python3
"""Pack the raw eMMC "boot" partition the cfw SPL reads (linux_boot.c).

  offset 0       arm64 Image (initramfs built in)
  48 MiB - 512   trailer: b"CFWS", Image file length (u32 LE)
  offset 48 MiB  flattened device tree

usage: mk_cfw_boot.py Image board.dtb boot.img
"""
import struct
import sys

DTB_OFFSET = 48 << 20
PART_SIZE = 64 << 20
KERNEL_LIMIT = 0xa5000000 - 0x9f000000  # must match CFW_KERNEL_BASE/LIMIT
DTB_MAX = 1 << 20

image, dtb, out = sys.argv[1:4]
k = open(image, "rb").read()
d = open(dtb, "rb").read()

if len(k) < 64 or k[56:60] != b"ARM\x64":
    sys.exit("error: %s is not an arm64 Image" % image)
text_offset, image_size = struct.unpack_from("<QQ", k, 8)
if text_offset + image_size > KERNEL_LIMIT:
    sys.exit("error: kernel image_size 0x%x exceeds the SPL load window" % image_size)
if len(k) > DTB_OFFSET - 512:
    sys.exit("error: Image (%d bytes) overlaps the dtb offset" % len(k))
if d[:4] != b"\xd0\x0d\xfe\xed" or len(d) > DTB_MAX:
    sys.exit("error: %s is not a dtb under 1 MiB" % dtb)

buf = bytearray(PART_SIZE)
buf[:len(k)] = k
buf[DTB_OFFSET - 512:DTB_OFFSET - 504] = b"CFWS" + struct.pack("<I", len(k))
buf[DTB_OFFSET:DTB_OFFSET + len(d)] = d
open(out, "wb").write(buf)
print("%s: Image %d bytes (image_size 0x%x), dtb %d bytes" % (out, len(k), image_size, len(d)))
