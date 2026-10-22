#!/usr/bin/env python3
"""Decode the cfw SPL's records from a uboot_log partition dump.

  sector 0  EL3 trap record ("CFWX")
  sector 1  rescued kernel log header ("CFWL"), log text from sector 8
  sector 2  progress probe ("CFWB"): which 16 KiB chunks past the Image file
            the previous kernel wrote
  sector 3  EL2 stub record: "EL2ENTRY", or "EL2TRAP!" + syndrome
  sector 4  PUB memory firewall base registers as found at boot (raw)
  sector 5-6 PUB memory firewall segments 0..7 as found at boot
  sector 2056 on: raw ramoops region (256 KiB); the console zone is printed
            with --console

usage: cfw_probe.py uboot_log.bin [System.map] [--console]
"""
import struct
import sys

CHUNK = 0x4000
STATE = {0: "untouched", 1: "partly written", 2: "written"}
# Landmarks in the kernel's post-file region. Write order on boot:
# early_init_stack (primary_entry's first instructions, MMU off), then
# init_pg_dir (early_map_kernel, MMU on via the idmap) and the BSS clear.
LANDMARKS = ("__bss_start", "__bss_stop", "init_pg_dir", "__pi_init_pg_dir",
             "init_pg_end", "__pi_init_pg_end", "early_init_stack", "_end")

args = [a for a in sys.argv[1:] if a != "--console"]
log = open(args[0], "rb").read()
if "--console" in sys.argv:
    # ramoops layout for the rg-rotate dts: 6 dmesg records of 0x8000, then
    # the console zone; each zone is a persistent_ram_buffer (sig, start, size).
    zone = log[(8 + 2048) * 512 + 6 * 0x8000:][:0x8000]
    sig, start, size = struct.unpack_from("<III", zone)
    data = zone[12:]
    if sig != 0x43474244:  # "DBGC"
        sys.exit("no ramoops console zone (sig 0x%x)" % sig)
    text = data[start:size] + data[:start] if size >= len(data) - 1 else data[:size]
    sys.stdout.write(text.decode("utf-8", "replace"))
    sys.exit(0)
syms = {}
if len(args) > 1:
    for line in open(args[1]):
        addr, _, name = line.split()
        syms[name] = int(addr, 16)

trap = log[0:512]
if trap[:4] == b"CFWX":
    vec, esr, elr, far, spsr, scr = struct.unpack_from("<I5Q", trap, 4)
    x = struct.unpack_from("<6Q", trap, 48)
    ec = esr >> 26
    print("EL3 trap: vector %d ESR 0x%x (EC 0x%x) ELR 0x%x FAR 0x%x SPSR 0x%x SCR 0x%x"
          % (vec, esr, ec, elr, far, spsr, scr))
    print("  trapped x0..x5: " + " ".join("0x%x" % v for v in x))
    if ec == 0x17:  # SMC64: one of our EL2 diagnostics
        imm = esr & 0xffff
        if imm == 1:
            print("  -> smc #1: the eret reached the DRAM stub at EL2 (stub at 0x%x)" % (elr - 4))
        elif imm == 2:
            e2 = x[1]
            print("  -> smc #2: EL2 exception before the stub: vector %d ESR_EL2 0x%x (EC 0x%x) "
                  "ELR_EL2 0x%x FAR_EL2 0x%x" % (x[0], e2, e2 >> 26, x[2], x[3]))

hdr = log[512:1024]
if hdr[:4] == b"CFWL":
    ln, addr, match = struct.unpack_from("<IQQ", hdr, 4)
    print("kernel log rescued: 0x%x bytes from 0x%x (first line at 0x%x), sector 8 on" % (ln, addr, match))

fw = struct.unpack_from("<128I", log, 2048)
segs = log[2560:3584]
print("PUB firewall as found at boot (base regs, nonzero):",
      " ".join("+0x%x=%08x" % (i * 4, v) for i, v in enumerate(fw) if v) or "all zero")
for n in range(8):
    w = struct.unpack_from("<18I", segs, n * 0x80)
    first, last = w[0], w[1]
    span = ("off" if first > last else
            "0x%x-0x%x" % (0x80000000 + (first << 12), 0x80000000 + ((last + 1) << 12) - 1))
    print("  seg%d %-24s rd_s %s rd_ns %s wr_s %s wr_ns %s" % (
        n, span, *("".join("%08x" % x for x in w[2 + 4 * k:6 + 4 * k]) for k in range(4))))

el2 = log[1536:2048]
if el2[0x180:0x188] == b"EL2SMC1!":
    print("EL3 checkpoint: the eret reached the DRAM stub at EL2 (smc #1 returned)")
tag = el2[:8]
if tag == b"EL2ENTRY":
    print("EL2 stub: reached non-secure EL2 and branched to the kernel, no EL2 trap before its vectors")
elif tag == b"EL2TRAP!":
    vec, esr, elr, far, spsr = struct.unpack_from("<5Q", el2, 8)
    print("EL2 trap before the kernel's vectors: vector %d ESR 0x%x (EC 0x%x) ELR 0x%x FAR 0x%x SPSR 0x%x"
          % (vec, esr, esr >> 26, elr, far, spsr))
elif log[1024:1028] == b"CFWB":
    print("EL2 stub: record empty, the eret never reached the stub")

probe = log[1024:1536]
if probe[:4] != b"CFWB":
    sys.exit("no progress probe record (DRAM did not survive, or no reset since flashing)")
chunks, kernel, start, end = struct.unpack_from("<IQQQ", probe, 4)
smap = probe[32:32 + chunks]
print("kernel at 0x%x, probed 0x%x-0x%x (%d chunks)" % (kernel, start, end, chunks))

# Physical address of a link-time symbol: offset from _text plus the load address.
marks = {}
if "_text" in syms:
    for name in LANDMARKS:
        if name in syms:
            marks.setdefault((syms[name] - syms["_text"] + kernel - start) // CHUNK, []).append(name)

prev = None
for i, st in enumerate(smap):
    here = marks.get(i)
    if st != prev or here:
        print("  0x%x  %-15s %s" % (start + i * CHUNK, STATE.get(st, "?"),
                                     " ".join(here) if here else ""))
    prev = st
