#!/usr/bin/env python3
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.

from sys import argv

def value_to_bytes (value, length):
    return value.to_bytes(length, 'little')

def bytes_to_value (bytes):
    return int.from_bytes (bytes, 'little')

def bytes_to_dwords (bytes):
    remain = len(bytes) % 4
    if remain:
        bins = bytes + b'\x00' * (4 - remain)
    else:
        bins = bytes
    dws = []
    for i in range(0, len(bins), 4):
        dws.append(bytes_to_value(bins[i:i+4]))
    return dws

fwfile    = argv[1]
linuxfile = argv[2]
nwords    = int(argv[3])
offset    = int(argv[4])
output    = argv[5]
with open(fwfile, "rb") as f:
    fwdata = f.read()
fw_dws = bytes_to_dwords(fwdata)

with open(linuxfile, "rb") as f:
    linuxdata = f.read()
ll_dws = bytes_to_dwords(linuxdata)

gap1 = offset - len(ll_dws)
gap2 = nwords - offset - len(fw_dws)

lines = []
for w in ll_dws:
    lines.append("%08x" % (w))
for i in range(gap1):
    lines.append("0")
for w in fw_dws:
    lines.append("%08x" % (w))
for i in range(gap2):
    lines.append("0")

if 1:
  fp = open(output, 'w')
  fp.write('\n'.join(lines))
  fp.close()

