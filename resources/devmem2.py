#!/usr/bin/python3

# simply a python3 version of the 'devmem2' program written by Jan-Derk Bakker (2010)
# this one is written by Pat Beirne <patb@pbeirne.com>

from mmap import mmap
import os, sys, time, struct

if __name__ == "__main__":
  if len(sys.argv) == 1:
    print("devmem2: read/write memory location, version 1.0")
    print("  python3 version; only operates on 32 bit words (w)")
    print("  syntax:     devmem2 address [ type [ write-data ]]")
    sys.exit(1)

  write_mode = False
  if len(sys.argv) == 4:
    if sys.argv[2] != 'w':
      print("error: only w is permitted in the 2nd parameter....32bit word")
      sys.exit(1)
    try:
      write_data = int(sys.argv[3],0)
    except:
      print("error: the 3rd parameter must be a number, not {}".format(sys.argv[3]))
      sys.exit(1)
    write_mode = True
  try:
    address = int(sys.argv[1],0)
  except:
    print("error: the 1st parameter must be a number, not {}".format(sys.argv[1]))
    sys.exit(1)

  # print("got past parameter check")

  with open("/dev/mem","r+b") as m:
    address_page = address & ~0xFFF
    address_offset = address & 0xFFF
    mem = mmap(m.fileno(), 32, offset = address_page)
    # read 4-byte word....often a requirement on ARM devices
    read_data = struct.unpack("<L", mem[address_offset: address_offset+4])[0]  
    print("Value at address {}: {}".format(hex(address),hex(read_data)))
    if write_mode == False:
      sys.exit(0)
    mem[address_offset:address_offset+4] = struct.pack("<L",write_data)
    read_data = struct.unpack("<L", mem[address_offset: address_offset+4])[0]  
    print("Written {}; readback {}".format(hex(write_data),hex(read_data)))
    sys.exit(0)

