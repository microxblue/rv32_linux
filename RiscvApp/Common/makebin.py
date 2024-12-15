#!/usr/bin/python3
#
# makebin.py 
#
# Prepares memory initialization files for the Verilog readmemb() functions generated by SpinalHDL.
#
# Input:  A memory image file, hexadecimal, one 32 bit memory location per line.
#
# Output: Four, byte wide, symbol files in binary.
#

# The memory image file in hexadecimal, one 32 bit location content per line.
hexFileName = "firmware.hex"

# The binary symbol (byte) output files
outFileName0 = "Memory.v_toplevel_memory_1_symbol0.bin"
outFileName1 = "Memory.v_toplevel_memory_1_symbol1.bin"
outFileName2 = "Memory.v_toplevel_memory_1_symbol2.bin"
outFileName3 = "Memory.v_toplevel_memory_1_symbol3.bin"

hexFile = open(hexFileName, "r")

outFile0 = open(outFileName0, "w+") 
outFile1 = open(outFileName1, "w+") 
outFile2 = open(outFileName2, "w+") 
outFile3 = open(outFileName3, "w+") 

# Read memory location values one per line
for line in hexFile:

    # Convert to 32 bit integer
    data = int(line, 16)

    # Split into 4 bytes
    byte0 = (data >>  0) & 0xFF
    byte1 = (data >>  8) & 0xFF
    byte2 = (data >> 16) & 0xFF
    byte3 = (data >> 24) & 0xFF

    # Write symbol ouptut files, one byte in binary per line 
    outFile0.write(format(byte0, '08b') + '\n')
    outFile1.write(format(byte1, '08b') + '\n')
    outFile2.write(format(byte2, '08b') + '\n')
    outFile3.write(format(byte3, '08b') + '\n')
  
# Close up the shop.  
outFile0.close()
outFile1.close()
outFile2.close()
outFile3.close()

