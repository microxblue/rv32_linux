import sys


def GenerateH(bin_path, h_path):
    fin = open (bin_path, "rb")
    bins = fin.read()
    fin.close()
    cnt  = 0

    lines = []
    line  = []

    lines.append("const uint8_t  dtb_bin[] = {")
    for each in bins:
      line.append("0x%02x," % each)
      cnt = cnt + 1
      if (cnt & 15) == 0:
        lines.append('    ' + ' '.join(line))
        line = []
    lines.append("};\n")

    fout =  open (h_path, "w")
    fout.write('\n'.join(lines))
    fout.close()

GenerateH (sys.argv[1], sys.argv[2])