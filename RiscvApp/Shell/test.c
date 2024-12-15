#include "shell.h"

int hyram_test (void)
{
  uint32_t  start = HYRAM_BASE;
  uint32_t  end   = HYRAM_BASE + HYRAM_SIZE;

  uint32_t  fail = 0;
  uint32_t  step = 0x10;
  uint32_t  addr;
  uint32_t  val;

  for (addr = start; addr < end; addr += step) {
    MMIO (addr) = addr + ((addr - HYRAM_BASE) >> 12);
  }

  for (addr = start; addr < end; addr += step) {
    val = addr + ((addr - HYRAM_BASE) >> 12);
    if (MMIO (addr) != val) {
      printf ("%08x: %08x (%08x)\n", addr, MMIO (addr), val);
      fail = 1;
      break;
    }
  }

  printf ("Test: %s\n", fail ? "FAIL" : "PASS");

  return 0;
}


