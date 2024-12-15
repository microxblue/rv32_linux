// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "firmware.h"
#include "common.h"

BYTE VgaPosX = 0;
BYTE VgaPosY = 0;

DWORD tick_to_us (UINT64 Tick)
{
  return (DWORD)(Tick / CLK_FREQ_MHZ);
}

void delay (DWORD us)
{
  UINT64     End;
  UINT64_HL  Curr;

  End = GET_TSC(Curr) + CLK_FREQ_MHZ * us;
  while (Curr.Value < End) {
    (void)GET_TSC(Curr);
  }
}
