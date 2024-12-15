#include "shell.h"
#include "rv32dtb.h"

// Set to 0 to allow OS handles trap/irq directly
// Set to 1 to allow FW to chain trap/irq handler before OS handler
#define HOOK_OS_TRAP (0)

#define REG(x)       ((x) ? irq_regs[x] : 0)
#define REGSET(x,v)  if (x) irq_regs[x] = v

extern uint32_t irq_regs[32];

static void dump_regs (uint32_t trap)
{
  uint32_t mepc;
  uint32_t mtval;
  uint32_t mcause;
  uint32_t mscratch;
  uint32_t mstatus;
  uint32_t mip, mie;
  uint32_t ir;

  CSRR_READ  (mie, mie);
  CSRR_READ  (mip, mip);
  CSRR_READ  (mstatus, mstatus);
  CSRR_READ  (mepc, mepc);
  CSRR_READ  (mtval, mtval);
  CSRR_READ  (mcause, mcause);
  CSRR_READ  (mscratch, mscratch);
  ir = MMIO (mepc);

  printf ("%s @ (pc=0x%08x [0x%08x], addr=0x%08x cause=0x%x)\n", trap ? "trap" : "irq", (uint32_t)mepc, ir,
          (uint32_t)mtval, mcause);

  printf ("  mie      = 0x%08x\n", mie);
  printf ("  mip      = 0x%08x\n", mip);
  printf ("  mstatus  = 0x%08x\n", mstatus);
  printf ("  mscratch = 0x%08x\n", mscratch);
  printf ("  mepc     = 0x%08x\n", mepc);
  printf ("  z0=%08x   ra=%08x   sp=%08x   gp=%08x\n",   0,         *(uint32_t *)4,  irq_regs[0],  irq_regs[3]);
  printf ("  tp=%08x   t0=%08x   t1=%08x   t2=%08x\n",   irq_regs[4], irq_regs[5],  irq_regs[6],  irq_regs[7]);
  printf ("  s0=%08x   s1=%08x   a0=%08x   a1=%08x\n",   irq_regs[8], irq_regs[9],  irq_regs[10], irq_regs[11]);
  printf ("  a2=%08x   a3=%08x   a4=%08x   a5=%08x\n",   irq_regs[12], irq_regs[13], irq_regs[14], irq_regs[15]);
  if (0) {
    // Dump the register 16-31
    printf ("  a6=%08x   a7=%08x   s2=%08x   s3=%08x\n",   irq_regs[16], irq_regs[17], irq_regs[18], irq_regs[19]);
    printf ("  s4=%08x   s5=%08x   s6=%08x   s7=%08x\n",   irq_regs[20], irq_regs[21], irq_regs[22], irq_regs[23]);
    printf ("  s8=%08x   s9=%08x   sa=%08x   sb=%08x\n",   irq_regs[24], irq_regs[25], irq_regs[26], irq_regs[27]);
    printf ("  t3=%08x   t4=%08x   t5=%08x   t6=%08x\n",   irq_regs[28], irq_regs[29], irq_regs[30], irq_regs[31]);
  }
  printf ("\n");
}

int irq (uint32_t mip, uint32_t irqs)
{
  putchar('.');
  // Chain to OS handler pointed by mtvec
  return 1;
}

bool trap (uint32_t mepc, uint32_t mcause, void *mtval)
{
  uint32_t   ir;
  uint8_t    op;
  uint32_t   rdid;

  ir   = *(uint32_t *)(uintptr_t)mepc;
  op   = ir & 0x7f;
  rdid = (ir >> 7) & 0x1f;

  bool handled = false;
  switch (mcause) {
  case 4:
    // unaligned load
    switch (op) {
    case 0x03:
      // load
      {
        uint8_t  rs1_idx = (ir >> 15) & 0x1f;
        uint32_t rs1 = REG (rs1_idx);
        uint32_t imm = ir >> 20;
        int32_t  imm_se = imm | (( imm & 0x800 ) ? 0xfffff000 : 0);
        uint32_t rsval = rs1 + imm_se;
        uint32_t rval = 0;
        uint8_t  acc = ( ir >> 12 ) & 0x7;

        switch (acc) {
        //LB, LH, LW, LBU, LHU
        case 1:
          if (rsval & 1) {
            handled = true;
            rval = (int32_t)(int16_t)(MMIO_B ( rsval ) + (MMIO_B ( rsval + 1) << 8));
          }
          break;

        case 2:
          if ((rsval & 1) == 1) {
            handled = true;
            rval = (int32_t)(MMIO_B ( rsval ) + (MMIO_S ( rsval + 1) << 8) + (MMIO_B ( rsval + 3) << 24));
          } else if ((rsval & 3) == 2) {
            handled = true;
            rval = (int32_t)(MMIO_S ( rsval ) + (MMIO_S ( rsval + 2) << 16));
          }
          break;

        default:
          break;
        }

        if (handled && rdid) {
          REGSET ( rdid, rval );
        }
      }
      break;
    }
    break;

  case 6:
    // unaligned store
    switch (op) {
    case 0x23:
      // store
      {
        uint8_t  rs1_idx = (ir >> 15) & 0x1f;
        uint32_t rs1 = REG (rs1_idx);
        uint8_t  rs2_idx = (ir >> 20) & 0x1f;
        uint32_t rs2 = REG (rs2_idx);
        uint32_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
        if( addy & 0x800 ) addy |= 0xfffff000;
        addy += rs1;
        rdid = 0;
        uint8_t  acc = ( ir >> 12 ) & 0x7;

        //printf ("%x %d %x %x\n", addy, acc, rs1, rs2);

        switch (acc) {
        case 1: // hw
          if (addy & 1) {
            handled = 1;
            MMIO_B ( addy    ) = rs2;
            MMIO_B ( addy + 1) = rs2>>8;
          }
          break;

        case 2: // dw
          if ((addy & 1) == 1) {
            handled = 1;
            MMIO_B ( addy    ) = rs2;
            MMIO_S ( addy + 1) = rs2 >> 8;
            MMIO_B ( addy + 3) = rs2 >> 24;

          } else if ((addy & 3) == 2) {
            handled = 1;
            MMIO_S ( addy) = rs2;
            MMIO_S ( addy+2) = rs2>>16;
          }
          break;

        default:
          break;
        }

        if (handled && rdid) {
          REGSET ( rdid, 0 );
        }

      }
      break;
    }
    break;

  default:
    break;
  }

  if (!handled) {
    // Unhandled trap
    dump_regs (1);
    return 0;
  } else {
    return 1;
  }
}

static
void patch_trap (void)
{
  if (!HOOK_OS_TRAP) {
    // Let OS handle everything
    MMIO (TRAP_VECT + 4) = 0xffc00067;
  } else {
    // Avoid patching the first DWORD pointed by mtvec since the first inst might be cached.
    // Patch TRAP entry to jump to $+0x200 Shell irq_vec() if SIM
    if (!SIM) {
      MMIO (TRAP_VECT + 4) = 0x2000006f; // nop / j $+0x200
    }
  }
}

int boot (uint32_t arg0, uint32_t arg1)
{
  const uint32_t  knl_base = 0x80000000;
  const uint32_t  dtb_off  = 0x00fc0000;
  const uint32_t  dtb_base = knl_base + dtb_off;

  patch_trap ();

  // Copy device tree binary
  memcpy ((void *)dtb_base, (void *)(uintptr_t)dtb_bin, sizeof (dtb_bin));

  // Jump to kernel entry
  __asm volatile  (
    "li   a1, %0\t\n"  // Device tree
    "li 	ra, %1\t\n"  // Kernal entry
    "jalr	0(ra)\t\n"
    : : "i" (dtb_base), "i" (knl_base)
    );

  return 0;
}
