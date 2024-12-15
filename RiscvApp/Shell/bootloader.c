#include "common.h"
#include "firmware.h"
#include "shell.h"

/* ------------------------------------------------------------------------- */
/* Intel HEX file record types                                               */
/* ------------------------------------------------------------------------- */
#define HEX_DATA_REC          0   /* data record */
#define HEX_EOF_REC           1   /* end of file record */
#define HEX_SEG_ADDR_REC      3   /* start segment address record */
#define HEX_EXT_ADDR_REC      4   /* extened linear address record */
#define HEX_START_ADDR_REC    5   /* start linear address record */
#define HEX_READ_ADDR_REC     6   /* read data at address record */

#define MAX_LEN  0x80

#define FLAG_PRINT_DOT   0x01

typedef int (*IMG_ENTRY)(void);

static
void  memcopy (volatile DWORD *dest, volatile DWORD *src, DWORD cont)
{
  cont = (cont + 3) >> 2;
  while (cont--) {
    *dest++ = *src++;
  }
}

#define WR_MEM   0x01
#define RD_MEM   0x02
#define WR_CONT  0x03
#define RD_CONT  0x04
#define WR_FLAG  0x05
#define RD_SPI   0x06
#define WR_MMIO  0x09
#define OP_DONE  0x0D
#define OP_EXE   0x0E

#define SCMD_MMIO_WR           0x01
#define SCMD_MMIO_WR_THEN_OR   0x02
#define SCMD_MMIO_FILL         0x03

#define FLAG_PRINT_DOT   0x01

#define ARG_IDX  0x00
#define CMD_IDX  0x01

void bootloader(void)
{
  DWORD   wcont;
  DWORD   waddr;
  DWORD   rcont;
  DWORD   raddr;
  DWORD   command;
  DWORD   pcnt;
  BYTE    cmdtype;
  BYTE    loop;
  BYTE    scmd;
  BYTE    flag;
  WORD    index;

  IMG_ENTRY entry;
  volatile DWORD *dpramdat = (volatile DWORD *)DPRAM_DAT;
  volatile DWORD *dpramcmd = (volatile DWORD *)DPRAM_CMD;

  raddr = 0;
  waddr = 0;
  pcnt  = 0;
  dpramcmd[CMD_IDX] = 0;

  loop = 1;
  while (loop) {

    command = dpramcmd[CMD_IDX];
    if ((!(command & 0xF000)) || (0xF000 == (command & 0xF000))) {  // No commands
      if (haschar()) {
        loop = 0;
      }
      continue;
    }

    cmdtype = (command & 0xF000) >> 12;
    switch (cmdtype) {

    case RD_MEM: // rd data from memory location
      raddr  = dpramcmd[ARG_IDX];

    case RD_CONT:
      rcont  = command & 0x03FF;
      if (rcont > 512) rcont = 512;
      memcopy (dpramdat, (DWORD *)raddr, rcont);
      raddr += rcont;
      if ((pcnt & 0x1F) == 0) {
        putchar('.');
      }
      break;

    case WR_MEM: // wr data to memory location
      waddr  = dpramcmd[ARG_IDX];

    case WR_CONT:
      wcont  = command & 0x03FF;
      if (wcont > 512) wcont = 512;
      memcopy ((DWORD *)waddr, dpramdat, wcont);
      waddr += wcont;
      if ((pcnt & 0x1F) == 0) {
        putchar('.');
      }
      break;

    case WR_MMIO:
      wcont  = (command & 0x03FF) >> 2;
      index  = 0;
      while (index < wcont) {
        scmd = (dpramdat[index] >> 0) & 0xFF;
        flag = (dpramdat[index] >> 8) & 0xFF;
        switch (scmd) {
        case SCMD_MMIO_WR:
          if (flag == 1) {
            MMIO_B(dpramdat[index+1]) = dpramdat[index+2] & 0xFF;
          } else if (flag == 2) {
            MMIO_S(dpramdat[index+1]) = dpramdat[index+2] & 0xFFFF;
          } else {
            MMIO(dpramdat[index+1]) = dpramdat[index+2];
          }
          index += 3;
          break;
        case SCMD_MMIO_WR_THEN_OR:
          if (flag == 1) {
            MMIO_B_AND_OR(dpramdat[index+1], dpramdat[index+2] & 0xFF, dpramdat[index+3] & 0xFF);
          } else if (flag == 2) {
            MMIO_S_AND_OR(dpramdat[index+1], dpramdat[index+2] & 0xFFFF, dpramdat[index+3] & 0xFFFF);
          } else {
            MMIO_AND_OR(dpramdat[index+1], dpramdat[index+2], dpramdat[index+3]);
          }
          index += 4;
          break;
        case SCMD_MMIO_FILL:
          if (flag == 1) {
            memset  ((void *)dpramdat[index+1], dpramdat[index+2] & 0xFF, dpramdat[index+3]);
          } else if (flag == 2) {
            memsetw ((void *)dpramdat[index+1], dpramdat[index+2] & 0xFFFF, dpramdat[index+3]);
          } else {
            memsetd ((void *)dpramdat[index+1], dpramdat[index+2], dpramdat[index+3]);
          }
          index += 4;
          break;
        default:
          index += 1;
          break;
        }
      }
      break;

    case OP_EXE:
      dpramcmd[CMD_IDX] = 0;
      entry  = (IMG_ENTRY)(dpramcmd[ARG_IDX]);
      // flush cache
      entry();
      break;

    case OP_DONE:
      // flush cache
      printf("\nDone\n");
      loop = 0;
      break;

    default:
      break;
    }
    dpramcmd[CMD_IDX] = 0;
    pcnt++;
  }
}

