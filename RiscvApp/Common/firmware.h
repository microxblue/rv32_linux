// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "common.h"

#define RESET_VECT           0x00
#define TRAP_VECT            0x10

#define FLASH_SIZE           0x800000
#define FLASH_WR_SIZE        0x100

#define FREQ_US              80
#define FREQ_MS              80000

#define HYRAM_BASE           0x80000000
#define HYRAM_SIZE           0x01000000

// HyperRAM 66WVH8M8ALL
#define HYRAM_ID_0           (HYRAM_BASE + 0x01000000)
#define HYRAM_ID_1           (HYRAM_BASE + 0x01100000)
#define HYRAM_REG_0          (HYRAM_BASE + 0x01001000)
#define HYRAM_REG_1          (HYRAM_BASE + 0x01101000)
#define HYRAM_DIE_2          (0x00800000)

#define SPI_FLASH_BASE       0xFFFF0100
#define SPI_CMD_OFF          0x00
#define SPI_CTL_OFF          0x04
#define   B_SPI_STS_BUSY       0x80000000
#define   B_SPI_CTL_RUN        0x80000000
#define   B_SPI_CTL_CS_ASSERT    0x00000300
#define   B_SPI_CTL_CS_RELEASE   0x00000100
#define   B_SPI_CTL_CNT        0x000000FF  // 0:4 bytes 1:1 bytes  2:2 bytes  3:3 bytes
#define SPI_DATA_OFF         0x08

#define GPIO_BASE            0xFFFF0200
#define GPIO_BTN_DIP         0xFFFF0210
#define   B_DIP_1                BIT6
#define   B_DIP_2                BIT5
#define   B_DIP_3                BIT4

#define CLK_FREQ_MHZ         80
#define TIMER_BASE           0xFFFF0300
#define TIMER_CNT0           0xFFFF0300
#define TIMER_CNT1           0xFFFF0304
#define TIMER_CNT2           0xFFFF0308
#define TIMER_CNT3           0xFFFF030C
#define TIMER_CNT4           0xFFFF0310
#define TIMER_CNT5           0xFFFF0314

#define TIMER_CNTN           0xFFFF0310
#define TIMER_CNTN_LI        0xFFFF0310
#define TIMER_CNTN_HI        0xFFFF0314

#define SYS_CTRL_BASE        0xFFFF0500
#define SYS_CTRL_JOYSTICK    0xFFFF0500
#define SYS_CTRL_RESET       0xFFFF05FC

#define UART_DAT             0xFFFF0400
#define UART_LSR             0xFFFF0414
#define LSR_TXRDY                  0x20
#define LSR_RXDA                   0x01
#define LSR_ERR_OVR                0x02
#define LSR_ERR_FRM                0x08

#define PS2_DAT              0xFFFF0600
#define PS2_STS              0xFFFF0604
#define   PS2_STS_AVL              0x01

#define DPRAM_BASE           0xFFFF1000
#define DPRAM_DAT            (DPRAM_BASE + 0x000)
#define DPRAM_CMD            (DPRAM_BASE + 0x600)
#define DPRAM_SHELL_CMD      (DPRAM_BASE + 0x7C0)
#define DPRAM_ARG(x)         (DPRAM_CMD + (x) * 4)

#define GET_RAND(x)        (MMIO(TIMER_CNTN_LI))
#define GET_TSC(Curr)      (Curr.Part.Lo = MMIO(TIMER_CNTN_LI),  Curr.Part.Hi = MMIO(TIMER_CNTN_HI),  Curr.Value)

void  delay (DWORD us);
DWORD tick_to_us (UINT64 Tick);

void  putchar_uart (char ch);

uint32_t _picorv32_irq_timer( uint32_t tVal );
uint32_t _picorv32_irq_disable( uint32_t irqsToDisable );
uint32_t _picorv32_irq_enable( uint32_t irqsToEnable );
uint32_t _picorv32_irq_mask( uint32_t iMask );

#endif
