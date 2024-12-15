## Introduction
This project aims to explore the minimum requirements for a RV32 core to boot a Linux kernel image without MMU.

For RV32 core, it has many implementations already. I picked the [nanorv32](https://github.com/elvisfox/nanorv32). It is based on the popular [picorv32](https://github.com/YosysHQ/picorv32) core with some enhancement on interrupt and trap handling.

## Build and Run Instrucitons

### Run on Mini-Rv32 host emulator
To boot Linux on the mini-rv32ima emulator, run:
```c
    # Run emulation
	make emurun
```

### Run on RTL verilator simulation
To boot Linux on the RTL verilator simulation:
```c
    # Build simulation FW
    make simfw
	# Run simulation
	make simrun
```

### Run on FPGA board
To boot Linux on the [Intel Cyclone LP10 evaluation kit](https://www.intel.com/content/www/us/en/developer/topic-technology/edge-5g/hardware/fpga-intel-cyclone10-lp.html):

- Connect FPGA SPI debug interface to FTDI FT2232HQ chip and then connect FTDI FT2232HQ to PC host through USB. Please replace the default UART driver with libusb-win32 driver for its interface 0 using zadig tool.

- Connect FPGA UART interface to PC host through a USB to serial adaptor. Open the UART with a terminal tool such as "Putty", and set baud rate to 115200 without flow control.

- Build and download compiled bits to FPGA
```
	# Build bootloader firmware hex file
	make loader

	# Build bootloader Shell binary
	make fw

    # Build Quartus project under Rv32Soc with Quartus 18.0

	# Download the SOF file to FPGA through JTAG port.
	  UART shoud display:
      "CYCLONE10LP RISC-V Loader v1.0: OK"

	# Download Shell to RV32 soc and load Linux via SPI debug adaptor
	make run

	# Linux should boot to bash prompt
```

- Boot log on FPGA UART
```
CYCLONE10LP RISC-V Loader v1.0: OK

.
*********************************
*       Welcome to Risc-V       *
*         (2011-2021)           *
*        Mini Shell 1.1         *
*********************************
>dl
.......................................................................................
Done

>bt
[    0.000000] Linux version 6.10.0mini-rv32ima-00001-g9568811284f3-dirty (test@ubuntu) (riscv32-buildroot-linux-uclibc-gcc.br_real (Buildroot -g851edd24) 13.2.0, GNU ld (GNU Binutils) 2.40) #13 SunDec 15 11:06:43 PST 2024
[    0.000000] Machine model: riscv-minimal-nommu,qemu
[    0.000000] earlycon: uart8250 at MMIO32 0xffff0400 (options '1000000')
[    0.000000] printk: legacy bootconsole [uart8250] enabled
[    0.000000] Zone ranges:
[    0.000000]   Normal   [mem 0x0000000080000000-0x0000000080fbffff]
[    0.000000] Movable zone start for each node
[    0.000000] Early memory node ranges
[    0.000000]   node   0: [mem 0x0000000080000000-0x0000000080fbffff]
[    0.000000] Initmem setup node 0 [mem 0x0000000080000000-0x0000000080fbffff]
[    0.000000] riscv: base ISA extensions
[    0.000000] riscv: ELF capabilities
[    0.000000] Kernel command line: earlycon=uart8250,mmio32,0xffff0400,1000000 console=ttyS0
[    0.000000] Dentry cache hash table entries: 2048 (order: 1, 8192 bytes, linear)
[    0.000000] Inode-cache hash table entries: 1024 (order: 0, 4096 bytes, linear)
[    0.000000] Built 1 zonelists, mobility grouping off.  Total pages: 4032
[    0.000000] mem auto-init: stack:off, heap alloc:off, heap free:off
[    0.000000] Memory: 13440K/16128K available (1312K kernel code, 276K rwdata, 143K rodata, 673K init, 92K bss, 2688K reserved, 0K cma-reserved)
[    0.000000] SLUB: HWalign=64, Order=0-1, MinObjects=0, CPUs=1, Nodes=1
[    0.000000] NR_IRQS: 64, nr_irqs: 64, preallocated irqs: 0
[    0.000000] riscv-intc: 32 local interrupts mapped
[    0.000000] clint: clint@11000000: timer running at 1000000 Hz
[    0.000000] clocksource: clint_clocksource: mask: 0xffffffffffffffff max_cycles: 0x1d854df40, max_idle_ns: 3526361616960 ns
[    0.000407] sched_clock: 64 bits at 1000kHz, resolution 1000ns, wraps every 2199023255500ns
[    0.140516] Console: colour dummy device 80x25
[    0.182335] Calibrating delay loop (skipped), value calculated using timer frequency.. 2.00 BogoMIPS (lpj=10000)
[    0.254112] pid_max: default: 32768 minimum: 301
[    0.314111] Mount-cache hash table entries: 1024 (order: 0, 4096 bytes, linear)
[    0.370748] Mountpoint-cache hash table entries: 1024 (order: 0, 4096 bytes, linear)
[    1.090743] devtmpfs: initialized
[    1.623351] clocksource: jiffies: mask: 0xffffffff max_cycles: 0xffffffff, max_idle_ns: 19112604462750000 ns
[    1.694119] futex hash table entries: 256 (order: -1, 3072 bytes, linear)
[    2.551196] clocksource: Switched to clocksource clint_clocksource
[    6.999257] workingset: timestamp_bits=30 max_order=12 bucket_order=0
[   20.405749] Serial: 8250/16550 driver, 1 ports, IRQ sharing disabled
[   20.975366] of_serial ffff0400.uart: error -ENXIO: IRQ index 0 not found
[   21.268795] printk: legacy console [ttyS0] disabled
[   21.503556] ffff0400.uart: ttyS0 at MMIO 0xffff0400 (irq = 0, base_baud = 62500) is a 16550
[   21.586087] printk: legacy console [ttyS0] enabled
[   21.586087] printk: legacy console [ttyS0] enabled
[   21.668071] printk: legacy bootconsole [uart8250] disabled
[   21.668071] printk: legacy bootconsole [uart8250] disabled
[   24.129051] clk: Disabling unused clocks
[   27.937543] Freeing unused kernel image (initmem) memory: 668K
[   27.997263] This architecture does not have kernel memory protection.
[   28.057894] Run /init as init process
Welcome to mini-rv32ima Linux
Jan  1 00:00:43 login[22]: root login on 'console'
~ #
```

### References
This project reused some souce code from:
- https://github.com/YosysHQ/picorv32
- https://github.com/elvisfox/nanorv32
- https://github.com/cnlohr/mini-rv32ima
