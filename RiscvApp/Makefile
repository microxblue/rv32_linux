PYTHON        := python
ifeq ($(OS), )
  BUSYBOX     :=
else
  BUSYBOX     := busybox
endif

APP             ?= Shell
COMPRESSED_ISA   =
GCC_WARNS        = -Wall -Wextra -Wshadow -Wundef -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wno-implicit-fallthrough -Wno-unused-parameter -Wno-missing-prototypes -Wno-unused-variable
GCC_WARNS       += -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes -pedantic
GCC_WARNS       += -Wno-unused-function -Wno-cast-align
TOOLCHAIN_PREFIX = riscv-none-embed-
SRCDIR           = $(APP)
CC               = $(TOOLCHAIN_PREFIX)gcc
CPFLAGS          = -march=rv32ima -mabi=ilp32 -O2 --std=c11 $(GCC_WARNS) -ffreestanding -nostdlib -ICommon
ASFLAGS          = -march=rv32ima
LDFLAGS          = -O2 -ffreestanding -nostdlib
TOOLDIR          = ../Tools
IMAGEDIR         = ../Images
BLD_TGT         ?=
DTS_APP         ?= dtc

ifeq ($(BLD_TGT),SIM)
MEM_TGT         ?= sram
OUTDIR           = out\$(SRCDIR)Sim
CPFLAGS         += -DSIM=1
else
MEM_TGT         ?= sram2
OUTDIR           = out\$(SRCDIR)
endif

ifeq ($(APP),Shell)
	ifeq ($(MEM_TGT),sdram)
	  TGT_ADDR = 0x80000000
	  LDS_FILE = sdram
	endif
	ifeq ($(MEM_TGT),sram)
	  TGT_ADDR = 0
	  LDS_FILE = sram
	endif
	ifeq ($(MEM_TGT),sram2)
	  TGT_ADDR = 0x200
	  LDS_FILE = sram2
	endif
	TGT_MIF =
else
	TGT_ADDR = 0
	LDS_FILE = sram_loader
	TGT_MIF  = $(OUTDIR)/$(APP).mif
endif

all: prebuild $(OUTDIR)/$(APP).bin $(TGT_MIF)

FIRMWARE_OBJS=\
    $(OUTDIR)/start.o \
    $(OUTDIR)/entry.o \
    $(OUTDIR)/tprintf.o \
    $(OUTDIR)/common.o

ifeq ($(APP),Shell)
	FIRMWARE_OBJS += \
		$(OUTDIR)/memory.o \
		$(OUTDIR)/firmware.o \
		$(OUTDIR)/bootloader.o \
		$(OUTDIR)/flash.o \
		$(OUTDIR)/shell.o \
		$(OUTDIR)/test.o \
		$(OUTDIR)/trap.o

else
	FIRMWARE_OBJS += \
	    $(OUTDIR)/bootloader.o

endif

$(OUTDIR)/%.o : $(CURDIR)/Common/%.c
		$(CC) -c $(CPFLAGS) $< -o $@

$(OUTDIR)/%.o : $(CURDIR)/Common/%.S
		$(CC) -c $(ASFLAGS) $< -o $@

$(OUTDIR)/%.o : $(CURDIR)/$(SRCDIR)/%.c
		$(CC) -c $(CPFLAGS) $< -o $@

$(OUTDIR)/%.o : $(CURDIR)/$(SRCDIR)/%.S
		$(CC) -c $(ASFLAGS) $< -o $@

$(OUTDIR)/$(APP).mif: $(OUTDIR)/$(APP).bin Common/makemif.py
		python Common/makemif.py  $< 10240 > $@
		python Common/makebins.py $< 10240 $(OUTDIR)/$(APP)

$(OUTDIR)/$(APP).hex: prebuild $(OUTDIR)/$(APP).bin $(IMAGEDIR)/Image $(TOOLDIR)/make_linux_hex.py
		python $(TOOLDIR)/make_linux_hex.py  $(OUTDIR)/$(APP).bin $(IMAGEDIR)/Image 4194304 4161536 $@

$(OUTDIR)/$(APP).bin: $(OUTDIR)/$(APP).elf
		$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@

$(OUTDIR)/shell.o: $(SRCDIR)/rv32dtb.h

$(OUTDIR)/$(APP).elf: $(FIRMWARE_OBJS) Common/$(LDS_FILE).lds Makefile
		$(CC) $(LDFLAGS) -o $@ -Wl,-Bstatic,-T,Common/$(LDS_FILE).lds,-Map,$(OUTDIR)/$(APP).map,--strip-debug $(FIRMWARE_OBJS) -lgcc
		$(TOOLCHAIN_PREFIX)objdump -S $@ > $@.s

$(SRCDIR)/rv32dtb.h:  $(OUTDIR)/rv32.dtb
		$(PYTHON) $(TOOLDIR)/bin2h.py $(OUTDIR)/rv32.dtb $@

$(OUTDIR)/rv32.dtb : $(CURDIR)/$(SRCDIR)/rv32.dts
		$(DTS_APP) -I dts -O dtb -o $@ $^ -S 1536

hex: $(OUTDIR)/$(APP).hex

run: $(OUTDIR)/$(APP).bin
		@python $(TOOLDIR)/FtdiTalk.py none 0x0080 DEDIPROG:ftdi:2232/1 r
		@python $(TOOLDIR)/FtdiTalk.py $(OUTDIR)/$(APP).bin $(TGT_ADDR) DEDIPROG:ftdi:2232/1 e
		@python $(TOOLDIR)/FtdiTalk.py "dl" 0x0000  DEDIPROG:ftdi:2232/1 c
		@python $(TOOLDIR)/FtdiTalk.py $(IMAGEDIR)\Image 0x80000000 DEDIPROG:ftdi:2232/1 u
		@python $(TOOLDIR)/FtdiTalk.py "bt" 0x0000  DEDIPROG:ftdi:2232/1 c

sts:
		@python $(TOOLDIR)/FtdiTalk.py  none 0      DEDIPROG:ftdi:2232/1 v

prebuild:
		@$(BUSYBOX) mkdir -p $(OUTDIR)

clean:
		@$(BUSYBOX) rm -rf $(OUTDIR)

.PHONY:  clean
