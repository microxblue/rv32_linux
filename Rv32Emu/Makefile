DTS_APP  ?= dtc
GCC_APP  := gcc
OUT_DIR  := out
TOOL_DIR := ../Tools
IMG_DIR  := ../Images

PYTHON        := python
ifeq ($(OS), )
  BUSYBOX     :=
else
  BUSYBOX     := busybox
endif

all: prebuild $(OUT_DIR)/rv32_emu

run: $(OUT_DIR)/rv32_emu
	$(OUT_DIR)/rv32_emu $(IMG_DIR)/Image

$(OUT_DIR)/rv32_emu: rv32dtb.h mini-rv32ima.c
	@$(BUSYBOX) mkdir -p $(OUT_DIR)
	$(GCC_APP) -I. -O2 mini-rv32ima.c -o $@

$(OUT_DIR)/rv32.dtb : rv32.dts
	@$(BUSYBOX) mkdir -p $(OUT_DIR)
	$(DTS_APP) -I dts -O dtb -o $@ $^ -S 1536

rv32dtb.h: $(OUT_DIR)/rv32.dtb
	@$(BUSYBOX) mkdir -p $(OUT_DIR)
	$(PYTHON) $(TOOL_DIR)/bin2h.py $(OUT_DIR)/rv32.dtb rv32dtb.h

prebuild:


clean:
	@$(BUSYBOX) rm -rf out rv32dtb.h

