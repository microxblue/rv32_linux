ifeq ($(OS), )
  BUSYBOX     :=
else
  BUSYBOX     := busybox
endif

emurun:
	cd Rv32Emu && make run

loader:
	cd RiscvApp && make APP=Bootloader

run:
	cd RiscvApp && make run

fw:
	cd RiscvApp && make

simfw:
	cd RiscvApp && make BLD_TGT=SIM hex

sim:
	cd Rv32Sim && make

simrun:
	cd Rv32Sim && make run

cleanfw:
	cd RiscvApp && make clean && make BLD_TGT=SIM clean

cleanloader:
	cd RiscvApp && make APP=Bootloader clean

cleanemu:
	cd Rv32Emu && make clean