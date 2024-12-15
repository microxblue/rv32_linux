######################################################################
#
# DESCRIPTION: Verilator Example: Small Makefile
#
# This calls the object directory makefile.  That allows the objects to
# be placed in the "current directory" which simplifies the Makefile.
#
# Copyright 2003-2018 by Wilson Snyder. This program is free software; you can
# redistribute it and/or modify it under the terms of either the GNU
# Lesser General Public License Version 3 or the Perl Artistic License
# Version 2.0.
#
######################################################################
# Check for sanity to avoid later confusion
RTL_TOP  := picorv32_wrapper
RTL_DIR  := ../Rv32Soc/cpu
DPI_DIR  := .
MSYS_DIR ?=
VERILATOR_ROOT ?=

ifeq ($(OS),Windows_NT)
export PATH := $(PATH):$(MSYS_DIR)\usr\bin
BZ      := busybox
endif

ifneq ($(words $(CURDIR)),1)
 $(error Unsupported: GNU Make cannot build in directories containing spaces, build elsewhere: '$(CURDIR)')
endif


######################################################################
# Set up variables

# If $VERILATOR_ROOT isn't in the environment, we assume it is part of a
# package inatall, and verilator is in your path. Otherwise find the
# binary relative to $VERILATOR_ROOT (such as when inside the git sources).
ifeq ($(VERILATOR_ROOT),)
VERILATOR = verilator_bin
VERILATOR_COVERAGE = verilator_coverage
else
export VERILATOR_ROOT
VERILATOR = $(VERILATOR_ROOT)/bin/verilator_bin
VERILATOR_COVERAGE = $(VERILATOR_ROOT)/bin/verilator_coverage
endif

# Change timer div for sim
VERILATOR_FLAGS =
# Generate C++ in executable form
VERILATOR_FLAGS += -cc --exe --top-module $(RTL_TOP)

# Generate makefile dependencies (not shown as complicates the Makefile)
#VERILATOR_FLAGS += -MMD
# Optimize
VERILATOR_FLAGS += -O2 -x-assign 0
# Warn abount lint issues; may not want this on less solid designs
VERILATOR_FLAGS +=
# Make waveforms
VERILATOR_FLAGS += --trace
# Check SystemVerilog assertions
VERILATOR_FLAGS += -D__VARILATOR__ -Wno-TIMESCALEMOD -Wno-lint -Wno-fatal -Wno-WIDTH -Wno-CASEINCOMPLETE -Wno-IMPLICIT -Wno-UNSIGNED -Wno-CMPCONST -Wno-PINMISSING -Wno-PINNOCONNECT -Wno-UNUSED -Wno-DEFPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-UNDRIVEN -Wno-SYNCASYNCNET
# Generate coverage analysis
VERILATOR_FLAGS += -I. -I$(RTL_DIR)

# Run Verilator in debug mode
#VERILATOR_FLAGS += --debug
# Add this trace to get a backtrace in gdb
#VERILATOR_FLAGS += --gdbbt

######################################################################
default: bld

gen:
	@$(BZ) echo "-- GENERATE CFG ----------------"
	python gen_xbar.py 2   > lbus_mux_gen.vh
	python gen_xbar.py 1 2 > axi_xbar_gen.vh

fw:
	cd loader && make clean && make

bld:
	@$(BZ) echo
	@$(BZ) echo "-- VERILATE ----------------"
	$(VERILATOR) $(VERILATOR_FLAGS) -f input.vc top.v sim_main.cpp $(DPI_SRC)

	@$(BZ) echo
	@$(BZ) echo "-- COMPILE -----------------"
	$(MAKE) -C obj_dir -f Vpicorv32_wrapper.mk

run:
	@$(BZ) echo "-- RUN ---------------------"
	obj_dir/V$(RTL_TOP) $(ARGS)

trace:
	@$(BZ) echo "-- TRACE ---------------------"
	obj_dir/V$(RTL_TOP) +trace

vcd:
	@$(BZ) echo "-- WAVE ---------------------"
	obj_dir/V$(RTL_TOP) +vcd

.PHONY: run gen bld trace

######################################################################
# Other targets

show-config:
	$(VERILATOR) -V

maintainer-copy::
clean mostlyclean distclean maintainer-clean::
	-$(BZ) rm -rf obj_dir logs *.log *.dmp *.vpd coverage.dat core
