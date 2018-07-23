# Example:
# make CHISEL=???

CHISEL ?= ../riscv32i-cpu-chisel
VSRC ?= ./thinpad_top.srcs/sources_1/new

all:
	cd $(CHISEL) && sbt run --target-dir=out
	cp $(CHISEL)/out/ChiselTop.v $(VSRC)
	