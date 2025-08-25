all: macro copy-macro

PDK_ROOT ?= ~/.ciel
PDK ?= ihp-sg13g2

RUN_TAG = $(shell ls librelane/runs/ -1 | tail -n 1)

# Macro - LibreLane

macro:
	cd librelane; librelane config.yaml --pdk $(PDK)
.PHONY: macro

macro-openroad:
	cd librelane; librelane config.yaml --pdk $(PDK) --last-run --flow OpenInOpenROAD
.PHONY: macro-openroad

macro-klayout:
	cd librelane; librelane config.yaml --pdk $(PDK) --last-run --flow OpenInKLayout
.PHONY: macro-klayout

copy-macro:
	mkdir -p macro/
	rm -rf macro/*
	cp -r librelane/runs/${RUN_TAG}/final/* macro/
.PHONY: copy-macro

# Simulation & Verification

sim:
	cd tb; python3 testbench.py
.PHONY: sim

sim-gl:
	cd tb; GL=1 python3 testbench.py
.PHONY: sim-gl

# FPGA Emulation

## iCE40HX8K

ICE40HX8K_SOURCES = $(wildcard fpga/ice40hx8k/*.sv) $(wildcard src/*.sv)

synth-ice40hx8k: ice40hx8k.json

pnr-ice40hx8k: ice40hx8k.bit

upload-ice40hx8k: ice40hx8k.bit
	openFPGALoader --board=ice40_generic -f ice40hx8k.bit

ice40hx8k.json: $(ICE40HX8K_SOURCES)
	yosys -l $(basename $@)-yosys.log -DSYNTHESIS -DICE40HX8K -p 'synth_ice40 -top ice40hx8k_top -json $@' $^

ice40hx8k.asc: ice40hx8k.json fpga/ice40hx8k/ice40hx8k-evb.pcf
	nextpnr-ice40 --hx8k --json $< \
		--pcf fpga/ice40hx8k/ice40hx8k-evb.pcf \
		--package ct256 \
		--asc $@

ice40hx8k.bit: ice40hx8k.asc
	icepack $< $@

## iCEBreaker

ICEBREAKER_SOURCES = $(wildcard fpga/icebreaker/*.sv) $(wildcard src/*.sv)

synth-icebreaker: icebreaker.json

pnr-icebreaker: icebreaker.bit

upload-icebreaker: icebreaker.bit
	openFPGALoader --board=ice40_generic -f icebreaker.bit

icebreaker.json: $(ICEBREAKER_SOURCES)
	yosys -l $(basename $@)-yosys.log -DSYNTHESIS -DICEBREAKER -p 'synth_ice40 -top icebreaker_top -json $@' $^

icebreaker.asc: icebreaker.json fpga/icebreaker/icebreaker.pcf
	nextpnr-ice40 --up5k --json $< \
		--pcf fpga/icebreaker/icebreaker.pcf \
		--package sg48 \
		--asc $@

icebreaker.bit: icebreaker.asc
	icepack $< $@

## ULX3S

ULX3S_SOURCES = $(wildcard fpga/ulx3s/*.sv) $(wildcard src/*.sv)

synth-ulx3s: ulx3s.json

pnr-ulx3s: ulx3s.bit

upload-ulx3s: ulx3s.bit
	openFPGALoader --board=ulx3s -f ulx3s.bit

ulx3s.json: $(ULX3S_SOURCES)
	yosys -l $(basename $@)-yosys.log -DSYNTHESIS -DULX3S -p 'synth_ecp5 -top ulx3s_top -json $@' $^

ulx3s.config: ulx3s.json fpga/ulx3s/ulx3s_v20.lpf
	nextpnr-ecp5 --85k --json $< \
		--lpf fpga/ulx3s/ulx3s_v20.lpf \
		--package CABGA381 \
		--textcfg $@

ulx3s.bit: ulx3s.config
	ecppack $< $@ --compress


## TANG NANO 9K

NANO9K_SOURCES = $(wildcard fpga/nano9k/*.sv) $(wildcard src/*.sv)

synth-nano9k: nano9k.json

pnr-nano9k: nano9k.fs

upload-nano9k: nano9k.fs
	openFPGALoader --board=tangnano9k nano9k.fs

nano9k.json: $(NANO9K_SOURCES)
	yosys -l $(basename $@)-yosys.log -DSYNTHESIS -DNANO9k -p 'synth_gowin -top nano9k_top -json $@' $^

nano9k_pnr.json: nano9k.json fpga/nano9k/nano9k.cst
	nextpnr-himbaechel --json $< --write $@ \
		--device GW1NR-LV9QN88PC6/I5 \
		--vopt family=GW1N-9C \
		--vopt cst=fpga/nano9k/nano9k.cst

nano9k.fs: nano9k_pnr.json
	gowin_pack -d GW1N-9C -o $@ $<

## Basys 3

BASYS3_SOURCES = $(wildcard fpga/basys3/*.sv) $(wildcard src/*.sv)

synth-basys3: basys3.json

pnr-basys3: basys3.bit

upload-basys3: basys3.bit
	openFPGALoader --board=basys3 -f basys3.bit

basys3.json: $(BASYS3_SOURCES)
	yosys -l $(basename $@)-yosys.log -DSYNTHESIS -DBASYS3 -p "synth_xilinx -flatten -abc9 ${SYNTH_OPTS} -arch xc7 -top basys3_top; write_json basys3.json" $^

# The chip database only needs to be generated once
# that is why we don't clean it with make clean
nix-openxc7/xc7a35tcpg236.bin:
	pypy3 ${NEXTPNR_XILINX_PYTHON_DIR}/bbaexport.py --device xc7a35tcpg236-1 --bba xc7a35tcpg236.bba
	bbasm -l xc7a35tcpg236.bba nix-openxc7/xc7a35tcpg236.bin
	rm -f xc7a35tcpg236.bba

basys3.fasm: basys3.json nix-openxc7/xc7a35tcpg236.bin fpga/basys3/Basys-3-Master.xdc
	nextpnr-xilinx --chipdb nix-openxc7/xc7a35tcpg236.bin --xdc fpga/basys3/Basys-3-Master.xdc --json basys3.json --fasm $@ ${PNR_ARGS} ${PNR_DEBUG}
	
basys3.frames: basys3.fasm
	fasm2frames --part xc7a35tcpg236-1 --db-root ${PRJXRAY_DB_DIR}/artix7 $< > $@

basys3.bit: basys3.frames
	xc7frames2bit --part_file ${PRJXRAY_DB_DIR}/artix7/xc7a35tcpg236-1/part.yaml --part_name xc7a35tcpg236-1 --frm_file $< --output_file $@

# Common

clean:
	rm -f ice40hx8k.json ice40hx8k.asc ice40hx8k.bit ice40hx8k-yosys.log
	rm -f icebreaker.json icebreaker.asc icebreaker.bit icebreaker-yosys.log
	rm -f ulx3s.json ulx3s.config ulx3s.bit ulx3s-yosys.log
	rm -f basys3.json basys3.bin basys3.fasm basys3.frames basys3.bit basys3-yosys.log
	rm -f nano9k.json nano9k.bin nano9k.fasm nano9k.frames nano9k.fs nano9k-yosys.log nano9k_pnr.json
