# Remove the existion library
rm -rf work

# create the library
questa-2019.3 vlib work

bender script vsim -t simulation > heichips.f

# compile the source codes described in the file list
# TODO: Add the netlist into the file list
questa-2019.3 vlog -f heichips.f

# optimize the design.
# Notice here we include the technology libraries, which are needed for the SoC design
# sg13g2_stdcell standard cells
# sg13g2_io IO cell library
# RM_IHPSG13 sram cells
questa-2019.3 vopt  -work work \
  -o testbench_opt testbench

# run the simulation
# 3009 is complaining the missing timeunit/timeprecision for the SRAM behavior model
# We can suppress it as we are using SDF to replace the delay there
# 12088 and 12090 complains some path specified in SDF cannot be found in the verilog behavior model
# This is because of the incomplete behavior model of SRAM and Pad cells
# We have contacted the PDK manufactuer about it and they are working on fixing it
# Currently, we suppress these error message
questa-2019.3 vsim -t 1ps -lib work \
  testbench_opt

# The disable_cdc_check.tcl suppress the timing checks on the JTAG CDC registers
# The Clock Domain Crossing (CDC) circuit moves signals between asynchronous clock domains
# which often causes intentionally setup/hold violations, which would be handled by the special
# cells
# The setup/hold assertion will cause `x` into the circuit (metalstablity) and make simulation
# failed, so we disable checks on these registers.
