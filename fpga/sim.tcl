#------------------------------------------------------------------------------
# Run the simulation testbench in ModelSim: recompile both Verilog source
# files, then start the simulation, add a lot of signals to the waveform
# viewer, and run. I should (TODO) fix the absolute paths at some point.
#
# Jonathan Westhues, Mar 
#
# run 'vlib work' the first time
#------------------------------------------------------------------------------

vlog -work work -O0 fpga.v
vlog -work work -O0 testbed_fpga.v

vopt +acc work.testbed_fpga -o testbed_fpga_opt

vsim testbed_fpga_opt

add wave sim:/testbed_fpga/adc_clk
add wave sim:/testbed_fpga/adc_d
add wave sim:/testbed_fpga/pwr_lo
add wave sim:/testbed_fpga/ssp_clk
add wave sim:/testbed_fpga/ssp_frame
add wave sim:/testbed_fpga/ssp_din
add wave sim:/testbed_fpga/ssp_dout

# add wave sim:/testbed_fpga/dut/clk_lo
add wave sim:/testbed_fpga/dut/spck
# add wave sim:/testbed_fpga/dut/carrier_divider_lo
# add wave sim:/testbed_fpga/dut/conf_word

run 30000
