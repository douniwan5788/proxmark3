vlog -work work -O0 hi_iso14443a.v
vlog -work work -O0 hi_iso14443a_tb.v
vlog -work work -O0 fifo_8in_1out.v

vopt -L xilinxcorelib_ver +acc work.hi_iso14443a_tb -o optd

vsim optd

# add wave -unsi sim:/hi_iso14443a_tb/dut/*

# delete wave *adc_d
# add wave -position 1 -unsi sim:/hi_iso14443a_tb/dut/adc_d
# delete wave *mod_type
# add wave -position 1 -bin sim:/hi_iso14443a_tb/dut/mod_type 

# delete wave *pck0 
# delete wave *ck_1356megb 
# delete wave *pwr_lo 
# delete wave *pwr_oe1 
# delete wave *pwr_oe2 
# delete wave *pwr_oe3
# delete wave *cross_hi 
# delete wave *cross_lo
# delete wave *dbg   

# delete wave *input_prev_*
# delete wave *adc_d_times*
# delete wave *tmp_*

# delete wave *pwr_oe4 

# add wave -unsi sim:/hi_iso14443a_tb/dut/mod_sig_buf 
# add wave -unsi sim:/hi_iso14443a_tb/dut/mod_sig_ptr 
# add wave -unsi sim:/hi_iso14443a_tb/dut/new_mod_sig_ptr 
# add wave -unsi sim:/hi_iso14443a_tb/dut/mod_sig 

# add wave -unsi sim:/hi_iso14443a_tb/dut/mod_sig_flip 
# add wave -unsi sim:/hi_iso14443a_tb/dut/fdt_reset 



# add wave -unsi sim:/hi_iso14443a_tb/dut/adc_clk 
# add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier_cnt 
# add wave -unsi sim:/hi_iso14443a_tb/dut/pre_sub_carrier_cnt1 
# add wave -unsi sim:/hi_iso14443a_tb/dut/pre_sub_carrier_cnt2

# add wave -unsi sim:/hi_iso14443a_tb/dut/reg_dbg1
# add wave -unsi sim:/hi_iso14443a_tb/dut/reg_dbg2

add wave -unsi sim:/hi_iso14443a_tb/dut/mod_sig_coil 
add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier
add wave -unsi sim:/hi_iso14443a_tb/dut/pwr_oe4 

# add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier_halfpulse_counter1 
# add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier_halfpulse_counter2 
add wave -unsi sim:/hi_iso14443a_tb/dut/half_bit_send_over
add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier1 
add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier2 
add wave -unsi sim:/hi_iso14443a_tb/dut/pwr_oe4
add wave -unsi sim:/hi_iso14443a_tb/dut/new_mod_sig_coil 



add wave -unsi sim:/hi_iso14443a_tb/dut/fdt_counter
add wave -unsi sim:/hi_iso14443a_tb/dut/fdt_elapsed
add wave -unsi sim:/hi_iso14443a_tb/dut/fdt_indicator

add wave -hex sim:/hi_iso14443a_tb/dut/recv_buf
add wave -hex sim:/hi_iso14443a_tb/dut/to_arm
add wave -hex sim:/hi_iso14443a_tb/rx



add wave -unsi sim:/hi_iso14443a_tb/dut/ssp_frame
add wave -unsi sim:/hi_iso14443a_tb/dut/init_fifo_wr_clk
# add wave -hex sim:/hi_iso14443a_tb/send_buf
add wave -unsi sim:/hi_iso14443a_tb/dut/fifo/*


add wave -unsi sim:/hi_iso14443a_tb/dut/ssp_din



add wave -unsi sim:/hi_iso14443a_tb/dut/mod_type 

add wave -unsi sim:/hi_iso14443a_tb/dut/ssp_dout
add wave -unsi sim:/hi_iso14443a_tb/dut/ssp_clk

add wave sim:/hi_iso14443a_tb/dut/negedge_cnt



# add wave -unsi sim:/hi_iso14443a_tb/dut/sub_carrier 
# add wave -unsi sim:/hi_iso14443a_tb/dut/old_pwr_oe4 

view wave
run 80000ns



