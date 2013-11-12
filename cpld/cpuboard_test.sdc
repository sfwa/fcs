## Generated SDC file "cpuboard_test.sdc"

## Copyright (C) 1991-2013 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, Altera MegaCore Function License 
## Agreement, or other applicable license agreement, including, 
## without limitation, that your use is for the sole purpose of 
## programming logic devices manufactured by Altera and sold by 
## Altera or its authorized distributors.  Please refer to the 
## applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 13.0.1 Build 232 06/12/2013 Service Pack 1 SJ Web Edition"

## DATE    "Tue Nov 12 22:20:15 2013"

##
## DEVICE  "EPM1270M256C5"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {int_osc|maxii_ufm_block1|osc} -period 181.818 -waveform { 0.000 90.909 } [get_pins {int_osc|maxii_ufm_block1|osc}]
create_clock -name {gpio[0]} -period 10000.000 -waveform { 0.000 5000.000 } [get_ports { gpio[0] }]


#**************************************************************
# Create Generated Clock
#**************************************************************

create_generated_clock -name {osc_clk_scaler[8]} -source [get_nets {int_osc|wire_maxii_ufm_block1_osc}] -divide_by 512 -master_clock {int_osc|maxii_ufm_block1|osc} [get_nets {osc_clk_scaler[8]}] 


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************



#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

