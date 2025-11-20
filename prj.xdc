## Clock Signal (25MHz on Nexys 4 DDR)
set_property PACKAGE_PIN E3 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
# 25MHz clock = 40.000 ns period
create_clock -period 40.000 -name sys_clk_pin -waveform {0.000 20.000} [get_ports clk]

## --- Inputs ---

## Reset Button (CPU_RESET)
set_property PACKAGE_PIN C12 [get_ports reset]
set_property IOSTANDARD LVCMOS33 [get_ports reset]

## Start Switch (SW0)
set_property PACKAGE_PIN J15 [get_ports start]
set_property IOSTANDARD LVCMOS33 [get_ports start]

## --- Outputs ---

# Map the 7-bit 'debiased_count' to LEDs LD0-LD6
# (Matches the 'debiased_count' port in your Verilog)
set_property PACKAGE_PIN H17 [get_ports {debiased_count[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[0]}]
set_property PACKAGE_PIN K15 [get_ports {debiased_count[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[1]}]
set_property PACKAGE_PIN J13 [get_ports {debiased_count[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[2]}]
set_property PACKAGE_PIN N14 [get_ports {debiased_count[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[3]}]
set_property PACKAGE_PIN R18 [get_ports {debiased_count[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[4]}]
set_property PACKAGE_PIN V17 [get_ports {debiased_count[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[5]}]
set_property PACKAGE_PIN U17 [get_ports {debiased_count[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {debiased_count[6]}]

# Map the 1-bit 'o_random_led' to LED LD7
# (Matches the 'o_random_led' port in your Verilog)
set_property PACKAGE_PIN U16 [get_ports o_random_led]
set_property IOSTANDARD LVCMOS33 [get_ports o_random_led]

# You are not using LEDs 8-15, so they are not included.