create_clock -name clk -period 16.5 [get_ports clk]
set_clock_uncertainty 0.1  [get_clocks clk]

set_clock_uncertainty -setup 0.01 [get_clocks clk]
set_clock_uncertainty -hold 0.01 [get_clocks clk]

set_output_delay -clock clk -max 0.01 [get_ports inst_mem_out[*]]
set_output_delay -clock clk -min 0.01 [get_ports inst_mem_out[*]]

set_output_delay -clock clk -max 0.01 [get_ports pc_out[*]]
set_output_delay -clock clk -min 0.01 [get_ports pc_out[*]]

set_output_delay -clock clk -max 0.01 [get_ports reg_write_wb_out]
set_output_delay -clock clk -min 0.01 [get_ports reg_write_wb_out]

set_output_delay -clock clk -max 0.01 [get_ports final_result_out[*]]
set_output_delay -clock clk -min 0.01 [get_ports final_result_out[*]]

set_output_delay -clock clk -max 0.01 [get_ports write_wb_addr_out[*]]
set_output_delay -clock clk -min 0.01 [get_ports write_wb_addr_out[*]]

set_output_delay -clock clk -max 0.01 [get_ports ecall_signal]
set_output_delay -clock clk -min 0.01 [get_ports ecall_signal]

set_output_delay -clock clk -max 0.01 [get_ports ebreak_signal]
set_output_delay -clock clk -min 0.01 [get_ports ebreak_signal]

  
