/* riscv-assembly   Description   Address    Machine Code

 main: addi x2, x0, 5      # x2 = 5                   0       00500113
      addi x3, x0, 12     # x3 = 12                  4       00C00193
      addi x7, x3, -9     # x7 = (12 - 9) = 3        8       FF718393
      or   x4, x7, x2     # x4 = (3 OR 5) = 7        C       0023E233
      and  x5, x3, x4     # x5 = (12 AND 7) = 4      10      0041F2B3
      add  x5, x5, x4     # x5 = 4 + 7 = 11          14      004282B3
      beq  x5, x7, end    # shouldn't be taken       18      02728863
      slt  x4, x3, x4     # x4 = (12 < 7) = 0        1C      0041A233
      beq  x4, x0, around # should be taken          20      00020463
      addi x5, x0, 0      # shouldn't execute        24      00000293

around: slt  x4, x7, x2   # x4 = (3 < 5) = 1         28      0023A233
      add  x7, x4, x5     # x7 = (1 + 11) = 12       2C      005203B3
      sub  x7, x7, x2     # x7 = (12 - 5) = 7        30      402383B3
      sw   x7, 84(x3)     # [96] = 7                 34      0471AA23
      lw   x2, 96(x0)     # x2 = [96] = 7            38      06002103
      add  x9, x2, x5     # x9 = (7 + 11) = 18       3C      005104B3
      jal  x3, end        # jump to end, x3 = 0x44   40      008001EF
      addi x2, x0, 1      # shouldn't execute        44      00100113

end:   add  x2, x2, x9    # x2 = (7 + 18) = 25       48      00910133
      sw   x2, 0x20(x3)   # [100] = 25               4C      0221A023
done:  beq  x2, x2, done  # infinite loop            50      00210063
*/

`timescale 1ns/1ps

module riscv_5stage_hazard_tb();
    
    reg clk = 1'b0;
    reg rst; 
    
    // All output wires from your design
    wire [31:0] inst_mem_out;
    wire [31:0] pc_adder_out;
    wire [31:0] pc_out;
    

    wire reg_write_signal_out, mem_write_signal_out, jump_signal_out, branch_signal_out, alu_src_signal_out;
    wire [1:0] result_src_signal_out;
    wire [2:0] alu_control_out; 
    wire [31:0] reg_data_1_out, reg_data_2_out, imm_ext_out, pc_decode_out, pc_adder_decode_out;
    wire [4:0] write_reg_addr_decode_out, src1_D_haz, src2_D_haz;
    wire [4:0] source1_addr_decode, source2_addr_decode;

    wire reg_write_exe_out, mem_write_exe_out, pc_src_out, o_pc_src_exe_hazard;
    wire [1:0] result_src_exe_out;
    wire [31:0] alu_result_exe_out, reg_data_2_out_exe_out, pc_adder_exe_out, pc_target_out;
    wire [4:0] write_reg_addr_exe_out, source1_addr_exe, source2_addr_exe, O_write_reg_add_E_haz;
    wire [1:0] O_result_src_E_haz;

    wire reg_write_mem_out;
    wire [1:0] result_src_mem_out;
    wire [31:0] mem_data_out_mem_out, alu_result_out_mem_out, pc_adder_out_mem_out;
    wire [4:0] write_reg_file_addr_out, write_reg_addr_mem_haz_wire;
    wire reg_write_mem_haz; 
    wire [31:0] alu_result_exe_haz;

    wire [31:0] final_result_out;
    wire reg_write_wb_out;
    wire [4:0] write_wb_addr_out;

    wire [1:0] forwardA_exe, forwardB_exe;
    wire StallF, StallD, FlushE, i_flushD;
    
    // Instantiate the design with ALL outputs connected
    riscv_5stage_hazard dut (
        .clk(clk), 
        .rst(rst),
        .inst_mem_out(inst_mem_out),
        .pc_out(pc_out),
        .pc_adder_out(pc_adder_out),
        .reg_write_signal_out(reg_write_signal_out),
        .mem_write_signal_out(mem_write_signal_out),
        .jump_signal_out(jump_signal_out),
        .branch_signal_out(branch_signal_out),
        .alu_src_signal_out(alu_src_signal_out),
        .result_src_signal_out(result_src_signal_out),
        .alu_control_out(alu_control_out),
        .reg_data_1_out(reg_data_1_out),
        .reg_data_2_out(reg_data_2_out),
        .imm_ext_out(imm_ext_out),
        .pc_decode_out(pc_decode_out),
        .pc_adder_decode_out(pc_adder_decode_out),
        .write_reg_addr_decode_out(write_reg_addr_decode_out),
        .src1_D_haz(src1_D_haz),
        .src2_D_haz(src2_D_haz),
        .source1_addr_decode(source1_addr_decode),
        .source2_addr_decode(source2_addr_decode),
        .reg_write_exe_out(reg_write_exe_out),
        .mem_write_exe_out(mem_write_exe_out),
        .pc_src_out(pc_src_out),
        .o_pc_src_exe_hazard(o_pc_src_exe_hazard),
        .result_src_exe_out(result_src_exe_out),
        .alu_result_exe_out(alu_result_exe_out),
        .reg_data_2_out_exe_out(reg_data_2_out_exe_out),
        .pc_adder_exe_out(pc_adder_exe_out),
        .pc_target_out(pc_target_out),
        .write_reg_addr_exe_out(write_reg_addr_exe_out),
        .source1_addr_exe(source1_addr_exe),
        .source2_addr_exe(source2_addr_exe),
        .O_write_reg_add_E_haz(O_write_reg_add_E_haz),
        .O_result_src_E_haz(O_result_src_E_haz),
        .reg_write_mem_out(reg_write_mem_out),
        .result_src_mem_out(result_src_mem_out),
        .mem_data_out_mem_out(mem_data_out_mem_out),
        .alu_result_out_mem_out(alu_result_out_mem_out),
        .pc_adder_out_mem_out(pc_adder_out_mem_out),
        .write_reg_file_addr_out(write_reg_file_addr_out),
        .write_reg_addr_mem_haz_wire(write_reg_addr_mem_haz_wire),
        .reg_write_mem_haz(reg_write_mem_haz),
        .alu_result_exe_haz(alu_result_exe_haz),
        .final_result_out(final_result_out),
        .reg_write_wb_out(reg_write_wb_out),
        .write_wb_addr_out(write_wb_addr_out),
        .forwardA_exe(forwardA_exe),
        .forwardB_exe(forwardB_exe),
        .StallF(StallF),
        .StallD(StallD),
        .FlushE(FlushE),
        .i_flushD(i_flushD)
    );

    always begin
        clk = ~clk;
        #50;
    end
    
    // Reset generation
    initial begin
        rst <= 1'b1;      
        #100;
        rst <= 1'b0;      
        #3000;
        $finish;    
    end
    
       
endmodule
