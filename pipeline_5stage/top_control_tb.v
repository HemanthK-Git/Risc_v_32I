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
