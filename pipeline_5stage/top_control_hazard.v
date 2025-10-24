`timescale 1ns / 1ps

module riscv_5stage_hazard (
    input clk, rst,

    output [31:0] inst_mem_out,
    output [31:0] pc_out,
    output [31:0] pc_adder_out,

    output reg_write_signal_out, mem_write_signal_out, jump_signal_out, branch_signal_out, alu_src_signal_out,
    output [1:0] result_src_signal_out,
    output [3:0] alu_control_out, 
    output [31:0] reg_data_1_out, reg_data_2_out, imm_ext_out, pc_decode_out, pc_adder_decode_out,
    output [4:0] write_reg_addr_decode_out, src1_D_haz, src2_D_haz,
    output [4:0] source1_addr_decode, source2_addr_decode,
    output [2:0] funct3_decode_out,
    output [6:0] opcode_decode_out,

    output reg_write_exe_out,  mem_write_exe_out  , pc_src_out , o_pc_src_exe_hazard,
    output [1:0] result_src_exe_out,
    output [31:0] alu_result_exe_out, reg_data_2_out_exe_out, pc_adder_exe_out, pc_target_out,
    output [4:0] write_reg_addr_exe_out, source1_addr_exe, source2_addr_exe, O_write_reg_add_E_haz,
    output [1:0] O_result_src_E_haz,
    output [2:0] funct3_exe_out,
    output [6:0] opcode_exe_out,
    output [31:0] imm_ext_exe_out,

    output reg_write_mem_out,
    output [1:0] result_src_mem_out,
    output [31:0] mem_data_out_mem_out, alu_result_out_mem_out, pc_adder_out_mem_out,
    output [4:0] write_reg_file_addr_out, write_reg_addr_mem_haz_wire,
    output  reg_write_mem_haz, 
    output [31:0] alu_result_exe_haz,
    output [31:0] imm_ext_mem_out,

    output [31:0] final_result_out,
    output reg_write_wb_out,
    output [4:0] write_wb_addr_out,

    output [1:0] forwardA_exe, forwardB_exe,
    output StallF, StallD, FlushE, i_flushD,

    output ecall_signal,
    output ebreak_signal
    
);

wire ecall_wire, ebreak_wire;

// fetch stage 
fetch_cycle fetch (
    .clk(clk),
    .rst(rst),
    .pc_src(pc_src_out),
    .pc_target(pc_target_out),
    .stallD(StallD),
    .stallF(StallF),
    .flushD(i_flushD),

    .inst_mem_delay(inst_mem_out),
    .pc_delay(pc_out),
    .pc_adder_delay(pc_adder_out)
    
);

// decode stage
decode_cycle decode (
    .clk(clk),
    .rst(rst),
    .reg_write(reg_write_wb_out),
    .inst_mem_in(inst_mem_out),
    .pc_in(pc_out),
    .pc_adder_in(pc_adder_out),
    .result_in(final_result_out),
    .write_reg_in(write_wb_addr_out),
    .flushE(FlushE),

    .reg_write_decode(reg_write_signal_out),
    .mem_write_decode(mem_write_signal_out),
    .jump_decode(jump_signal_out),
    .branch_decode(branch_signal_out),
    .alu_src_decode(alu_src_signal_out),
    .result_src_decode(result_src_signal_out),
    .alu_control_decode(alu_control_out),
    .reg_data_1_out_decode(reg_data_1_out),
    .reg_data_2_out_decode(reg_data_2_out),
    .imm_ext_decode(imm_ext_out),
    .pc_decode(pc_decode_out),
    .write_reg_decode(write_reg_addr_decode_out),
    .source1_decode(source1_addr_decode), 
    .source2_decode(source2_addr_decode), 
    .pc_adder_decode(pc_adder_decode_out),
    .source1_dec_hazard(src1_D_haz),
    .source2_dec_hazard(src2_D_haz),
    .funct3_decode(funct3_decode_out),
    .opcode_decode(opcode_decode_out),
    .ecall_decode(ecall_wire),
    .ebreak_decode(ebreak_wire)
);

    assign ecall_signal = ecall_wire;
    assign ebreak_signal = ebreak_wire;

// execute stage
execute_cycle execute (
    .clk(clk),
    .rst(rst), 
    .reg_write(reg_write_signal_out),
    .mem_write(mem_write_signal_out),
    .jump(jump_signal_out),
    .branch(branch_signal_out),          
    .alu_src(alu_src_signal_out),
    .result_src(result_src_signal_out),
    .alu_control_exe(alu_control_out),
    .reg_data_1_in(reg_data_1_out),
    .reg_data_2_in(reg_data_2_out),
    .pc_in(pc_decode_out),
    .imm_ext_in(imm_ext_out),
    .pc_adder_in(pc_adder_decode_out),
    .write_reg_addr_in(write_reg_addr_decode_out),
    .forwardA(forwardA_exe), 
    .forwardB(forwardB_exe),
    .alu_result_mem_in(alu_result_exe_haz),
    .final_result_wb_in(final_result_out),
    .source1_addr_in(source1_addr_decode),
    .source2_addr_in(source2_addr_decode),
    .funct3_in(funct3_decode_out),
    .opcode_in(opcode_decode_out),
    
    .reg_write_exe(reg_write_exe_out),
    .funct3_exe(funct3_exe_out),
    .mem_write_exe(mem_write_exe_out),
    .pc_src_exe(pc_src_out),
    .result_src_exe(result_src_exe_out),
    .alu_result_exe(alu_result_exe_out),
    .reg_data_2_out_exe(reg_data_2_out_exe_out),
    .pc_adder_exe(pc_adder_exe_out),
    .pc_target_exe(pc_target_out),
    .imm_ext_exe(imm_ext_exe_out),
    .write_reg_addr_out(write_reg_addr_exe_out),
    .source1_addr_exe(source1_addr_exe),
    .source2_addr_exe(source2_addr_exe),
    .write_reg_addr_out_hazard(O_write_reg_add_E_haz),
    .result_src_exe_hazard(O_result_src_E_haz),
    .pc_src_exe_hazard(o_pc_src_exe_hazard)
);

// memory stage
memory_cycle memory (
    .clk(clk),
    .rst(rst),
    .reg_write(reg_write_exe_out),
    .mem_write(mem_write_exe_out),
    .result_src(result_src_exe_out),
    .alu_result_in(alu_result_exe_out),
    .reg_data_2_in(reg_data_2_out_exe_out),
    .pc_adder_in(pc_adder_exe_out),
    .write_reg_addr_in(write_reg_addr_exe_out),
    .funct3(funct3_exe_out),
    .imm_ext_in(imm_ext_exe_out),

    .reg_write_mem(reg_write_mem_out),
    .result_src_mem(result_src_mem_out),
    .mem_data_out_mem(mem_data_out_mem_out),
    .alu_result_out_mem(alu_result_out_mem_out),
    .pc_adder_out_mem(pc_adder_out_mem_out),
    .write_reg_addr_out_mem(write_reg_file_addr_out),
    .write_reg_addr_out_mem_haz(write_reg_addr_mem_haz_wire),
    .reg_write_mem_out_haz(reg_write_mem_haz),
    .alu_result_out_mem_haz(alu_result_exe_haz),
    .imm_ext_out_mem(imm_ext_mem_out)
);

// writeback stage
writeback_cycle writeback (
    .reg_write(reg_write_mem_out),
    .result_src(result_src_mem_out),
    .mem_data_in(mem_data_out_mem_out),
    .alu_result_in(alu_result_out_mem_out),
    .pc_adder_in(pc_adder_out_mem_out), 
    .write_reg_addr_in(write_reg_file_addr_out),
    .imm_ext_in(imm_ext_mem_out),

    .final_result(final_result_out),
    .reg_write_out(reg_write_wb_out),
    .write_reg_addr_out(write_wb_addr_out)
    
);

hazard hazard_detection (
    .rst(rst),
    .reg_write_mem_hazard(reg_write_mem_haz),
    .reg_write_wb_hazard(reg_write_wb_out),
    .write_reg_addr_wb_hazard(write_wb_addr_out),
    .write_reg_addr_mem_hazard(write_reg_addr_mem_haz_wire),
    .source1_addr_hazard(source1_addr_exe),
    .source2_addr_hazard(source2_addr_exe),
    .source1_addr_dec_hazard(src1_D_haz),
    .source2_addr_dec_hazard(src2_D_haz),
    .result_src_exe_hazard(O_result_src_E_haz),
    .write_reg_addr_exe_hazard(O_write_reg_add_E_haz),
    .pc_src_exe_haz(o_pc_src_exe_hazard),

    .forwardA_hazard(forwardA_exe),
    .forwardB_hazard(forwardB_exe),
    .stallF(StallF),
    .stallD(StallD),
    .flushE(FlushE),
    .flushD(i_flushD)

);



endmodule

// 1. Fetch Cycle
module fetch_cycle (
    input clk,pc_src,rst,
    input [31:0] pc_target,
    input stallF,
    input stallD,
    input flushD,

    output [31:0] inst_mem_delay, pc_delay, pc_adder_delay
    );    

    wire [31:0] pc_out_wire, pc_adder_out_wire, pc_mux_out_wire, inst_mem_out_wire;
    reg [31:0] pc_reg, inst_mem_reg, pc_adder_reg;

    mux_2x1 mux_before_pc (
        .mux_input_a(pc_adder_out_wire),
        .mux_input_b(pc_target),
        .mux_select(pc_src),
        .mux_output(pc_mux_out_wire)
    );

    program_counter pc (
        .clk(clk),
        .rst(rst),
        .stallF(stallF),
        .pc_in(pc_mux_out_wire),
        .pc_out(pc_out_wire)
    );

    instruction_memory im (
        .inst_mem_in(pc_out_wire),
        .rst(rst),
        .instruction_out(inst_mem_out_wire)
    );  

    pc_adder pca (
        .pc_adder_in_a(pc_out_wire),
        .pc_adder_in_b(32'd4),
        .pc_adder_out(pc_adder_out_wire)
    );

    // Fetch Pipeline registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_reg <= 32'd0;
            inst_mem_reg <= 32'd0;
            pc_adder_reg <= 32'd0;
        end else if (flushD) begin
            pc_reg <= 32'd0;
            inst_mem_reg <= 32'd0;
            pc_adder_reg <= 32'd0;
        end
        else if (!stallD) begin
            pc_reg <= pc_out_wire;
            inst_mem_reg <= inst_mem_out_wire;
            pc_adder_reg <= pc_adder_out_wire;
        end
    end

    assign inst_mem_delay = (rst == 1'b1) ? 32'd0 : inst_mem_reg;
    assign pc_delay =  (rst == 1'b1) ? 32'd0 : pc_reg;
    assign pc_adder_delay =  (rst == 1'b1) ? 32'd0 : pc_adder_reg;

endmodule


// 2. Decode Cycle
module decode_cycle (
    input clk, rst, reg_write,
    input [31:0] inst_mem_in, pc_in, pc_adder_in, result_in,
    input [4:0] write_reg_in,
    input flushE, 

    output reg_write_decode, mem_write_decode, jump_decode, branch_decode, alu_src_decode,
    output [1:0] result_src_decode,
    output [3:0] alu_control_decode,
    output [31:0] reg_data_1_out_decode, reg_data_2_out_decode,
    output [31:0] imm_ext_decode,
    output [31:0] pc_decode,
    output [4:0] write_reg_decode, source1_decode, source2_decode,
    output [31:0] pc_adder_decode,
    output [4:0] source1_dec_hazard, source2_dec_hazard,
    output [2:0] funct3_decode,
    output [6:0] opcode_decode,
    output ecall_decode,
    output ebreak_decode
);

wire jump_out, branch_out, reg_write_out, mem_write_out, alu_src_out;
wire [1:0] result_src_out;
wire [3:0] alu_control_out;
wire [31:0] reg_data_1_out_wire, reg_data_2_out_wire, imm_ext_wire;
wire [2:0] imm_src_decode;
reg [6:0] opcode_reg;

wire ecall_out, ebreak_out;

reg reg_write_reg, mem_write_reg, jump_reg, branch_reg, alu_src_reg;
reg [3:0] alu_control_reg;
reg [1:0] result_src_reg;
reg [31:0] reg_data_1_out_reg, reg_data_2_out_reg, imm_ext_reg;
reg [31:0] pc_reg;
reg [4:0] write_reg_reg, source1_reg, source2_reg;
reg [31:0] pc_adder_reg;

reg [2:0] funct3_reg;
reg ecall_reg, ebreak_reg;

control_unit control (
    .opcode(inst_mem_in[6:0]),
    .funct3(inst_mem_in[14:12]),
    .funct7_5(inst_mem_in[30]),
    .imm_bits(inst_mem_in[31:20]),
    .jump(jump_out),
    .branch(branch_out),
    .mem_write(mem_write_out),
    .alu_src(alu_src_out),
    .result_src(result_src_out),
    .reg_write(reg_write_out),
    .imm_src(imm_src_decode),
    .alu_control(alu_control_out),
    .ecall(ecall_out),
    .ebreak(ebreak_out)
);

register_file reg_file (
    .clk(clk),
    .rst(rst),
    .reg_write_enable(reg_write),
    .reg_addr1(inst_mem_in[19:15]),
    .reg_addr2(inst_mem_in[24:20]),
    .reg_write_addr(write_reg_in),
    .write_data(result_in),
    .read_data1(reg_data_1_out_wire),
    .read_data2(reg_data_2_out_wire)
);

sign_extension imm_gen (
    .imm_ext_input(inst_mem_in),
    .imm_select(imm_src_decode),
    .imm_ext_output(imm_ext_wire)
);

// Decode Pipeline registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            reg_write_reg <= 1'b0;
            mem_write_reg <= 1'b0;
            jump_reg <= 1'b0;
            branch_reg <= 1'b0;
            opcode_reg <= 7'b0;
            alu_src_reg <= 1'b0;
            funct3_reg <= 3'b0;
            alu_control_reg <= 4'b0000;
            result_src_reg <= 2'b00;
            reg_data_1_out_reg <= 32'd0;
            reg_data_2_out_reg <= 32'd0;
            imm_ext_reg <= 32'd0;
            pc_reg <= 32'd0;
            write_reg_reg <= 5'd0;
            pc_adder_reg <= 32'd0;
            source1_reg <= 5'd0;
            source2_reg <= 5'd0;
            ecall_reg <= 1'b0;
            ebreak_reg <= 1'b0;
        end else if (flushE) begin
            reg_write_reg <= 1'b0;
            mem_write_reg <= 1'b0;
            opcode_reg <= 7'b0;
            funct3_reg <= 3'b0;
            jump_reg <= 1'b0;
            branch_reg <= 1'b0;
            alu_src_reg <= 1'b0;
            alu_control_reg <= 4'b0000;
            result_src_reg <= 2'b00;
            reg_data_1_out_reg <= 32'd0;
            reg_data_2_out_reg <= 32'd0;
            imm_ext_reg <= 32'd0;
            pc_reg <= 32'd0;
            write_reg_reg <= 5'd0;
            pc_adder_reg <= 32'd0;
            source1_reg <= 5'd0;
            source2_reg <= 5'd0;
            ecall_reg <= 1'b0;
            ebreak_reg <= 1'b0;
        end else  begin
            reg_write_reg <= reg_write_out;
            result_src_reg <= result_src_out;
            mem_write_reg <= mem_write_out;
            funct3_reg <= inst_mem_in[14:12];
            opcode_reg <= inst_mem_in[6:0];
            jump_reg <= jump_out;
            branch_reg <= branch_out;
            alu_src_reg <= alu_src_out;
            alu_control_reg <= alu_control_out;
            reg_data_1_out_reg <= reg_data_1_out_wire;
            reg_data_2_out_reg <= reg_data_2_out_wire;
            imm_ext_reg <= imm_ext_wire;

            pc_reg <= pc_in;
            write_reg_reg <= inst_mem_in[11:7];
            source1_reg <= inst_mem_in[19:15];
            source2_reg <= inst_mem_in[24:20];
            pc_adder_reg <= pc_adder_in;
            ecall_reg <= ecall_out;
            ebreak_reg <= ebreak_out;
        end
    end

    assign reg_write_decode = reg_write_reg;
    assign result_src_decode = result_src_reg;
    assign mem_write_decode = mem_write_reg;
    assign jump_decode = jump_reg;  
    assign funct3_decode = funct3_reg;
    assign branch_decode = branch_reg;
    assign alu_src_decode = alu_src_reg;
    assign alu_control_decode = alu_control_reg;
    assign reg_data_1_out_decode = reg_data_1_out_reg;
    assign reg_data_2_out_decode = reg_data_2_out_reg;
    assign imm_ext_decode = imm_ext_reg;
    assign pc_decode = pc_reg;
    assign write_reg_decode = write_reg_reg;
    assign source1_decode = source1_reg;
    assign source2_decode = source2_reg;
    assign pc_adder_decode = pc_adder_reg;
    assign source1_dec_hazard = inst_mem_in[19:15];
    assign source2_dec_hazard = inst_mem_in[24:20];
    assign opcode_decode = opcode_reg;
    assign ecall_decode = ecall_reg;
    assign ebreak_decode = ebreak_reg;

endmodule


// 3. Execute Cycle
module execute_cycle (
    input clk, rst, reg_write, mem_write, jump, branch, alu_src,
    input [1:0] result_src,
    input [3:0] alu_control_exe,
    input [31:0] reg_data_1_in, reg_data_2_in, pc_in, imm_ext_in, pc_adder_in,
    input [4:0] write_reg_addr_in,
    input [4:0] source1_addr_in, source2_addr_in,
    input [1:0] forwardA, forwardB,
    input [31:0] alu_result_mem_in, final_result_wb_in,
    input [2:0] funct3_in,
    input [6:0] opcode_in,

    output reg_write_exe, mem_write_exe, pc_src_exe, 
    output [1:0] result_src_exe,
    output [31:0] alu_result_exe, reg_data_2_out_exe, pc_adder_exe, pc_target_exe,
    output [31:0] imm_ext_exe,
    output [4:0] write_reg_addr_out,
    output [4:0] source1_addr_exe, source2_addr_exe,
    output [4:0] write_reg_addr_out_hazard,
    output [1:0] result_src_exe_hazard,
    output [2:0] funct3_exe,
    output pc_src_exe_hazard
);

wire [31:0] mux1_out_wire, alu_result_wire, mux1_alu_wire, mux2_out_wire;
wire zero, negative, carry, overflow;
wire branch_taken;
reg [2:0] funct3_reg;
wire [31:0] pc_target_calc;
wire is_jalr;
wire is_auipc;
// For JALR
wire [31:0] jalr_target_calc;

reg reg_write_reg, mem_write_reg;
reg [1:0] result_src_reg;
reg [31:0] alu_result_reg, reg_data_2_out_reg;
reg [4:0] write_reg_addr_reg;
reg [31:0] pc_adder_reg;
reg [31:0] imm_ext_reg;

// Detect JALR and auipc instruction (opcode = 1100111)
assign is_jalr = (opcode_in == 7'b1100111) && (funct3_in == 3'b000);
assign is_auipc = (opcode_in == 7'b0010111);

wire [31:0] alu_input_a_sel;
assign alu_input_a_sel = is_auipc ? pc_in : mux1_alu_wire;

// Branch decision logic - evaluates all 6 branch types
assign branch_taken = (funct3_in == 3'b000) ? zero :              // BEQ: branch if equal
                      (funct3_in == 3'b001) ? ~zero :             // BNE: branch if not equal
                      (funct3_in == 3'b100) ? (negative ^ overflow) : // BLT: branch if less than (signed)
                      (funct3_in == 3'b101) ? ~(negative ^ overflow) : // BGE: branch if greater/equal (signed)
                      (funct3_in == 3'b110) ? ~carry :            // BLTU: branch if less than (unsigned)
                      (funct3_in == 3'b111) ? carry :             // BGEU: branch if greater/equal (unsigned)
                      1'b0;                                       // Default: no branch

assign pc_src_exe = (branch && branch_taken) || jump;  // CHANGE THIS LINE
assign pc_src_exe_hazard = pc_src_exe;

mux_2x1 reg_to_alu_mux (
    .mux_input_a(mux2_out_wire),
    .mux_input_b(imm_ext_in),
    .mux_select(alu_src),
    .mux_output(mux1_out_wire)
);

alu alu_unit (
    .a(alu_input_a_sel),
    .b(mux1_out_wire),
    .alu_control(alu_control_exe),
    .result(alu_result_wire),
    .flag_zero(zero),
    .flag_negative(negative),     
    .flag_carry(carry),            
    .flag_overflow(overflow) 
);



mux_4to1 forwardA_mux (
    .mux_input_0(reg_data_1_in),
    .mux_input_1(final_result_wb_in),
    .mux_input_2(alu_result_mem_in),
    .mux_input_3(32'd0), // Not used
    .mux_select(forwardA),
    .mux_output(mux1_alu_wire) 
);


mux_4to1 forwardB_mux (
    .mux_input_0(reg_data_2_in),
    .mux_input_1(final_result_wb_in),
    .mux_input_2(alu_result_mem_in),
    .mux_input_3(32'd0), // Not used
    .mux_select(forwardB),
    .mux_output(mux2_out_wire) 
);

assign pc_target_calc = pc_in + imm_ext_in;
assign jalr_target_calc = mux1_alu_wire + imm_ext_in;

// Final target selection with JALR LSB clearing
assign pc_target_exe = is_jalr ? {jalr_target_calc[31:1], 1'b0} : pc_target_calc;

// Execute Pipeline registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            reg_write_reg <= 1'b0;
            mem_write_reg <= 1'b0;
            result_src_reg <= 2'b00;
            alu_result_reg <= 32'd0;
            reg_data_2_out_reg <= 32'd0;
            write_reg_addr_reg <= 5'd0;
            pc_adder_reg <= 32'd0;
            funct3_reg <= 3'b0;
            imm_ext_reg <= 32'd0;
        end else begin
            reg_write_reg <= reg_write;
            mem_write_reg <= mem_write;
            result_src_reg <= result_src;
            alu_result_reg <= alu_result_wire;
            reg_data_2_out_reg <= mux2_out_wire;
            write_reg_addr_reg <= write_reg_addr_in;
            pc_adder_reg <= pc_adder_in;
            funct3_reg <= funct3_in;
            imm_ext_reg <= imm_ext_in;
        end
    end

    assign alu_result_exe = alu_result_reg;
    assign reg_data_2_out_exe = reg_data_2_out_reg;
    assign write_reg_addr_out = write_reg_addr_reg;
    assign pc_adder_exe = pc_adder_reg;
    assign reg_write_exe = reg_write_reg;
    assign mem_write_exe = mem_write_reg;
    assign result_src_exe = result_src_reg;
    assign funct3_exe = funct3_reg;
    assign source1_addr_exe = source1_addr_in;
    assign source2_addr_exe = source2_addr_in;
    assign imm_ext_exe = imm_ext_reg;

endmodule

// Memory Cycle
module memory_cycle (
    input clk, rst, reg_write, mem_write,
    input [1:0] result_src,
    input [31:0] alu_result_in, reg_data_2_in, pc_adder_in,
    input [4:0] write_reg_addr_in,
    input [2:0] funct3,
    input [31:0] imm_ext_in, 

    output reg_write_mem, 
    output [1:0] result_src_mem,
    output [31:0] mem_data_out_mem, alu_result_out_mem, pc_adder_out_mem,
    output [4:0] write_reg_addr_out_mem,
    output [31:0] alu_result_out_mem_haz,
    output [4:0] write_reg_addr_out_mem_haz,
    output reg_write_mem_out_haz,
    output [31:0] imm_ext_out_mem
);

wire [31:0] mem_data_out;
reg reg_write_reg;
reg [1:0] result_src_reg;
reg [31:0] alu_result_reg, mem_data_out_reg, pc_adder_reg;
reg [4:0] write_reg_addr_reg;
reg [31:0] imm_ext_reg;

data_memory data_mem (
    .clk(clk),
    .rst(rst),
    .data_mem_write_enable(mem_write),
    .data_mem_address(alu_result_in),
    .funct3(funct3),
    .data_mem_write_data(reg_data_2_in),
    .data_mem_read_data(mem_data_out)
);

// Memory Pipeline registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            reg_write_reg <= 1'b0;
            result_src_reg <= 2'b00;
            alu_result_reg <= 32'd0;
            mem_data_out_reg <= 32'd0;
            pc_adder_reg <= 32'd0;
            write_reg_addr_reg <= 5'd0;
            imm_ext_reg <= 32'd0;
        end else begin
            reg_write_reg <= reg_write;
            result_src_reg <= result_src;
            alu_result_reg <= alu_result_in;
            mem_data_out_reg <= mem_data_out;
            write_reg_addr_reg <= write_reg_addr_in;
            pc_adder_reg <= pc_adder_in;
            imm_ext_reg <= imm_ext_in;
        end
    end

    assign reg_write_mem = reg_write_reg;
    assign result_src_mem = result_src_reg;
    assign mem_data_out_mem = mem_data_out_reg;
    assign alu_result_out_mem = alu_result_reg;
    assign pc_adder_out_mem = pc_adder_reg;
    assign write_reg_addr_out_mem = write_reg_addr_reg;
    assign alu_result_out_mem_haz = alu_result_in;
    assign write_reg_addr_out_mem_haz = write_reg_addr_in;
    assign reg_write_mem_out_haz = reg_write;
    assign imm_ext_out_mem = imm_ext_reg;

endmodule

// Write Back Cycle
module writeback_cycle (
    input reg_write,
    input [1:0] result_src,
    input [31:0] mem_data_in, alu_result_in, pc_adder_in,
    input [4:0] write_reg_addr_in,
    input [31:0] imm_ext_in,

    output [31:0] final_result,
    output reg_write_out,
    output [4:0] write_reg_addr_out
);

mux_4to1 writeback_mux (
    .mux_input_0(alu_result_in),
    .mux_input_1(mem_data_in),
    .mux_input_2(pc_adder_in),
    .mux_input_3(imm_ext_in), 
    .mux_select(result_src),
    .mux_output(final_result) 
);

assign reg_write_out = reg_write;
assign write_reg_addr_out = write_reg_addr_in;


endmodule



// 1. multiplexer 2:1 
module mux_2x1 (
    input  [31:0] mux_input_a,
    input  [31:0] mux_input_b,
    input         mux_select,
    output [31:0] mux_output
);
    assign mux_output = (mux_select == 1'b0) ? mux_input_a : mux_input_b;   
endmodule

// 2. Program Counter (PC)
module program_counter (
    input        clk,
    input        rst,
    input        stallF,
    input  [31:0] pc_in,
    output [31:0] pc_out
);
    reg [31:0] pc_reg ;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_reg <= 32'd0; // Reset PC to 0
        end else if (!stallF) begin
            pc_reg <= pc_in; // Update PC with new value
        end
    end

    assign pc_out = pc_reg;

endmodule

// 3. Instruction Memory (IM)
module instruction_memory (
    input  [31:0] inst_mem_in,
    input rst,
    output [31:0] instruction_out
);
    reg [31:0] inst_memory [1023:0]; // 1024 words of 32-bit memory

    initial begin
        $readmemh("mem_file.mem", inst_memory); // Load instructions from a file
    end

    // initial begin
    //    inst_memory[0] = 32'hFFC4A303; // addi x1, x0, 1
    //    inst_memory[1] = 32'h00832383;
    // end

    assign instruction_out = (rst == 1'b1) ? 32'd0 : inst_memory[inst_mem_in[31:2]]; // Word-aligned access (pc[31:2] for 1024 words)

endmodule

// 4. Program Counter Adder (PCA)
module pc_adder (
    input  [31:0] pc_adder_in_a,
    input [31:0] pc_adder_in_b,
    output [31:0] pc_adder_out

);
    assign pc_adder_out = pc_adder_in_a + pc_adder_in_b;

endmodule

// 5. Control Unit
module control_unit (
    input  [6:0] opcode,
    input [2:0] funct3,
    input [11:0] imm_bits,
    input      funct7_5,
    output jump, branch,
    output       mem_write,
    output       alu_src,
    output  [1:0]     result_src,
    output       reg_write,
    output [2:0] imm_src,
    output [3:0] alu_control,
    output ecall,
    output ebreak
);
    wire [1:0] alu_op_wire;
    wire is_system_inst;

    // Detect system instruction
    assign is_system_inst = (opcode == 7'b1110011) && (funct3 == 3'b000);
    
    // Differentiate ECALL and EBREAK using imm[11:0]
    assign ecall  = is_system_inst && (imm_bits == 12'h000);
    assign ebreak = is_system_inst && (imm_bits == 12'h001);
    
    main_decoder md (
        .opcode(opcode),
        .mem_write(mem_write),
        .branch(branch),
        .alu_src(alu_src),
        .result_src(result_src),
        .reg_write( reg_write ),
        .alu_op(alu_op_wire),
        .imm_src(imm_src),
        .jump(jump)
    );

   alu_decoder acu (
       .alu_op(alu_op_wire),
       .funct3(funct3),
       .op_5(opcode[5]),
       .funct7_5(funct7_5),
       .alu_control(alu_control)
   );

endmodule

// 6.1 Main Decoder
module main_decoder(
    input [6:0] opcode,

    output       mem_write,
    output       branch,
    output       alu_src,
    output [1:0] result_src,
    output       reg_write,
    output [1:0] alu_op,
    output [2:0] imm_src,
    output jump
);


    assign reg_write = (opcode == 7'b1110011) ? 1'b0 :
                    (opcode == 7'b0000011 || opcode == 7'b0110011 || opcode == 7'b0010011 || 
                    opcode == 7'b1101111 || opcode == 7'b1100111 || opcode == 7'b0110111 || 
                    opcode == 7'b0010111) ? 1'b1 : 1'b0;

    assign imm_src = (opcode == 7'b0100011) ? 3'b001 : // SW - S-type
                 (opcode == 7'b1100011) ? 3'b010 : // BEQ - B-type
                 (opcode == 7'b1101111) ? 3'b011 : // JAL - J-type
                 (opcode == 7'b0110111 || opcode == 7'b0010111) ? 3'b100 : // LUI, AUIPC - U-type
                 3'b000; // LW, I-type ALU, JALR - I-type

    assign alu_src = (opcode == 7'b0100011 || opcode == 7'b0000011 || opcode == 7'b0010011 || 
                  opcode == 7'b1100111 || opcode == 7'b0010111) ? 1'b1 : 1'b0;

    assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // SW
    
    assign result_src = (opcode == 7'b0000011) ? 2'b01 : // LW - Memory
                    (opcode == 7'b1101111 || opcode == 7'b1100111) ? 2'b10 : // JAL, JALR - PC+4
                    (opcode == 7'b0110111) ? 2'b11 : // LUI - Immediate
                    2'b00; // R-type, I-type ALU, SW, BEQ, AUIPC - ALU result  

    assign branch   = (opcode == 7'b1100011) ? 1'b1 : 1'b0; // BEQ

    assign alu_op    = (opcode == 7'b0110011 || opcode == 7'b0010011) ? 2'b10 : // R-type and I-type
                      (opcode == 7'b1100011) ? 2'b01 : 2'b00; // R-type : BEQ : LW/SW

    assign jump = (opcode == 7'b1101111 || opcode == 7'b1100111) ? 1'b1 : 1'b0; // JAL  and JALR


endmodule

// 6.2 ALU Decoder
module alu_decoder(
    input  [1:0] alu_op,
    input  [2:0] funct3,
    input        op_5,
    input        funct7_5,
    output [3:0] alu_control
);

    assign alu_control = (alu_op == 2'b00) ? 4'b0000 : // ADD (LW/SW)
                     (alu_op == 2'b01) ? 4'b0001 : // SUB (Branch)
                     // R-type and I-type ALU operations (same handling)
                     (alu_op == 2'b10) ? (
                         (funct3 == 3'b000) ? ((op_5 == 1'b1 && funct7_5 == 1'b1) ? 4'b0001 : 4'b0000) : // SUB or ADD/ADDI
                         (funct3 == 3'b111) ? 4'b0010 : // AND/ANDI
                         (funct3 == 3'b110) ? 4'b0011 : // OR/ORI
                         (funct3 == 3'b100) ? 4'b0100 : // XOR/XORI
                         (funct3 == 3'b010) ? 4'b0101 : // SLT/SLTI
                         (funct3 == 3'b011) ? 4'b0110 : // SLTU/SLTIU
                         (funct3 == 3'b001) ? 4'b0111 : // SLL/SLLI
                         (funct3 == 3'b101 && funct7_5 == 1'b0) ? 4'b1000 : // SRL/SRLI
                         (funct3 == 3'b101 && funct7_5 == 1'b1) ? 4'b1001 : // SRA/SRAI
                         4'b0000
                     ) : 4'b0000; // Default


endmodule

// 7. Register File (RF)
module register_file (
    input        clk,
    input        rst,
    input        reg_write_enable,
    input  [4:0] reg_addr1,
    input  [4:0] reg_addr2,
    input  [4:0] reg_write_addr,
    input  [31:0] write_data,    
    output [31:0] read_data1,
    output [31:0] read_data2
);
    reg [31:0] registers [31:0]; 
    integer i;

    // Read ports (combinational)
    assign read_data1 = (rst == 1'b1) ? 32'd0 : registers[reg_addr1];
    assign read_data2 = (rst == 1'b1) ? 32'd0 : registers[reg_addr2];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) begin  
                registers[i] <= 32'd0;
            end
        end
    end

    // Write port (sequential)
    always @(negedge clk ) begin
        if (reg_write_enable && reg_write_addr != 5'b00000) begin
            registers[reg_write_addr] <= write_data; 
        end
    end


endmodule

// 8. Immediate Generator (IG)
module sign_extension (
    input [31:0] imm_ext_input,
    input [2:0] imm_select, // 000: I-type, 001: S-type, 010: B-type, 011: J-type, 100: U-type
    output [31:0] imm_ext_output
);

    // For I-type shift instructions
    wire is_shift_imm;
    wire is_slli, is_srli, is_srai;
    
    // SLLI: funct3 = 001, funct7 = 0000000
    assign is_slli = (imm_ext_input[14:12] == 3'b001) && (imm_ext_input[31:25] == 7'b0000000);
    
    // SRLI: funct3 = 101, funct7 = 0000000
    assign is_srli = (imm_ext_input[14:12] == 3'b101) && (imm_ext_input[31:25] == 7'b0000000);
    
    // SRAI: funct3 = 101, funct7 = 0100000
    assign is_srai = (imm_ext_input[14:12] == 3'b101) && (imm_ext_input[30] == 1'b1);

    assign is_shift_imm = (imm_select == 3'b000) && (is_slli || is_srli || is_srai);

    assign imm_ext_output = (imm_select == 3'b000 && is_shift_imm) ? 
                            {27'b0, imm_ext_input[24:20]} : // I-type shift: zero-extend shamt 
                            (imm_select == 3'b000) ? 
                            {{20{imm_ext_input[31]}}, imm_ext_input[31:20]} : // I-type: normal sign-extend
                            (imm_select == 3'b001) ? 
                            {{20{imm_ext_input[31]}}, imm_ext_input[31:25], imm_ext_input[11:7]} : // S-type
                            (imm_select == 3'b010) ?
                            {{19{imm_ext_input[31]}}, imm_ext_input[31], imm_ext_input[7], imm_ext_input[30:25], imm_ext_input[11:8], 1'b0} : // B-type
                            (imm_select == 3'b011) ?
                            {{11{imm_ext_input[31]}}, imm_ext_input[31], imm_ext_input[19:12], imm_ext_input[20], imm_ext_input[30:21], 1'b0} : // J-type
                            (imm_select == 3'b100) ?
                            {imm_ext_input[31:12], 12'b0} : // U-type
                            32'b0; // Default case


endmodule

// 9. ALU
module alu (
    input  [31:0] a, b,
    input  [3:0]  alu_control,
    output [31:0] result,
    output  flag_zero,
    output  flag_negative,    
    output  flag_carry,      
    output  flag_overflow    
);

    wire [31:0] sum;
    wire cout;
    wire slt, sltu;
    wire [31:0] mux1;
    wire carry, overflow, zero, negative;
    wire [31:0] shift_left, shift_right_logical, shift_right_arithmetic;
    wire [31:0] xor_result;
    
    assign mux1 = (alu_control[0] == 1'b0) ? b : ~b; // Mux for ADD/SUB

    assign {cout,sum} = a + mux1 + alu_control[0]; // Sum for ADD/SUB

    assign slt = sum[31] ^ overflow; // Set Less Than for SLT
    assign sltu = ~cout; // Set Less Than Unsigned

    // Bitwise operations
    assign xor_result = a ^ b;

    // Shift operations
    assign shift_left = a << b[4:0]; // SLL - shift left logical
    assign shift_right_logical = a >> b[4:0]; // SRL - shift right logical
    assign shift_right_arithmetic = $signed(a) >>> b[4:0]; // SRA - shift right arithmetic

    // Result selection based on alu_control
    assign result = (alu_control == 4'b0000) ? sum :                      // ADD
                    (alu_control == 4'b0001) ? sum :                      // SUB
                    (alu_control == 4'b0010) ? (a & b) :                  // AND
                    (alu_control == 4'b0011) ? (a | b) :                  // OR
                    (alu_control == 4'b0100) ? xor_result :               // XOR
                    (alu_control == 4'b0101) ? {{31'd0}, slt} :           // SLT
                    (alu_control == 4'b0110) ? {{31'd0}, sltu} :          // SLTU
                    (alu_control == 4'b0111) ? shift_left :               // SLL
                    (alu_control == 4'b1000) ? shift_right_logical :      // SRL
                    (alu_control == 4'b1001) ? shift_right_arithmetic :   // SRA
                    32'd0; // Default

    // Flags calculation
    assign carry     = (~alu_control[1]) & cout;  
    assign overflow  = (~alu_control[1]) & (a[31] ^ sum[31]) & (~(a[31] ^ b[31] ^ alu_control[0]));
    assign zero      = &(~result);
    assign negative  = result[31];

    assign flag_zero = zero;
    assign flag_negative = negative;     
    assign flag_carry = carry;           
    assign flag_overflow = overflow;     

endmodule


// 10. Data Memory (DM)
module data_memory (
    input        clk,
    input        rst,
    input        data_mem_write_enable,
    input  [31:0] data_mem_address,
    input  [31:0] data_mem_write_data,
    input  [2:0]  funct3, 

    output [31:0] data_mem_read_data
);
    reg [31:0] data_memory [1023:0]; // 1024 words of 32-bit memory
    integer i;

    wire [1:0] byte_offset;
    wire [31:0] word_address;
    wire [31:0] word_data;
    
    assign byte_offset = data_mem_address[1:0];
    assign word_address = data_mem_address[31:2];
    assign word_data = data_memory[word_address];

    // Reset Logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 1024; i = i + 1) begin
                data_memory[i] <= 32'd0;
            end
        end
    end
    
    // Write port - supports byte, half-word, and word writes
    always @(posedge clk) begin
        if (data_mem_write_enable && !rst) begin
            case (funct3)
                3'b000: begin // SB - Store Byte
                    case (byte_offset)
                        2'b00: data_memory[word_address][7:0]   <= data_mem_write_data[7:0];
                        2'b01: data_memory[word_address][15:8]  <= data_mem_write_data[7:0];
                        2'b10: data_memory[word_address][23:16] <= data_mem_write_data[7:0];
                        2'b11: data_memory[word_address][31:24] <= data_mem_write_data[7:0];
                    endcase
                end
                3'b001: begin // SH - Store Half-word
                    case (byte_offset[1])
                        1'b0: data_memory[word_address][15:0]  <= data_mem_write_data[15:0];
                        1'b1: data_memory[word_address][31:16] <= data_mem_write_data[15:0];
                    endcase
                end
                3'b010: begin // SW - Store Word
                    data_memory[word_address] <= data_mem_write_data;
                end
                default: begin
                    data_memory[word_address] <= data_mem_write_data;
                end
            endcase
        end
    end

    // Read port - byte, half-word, and word 
    reg [31:0] read_data_temp;
    
    always @(*) begin
        if (rst) begin
            read_data_temp = 32'd0;
        end else begin
            case (funct3)
                3'b000: begin // LB - Load Byte 
                    case (byte_offset)
                        2'b00: read_data_temp = {{24{word_data[7]}},  word_data[7:0]};
                        2'b01: read_data_temp = {{24{word_data[15]}}, word_data[15:8]};
                        2'b10: read_data_temp = {{24{word_data[23]}}, word_data[23:16]};
                        2'b11: read_data_temp = {{24{word_data[31]}}, word_data[31:24]};
                    endcase
                end
                3'b001: begin // LH - Load Half-word 
                    case (byte_offset[1])
                        1'b0: read_data_temp = {{16{word_data[15]}}, word_data[15:0]};
                        1'b1: read_data_temp = {{16{word_data[31]}}, word_data[31:16]};
                    endcase
                end
                3'b010: begin // LW - Load Word
                    read_data_temp = word_data;
                end
                3'b100: begin // LBU - Load Byte Unsigned (zero-extended)
                    case (byte_offset)
                        2'b00: read_data_temp = {24'b0, word_data[7:0]};
                        2'b01: read_data_temp = {24'b0, word_data[15:8]};
                        2'b10: read_data_temp = {24'b0, word_data[23:16]};
                        2'b11: read_data_temp = {24'b0, word_data[31:24]};
                    endcase
                end
                3'b101: begin // LHU - Load Half-word Unsigned (zero-extended)
                    case (byte_offset[1])
                        1'b0: read_data_temp = {16'b0, word_data[15:0]};
                        1'b1: read_data_temp = {16'b0, word_data[31:16]};
                    endcase
                end
                default: begin
                    read_data_temp = word_data;
                end
            endcase
        end
    end

    assign data_mem_read_data = read_data_temp;

endmodule

// 11. multiplexer 4:1
module mux_4to1 (
    input  [31:0] mux_input_0,
    input  [31:0] mux_input_1,
    input  [31:0] mux_input_2,
    input  [31:0] mux_input_3,
    input  [1:0]  mux_select,
    output [31:0] mux_output
);
    assign mux_output = (mux_select == 2'b00) ? mux_input_0 :
                        (mux_select == 2'b01) ? mux_input_1 :
                        (mux_select == 2'b10) ? mux_input_2 :
                        mux_input_3; // 2'b11
endmodule

// 12. Hazard Detection Unit (HDU)
module hazard(
    input rst, reg_write_mem_hazard, reg_write_wb_hazard,
    input [1:0] result_src_exe_hazard,
    input [4:0] write_reg_addr_exe_hazard,  source1_addr_dec_hazard, source2_addr_dec_hazard,
    input [4:0] write_reg_addr_wb_hazard, write_reg_addr_mem_hazard, source1_addr_hazard, source2_addr_hazard,
    input pc_src_exe_haz,

    output [1:0] forwardA_hazard, forwardB_hazard,
    output stallF, 
    output stallD, 
    output flushE,
    output flushD    
);

    wire lw_stall;

    assign lw_stall = result_src_exe_hazard[0] & 
                    ((write_reg_addr_exe_hazard == source1_addr_dec_hazard) | 
                    (write_reg_addr_exe_hazard == source2_addr_dec_hazard)) &
                    (write_reg_addr_exe_hazard != 5'd0);
    
    // Stall and flush signals
    assign stallF = lw_stall;
    assign stallD = lw_stall;
    assign flushE = lw_stall || pc_src_exe_haz;
    assign flushD = pc_src_exe_haz;

    assign forwardA_hazard = (rst == 1'b1) ? 2'b00 :
                             ((reg_write_mem_hazard == 1'b1) && 
                              (write_reg_addr_mem_hazard != 5'd0) && 
                              (write_reg_addr_mem_hazard == source1_addr_hazard)) ? 2'b10 :
                             ((reg_write_wb_hazard == 1'b1) && 
                              (write_reg_addr_wb_hazard != 5'd0) && 
                              (write_reg_addr_wb_hazard == source1_addr_hazard)) ? 2'b01 :
                             2'b00;

    assign forwardB_hazard = (rst == 1'b1) ? 2'b00 :
                             ((reg_write_mem_hazard == 1'b1) && 
                              (write_reg_addr_mem_hazard != 5'd0) && 
                              (write_reg_addr_mem_hazard == source2_addr_hazard)) ? 2'b10 :
                             ((reg_write_wb_hazard == 1'b1) && 
                              (write_reg_addr_wb_hazard != 5'd0) && 
                              (write_reg_addr_wb_hazard == source2_addr_hazard)) ? 2'b01 :
                             2'b00;


endmodule
