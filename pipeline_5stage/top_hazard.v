`timescale 1ns / 1ps

module riscv_5stage_hazard (
    input clk, rst
);

wire pc_src_out, reg_write_out, reg_write_signal_out, mem_write_signal_out;
wire jump_signal_out, branch_signal_out, alu_src_signal_out;
wire [1:0] result_src_signal_out;
wire [31:0] pc_target_out, inst_mem_out, pc_out, pc_adder_out, final_result_out;
wire [4:0] write_reg_file_addr_out;
wire [2:0] alu_control_out;
wire [31:0] reg_data_1_out, reg_data_2_out, imm_ext_out, pc_decode_out;
wire [4:0] write_reg_addr_decode_out;
wire [31:0] pc_adder_decode_out;

wire reg_write_exe_out, mem_write_exe_out;
wire [1:0] result_src_exe_out;
wire [31:0] alu_result_exe_out, reg_data_2_out_exe_out, pc_adder_exe_out;
wire [4:0] write_reg_addr_exe_out;

wire reg_write_mem_out;
wire [1:0] result_src_mem_out;
wire [31:0] mem_data_out_mem_out, alu_result_out_mem_out, pc_adder_out_mem_out;
wire reg_write_wb_out;
wire [4:0] write_wb_addr_out;

wire [4:0] source1_addr_decode, source2_addr_decode;
wire [4:0] source1_addr_exe, source2_addr_exe;
wire [1:0] forwardA_exe, forwardB_exe;
wire [31:0] alu_result_exe_haz;
wire reg_write_mem_haz;
wire [4:0] write_reg_addr_mem_haz_wire;
wire reg_write_mem_haz_wire;


// fetch stage 
fetch_cycle fetch (
    .clk(clk),
    .rst(rst),
    .pc_src(pc_src_out),
    .pc_target(pc_target_out),

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
    .pc_adder_decode(pc_adder_decode_out)
);

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

    .reg_write_exe(reg_write_exe_out),
    .mem_write_exe(mem_write_exe_out),
    .pc_src_exe(pc_src_out),
    .result_src_exe(result_src_exe_out),
    .alu_result_exe(alu_result_exe_out),
    .reg_data_2_out_exe(reg_data_2_out_exe_out),
    .pc_adder_exe(pc_adder_exe_out),
    .pc_target_exe(pc_target_out),
    .write_reg_addr_out(write_reg_addr_exe_out),
    .source1_addr_exe(source1_addr_exe),
    .source2_addr_exe(source2_addr_exe)
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

    .reg_write_mem(reg_write_mem_out),
    .result_src_mem(result_src_mem_out),
    .mem_data_out_mem(mem_data_out_mem_out),
    .alu_result_out_mem(alu_result_out_mem_out),
    .pc_adder_out_mem(pc_adder_out_mem_out),
    .write_reg_addr_out_mem(write_reg_file_addr_out),
    .write_reg_addr_out_mem_haz(write_reg_addr_mem_haz_wire),
    .reg_write_mem_out_haz(reg_write_mem_haz),
    .alu_result_out_mem_haz(alu_result_exe_haz)
);

// writeback stage
writeback_cycle writeback (
    .reg_write(reg_write_mem_out),
    .result_src(result_src_mem_out),
    .mem_data_in(mem_data_out_mem_out),
    .alu_result_in(alu_result_out_mem_out),
    .pc_adder_in(pc_adder_out_mem_out), 
    .write_reg_addr_in(write_reg_file_addr_out),

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
    .forwardA_hazard(forwardA_exe),
    .forwardB_hazard(forwardB_exe)

);



endmodule










// 1. Fetch Cycle
module fetch_cycle (
    input clk,pc_src,rst,
    input [31:0] pc_target,
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
    always @(posedge clk ) begin
        if (rst) begin
            pc_reg <= 32'd0;
            inst_mem_reg <= 32'd0;
            pc_adder_reg <= 32'd0;
        end else begin
            pc_reg <= pc_out_wire;
            inst_mem_reg <= inst_mem_out_wire;
            pc_adder_reg <= pc_adder_out_wire;
        end
    end

    assign inst_mem_delay =  inst_mem_reg;
    assign pc_delay =  pc_reg;
    assign pc_adder_delay =  pc_adder_reg;

endmodule


// 2. Decode Cycle
module decode_cycle (
    input clk, rst, reg_write,
    input [31:0] inst_mem_in, pc_in, pc_adder_in, result_in,
    input [4:0] write_reg_in,

    output reg_write_decode, mem_write_decode, jump_decode, branch_decode, alu_src_decode,
    output [1:0] result_src_decode,
    output [2:0] alu_control_decode,
    output [31:0] reg_data_1_out_decode, reg_data_2_out_decode,
    output [31:0] imm_ext_decode,
    output [31:0] pc_decode,
    output [4:0] write_reg_decode, source1_decode, source2_decode,
    output [31:0] pc_adder_decode
);

wire jump_out, branch_out, reg_write_out, mem_write_out, alu_src_out;
wire [1:0] result_src_out;
wire [2:0] alu_control_out;
wire [31:0] reg_data_1_out_wire, reg_data_2_out_wire, imm_ext_wire;
wire [1:0] imm_src_decode;

reg reg_write_reg, mem_write_reg, jump_reg, branch_reg, alu_src_reg;
reg [2:0] alu_control_reg;
reg [1:0] result_src_reg;
reg [31:0] reg_data_1_out_reg, reg_data_2_out_reg, imm_ext_reg;
reg [31:0] pc_reg;
reg [4:0] write_reg_reg, source1_reg, source2_reg;
reg [31:0] pc_adder_reg;

control_unit control (
    .opcode(inst_mem_in[6:0]),
    .funct3(inst_mem_in[14:12]),
    .funct7_5(inst_mem_in[30]),
    .jump(jump_out),
    .branch(branch_out),
    .mem_write(mem_write_out),
    .alu_src(alu_src_out),
    .result_src(result_src_out),
    .reg_write(reg_write_out),
    .imm_src(imm_src_decode),
    .alu_control(alu_control_out)
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
            alu_src_reg <= 1'b0;
            alu_control_reg <= 3'b000;
            result_src_reg <= 2'b00;
            reg_data_1_out_reg <= 32'd0;
            reg_data_2_out_reg <= 32'd0;
            imm_ext_reg <= 32'd0;
            pc_reg <= 32'd0;
            write_reg_reg <= 5'd0;
            pc_adder_reg <= 32'd0;
            source1_reg <= 5'd0;
            source2_reg <= 5'd0;
        end else begin
            reg_write_reg <= reg_write_out;
            result_src_reg <= result_src_out;
            mem_write_reg <= mem_write_out;
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
        end
    end

    assign reg_write_decode = reg_write_reg;
    assign result_src_decode = result_src_reg;
    assign mem_write_decode = mem_write_reg;
    assign jump_decode = jump_reg;  
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


endmodule


// 3. Execute Cycle
module execute_cycle (
    input clk, rst, reg_write, mem_write, jump, branch, alu_src,
    input [1:0] result_src,
    input [2:0] alu_control_exe,
    input [31:0] reg_data_1_in, reg_data_2_in, pc_in, imm_ext_in, pc_adder_in,
    input [4:0] write_reg_addr_in,
    input [4:0] source1_addr_in, source2_addr_in,
    input [1:0] forwardA, forwardB,
    input [31:0] alu_result_mem_in, final_result_wb_in,

    output reg_write_exe, mem_write_exe, pc_src_exe, 
    output [1:0] result_src_exe,
    output [31:0] alu_result_exe, reg_data_2_out_exe, pc_adder_exe, pc_target_exe,
    output [4:0] write_reg_addr_out,
    output [4:0] source1_addr_exe, source2_addr_exe
);

wire [31:0] mux1_out_wire, alu_result_wire, mux1_alu_wire, mux2_out_wire;
wire zero;

reg reg_write_reg, mem_write_reg;
reg [1:0] result_src_reg;
reg [31:0] alu_result_reg, reg_data_2_out_reg;
reg [4:0] write_reg_addr_reg;
reg [31:0] pc_adder_reg;

assign pc_src_exe = (branch && zero) || jump;

mux_2x1 reg_to_alu_mux (
    .mux_input_a(mux2_out_wire),
    .mux_input_b(imm_ext_in),
    .mux_select(alu_src),
    .mux_output(mux1_out_wire)
);

alu alu_unit (
    .a(mux1_alu_wire),
    .b(mux1_out_wire),
    .alu_control(alu_control_exe),
    .result(alu_result_wire),
    .flag_zero(zero)
);

pc_adder pca_exe (
    .pc_adder_in_a(pc_in),
    .pc_adder_in_b(imm_ext_in),
    .pc_adder_out(pc_target_exe)
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
        end else begin
            reg_write_reg <= reg_write;
            mem_write_reg <= mem_write;
            result_src_reg <= result_src;
            alu_result_reg <= alu_result_wire;
            reg_data_2_out_reg <= mux2_out_wire;
            write_reg_addr_reg <= write_reg_addr_in;
            pc_adder_reg <= pc_adder_in;
        end
    end

    assign alu_result_exe = alu_result_reg;
    assign reg_data_2_out_exe = reg_data_2_out_reg;
    assign write_reg_addr_out = write_reg_addr_reg;
    assign pc_adder_exe = pc_adder_reg;
    assign reg_write_exe = reg_write_reg;
    assign mem_write_exe = mem_write_reg;
    assign result_src_exe = result_src_reg;
    assign source1_addr_exe = source1_addr_in;
    assign source2_addr_exe = source2_addr_in;

endmodule

// Memory Cycle
module memory_cycle (
    input clk, rst, reg_write, mem_write,
    input [1:0] result_src,
    input [31:0] alu_result_in, reg_data_2_in, pc_adder_in,
    input [4:0] write_reg_addr_in,

    output reg_write_mem, 
    output [1:0] result_src_mem,
    output [31:0] mem_data_out_mem, alu_result_out_mem, pc_adder_out_mem,
    output [4:0] write_reg_addr_out_mem,
    output [31:0] alu_result_out_mem_haz,
    output [4:0] write_reg_addr_out_mem_haz,
    output reg_write_mem_out_haz
);

wire [31:0] mem_data_out;
reg reg_write_reg;
reg [1:0] result_src_reg;
reg [31:0] alu_result_reg, mem_data_out_reg, pc_adder_reg;
reg [4:0] write_reg_addr_reg;

data_memory data_mem (
    .clk(clk),
    .rst(rst),
    .data_mem_write_enable(mem_write),
    .data_mem_address(alu_result_in),
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
        end else begin
            reg_write_reg <= reg_write;
            result_src_reg <= result_src;
            alu_result_reg <= alu_result_in;
            mem_data_out_reg <= mem_data_out;
            write_reg_addr_reg <= write_reg_addr_in;
            pc_adder_reg <= pc_adder_in;
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

endmodule

// Write Back Cycle
module writeback_cycle (
    input reg_write,
    input [1:0] result_src,
    input [31:0] mem_data_in, alu_result_in, pc_adder_in,
    input [4:0] write_reg_addr_in,

    output [31:0] final_result,
    output reg_write_out,
    output [4:0] write_reg_addr_out
);

mux_4to1 writeback_mux (
    .mux_input_0(alu_result_in),
    .mux_input_1(mem_data_in),
    .mux_input_2(pc_adder_in),
    .mux_input_3(32'd0), // Not used
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
    input  [31:0] pc_in,
    output reg [31:0] pc_out
);
    always @(posedge clk) begin
        if (rst) begin
            pc_out <= 32'b0; // Reset PC to 0
        end else begin
            pc_out <= pc_in; // Update PC with new value
        end
    end

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

    assign instruction_out = (rst == 1'b1) ? 32'b0 : inst_memory[inst_mem_in[31:2]]; // Word-aligned access (pc[31:2] for 1024 words)

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
    input      funct7_5,
    output jump, branch,
    output       mem_write,
    output       alu_src,
    output  [1:0]     result_src,
    output       reg_write,
    output [1:0] imm_src,
    output [2:0] alu_control
);
    wire [1:0] alu_op_wire;
    
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
    output [1:0] imm_src,
    output jump
);

    assign reg_write = (opcode == 7'b0000011 || opcode == 7'b0110011 || opcode == 7'b1101111 || opcode == 7'b0010011) ? 1'b1 : 1'b0; // LW, R-type, I-type, JAL 
    assign imm_src = (opcode == 7'b1100011) ? 2'b10 : // BEQ
                     (opcode == 7'b0100011) ? 2'b01 : // SW
                     (opcode == 7'b1101111) ? 2'b11 : 2'b00; // JAL : LW, I-type & R-type
    assign alu_src   = (opcode == 7'b0100011) || (opcode == 7'b0000011 || opcode == 7'b0010011) ? 1'b1 : 1'b0; // SW, LW and I-type
    assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // SW
    assign result_src = (opcode == 7'b0000011) ? 2'b01 : // LW
                        (opcode == 7'b1101111) ? 2'b10 : 2'b00; // JAL : I-type, R-type, SW, BEQ   
    assign branch   = (opcode == 7'b1100011) ? 1'b1 : 1'b0; // BEQ
    assign alu_op    = (opcode == 7'b0110011 || opcode == 7'b0010011) ? 2'b10 : // R-type and I-type
                      (opcode == 7'b1100011) ? 2'b01 : 2'b00; // R-type : BEQ : LW/SW
    assign jump = (opcode == 7'b1101111) ? 1'b1 : 1'b0; // JAL  

endmodule

// 6.2 ALU Decoder
module alu_decoder(
    input  [1:0] alu_op,
    input  [2:0] funct3,
    input        op_5,
    input        funct7_5,
    output [2:0] alu_control
);
    wire [1:0] concatenation;
    assign concatenation = {op_5, funct7_5};
    assign alu_control = (alu_op == 2'b00) ? 3'b000 : // LW or SW -> ADD
                         (alu_op == 2'b01) ? 3'b001 : // BEQ -> SUB
                         (alu_op == 2'b10 && funct3 == 3'b000 && (concatenation != 2'b11)) ? 3'b000 : // ADD
                         (alu_op == 2'b10 && funct3 == 3'b000 && (concatenation == 2'b11)) ? 3'b001 : // SUB
                         (alu_op == 2'b10 && funct3 == 3'b010) ? 3'b101 : // SLT
                         (alu_op == 2'b10 && funct3 == 3'b110) ? 3'b011 : // OR
                         (alu_op == 2'b10 && funct3 == 3'b111) ? 3'b010 : // AND
                         3'b000; // Undefined

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
    reg [31:0] registers [31:0]; // 32 registers of 32 bits each
    integer i;

    // Read ports (combinational)
    assign read_data1 = (rst == 1'b1) ? 32'b0 : registers[reg_addr1];
    assign read_data2 = (rst == 1'b1) ? 32'b0 : registers[reg_addr2];

    // Write port (sequential)
    always @(negedge clk ) begin
        if (reg_write_enable && reg_write_addr != 5'b00000) begin
            registers[reg_write_addr] <= write_data; // Ensure x0 is always 0
        end
    end

    initial begin
        registers[0] = 32'd0; // x0 = 0
        registers[5] = 32'd10; // x5 = 10
    end

endmodule

// 8. Immediate Generator (IG)
module sign_extension (
    input [31:0] imm_ext_input,
    input [1:0] imm_select, // 0 for I-type, 1 for S-type
    output [31:0] imm_ext_output
);

    assign imm_ext_output = (imm_select == 2'b01) ? 
                            {{20{imm_ext_input[31]}}, imm_ext_input[31:25], imm_ext_input[11:7]} : // S-type
                            (imm_select == 2'b10) ?
                            {{19{imm_ext_input[31]}}, imm_ext_input[31], imm_ext_input[7], imm_ext_input[30:25], imm_ext_input[11:8], 1'b0} : // B-type
                            (imm_select == 2'b11) ?
                            {{11{imm_ext_input[31]}},imm_ext_input[31], imm_ext_input[19:12], imm_ext_input[20], imm_ext_input[30:21], 1'b0} : // J-type
                            (imm_select == 2'b00) ?
                            {{20{imm_ext_input[31]}}, imm_ext_input[31:20]} : // I-type
                            32'b0; // Default case

endmodule

// 9. ALU
module alu (
    input  [31:0] a, b,
    input  [2:0]  alu_control,
    output [31:0] result,
    output  flag_zero   // {Negative, Zero, Carry, OverFlow}
);

    wire [31:0] sum;
    wire cout;
    wire slt;
    wire [31:0] mux1;
    wire carry, overflow, zero, negative;
    
    assign mux1 = (alu_control[0] == 1'b0) ? b : ~b; // Mux for ADD/SUB

    assign {cout,sum} = a + mux1 + alu_control[0]; // Sum for ADD/SUB

    assign slt = sum[31] ^ overflow; // Set Less Than for SLT

    // Result selection
    assign result = (alu_control == 3'b000) ? sum :                  // ADD
                    (alu_control == 3'b001) ? sum :                  // SUB
                    (alu_control == 3'b010) ? (a & b) :             // AND logical
                    (alu_control == 3'b011) ? (a | b) :            // OR logical
                    (alu_control == 3'b101) ? {{31'd0}, slt} : 32'd0; // SLT

    // Flags calculation
    assign carry     = (~alu_control[1]) & cout;  
    assign overflow  = (~alu_control[1]) & (a[31] ^ sum[31]) & (~(a[31] ^ b[31] ^ alu_control[0]));
    assign zero      = &(~result);
    assign negative  = result[31];

    assign flag_zero = zero;

endmodule


// 10. Data Memory (DM)
module data_memory (
    input        clk,
    input        rst,
    input        data_mem_write_enable,
    input  [31:0] data_mem_address,
    input  [31:0] data_mem_write_data,
    output [31:0] data_mem_read_data
);
    reg [31:0] data_memory [1023:0]; // 1024 words of 32-bit memory

    // Read port (combinational)
    assign data_mem_read_data = (rst == 1'b1) ? 32'b0 : data_memory[data_mem_address[31:2]]; // Word-aligned access (address[31:2] for 1024 words)

    // Write port (sequential)
    always @(posedge clk) begin
        if (data_mem_write_enable && !rst) begin
            data_memory[data_mem_address[31:2]] <= data_mem_write_data; // Word-aligned access
        end
    end 
    
    initial begin
        data_memory [0] = 32'd0;
        data_memory [40] = 32'h00000002;
    end

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
    input [4:0] write_reg_addr_wb_hazard, write_reg_addr_mem_hazard, source1_addr_hazard, source2_addr_hazard,

    output [1:0] forwardA_hazard, forwardB_hazard
);
    assign forwardA_hazard = (rst == 1'b1) ? 2'b00 :
                             ((reg_write_mem_hazard == 1'b1) && (write_reg_addr_mem_hazard != 5'd0) && (write_reg_addr_mem_hazard == source1_addr_hazard)) ? 2'b10 :
                             ((reg_write_wb_hazard == 1'b1) && (write_reg_addr_wb_hazard != 5'd0) && (write_reg_addr_wb_hazard == source1_addr_hazard)) ? 2'b01 :
                             2'b00;

    assign forwardB_hazard = (rst == 1'b1) ? 2'b00 :
                              ((reg_write_mem_hazard == 1'b1) && (write_reg_addr_mem_hazard != 5'd0) && (write_reg_addr_mem_hazard == source2_addr_hazard)) ? 2'b10 :
                              ((reg_write_wb_hazard == 1'b1) && (write_reg_addr_wb_hazard != 5'd0) && (write_reg_addr_wb_hazard == source2_addr_hazard)) ? 2'b01 :
                              2'b00;

endmodule



