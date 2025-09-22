`timescale 1ns/1ps

module single_cycle_riscv (
    input clk,rst
);

wire [31:0] pc_out_wire, inst_mem_out_wire, reg_file_data_out1_wire, reg_file_data_out2_wire;
wire [31:0] imm_ext_out_wire, alu_output_wire, data_mem_read_data_out_wire, pc_adder_out_wire;
wire [2:0] alu_control_wire;
wire control_reg_write_wire;

program_counter pc (
    .clk(clk),
    .rst(rst),
    .pc_in(pc_adder_out_wire),
    .pc_out(pc_out_wire)
);

instruction_memory im (
    .inst_mem_in(pc_out_wire),
    .rst(rst),
    .instruction_out(inst_mem_out_wire)
);

register_file rf (
    .clk(clk),
    .rst(rst),
    .reg_write_enable(control_reg_write_wire),
    .reg_addr1(inst_mem_out_wire[19:15]),
    .reg_addr2(),
    .reg_write_addr(inst_mem_out_wire[11:7]),
    .write_data(data_mem_read_data_out_wire),    
    .read_data1(reg_file_data_out1_wire),
    .read_data2()
);

sign_extension imm (
    .imm_ext_input(inst_mem_out_wire),
    .imm_ext_output(imm_ext_out_wire)
);

alu alu_unit (
    .a(reg_file_data_out1_wire),
    .b(imm_ext_out_wire),
    .alu_control(alu_control_wire),
    .result(alu_output_wire),
    .flag_zero()
);

control_unit cu (
    .opcode(inst_mem_out_wire[6:0]),
    .funct3(inst_mem_out_wire[14:12]),
    .funct7_5(inst_mem_out_wire[30]),
    .mem_write(),
    .branch(),
    .alu_src(),
    .result_src(),
    .reg_write(control_reg_write_wire),
    .alu_op(),
    .imm_src(),
    .alu_control(alu_control_wire)
);

data_memory dm (
    .clk(clk),
    .rst(rst),
    .data_mem_write_enable(),
    .data_mem_address(alu_output_wire),
    .data_mem_write_data(),
    .data_mem_read_data(data_mem_read_data_out_wire)
);

pc_adder pca (
    .pc_adder_in_a(pc_out_wire),
    .pc_adder_in_b(32'd4),
    .pc_adder_out(pc_adder_out_wire)
);

endmodule

////////////////////////////////////////////////////////////////////////////////////////

// 1. Program Counter (PC)
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

// 2. Instruction Memory (IM)
module instruction_memory (
    input  [31:0] inst_mem_in,
    input rst,
    output [31:0] instruction_out
);
    reg [31:0] inst_memory [1023:0]; // 1024 words of 32-bit memory

    initial begin
        // Load instructions from a file (for simulation purposes)
        inst_memory[0] = 32'hFFC4A303; // addi x1, x0, 1
        inst_memory[1] = 32'h00832383;
    end

    assign instruction_out = (rst == 1'b1) ? 32'b0 : inst_memory[inst_mem_in[31:2]]; // Word-aligned access (pc[31:2] for 1024 words)

endmodule

// 3. Register File (RF)
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
    always @(posedge clk ) begin
        if (reg_write_enable && reg_write_addr != 5'b00000) begin
            registers[reg_write_addr] <= write_data; // Ensure x0 is always 0
        end
    end

    initial begin
        registers[9] = 32'd20; // x9 = 20
    end

endmodule

// 4. Sign Extension Unit (SEU)
module sign_extension (
    input [31:0] imm_ext_input,
    output [31:0] imm_ext_output
);

    assign imm_ext_output = (imm_ext_input[31] == 1'b1) ? {20'b11111111111111111111, imm_ext_input[31:20]} : 
                                                          {20'b00000000000000000000, imm_ext_input[31:20]};


endmodule

// 5. Arithemtic Logic Unit (ALU)
module alu (
    input  [31:0] a, b,
    input  [2:0]  alu_control,
    output [31:0] result,
    output [3:0]  flag_zero   // {Negative, Zero, Carry, OverFlow}
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

    // Packed Flags: {Negative, Zero, Carry, OverFlow}
    assign flag_zero = zero;

endmodule

// 6. Control Unit (CU)
module control_unit (
    input  [6:0] opcode,
    input [2:0] funct3,
    input      funct7_5,
    output       mem_write,
    output       branch,
    output       alu_src,
    output       result_src,
    output       reg_write,
    output [1:0] alu_op,
    output [1:0] imm_src,
    output [2:0] alu_control
);
    wire [1:0] alu_op_wire;

    main_decoder md (
        .opcode(opcode),
        .mem_write(),
        .branch(branch),
        .alu_src(alu_src),
        .result_src(result_src),
        .reg_write( reg_write ),
        .alu_op(alu_op_wire),
        .imm_src(imm_src)
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
    output       result_src,
    output       reg_write,
    output [1:0] alu_op,
    output [1:0] imm_src
);

    assign reg_write = (opcode == 7'b0000011 || opcode == 7'b0110011) ? 1'b1 : 1'b0; // LW and R-type
    assign imm_src = (opcode == 7'b1100011) ? 2'b10 : 
                     (opcode == 7'b0100011) ? 2'b01 : 2'b00; // LW : SW : BEQ
    assign alu_src   = (opcode == 7'b0100011) || (opcode == 7'b0000011) ? 1'b1 : 1'b0; // SW and LW
    assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // SW
    assign result_src = (opcode == 7'b0000011) ? 1'b1 : 1'b0; // LW 
    assign branch   = (opcode == 7'b1100011) ? 1'b1 : 1'b0; // BEQ
    assign alu_op    = (opcode == 7'b0110011) ? 2'b10 :
                      (opcode == 7'b1100011) ? 2'b01 : 2'b00; // R-type : BEQ : LW/SW

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

// 7. Data Memory (DM)
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
    assign data_mem_read_data = (rst == 1'b1) ? 32'b0 : data_memory[data_mem_address[31:0]]; // Word-aligned access (address[31:2] for 1024 words)

    // Write port (sequential)
    always @(posedge clk) begin
        if (data_mem_write_enable && !rst) begin
            data_memory[data_mem_address[31:2]] <= data_mem_write_data; // Word-aligned access
        end
    end 
    
    initial begin
        data_memory [16] = 32'h00000020;
        data_memory [40] = 32'h00000002;
    end

endmodule

// 8. Program Counter Adder (PCA)
module pc_adder (
    input  [31:0] pc_adder_in_a,
    input [31:0] pc_adder_in_b,
    output [31:0] pc_adder_out
);
    assign pc_adder_out = pc_adder_in_a + pc_adder_in_b;

endmodule







