// alu with overflow 

`timescale 1ns/1ps


module alu (
    input  [31:0] a, b,
    input  [2:0]  alu_control,
    output [31:0] result,
    output [3:0]  flags   // {Negative, Zero, Carry, OverFlow}
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
    assign flags = {  negative, zero,  carry, overflow};

endmodule
