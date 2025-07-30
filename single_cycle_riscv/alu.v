module alu (
    input  [31:0] a, b,
    input  [2:0]  alu_control,
    output [31:0] result,
    output [3:0]  flags   // {Negative, Zero, Carry, OverFlow}
);

    wire [31:0] sum;
    wire cout;
    wire carry, overflow, zero, negative;

    // ADD or SUB
    assign {cout, sum} = (alu_control[0] == 1'b0) ? {1'b0, a} + {1'b0, b} :
                                             {1'b0, a} + {1'b0, ~b + 1'b1};

    // Result selection
    assign result = (alu_control == 3'b000) ? sum :                  // ADD
                    (alu_control == 3'b001) ? sum :                  // SUB
                    (alu_control == 3'b010) ? (a & b) :
                    (alu_control == 3'b011) ? (a | b) :
                    (alu_control == 3'b101) ? {{31{1'b0}}, ($signed(a) < $signed(b))} :
                    32'b0;

    // Flags calculation
    assign carry     = (~alu_control[1]) & cout;  
    assign overflow  = (~alu_control[1]) & ((a[31] == b[31]) && (result[31] != a[31])) & (alu_control[0] == 1'b0);
    assign zero      = ~( |result );
    assign negative  = result[31];

    // Packed Flags: {Negative, Zero, Carry, OverFlow}
    assign flags = {  overflow, carry,negative, zero};

endmodule
