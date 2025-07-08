`timescale 1ns / 1ps

module pc(
    input clk,rst,
    input [31:0] nxt_pc,
    output reg [31:0] pc_out,
    output     [31:0] pc_plus4
    );
    
    assign pc_plus4 = pc_out + 32'd4;
    
    always@(posedge clk or posedge rst)
        if(rst)
            pc_out <= 32'd0;
        else
            begin
                pc_out <= nxt_pc;         
            end

endmodule
