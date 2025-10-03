// addi x5,x0,0x5 
// addi x6, x0,0x3 
// add x7, x5, x6

// lw x8, 0x0(x0)
// addi x9, x0, 0x1
// add x10, x8, x9


`timescale 1ns/1ps

module riscv_5stage_hazard_tb();
    reg rst; 
    reg clk = 1'b0;
    
    riscv_5stage_hazard dut (
        .clk(clk), 
        .rst(rst)
);

// Clock generation - 100ns period
    always begin
        clk = ~clk;
        #50;
    end
    
    // Reset generation
    initial begin
        rst <= 1'b1;      // Reset HIGH (active)
        #100;
        rst <= 1'b0;      // Reset LOW (inactive)
        #1500;
        $finish;    
    end
    
    
    

       
endmodule
