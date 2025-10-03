


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
