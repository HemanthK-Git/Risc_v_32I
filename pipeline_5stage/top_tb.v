`timescale 1ns/1ps

module riscv_5stage_tb();
    reg clk, rst; 
    
    riscv_5stage dut (
        .clk(clk), 
        .rst(rst)
);

    // Clock generation
    always begin
        #50 clk = ~clk; // Toggle clock every 5 time units
    end  


    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;

        // Release reset after some time
        #200;
        rst = 0;

        // Run the simulation for a certain period
        #1500;    

        // Finish the simulation
        $finish;
    end

       
endmodule
