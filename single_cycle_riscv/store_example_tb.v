`timescale 1ns/1ps

module single_cycle_riscv_tb;
    reg clk;
    reg rst;

    single_cycle_riscv uut (
        .clk(clk),
        .rst(rst)
    );

    initial begin
        $dumpfile("single_cycle_riscv.vcd");
        $dumpvars(0);
    end


    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;

        // Release reset after some time
        #100;
        rst = 0;

        // Run the simulation for a certain period
        #300;
    

        // Finish the simulation
        $finish;
    end

    // Clock generation
    always begin
        #50 clk = ~clk; // Toggle clock every 5 time units
    end     
endmodule
