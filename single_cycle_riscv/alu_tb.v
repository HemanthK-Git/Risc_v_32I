`timescale 1ns/1ps

module alu_tb;
    reg [31:0] a, b;
    reg [2:0] alu_control;
    wire [31:0] result;
    wire [3:0] flags;
    
    // Instantiate the ALU
    alu uut (
        .a(a),
        .b(b),
        .alu_control(alu_control),
        .result(result),
        .flags(flags)
    );
    
    initial begin
        // Initialize inputs
        a = 0;
        b = 0;
        alu_control = 0;
        
        // Monitor changes
        $monitor("Time=%0t, a=%0d, b=%0d, alu_control=%b, result=%0d, flags=%b", 
                 $time, a, b, alu_control, result, flags);
        
        // Test ADD operation
        #10;
        a = 32'd10;
        b = 32'd20;
        alu_control = 3'b000; // ADD
        #10;
        
        // Test SUB operation
        a = 32'd30;
        b = 32'd10;
        alu_control = 3'b001; // SUB
        #10;
        
        // Test AND operation
        a = 32'hFFFF;
        b = 32'hF0F0;
        alu_control = 3'b010; // AND
        #10;
        
        // Test OR operation
        a = 32'h00FF;
        b = 32'h0F0F;
        alu_control = 3'b011; // OR
        #10;
        
        // Test SLT (Set Less Than) operation - a < b
        a = 32'd5;
        b = 32'd10;
        alu_control = 3'b101; // SLT
        #10;
        
        // Test SLT (Set Less Than) operation - a > b
        a = 32'd15;
        b = 32'd10;
        alu_control = 3'b101; // SLT
        #10;
        
        // Test negative numbers
        a = 32'hFFFFFFF6; // -10 in two's complement
        b = 32'd5;
        alu_control = 3'b000; // ADD
        #10;
        
        // Test overflow case
        a = 32'h7FFFFFFF; // Maximum positive number
        b = 32'd1;
        alu_control = 3'b000; // ADD
        #10;
        
        // Test zero flag
        a = 32'd10;
        b = 32'd0;
        alu_control = 3'b010; // AND
        #10;
        
        $display("Test completed");
        $finish;
    end
    
    // Generate waveform file
    initial begin
        $dumpfile("alu_wave.vcd");
        $dumpvars(0, alu_tb);
    end
endmodule
