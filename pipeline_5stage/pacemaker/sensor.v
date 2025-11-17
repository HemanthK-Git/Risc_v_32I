// Top-level module: RISC-V integrated with Pacemaker Sensing Block
module riscv_pacemaker_system (
    input wire clk,
    input wire rst,
    
    // Sensing Block Interface
    output reg sense_pol_a,           // Polarity select for Atrium
    output reg sense_pol_v,           // Polarity select for Ventricle
    output reg sense_pol_lv,          // Polarity select for Left Ventricle
    output reg [4:0] sense_a_th,      // Atrium threshold
    output reg [4:0] sense_v_th,      // Ventricle threshold
    output reg [4:0] sense_lv_th,     // Left Ventricle threshold
    output reg [4:0] sense_er_th,     // Event Recognition threshold
    
    input wire sense_a,                // Atrium sense detected (from Sensing Block)
    input wire sense_v,                // Ventricle sense detected
    input wire sense_lv,               // Left Ventricle sense detected
    
    // Pacing outputs (will connect to Pacing Block later)
    output reg pace_a,                 // Atrium pacing pulse
    output reg pace_rv,                // Right Ventricle pacing pulse
    output reg pace_lv                 // Left Ventricle pacing pulse
);

    // Memory-mapped register addresses
    localparam ADDR_SENSE_CONTROL  = 32'h4000_0000;  // Control register
    localparam ADDR_SENSE_STATUS   = 32'h4000_0004;  // Status register (read sensing)
    localparam ADDR_SENSE_TH_A     = 32'h4000_0008;  // Atrium threshold
    localparam ADDR_SENSE_TH_V     = 32'h4000_000C;  // Ventricle threshold
    localparam ADDR_SENSE_TH_LV    = 32'h4000_0010;  // Left Ventricle threshold
    localparam ADDR_SENSE_TH_ER    = 32'h4000_0014;  // ER threshold
    localparam ADDR_PACE_CONTROL   = 32'h4000_0018;  // Pacing control
    localparam ADDR_TIMER_LRL      = 32'h4000_001C;  // Lower Rate Limit timer
    localparam ADDR_TIMER_AV       = 32'h4000_0020;  // AV delay timer
    localparam ADDR_TIMER_VA       = 32'h4000_0024;  // VA interval timer

    // RISC-V Core signals
    wire [31:0] instr_addr;
    wire [31:0] instr_data;
    wire [31:0] data_addr;
    wire [31:0] data_wdata;
    wire [31:0] data_rdata;
    wire [3:0] data_we;
    wire data_re;
    
    // Timer registers (for pacemaker timing control)
    reg [31:0] timer_lrl;      // Lower Rate Limit (e.g., 1000ms = 60 bpm)
    reg [31:0] timer_av;       // AV delay (120-200ms)
    reg [31:0] timer_va;       // VA interval
    reg [31:0] counter_lrl;    // Current LRL counter
    reg [31:0] counter_av;     // Current AV counter
    
    // Control and status registers
    reg [31:0] sense_control;
    reg [31:0] sense_status;
    reg [31:0] pace_control;
    
    // Instantiate your existing RISC-V 5-stage pipeline
    riscv_core cpu (
        .clk(clk),
        .rst(rst),
        .instr_addr(instr_addr),
        .instr_data(instr_data),
        .data_addr(data_addr),
        .data_wdata(data_wdata),
        .data_rdata(data_rdata),
        .data_we(data_we),
        .data_re(data_re)
    );
    
    // Instruction memory (contains pacemaker control firmware)
    instruction_memory imem (
        .addr(instr_addr),
        .data_out(instr_data)
    );
    
    // Memory-mapped I/O: Write operations
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sense_pol_a <= 0;
            sense_pol_v <= 0;
            sense_pol_lv <= 0;
            sense_a_th <= 5'd15;      // Default threshold
            sense_v_th <= 5'd15;
            sense_lv_th <= 5'd15;
            sense_er_th <= 5'd10;
            sense_control <= 0;
            pace_control <= 0;
            timer_lrl <= 32'd100_000_000;  // 1 second at 100MHz clock
            timer_av <= 32'd15_000_000;    // 150ms AV delay
            timer_va <= 32'd85_000_000;    // 850ms VA interval
            pace_a <= 0;
            pace_rv <= 0;
            pace_lv <= 0;
        end else begin
            // Write to memory-mapped registers
            if (|data_we) begin
                case (data_addr)
                    ADDR_SENSE_CONTROL: begin
                        sense_control <= data_wdata;
                        sense_pol_a <= data_wdata[0];
                        sense_pol_v <= data_wdata[1];
                        sense_pol_lv <= data_wdata[2];
                    end
                    
                    ADDR_SENSE_TH_A:  sense_a_th <= data_wdata[4:0];
                    ADDR_SENSE_TH_V:  sense_v_th <= data_wdata[4:0];
                    ADDR_SENSE_TH_LV: sense_lv_th <= data_wdata[4:0];
                    ADDR_SENSE_TH_ER: sense_er_th <= data_wdata[4:0];
                    
                    ADDR_PACE_CONTROL: begin
                        pace_control <= data_wdata;
                        pace_a <= data_wdata[0];
                        pace_rv <= data_wdata[1];
                        pace_lv <= data_wdata[2];
                    end
                    
                    ADDR_TIMER_LRL: timer_lrl <= data_wdata;
                    ADDR_TIMER_AV:  timer_av <= data_wdata;
                    ADDR_TIMER_VA:  timer_va <= data_wdata;
                endcase
            end
            
            // Clear pacing pulses after 1ms (pulse duration)
            if (pace_a || pace_rv || pace_lv) begin
                pace_a <= 0;
                pace_rv <= 0;
                pace_lv <= 0;
            end
        end
    end
    
    // Memory-mapped I/O: Read operations
    reg [31:0] read_data;
    always @(*) begin
        case (data_addr)
            ADDR_SENSE_STATUS:  read_data = {29'b0, sense_lv, sense_v, sense_a};
            ADDR_SENSE_CONTROL: read_data = sense_control;
            ADDR_SENSE_TH_A:    read_data = {27'b0, sense_a_th};
            ADDR_SENSE_TH_V:    read_data = {27'b0, sense_v_th};
            ADDR_SENSE_TH_LV:   read_data = {27'b0, sense_lv_th};
            ADDR_SENSE_TH_ER:   read_data = {27'b0, sense_er_th};
            ADDR_PACE_CONTROL:  read_data = pace_control;
            ADDR_TIMER_LRL:     read_data = timer_lrl;
            ADDR_TIMER_AV:      read_data = timer_av;
            ADDR_TIMER_VA:      read_data = timer_va;
            default:            read_data = 32'h0;
        endcase
    end
    
    assign data_rdata = read_data;

endmodule

// Simple instruction memory module
module instruction_memory (
    input wire [31:0] addr,
    output reg [31:0] data_out
);
    // This would contain your pacemaker control program
    // compiled from C code (shown below in comments)
    reg [31:0] mem [0:1023];
    
    initial begin
        // Load your compiled RISC-V instructions here
        $readmemh("pacemaker_firmware.hex", mem);
    end
    
    always @(*) begin
        data_out = mem[addr[11:2]];  // Word-aligned access
    end
endmodule

// Placeholder for your existing RISC-V core
module riscv_core (
    input wire clk,
    input wire rst,
    output wire [31:0] instr_addr,
    input wire [31:0] instr_data,
    output wire [31:0] data_addr,
    output wire [31:0] data_wdata,
    input wire [31:0] data_rdata,
    output wire [3:0] data_we,
    output wire data_re
);
    // Your existing 5-stage pipeline implementation goes here
    // This should include:
    // - Fetch stage
    // - Decode stage
    // - Execute stage
    // - Memory stage
    // - Writeback stage
    // - Hazard detection unit
    // - Forwarding unit
endmodule
