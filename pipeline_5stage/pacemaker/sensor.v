// Complete Hardware Integration: RISC-V with Pacemaker Control Logic
// No firmware needed - all control logic is in hardware

module riscv_pacemaker_system (
    input wire clk,
    input wire rst,
    
    // Sensing Block Interface (from heart electrodes)
    input wire sense_a,                // Atrium sense detected
    input wire sense_v,                // Ventricle sense detected  
    input wire sense_lv,               // Left Ventricle sense detected
    
    // Sensing Block Configuration (to sensing block)
    output reg sense_pol_a,            // Polarity select for Atrium
    output reg sense_pol_v,            // Polarity select for Ventricle
    output reg sense_pol_lv,           // Polarity select for Left Ventricle
    output reg [4:0] sense_a_th,       // Atrium threshold
    output reg [4:0] sense_v_th,       // Ventricle threshold
    output reg [4:0] sense_lv_th,      // Left Ventricle threshold
    output reg [4:0] sense_er_th,      // Event Recognition threshold
    
    // Pacing outputs (to Pacing Block)
    output reg pace_a,                 // Atrium pacing pulse
    output reg pace_rv,                // Right Ventricle pacing pulse
    output reg pace_lv                 // Left Ventricle pacing pulse
);

    // ========== PACEMAKER TIMING PARAMETERS ==========
    // Assuming 100MHz clock (10ns per cycle)
    parameter LRL_INTERVAL = 100_000_000;  // 1000ms = 60 bpm (Lower Rate Limit)
    parameter AV_DELAY     = 15_000_000;   // 150ms (AV delay)
    parameter PVARP        = 25_000_000;   // 250ms (Post Ventricular Atrial Refractory)
    parameter VRP          = 32_000_000;   // 320ms (Ventricular Refractory Period)
    parameter PULSE_WIDTH  = 100_000;      // 1ms pacing pulse width

    // ========== PACEMAKER STATE MACHINE ==========
    localparam IDLE            = 3'd0;
    localparam WAIT_LRL        = 3'd1;
    localparam PACE_ATRIUM     = 3'd2;
    localparam AV_DELAY_STATE  = 3'd3;
    localparam PACE_VENTRICLE  = 3'd4;
    localparam REFRACTORY      = 3'd5;

    reg [2:0] state, next_state;
    
    // ========== TIMING COUNTERS ==========
    reg [31:0] lrl_counter;      // Lower Rate Limit counter
    reg [31:0] av_counter;       // AV delay counter
    reg [31:0] pvarp_counter;    // PVARP counter
    reg [31:0] vrp_counter;      // VRP counter
    reg [31:0] pulse_counter;    // Pacing pulse duration counter
    
    // ========== STATUS FLAGS ==========
    reg in_av_delay;
    reg in_pvarp;
    reg in_vrp;
    reg lrl_expired;
    reg av_expired;
    reg pvarp_expired;
    reg vrp_expired;
    
    // ========== SENSE EDGE DETECTION ==========
    reg sense_a_prev, sense_v_prev;
    wire sense_a_edge, sense_v_edge;
    
    assign sense_a_edge = sense_a & ~sense_a_prev;  // Rising edge detection
    assign sense_v_edge = sense_v & ~sense_v_prev;
    
    // ========== INITIALIZATION ==========
    initial begin
        sense_pol_a = 0;
        sense_pol_v = 0;
        sense_pol_lv = 0;
        sense_a_th = 5'd15;
        sense_v_th = 5'd15;
        sense_lv_th = 5'd15;
        sense_er_th = 5'd10;
    end
    
    // ========== MAIN STATE MACHINE ==========
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            lrl_counter <= 0;
            av_counter <= 0;
            pvarp_counter <= 0;
            vrp_counter <= 0;
            pulse_counter <= 0;
            in_av_delay <= 0;
            in_pvarp <= 0;
            in_vrp <= 0;
            pace_a <= 0;
            pace_rv <= 0;
            pace_lv <= 0;
            sense_a_prev <= 0;
            sense_v_prev <= 0;
        end else begin
            state <= next_state;
            
            // Update edge detection registers
            sense_a_prev <= sense_a;
            sense_v_prev <= sense_v;
            
            // ========== TIMER MANAGEMENT ==========
            
            // LRL Timer (always counting unless reset)
            if (sense_a_edge || sense_v_edge || state == PACE_ATRIUM || state == PACE_VENTRICLE) begin
                lrl_counter <= 0;
            end else if (lrl_counter < LRL_INTERVAL) begin
                lrl_counter <= lrl_counter + 1;
            end
            
            // AV Delay Timer
            if (in_av_delay) begin
                if (av_counter < AV_DELAY) begin
                    av_counter <= av_counter + 1;
                end
            end else begin
                av_counter <= 0;
            end
            
            // PVARP Timer
            if (in_pvarp) begin
                if (pvarp_counter < PVARP) begin
                    pvarp_counter <= pvarp_counter + 1;
                end else begin
                    in_pvarp <= 0;
                end
            end else begin
                pvarp_counter <= 0;
            end
            
            // VRP Timer
            if (in_vrp) begin
                if (vrp_counter < VRP) begin
                    vrp_counter <= vrp_counter + 1;
                end else begin
                    in_vrp <= 0;
                end
            end else begin
                vrp_counter <= 0;
            end
            
            // ========== STATE-SPECIFIC ACTIONS ==========
            case (state)
                IDLE: begin
                    pace_a <= 0;
                    pace_rv <= 0;
                    pace_lv <= 0;
                end
                
                WAIT_LRL: begin
                    // Check for natural atrial sense (not in PVARP)
                    if (sense_a_edge && !in_pvarp) begin
                        // Natural P-wave detected
                        in_av_delay <= 1;
                        av_counter <= 0;
                    end
                    
                    // Check for natural ventricular sense (not in VRP)
                    if (sense_v_edge && !in_vrp) begin
                        // Natural R-wave detected
                        in_av_delay <= 0;
                        in_pvarp <= 1;
                        in_vrp <= 1;
                        pvarp_counter <= 0;
                        vrp_counter <= 0;
                    end
                end
                
                PACE_ATRIUM: begin
                    if (pulse_counter < PULSE_WIDTH) begin
                        pace_a <= 1;
                        pulse_counter <= pulse_counter + 1;
                    end else begin
                        pace_a <= 0;
                        pulse_counter <= 0;
                        in_av_delay <= 1;
                        av_counter <= 0;
                    end
                end
                
                AV_DELAY_STATE: begin
                    // Check for natural ventricular sense during AV delay
                    if (sense_v_edge && !in_vrp) begin
                        in_av_delay <= 0;
                        in_pvarp <= 1;
                        in_vrp <= 1;
                    end
                end
                
                PACE_VENTRICLE: begin
                    if (pulse_counter < PULSE_WIDTH) begin
                        pace_rv <= 1;
                        pulse_counter <= pulse_counter + 1;
                    end else begin
                        pace_rv <= 0;
                        pulse_counter <= 0;
                        in_av_delay <= 0;
                        in_pvarp <= 1;
                        in_vrp <= 1;
                        pvarp_counter <= 0;
                        vrp_counter <= 0;
                    end
                end
                
                REFRACTORY: begin
                    pace_a <= 0;
                    pace_rv <= 0;
                end
            endcase
        end
    end
    
    // ========== NEXT STATE LOGIC ==========
    always @(*) begin
        // Default
        next_state = state;
        lrl_expired = (lrl_counter >= LRL_INTERVAL);
        av_expired = (av_counter >= AV_DELAY);
        
        case (state)
            IDLE: begin
                next_state = WAIT_LRL;
            end
            
            WAIT_LRL: begin
                // If LRL expired and not in PVARP, pace atrium
                if (lrl_expired && !in_pvarp) begin
                    next_state = PACE_ATRIUM;
                end
                // If in AV delay and AV expired, pace ventricle
                else if (in_av_delay && av_expired && !in_vrp) begin
                    next_state = PACE_VENTRICLE;
                end
                // If in AV delay but not expired, go to AV_DELAY_STATE
                else if (in_av_delay) begin
                    next_state = AV_DELAY_STATE;
                end
            end
            
            PACE_ATRIUM: begin
                if (pulse_counter >= PULSE_WIDTH) begin
                    next_state = AV_DELAY_STATE;
                end
            end
            
            AV_DELAY_STATE: begin
                // If ventricular sense detected during AV delay
                if (sense_v_edge && !in_vrp) begin
                    next_state = REFRACTORY;
                end
                // If AV delay expired without ventricular sense
                else if (av_expired) begin
                    next_state = PACE_VENTRICLE;
                end
            end
            
            PACE_VENTRICLE: begin
                if (pulse_counter >= PULSE_WIDTH) begin
                    next_state = REFRACTORY;
                end
            end
            
            REFRACTORY: begin
                // Stay in refractory until PVARP and VRP expire
                if (!in_pvarp && !in_vrp) begin
                    next_state = WAIT_LRL;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

endmodule


// ========== TESTBENCH ==========
module tb_riscv_pacemaker;
    reg clk, rst;
    reg sense_a, sense_v, sense_lv;
    wire sense_pol_a, sense_pol_v, sense_pol_lv;
    wire [4:0] sense_a_th, sense_v_th, sense_lv_th, sense_er_th;
    wire pace_a, pace_rv, pace_lv;
    
    // Instantiate DUT
    riscv_pacemaker_system dut (
        .clk(clk),
        .rst(rst),
        .sense_a(sense_a),
        .sense_v(sense_v),
        .sense_lv(sense_lv),
        .sense_pol_a(sense_pol_a),
        .sense_pol_v(sense_pol_v),
        .sense_pol_lv(sense_pol_lv),
        .sense_a_th(sense_a_th),
        .sense_v_th(sense_v_th),
        .sense_lv_th(sense_lv_th),
        .sense_er_th(sense_er_th),
        .pace_a(pace_a),
        .pace_rv(pace_rv),
        .pace_lv(pace_lv)
    );
    
    // Clock generation (100MHz = 10ns period)
    always #5 clk = ~clk;
    
    initial begin
        clk = 0;
        rst = 1;
        sense_a = 0;
        sense_v = 0;
        sense_lv = 0;
        
        #100 rst = 0;
        
        // Test case 1: No heart activity - pacemaker should pace
        #1100000000;  // Wait for LRL to expire
        
        // Test case 2: Simulate natural P-wave
        #100000;
        sense_a = 1;
        #1000;
        sense_a = 0;
        
        // Test case 3: No R-wave after P-wave - should pace ventricle
        #200000000;
        
        // Test case 4: Natural P-wave followed by natural R-wave
        #800000000;
        sense_a = 1;
        #1000;
        sense_a = 0;
        #100000000;  // After 100ms
        sense_v = 1;
        #1000;
        sense_v = 0;
        
        #2000000000;
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("Time=%0t | State=%0d | Sense_A=%b Sense_V=%b | Pace_A=%b Pace_RV=%b | LRL_cnt=%0d AV_cnt=%0d", 
                 $time, dut.state, sense_a, sense_v, pace_a, pace_rv, 
                 dut.lrl_counter, dut.av_counter);
    end
    
    initial begin
        $dumpfile("pacemaker.vcd");
        $dumpvars(0, tb_riscv_pacemaker);
    end
endmodule
