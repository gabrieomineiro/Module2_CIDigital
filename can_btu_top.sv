//-----------------------------------------------------------------------------
// Module: can_btu (Bit Timing Unit)
// Description: Generates timing signals for CAN bit synchronization
//              Implements configurable bit timing per CAN specification
//              Supports hard and soft synchronization with edge detection
// Author: CAN Controller Project
// Version: 2.0
//-----------------------------------------------------------------------------

module can_btu_top #(
    parameter int CLK_FREQ_HZ = 50_000_000,  // System clock frequency
    parameter int BAUD_RATE   = 500_000      // Target CAN baud rate
)(
    input  logic        clk,           // System clock
    input  logic        rst_n,         // Active-low reset
    
    // Configuration inputs
    input  logic [7:0]  prescaler,     // Baud rate prescaler (1-256)
    input  logic [2:0]  prop_seg,      // Propagation segment (1-8 time quanta)
    input  logic [2:0]  phase_seg1,    // Phase segment 1 (1-8 time quanta)
    input  logic [2:0]  phase_seg2,    // Phase segment 2 (2-8 time quanta)
    input  logic [1:0]  sjw,           // Synchronization jump width (1-4 time quanta)
    
    // CAN bus input for edge detection
    input  logic        can_rx,        // CAN receive line
    
    // Synchronization inputs
    input  logic        sync_en,       // Enable synchronization
    input  logic        hard_sync,     // Hard synchronization request
    
    // Timing outputs
    output logic        bit_tick,      // Pulse at each bit time
    output logic        sample_tick,   // Pulse at sample point
    output logic        tx_tick,       // Pulse at transmission point
    output logic        sample_point,  // High during sample phase
    
    // Status outputs
    output logic [7:0]  bit_time_cnt,  // Current bit time counter
    output logic        sync_locked,   // Synchronization locked
    output logic        edge_detected, // Edge detected flag (for monitoring)
    output logic        sync_active    // Synchronization in progress
);

    //-----------------------------------------------------------------------------
    // Internal Signals
    //-----------------------------------------------------------------------------
    
    // Time quantum counter
    logic [7:0] tq_counter;
    logic [4:0] tq_limit;          // Changed to 5 bits to handle max 22 TQ
    
    // Calculated timing values (5 bits to handle max 22 TQ)
    logic [4:0] sample_tq_base;    // Base time quanta count for sample point
    logic [4:0] sample_tq_adj;     // Adjusted sample point for synchronization
    logic [4:0] tx_tq;             // Time quanta count for TX point
    logic [4:0] total_tq_base;     // Base total time quanta per bit
    logic [4:0] total_tq_adj;      // Adjusted total for synchronization
    
    // Prescaler counter
    logic [7:0] presc_counter;
    logic [7:0] prescaler_safe;    // Protected prescaler value
    logic       presc_tick;
    
    // Synchronization state
    typedef enum logic [1:0] {
        SYNC_IDLE,
        SYNC_WAIT_EDGE,
        SYNC_ADJUSTING,
        SYNC_COMPLETE
    } sync_state_t;
    
    sync_state_t sync_state;
    
    // Phase adjustment registers
    logic [3:0] phase_seg1_adj;    // Adjusted phase_seg1 (4 bits for overflow)
    logic [3:0] phase_seg2_adj;    // Adjusted phase_seg2 (4 bits for underflow)
    logic       sync_done;
    logic       phase_adjusted;    // Flag indicating phase was adjusted
    
    // Edge detection
    logic       prev_bus_value;
    logic       edge_detected_int; // Internal edge detection signal
    logic [3:0] phase_error;       // Phase error magnitude
    
    //-----------------------------------------------------------------------------
    // Input Protection
    //-----------------------------------------------------------------------------
    
    // Protect against zero prescaler
    assign prescaler_safe = (prescaler == 8'd0) ? 8'd1 : prescaler;
    
    //-----------------------------------------------------------------------------
    // Calculate Timing Parameters
    //-----------------------------------------------------------------------------
    
    // Total time quanta per bit = 1 (sync) + prop_seg + phase_seg1 + phase_seg2
    // Note: prop_seg, phase_seg1, phase_seg2 are 3-bit but represent 1-8 values
    // Maximum: 1 + 8 + 8 + 8 = 25 TQ (fits in 5 bits)
    assign total_tq_base = 5'd1 + {2'b0, prop_seg} + {2'b0, phase_seg1} + {2'b0, phase_seg2};
    
    // Sample point is after sync + prop_seg + phase_seg1
    assign sample_tq_base = 5'd1 + {2'b0, prop_seg} + {2'b0, phase_seg1};
    
    // TX point is at the end of sync segment (beginning of bit)
    assign tx_tq = 5'd1;
    
    // Adjusted timing values based on synchronization
    assign sample_tq_adj = 5'd1 + {2'b0, prop_seg} + phase_seg1_adj;
    assign total_tq_adj = 5'd1 + {2'b0, prop_seg} + phase_seg1_adj + phase_seg2_adj;
    
    // TQ limit for counter (use adjusted value when synchronizing)
    assign tq_limit = phase_adjusted ? (total_tq_adj - 5'd1) : (total_tq_base - 5'd1);
    
    //-----------------------------------------------------------------------------
    // Edge Detection
    //-----------------------------------------------------------------------------
    
    // Detect falling edge (recessive to dominant) for synchronization
    // CAN bus uses dominant (0) and recessive (1) states
    // A falling edge indicates a potential synchronization point
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_bus_value    <= 1'b1;  // Default to recessive
            edge_detected_int <= 1'b0;
        end else begin
            prev_bus_value    <= can_rx;
            // Detect falling edge (recessive -> dominant)
            edge_detected_int <= prev_bus_value && !can_rx;
        end
    end
    
    // Output edge detection status
    assign edge_detected = edge_detected_int;
    
    //-----------------------------------------------------------------------------
    // Prescaler
    //-----------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            presc_counter <= 8'd0;
            presc_tick    <= 1'b0;
        end else begin
            if (presc_counter >= prescaler_safe - 8'd1) begin
                presc_counter <= 8'd0;
                presc_tick    <= 1'b1;
            end else begin
                presc_counter <= presc_counter + 8'd1;
                presc_tick    <= 1'b0;
            end
        end
    end
    
    //-----------------------------------------------------------------------------
    // Time Quantum Counter
    //-----------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tq_counter <= 8'd0;
        end else if (presc_tick) begin
            if (hard_sync) begin
                // Hard sync: restart from beginning
                tq_counter <= 8'd0;
            end else if (tq_counter >= {3'b0, tq_limit}) begin
                tq_counter <= 8'd0;
            end else begin
                tq_counter <= tq_counter + 8'd1;
            end
        end
    end
    
    //-----------------------------------------------------------------------------
    // Synchronization State Machine
    //-----------------------------------------------------------------------------
    
    // Calculate phase error
    always_comb begin
        if (tq_counter < {3'b0, sample_tq_base}) begin
            phase_error = {1'b0, sample_tq_base[3:0]} - {1'b0, tq_counter[3:0]};
        end else begin
            phase_error = {1'b0, tq_counter[3:0]} - {1'b0, sample_tq_base[3:0]};
        end
    end
    
    // Synchronization state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_state     <= SYNC_IDLE;
            phase_seg1_adj <= {1'b0, phase_seg1};
            phase_seg2_adj <= {1'b0, phase_seg2};
            sync_done      <= 1'b0;
            phase_adjusted <= 1'b0;
            sync_active    <= 1'b0;
        end else begin
            // Default: reset adjustments at start of new bit
            if (presc_tick && tq_counter == 8'd0 && !hard_sync) begin
                phase_seg1_adj <= {1'b0, phase_seg1};
                phase_seg2_adj <= {1'b0, phase_seg2};
                sync_done      <= 1'b0;
                phase_adjusted <= 1'b0;
                sync_active    <= 1'b0;
                sync_state     <= SYNC_IDLE;
            end else begin
                case (sync_state)
                    SYNC_IDLE: begin
                        if (hard_sync) begin
                            // Hard sync: reset phase adjustments
                            phase_seg1_adj <= {1'b0, phase_seg1};
                            phase_seg2_adj <= {1'b0, phase_seg2};
                            sync_done      <= 1'b1;
                            sync_active    <= 1'b1;
                            sync_state     <= SYNC_COMPLETE;
                        end else if (sync_en && edge_detected_int && !sync_done) begin
                            sync_state  <= SYNC_WAIT_EDGE;
                            sync_active <= 1'b1;
                        end
                    end
                    
                    SYNC_WAIT_EDGE: begin
                        // Process the edge and calculate adjustment
                        if (tq_counter < {3'b0, sample_tq_base}) begin
                            // Phase error: edge before sample point, lengthen phase_seg1
                            // Limit adjustment to SJW
                            if (phase_error <= {2'b0, sjw}) begin
                                phase_seg1_adj <= {1'b0, phase_seg1} + phase_error[2:0];
                            end else begin
                                phase_seg1_adj <= {1'b0, phase_seg1} + {2'b0, sjw};
                            end
                            phase_seg2_adj <= {1'b0, phase_seg2};
                        end else begin
                            // Phase error: edge after sample point, shorten phase_seg2
                            // Limit adjustment to SJW
                            if (phase_error <= {2'b0, sjw}) begin
                                phase_seg2_adj <= {1'b0, phase_seg2} - phase_error[2:0];
                            end else begin
                                phase_seg2_adj <= {1'b0, phase_seg2} - {2'b0, sjw};
                            end
                            phase_seg1_adj <= {1'b0, phase_seg1};
                        end
                        phase_adjusted <= 1'b1;
                        sync_state     <= SYNC_ADJUSTING;
                    end
                    
                    SYNC_ADJUSTING: begin
                        // Wait for the adjusted timing to take effect
                        sync_done  <= 1'b1;
                        sync_state <= SYNC_COMPLETE;
                    end
                    
                    SYNC_COMPLETE: begin
                        // Synchronization complete, wait for next bit
                        sync_active <= 1'b0;
                        if (presc_tick && tq_counter == 8'd0) begin
                            sync_state <= SYNC_IDLE;
                        end
                    end
                    
                    default: begin
                        sync_state <= SYNC_IDLE;
                    end
                endcase
            end
        end
    end
    
    //-----------------------------------------------------------------------------
    // Output Generation
    //-----------------------------------------------------------------------------
    
    // Bit tick: pulse at start of each bit (TQ = 0)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_tick <= 1'b0;
        end else if (presc_tick && tq_counter == 8'd0) begin
            bit_tick <= 1'b1;
        end else begin
            bit_tick <= 1'b0;
        end
    end
    
    // Sample tick: pulse at sample point
    // Uses adjusted sample point when synchronization is active
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_tick <= 1'b0;
        end else if (presc_tick) begin
            if (phase_adjusted) begin
                // Use adjusted sample point
                if (tq_counter == {3'b0, sample_tq_adj}) begin
                    sample_tick <= 1'b1;
                end else begin
                    sample_tick <= 1'b0;
                end
            end else begin
                // Use base sample point
                if (tq_counter == {3'b0, sample_tq_base}) begin
                    sample_tick <= 1'b1;
                end else begin
                    sample_tick <= 1'b0;
                end
            end
        end else begin
            sample_tick <= 1'b0;
        end
    end
    
    // TX tick: pulse at transmission point (sync segment, TQ = 1)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_tick <= 1'b0;
        end else if (presc_tick && tq_counter == {3'b0, tx_tq}) begin
            tx_tick <= 1'b1;
        end else begin
            tx_tick <= 1'b0;
        end
    end
    
    // Sample point: high during sample phase (from sample point to end of bit)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_point <= 1'b0;
        end else if (presc_tick) begin
            if (phase_adjusted) begin
                sample_point <= (tq_counter >= {3'b0, sample_tq_adj});
            end else begin
                sample_point <= (tq_counter >= {3'b0, sample_tq_base});
            end
        end
    end
    
    // Bit time counter output
    assign bit_time_cnt = tq_counter;
    
    // Sync locked status: indicates timing is stable
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_locked <= 1'b0;
        end else begin
            // Locked when we've completed at least one bit or sync
            sync_locked <= sync_done || (tq_counter > 8'd0);
        end
    end

endmodule : can_btu_top
