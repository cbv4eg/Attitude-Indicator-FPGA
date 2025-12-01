/////////////////////////////////////////////////////////////////////
////                                                             ////
////  BNO055 I2C Read using Nandland Go Board                    ////
////  Continuously reads and displays Euler Angle Attitude       ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

// `timescale 1ns / 10ps

// Continuously reads Euler angle data from BNO055 registers 0x1A-0x1F
// Register 0x1A-0x1B: EUL_DATA_X (Heading) - LSB, MSB
// Register 0x1C-0x1D: EUL_DATA_Y (Roll)    - LSB, MSB  
// Register 0x1E-0x1F: EUL_DATA_Z (Pitch)   - LSB, MSB

module bno055_top (
        input CLK,      // Go Board clk
        input SW1,      // reset
        input SW2,
        inout PMOD1,    // sda
        inout PMOD7,    // scl
        output S1_A,
        output S1_B,
        output S1_C,
        output S1_D,
        output S1_E,
        output S1_F,
        output S1_G,
        output S2_A,
        output S2_B,
        output S2_C,
        output S2_D,
        output S2_E,
        output S2_F,
        output S2_G,
        output LED1,
        output LED2,
        output LED3,
        output LED4
    );

    // State machine to continuously read all 6 Euler bytes sequentially
    reg [2:0] read_state = 3'd0;
    reg [2:0] byte_count = 3'd0;
    reg start_read = 1'b0;
    wire read_done;

    localparam IDLE = 3'd0;
    localparam START_READ = 3'd1;
    localparam READ_BYTE = 3'd2;
    localparam WAIT_DONE = 3'd3;
    localparam NEXT_BYTE = 3'd4;
    localparam PROCESS = 3'd5;

    // Store the 6 bytes of Euler data
    reg [7:0] euler_data [0:5];
    wire [7:0] current_addr;

    // Current register address: 0x1A + byte_count
    assign current_addr = 8'h1A + byte_count;


    // Read one byte at a time
    wire [7:0] read_byte;
    bno055_read operation (
        .i_clk          (CLK),
        .i_rst          (SW1),
        .i_reg_addr     (current_addr),
        .i_read_chip_id (start_read),
        .o_chip_id      (read_byte),
        .o_done         (read_done),
        .io_sda         (PMOD1),
        .io_scl         (PMOD7)
    );

    // State machine to continuously read all 6 bytes in a loop
    always @(posedge CLK) begin
        if (SW1) begin
            read_state <= IDLE;
            byte_count <= 3'd0;
            start_read <= 1'b0;
        end else begin
            start_read <= 1'b0;  // Default
            
            case (read_state)
                IDLE: begin
                    // Automatically start reading after reset
                    byte_count <= 3'd0;
                    read_state <= START_READ;
                end
                
                START_READ: begin
                    // Small delay to ensure system is ready
                    read_state <= READ_BYTE;
                end
                
                READ_BYTE: begin
                    start_read <= 1'b1;
                    read_state <= WAIT_DONE;
                end
                
                WAIT_DONE: begin
                    if (read_done) begin
                        euler_data[byte_count] <= read_byte;
                        read_state <= NEXT_BYTE;
                    end
                end
                
                NEXT_BYTE: begin
                    if (byte_count == 3'd5) begin
                        // All 6 bytes read, process and restart
                        read_state <= PROCESS;
                    end else begin
                        byte_count <= byte_count + 1;
                        read_state <= READ_BYTE;
                    end
                end
                
                PROCESS: begin
                    // Data is now ready and displayed continuously
                    // Immediately start next read cycle
                    byte_count <= 3'd0;
                    read_state <= READ_BYTE;
                end
                
                default: begin
                    read_state <= IDLE;
                end
            endcase
        end
    end

    // Reconstruct 16-bit Euler angles from bytes
    // Note: BNO055 stores data as little-endian (LSB first)
    // wire [15:0] euler_heading;  // 0x1A (LSB), 0x1B (MSB) - Not used in Roll_Pitch_Encoder
    wire [15:0] euler_roll;     // 0x1C (LSB), 0x1D (MSB) - EUL_DATA_Y
    wire [15:0] euler_pitch;    // 0x1E (LSB), 0x1F (MSB) - EUL_DATA_Z

    // assign euler_heading = {euler_data[1], euler_data[0]};  // [MSB, LSB]
    assign euler_roll    = {euler_data[3], euler_data[2]};  // [MSB, LSB]
    assign euler_pitch   = {euler_data[5], euler_data[4]};  // [MSB, LSB]

    // Encode roll and pitch into 4-bit attitude
    wire [3:0] attitude;
    Roll_Pitch_Encoder encoder (
        .i_Roll_Raw     (euler_roll),
        .i_Pitch_Raw    (euler_pitch),
        .o_Attitude     (attitude)
    );

    // // Decode attitude to seven segment displays
    // SSD_Decoder ssd_display (
    //     .i_Attitude     (attitude),
    //     .seg_A1         (S1_A),
    //     .seg_B1         (S1_B),
    //     .seg_C1         (S1_C),
    //     .seg_D1         (S1_D),
    //     .seg_E1         (S1_E),
    //     .seg_F1         (S1_F),
    //     .seg_G1         (S1_G),
    //     .seg_A2         (S2_A),
    //     .seg_B2         (S2_B),
    //     .seg_C2         (S2_C),
    //     .seg_D2         (S2_D),
    //     .seg_E2         (S2_E),
    //     .seg_F2         (S2_F),
    //     .seg_G2         (S2_G)
    // );

    // Debounce SW2 to select which byte to display
    wire Cleaned_SW2;
    Debounce_Filter_Edge SW2_debouncer(
        .i_CLK          (CLK),
        .i_Bouncy       (SW2),
        .o_Debounced    (Cleaned_SW2)
    );

    // Counter to cycle through bytes 0-5
    reg [2:0] display_select = 3'd0;
    always @(posedge CLK) begin
        if (SW1) begin
            display_select <= 3'd0;
        end else if (Cleaned_SW2) begin
            if (display_select == 3'd5)
                display_select <= 3'd0;
            else
                display_select <= display_select + 1;
        end
    end

    // Select which byte to display
    reg [7:0] display_byte;
    always @(*) begin
        case (display_select)
            3'd0: display_byte = euler_data[0];  // Heading LSB
            3'd1: display_byte = euler_data[1];  // Heading MSB
            3'd2: display_byte = euler_data[2];  // Roll LSB
            3'd3: display_byte = euler_data[3];  // Roll MSB
            3'd4: display_byte = euler_data[4];  // Pitch LSB
            3'd5: display_byte = euler_data[5];  // Pitch MSB
            default: display_byte = 8'h00;
        endcase
    end

    // Display selected byte in hex on seven segment displays
    SSD_Bin_Decoder hex_display_upper(
        .bits(display_byte[7:4]),
        .seg_A(S1_A), .seg_B(S1_B), .seg_C(S1_C), .seg_D(S1_D), 
        .seg_E(S1_E), .seg_F(S1_F), .seg_G(S1_G)
    );

    SSD_Bin_Decoder hex_display_lower(
        .bits(display_byte[3:0]),
        .seg_A(S2_A), .seg_B(S2_B), .seg_C(S2_C), .seg_D(S2_D), 
        .seg_E(S2_E), .seg_F(S2_F), .seg_G(S2_G)
    );

    // Status LEDs show current byte being read
    assign {LED1, LED2, LED3, LED4} = {attitude};

endmodule
