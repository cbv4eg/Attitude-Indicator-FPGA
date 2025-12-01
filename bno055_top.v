/////////////////////////////////////////////////////////////////////
////                                                             ////
////  BNO055 I2C Operations using Nandland Go Board              ////
////  Displays CHIP ID reg values using 7-segment display        ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

`define STOP 0
`define READ 1
`define WRITE 2

// Continuously reads Euler angle data from BNO055 registers 0x1A-0x1F
// Register 0x1A-0x1B: EUL_DATA_X (Heading) - LSB, MSB
// Register 0x1C-0x1D: EUL_DATA_Y (Roll)    - LSB, MSB  
// Register 0x1E-0x1F: EUL_DATA_Z (Pitch)   - LSB, MSB

module bno055_top (
    input CLK,      // Go Board clk
    input SW1,      // reset
    input SW2,      // start next chip id read operation
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

// Debounce Switches
reg Cleaned_SW1;
Debounce_Filter_Edge SW1_debouncer(
    .i_CLK          (CLK),
    .i_Bouncy       (SW1),
    .o_Debounced    (Cleaned_SW1)
);
reg Cleaned_SW2;
Debounce_Filter_Edge SW2_debouncer(
    .i_CLK          (CLK),
    .i_Bouncy       (SW2),
    .o_Debounced    (Cleaned_SW2)
);

// BNO055 OPERATION STATE MACHINE: reset --> configure mode --> read continously 
reg [3:0] state = ST_CONFIG;
localparam
    ST_CONFIG = 1,
    ST_CONFIG_WAIT = 2,
    ST_READ_BYTE = 3,
    ST_READ_BYTE_WAIT = 4,
    ST_NEXT_BYTE = 5;

// Store the 4 bytes of Euler data (roll + pitch only)
reg [7:0] euler_data [0:3];
wire [7:0] current_addr;
reg [1:0] byte_count = 2'd0;

// Current register address: 0x1A + byte_count
assign current_addr = 8'h1A + {6'h0, byte_count};

// I2C transaction variables
reg [1:0] opcode;
reg [7:0] reg_addr, wr_reg_data, rd_reg_data;
wire op_done;

bno055_read_write transaction (
    .i_clk          (CLK),
    .i_rst          (Cleaned_SW1),
    .i_opcode       (opcode),
    .i_reg_addr     (reg_addr),
    .i_tx_data      (wr_reg_data),
    .o_reg_data     (rd_reg_data),
    .o_done         (op_done),
    .io_sda         (PMOD1),
    .io_scl         (PMOD7)
);

always @(posedge CLK) begin
    if (Cleaned_SW1) begin
        state <= ST_CONFIG;
        opcode <= `STOP; // 0
    end else begin
        case(state)

        ST_CONFIG: begin
            // Go straight into config sequence after reset
            reg_addr <= 8'h3D;   // OPR_MODE
            wr_reg_data <= 8'h0B;   // NDOF_FMC_OFF
            opcode <= `WRITE; // 2
            state <= ST_CONFIG_WAIT;
        end

        ST_CONFIG_WAIT: begin
            opcode <= `STOP; // 0
            if (op_done)
                state <= ST_READ_BYTE;   // 7ms delay
        end

        ST_READ_BYTE: begin
            reg_addr <= current_addr;   // OPR_MODE
            opcode <= `READ; // 1
            state <= ST_READ_BYTE_WAIT;
        end
                
        ST_READ_BYTE_WAIT: begin
            opcode <= `STOP; // 0
            if (op_done) begin
                euler_data[byte_count] <= rd_reg_data;
                state <= ST_NEXT_BYTE;
            end
        end
        
        ST_NEXT_BYTE: begin
            opcode <= `STOP; // 0
            if (byte_count == 2'd3) begin
                // All bytes read, process and restart
                byte_count <= 2'd0;
                state <= ST_READ_BYTE;
            end else begin
                byte_count <= byte_count + 1;
                state <= ST_READ_BYTE;
            end
        end
        endcase
    end
end

/*** INTERPRETATION & DISPLAY ***/

// Reconstruct 16-bit Euler angles from bytes
// Note: BNO055 stores data as little-endian (LSB first)
wire [15:0] euler_roll;     // 0x1C (LSB), 0x1D (MSB) - EUL_DATA_Y
wire [15:0] euler_pitch;    // 0x1E (LSB), 0x1F (MSB) - EUL_DATA_Z

assign euler_roll    = {euler_data[1], euler_data[0]};  // [MSB, LSB]
assign euler_pitch   = {euler_data[3], euler_data[2]};  // [MSB, LSB]

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

// Counter to cycle through bytes 0-3
reg [1:0] display_select;
Mod_N_Counter #(.WIDTH(2), .RESET_VALUE(2'd3)) ID_reg_counter (
    .reset(Cleaned_SW1),
    .clock(Cleaned_SW2),
    .o_Counter(display_select)
);

// Select which byte to display
reg [7:0] display_byte;
always @(*) begin
    case (display_select)
        2'd0: display_byte = euler_data[0];  // Roll LSB
        2'd1: display_byte = euler_data[1];  // Roll MSB
        2'd2: display_byte = euler_data[2];  // Pitch LSB
        2'd3: display_byte = euler_data[3];  // Pitch MSB
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