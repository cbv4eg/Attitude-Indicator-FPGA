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
    inout PMOD1,    // sda
    inout PMOD7,    // scl
    input RX,
    output TX
);

// Debounce Switches
reg Cleaned_SW1;
Debounce_Filter_Edge SW1_debouncer(
    .i_CLK          (CLK),
    .i_Bouncy       (SW1),
    .o_Debounced    (Cleaned_SW1)
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
assign current_addr = 8'h1C + {6'h0, byte_count};

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

wire [15:0] roll_deg = euler_roll >> 4;
wire [15:0] pitch_deg = euler_pitch >> 4;

/*** ORIENTATION ENCODER ***/

wire [3:0] orientation;  // 0-8 orientation code

orientation_encoder #(
    .THRESHOLD(16'd400)  // Raw threshold for tilt detection
) encoder (
    .roll_raw(euler_roll),
    .pitch_raw(euler_pitch),
    .orientation(orientation)
);

/*** UART TRANSMISSION ***/

// UART parameters for 115200 baud with 25MHz clock
// CLKS_PER_BIT = Clock_Freq / Baud_Rate = 25000000 / 115200 = 217
parameter CLKS_PER_BIT = 217;

// UART transmission state machine
reg [3:0] uart_state = 4'd0;
localparam
    UART_IDLE = 4'd0,
    UART_SEND_START = 4'd1,
    UART_WAIT_START = 4'd2,
    UART_SEND_ROLL_LSB = 4'd3,
    UART_WAIT_ROLL_LSB = 4'd4,
    UART_SEND_ROLL_MSB = 4'd5,
    UART_WAIT_ROLL_MSB = 4'd6,
    UART_SEND_PITCH_LSB = 4'd7,
    UART_WAIT_PITCH_LSB = 4'd8,
    UART_SEND_PITCH_MSB = 4'd9,
    UART_WAIT_PITCH_MSB = 4'd10,
    UART_SEND_END = 4'd11,
    UART_WAIT_END = 4'd12;

reg [7:0] tx_byte;
wire tx_done;
wire tx_active;

// Instantiate UART transmitter
uart_tx #(
    .CLKS_PER_BIT(CLKS_PER_BIT)
) uart_transmitter (
    .i_Clk(CLK),
    .i_tx_byte(tx_byte),
    .o_tx_active(tx_active),
    .o_tx_serial(TX),
    .o_tx_done(tx_done)
);

// Delay counter to wait between complete data packets
reg [23:0] packet_delay = 24'd0;
localparam PACKET_DELAY_MAX = 24'd2500000; // ~100ms at 25MHz

// Flag to indicate we have valid data (at least one complete read cycle)
reg data_valid = 1'b0;

always @(posedge CLK) begin
    if (Cleaned_SW1) begin
        data_valid <= 1'b0;
    end else if (state == ST_NEXT_BYTE && byte_count == 2'd3) begin
        data_valid <= 1'b1;  // Mark data as valid after first complete read
    end
end

// UART transmission control
always @(posedge CLK) begin
    if (Cleaned_SW1) begin
        uart_state <= UART_IDLE;
        packet_delay <= 24'd0;
    end else begin
        case(uart_state)
            UART_IDLE: begin
                if (data_valid) begin
                    if (packet_delay < PACKET_DELAY_MAX) begin
                        packet_delay <= packet_delay + 1;
                    end else begin
                        packet_delay <= 24'd0;
                        uart_state <= UART_SEND_START;
                    end
                end
            end

            UART_SEND_START: begin
                tx_byte <= 8'hAA;  // Start byte
                uart_state <= UART_WAIT_START;
            end

            UART_WAIT_START: begin
                if (tx_done) uart_state <= UART_SEND_ROLL_LSB;
            end

            UART_SEND_ROLL_LSB: begin
                tx_byte <= euler_roll[7:0];  // Roll LSB
                uart_state <= UART_WAIT_ROLL_LSB;
            end

            UART_WAIT_ROLL_LSB: begin
                if (tx_done) uart_state <= UART_SEND_ROLL_MSB;
            end

            UART_SEND_ROLL_MSB: begin
                tx_byte <= euler_roll[15:8];  // Roll MSB
                uart_state <= UART_WAIT_ROLL_MSB;
            end

            UART_WAIT_ROLL_MSB: begin
                if (tx_done) uart_state <= UART_SEND_PITCH_LSB;
            end

            UART_SEND_PITCH_LSB: begin
                tx_byte <= euler_pitch[7:0];  // Pitch LSB
                uart_state <= UART_WAIT_PITCH_LSB;
            end

            UART_WAIT_PITCH_LSB: begin
                if (tx_done) uart_state <= UART_SEND_PITCH_MSB;
            end

            UART_SEND_PITCH_MSB: begin
                tx_byte <= euler_pitch[15:8];  // Pitch MSB
                uart_state <= UART_WAIT_PITCH_MSB;
            end

            UART_WAIT_PITCH_MSB: begin
                if (tx_done) uart_state <= UART_SEND_END;
            end

            UART_SEND_END: begin
                tx_byte <= 8'h55;  // End byte
                uart_state <= UART_WAIT_END;
            end

            UART_WAIT_END: begin
                if (tx_done) uart_state <= UART_IDLE;
            end

            default: uart_state <= UART_IDLE;
        endcase
    end
end

endmodule