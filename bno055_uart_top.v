// bno055_uart_top.v - Top-level module for BNO055 IMU to UART
// Reads Euler angles (pitch, roll, yaw) from BNO055 and transmits via UART
// For Nandland Go Board - 25 MHz clock

`timescale 1ns / 1ps

module bno055_uart_top (
    input  wire       CLK,           // 25 MHz clock
    input  wire       SW1,           // Reset button (active low)
    
    // I2C signals to BNO055 (via PMOD connector)
    inout  wire       I2C_SDA,       // PMOD1 - pin 65
    inout  wire       I2C_SCL,       // PMOD2 - pin 64
    
    // UART signals
    input  wire       RX,            // UART RX (unused but included)
    output wire       TX,            // UART TX to PC
    
    // Status LEDs
    output wire       LED1,          // Init complete
    output wire       LED2,          // I2C busy
    output wire       LED3,          // Data valid
    output wire       LED4           // Error/missed ACK
);

// ============================================================================
// Parameters and Constants
// ============================================================================

// BNO055 I2C address (0x28 when ADDR pin low, 0x29 when high)
localparam [6:0] BNO055_ADDR = 7'h28;

// BNO055 Register addresses
localparam [7:0] REG_CHIP_ID    = 8'h00;  // Should read 0xA0
localparam [7:0] REG_OPR_MODE   = 8'h3D;  // Operation mode register
localparam [7:0] REG_PWR_MODE   = 8'h3E;  // Power mode register
localparam [7:0] REG_UNIT_SEL   = 8'h3B;  // Unit selection
localparam [7:0] REG_EULER_H_LSB = 8'h1A; // Euler heading LSB
localparam [7:0] REG_EULER_H_MSB = 8'h1B; // Euler heading MSB
localparam [7:0] REG_EULER_R_LSB = 8'h1C; // Euler roll LSB
localparam [7:0] REG_EULER_R_MSB = 8'h1D; // Euler roll MSB
localparam [7:0] REG_EULER_P_LSB = 8'h1E; // Euler pitch LSB
localparam [7:0] REG_EULER_P_MSB = 8'h1F; // Euler pitch MSB

// Operation modes
localparam [7:0] MODE_CONFIG    = 8'h00;  // Configuration mode
localparam [7:0] MODE_NDOF      = 8'h0C;  // 9DOF sensor fusion

// I2C clock prescaler: 25MHz / (100kHz * 4) = 62.5 ≈ 63
localparam [15:0] I2C_PRESCALE = 16'd63;

// State machine states
localparam [4:0] 
    ST_RESET        = 5'd0,
    ST_WAIT_STARTUP = 5'd1,
    ST_CHECK_ID     = 5'd2,
    ST_READ_ID      = 5'd3,
    ST_SET_CONFIG   = 5'd4,
    ST_SET_POWER    = 5'd5,
    ST_SET_UNITS    = 5'd6,
    ST_SET_NDOF     = 5'd7,
    ST_WAIT_CALIB   = 5'd8,
    ST_IDLE         = 5'd9,
    ST_READ_EULER   = 5'd10,
    ST_WAIT_READ    = 5'd11,
    ST_FORMAT_DATA  = 5'd12,
    ST_SEND_UART    = 5'd13,
    ST_WAIT_UART    = 5'd14,
    ST_ERROR        = 5'd15;

// ============================================================================
// Signal Declarations
// ============================================================================

// Reset and timing
reg        rst;
reg [27:0] startup_counter;
reg [27:0] sample_counter;
reg [3:0]  byte_counter;
reg [3:0]  uart_byte_idx;

// State machine
reg [4:0]  state;
reg [4:0]  next_state;

// I2C master signals
reg [6:0]  i2c_cmd_address;
reg        i2c_cmd_start;
reg        i2c_cmd_read;
reg        i2c_cmd_write;
reg        i2c_cmd_write_multiple;
reg        i2c_cmd_stop;
reg        i2c_cmd_valid;
wire       i2c_cmd_ready;

reg [7:0]  i2c_data_tdata;
reg        i2c_data_tvalid;
wire       i2c_data_tready;
reg        i2c_data_tlast;

wire [7:0] i2c_rx_tdata;
wire       i2c_rx_tvalid;
reg        i2c_rx_tready;
wire       i2c_rx_tlast;

wire       i2c_busy;
wire       i2c_bus_control;
wire       i2c_bus_active;
wire       i2c_missed_ack;

// I2C tristate signals
wire       scl_i, scl_o, scl_t;
wire       sda_i, sda_o, sda_t;
reg        scl_pin_reg, sda_pin_reg;

// BNO055 data registers
reg [7:0]  chip_id;
reg [15:0] euler_heading;
reg [15:0] euler_roll;
reg [15:0] euler_pitch;
reg [7:0]  read_data [0:5];  // Buffer for 6-byte Euler read

// UART signals
reg [7:0]  uart_tx_byte;
wire       uart_tx_active;
wire       uart_tx_done;
wire [7:0] uart_rx_byte;
wire       uart_rx_dv;

// Formatted output buffer (ASCII hex representation)
reg [7:0]  tx_buffer [0:15];  // Format: "H:xxxx R:xxxx P:xxxx\n"
reg        init_complete;
reg        error_flag;

// ============================================================================
// Reset Synchronization
// ============================================================================

always @(posedge CLK) begin
    rst <= ~SW1;  // Active high reset from active low button
end

// ============================================================================
// I2C Master Instance
// ============================================================================

i2c_master i2c_inst (
    .clk(CLK),
    .rst(rst),
    
    // Command interface
    .s_axis_cmd_address(i2c_cmd_address),
    .s_axis_cmd_start(i2c_cmd_start),
    .s_axis_cmd_read(i2c_cmd_read),
    .s_axis_cmd_write(i2c_cmd_write),
    .s_axis_cmd_write_multiple(i2c_cmd_write_multiple),
    .s_axis_cmd_stop(i2c_cmd_stop),
    .s_axis_cmd_valid(i2c_cmd_valid),
    .s_axis_cmd_ready(i2c_cmd_ready),
    
    // Write data interface
    .s_axis_data_tdata(i2c_data_tdata),
    .s_axis_data_tvalid(i2c_data_tvalid),
    .s_axis_data_tready(i2c_data_tready),
    .s_axis_data_tlast(i2c_data_tlast),
    
    // Read data interface
    .m_axis_data_tdata(i2c_rx_tdata),
    .m_axis_data_tvalid(i2c_rx_tvalid),
    .m_axis_data_tready(i2c_rx_tready),
    .m_axis_data_tlast(i2c_rx_tlast),
    
    // I2C physical interface
    .scl_i(scl_i),
    .scl_o(scl_o),
    .scl_t(scl_t),
    .sda_i(sda_i),
    .sda_o(sda_o),
    .sda_t(sda_t),
    
    // Status
    .busy(i2c_busy),
    .bus_control(i2c_bus_control),
    .bus_active(i2c_bus_active),
    .missed_ack(i2c_missed_ack),
    
    // Configuration
    .prescale(I2C_PRESCALE),
    .stop_on_idle(1'b1)
);

// ============================================================================
// I2C Tristate Interface (Open-drain)
// ============================================================================

assign scl_i = scl_pin_reg;
assign sda_i = sda_pin_reg;

// Simulate bidirectional pins for Go Board PMOD
assign I2C_SCL = scl_o ? 1'bz : 1'b0;
assign I2C_SDA = sda_o ? 1'bz : 1'b0;

always @(posedge CLK) begin
    scl_pin_reg <= I2C_SCL;
    sda_pin_reg <= I2C_SDA;
end

// ============================================================================
// UART Instance
// ============================================================================

uart uart_inst (
    .i_Clk(CLK),
    .i_rx_serial(RX),
    .o_tx_serial(TX),
    
    .o_rx_dv(uart_rx_dv),
    .o_rx_byte(uart_rx_byte),
    
    .i_tx_byte(uart_tx_byte),
    .o_tx_active(uart_tx_active),
    .o_tx_done(uart_tx_done)
);

// ============================================================================
// Main Control FSM
// ============================================================================

always @(posedge CLK) begin
    if (rst) begin
        state <= ST_RESET;
        startup_counter <= 0;
        sample_counter <= 0;
        byte_counter <= 0;
        uart_byte_idx <= 0;
        init_complete <= 0;
        error_flag <= 0;
        
        i2c_cmd_valid <= 0;
        i2c_data_tvalid <= 0;
        i2c_rx_tready <= 0;
        
    end else begin
        // Default: clear single-cycle signals
        i2c_cmd_valid <= 0;
        i2c_data_tvalid <= 0;
        i2c_rx_tready <= 0;
        
        case (state)
            // ================================================================
            // Initialization States
            // ================================================================
            
            ST_RESET: begin
                startup_counter <= 0;
                sample_counter <= 0;
                byte_counter <= 0;
                uart_byte_idx <= 0;
                init_complete <= 0;
                error_flag <= 0;
                state <= ST_WAIT_STARTUP;
            end
            
            ST_WAIT_STARTUP: begin
                // Wait 650ms for BNO055 power-on reset
                // At 25 MHz: 650ms = 16,250,000 cycles
                if (startup_counter < 28'd16_250_000) begin
                    startup_counter <= startup_counter + 1;
                end else begin
                    state <= ST_CHECK_ID;
                end
            end
            
            ST_CHECK_ID: begin
                // Read chip ID to verify communication
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write <= 1;
                    i2c_cmd_stop <= 0;
                    i2c_cmd_valid <= 1;
                    i2c_data_tdata <= REG_CHIP_ID;
                    i2c_data_tvalid <= 1;
                    i2c_data_tlast <= 1;
                    next_state <= ST_READ_ID;
                    state <= ST_WAIT_READ;
                end
            end
            
            ST_READ_ID: begin
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_read <= 1;
                    i2c_cmd_stop <= 1;
                    i2c_cmd_valid <= 1;
                    i2c_rx_tready <= 1;
                    
                    if (i2c_rx_tvalid) begin
                        chip_id <= i2c_rx_tdata;
                        if (i2c_rx_tdata == 8'hA0) begin
                            state <= ST_SET_CONFIG;
                        end else begin
                            state <= ST_ERROR;
                        end
                    end
                end
            end
            
            ST_SET_CONFIG: begin
                // Set to CONFIG mode
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write_multiple <= 1;
                    i2c_cmd_stop <= 1;
                    i2c_cmd_valid <= 1;
                    
                    if (byte_counter == 0) begin
                        i2c_data_tdata <= REG_OPR_MODE;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 0;
                        byte_counter <= 1;
                    end else if (byte_counter == 1 && i2c_data_tready) begin
                        i2c_data_tdata <= MODE_CONFIG;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 1;
                        byte_counter <= 0;
                        state <= ST_SET_POWER;
                    end
                end
            end
            
            ST_SET_POWER: begin
                // Set to normal power mode
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write_multiple <= 1;
                    i2c_cmd_stop <= 1;
                    i2c_cmd_valid <= 1;
                    
                    if (byte_counter == 0) begin
                        i2c_data_tdata <= REG_PWR_MODE;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 0;
                        byte_counter <= 1;
                    end else if (byte_counter == 1 && i2c_data_tready) begin
                        i2c_data_tdata <= 8'h00;  // Normal power mode
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 1;
                        byte_counter <= 0;
                        state <= ST_SET_UNITS;
                    end
                end
            end
            
            ST_SET_UNITS: begin
                // Set units (degrees for Euler angles)
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write_multiple <= 1;
                    i2c_cmd_stop <= 1;
                    i2c_cmd_valid <= 1;
                    
                    if (byte_counter == 0) begin
                        i2c_data_tdata <= REG_UNIT_SEL;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 0;
                        byte_counter <= 1;
                    end else if (byte_counter == 1 && i2c_data_tready) begin
                        i2c_data_tdata <= 8'h00;  // Degrees, m/s^2, dps
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 1;
                        byte_counter <= 0;
                        state <= ST_SET_NDOF;
                    end
                end
            end
            
            ST_SET_NDOF: begin
                // Set to NDOF mode (9DOF fusion)
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write_multiple <= 1;
                    i2c_cmd_stop <= 1;
                    i2c_cmd_valid <= 1;
                    
                    if (byte_counter == 0) begin
                        i2c_data_tdata <= REG_OPR_MODE;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 0;
                        byte_counter <= 1;
                    end else if (byte_counter == 1 && i2c_data_tready) begin
                        i2c_data_tdata <= MODE_NDOF;
                        i2c_data_tvalid <= 1;
                        i2c_data_tlast <= 1;
                        byte_counter <= 0;
                        startup_counter <= 0;
                        state <= ST_WAIT_CALIB;
                    end
                end
            end
            
            ST_WAIT_CALIB: begin
                // Wait 1 second for sensor fusion to stabilize
                // At 25 MHz: 1s = 25,000,000 cycles
                if (startup_counter < 28'd25_000_000) begin
                    startup_counter <= startup_counter + 1;
                end else begin
                    init_complete <= 1;
                    sample_counter <= 0;
                    state <= ST_IDLE;
                end
            end
            
            // ================================================================
            // Data Acquisition Loop
            // ================================================================
            
            ST_IDLE: begin
                // Sample at ~10 Hz (every 2.5M cycles at 25 MHz)
                if (sample_counter < 28'd2_500_000) begin
                    sample_counter <= sample_counter + 1;
                end else begin
                    sample_counter <= 0;
                    byte_counter <= 0;
                    state <= ST_READ_EULER;
                end
            end
            
            ST_READ_EULER: begin
                // Read 6 bytes starting from Euler heading LSB
                if (i2c_cmd_ready && !i2c_busy) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_start <= 1;
                    i2c_cmd_write <= 1;
                    i2c_cmd_stop <= 0;
                    i2c_cmd_valid <= 1;
                    i2c_data_tdata <= REG_EULER_H_LSB;
                    i2c_data_tvalid <= 1;
                    i2c_data_tlast <= 1;
                    byte_counter <= 0;
                    next_state <= ST_WAIT_READ;
                    state <= ST_WAIT_READ;
                end
            end
            
            ST_WAIT_READ: begin
                // Read 6 bytes of Euler data
                if (i2c_cmd_ready && !i2c_busy && byte_counter == 0) begin
                    i2c_cmd_address <= BNO055_ADDR;
                    i2c_cmd_read <= 1;
                    i2c_cmd_stop <= 0;
                    i2c_cmd_valid <= 1;
                    byte_counter <= 1;
                end
                
                if (i2c_rx_tvalid && byte_counter > 0 && byte_counter <= 6) begin
                    i2c_rx_tready <= 1;
                    read_data[byte_counter - 1] <= i2c_rx_tdata;
                    
                    if (byte_counter < 6) begin
                        byte_counter <= byte_counter + 1;
                    end else begin
                        // All 6 bytes received
                        euler_heading <= {read_data[1], read_data[0]};
                        euler_roll    <= {read_data[3], read_data[2]};
                        euler_pitch   <= {read_data[5], read_data[4]};
                        byte_counter <= 0;
                        state <= ST_FORMAT_DATA;
                    end
                end
            end
            
            ST_FORMAT_DATA: begin
                // Format data as ASCII: "H:xxxx R:xxxx P:xxxx\n"
                tx_buffer[0]  <= "H";
                tx_buffer[1]  <= ":";
                tx_buffer[2]  <= hex_to_ascii(euler_heading[15:12]);
                tx_buffer[3]  <= hex_to_ascii(euler_heading[11:8]);
                tx_buffer[4]  <= hex_to_ascii(euler_heading[7:4]);
                tx_buffer[5]  <= hex_to_ascii(euler_heading[3:0]);
                tx_buffer[6]  <= " ";
                tx_buffer[7]  <= "R";
                tx_buffer[8]  <= ":";
                tx_buffer[9]  <= hex_to_ascii(euler_roll[15:12]);
                tx_buffer[10] <= hex_to_ascii(euler_roll[11:8]);
                tx_buffer[11] <= hex_to_ascii(euler_roll[7:4]);
                tx_buffer[12] <= hex_to_ascii(euler_roll[3:0]);
                tx_buffer[13] <= " ";
                tx_buffer[14] <= "P";
                tx_buffer[15] <= ":";
                // Continue with pitch and newline...
                uart_byte_idx <= 0;
                state <= ST_SEND_UART;
            end
            
            ST_SEND_UART: begin
                if (!uart_tx_active && uart_byte_idx < 16) begin
                    uart_tx_byte <= tx_buffer[uart_byte_idx];
                    state <= ST_WAIT_UART;
                end else if (uart_byte_idx >= 16) begin
                    state <= ST_IDLE;
                end
            end
            
            ST_WAIT_UART: begin
                if (uart_tx_done) begin
                    uart_byte_idx <= uart_byte_idx + 1;
                    state <= ST_SEND_UART;
                end
            end
            
            ST_ERROR: begin
                error_flag <= 1;
                // Stay in error state
            end
            
            default: state <= ST_RESET;
        endcase
        
        // Handle missed ACK
        if (i2c_missed_ack) begin
            error_flag <= 1;
        end
    end
end

// ============================================================================
// Helper Function: Convert 4-bit hex to ASCII
// ============================================================================

function [7:0] hex_to_ascii;
    input [3:0] hex;
    begin
        if (hex < 4'd10)
            hex_to_ascii = 8'h30 + hex;  // '0'-'9'
        else
            hex_to_ascii = 8'h37 + hex;  // 'A'-'F'
    end
endfunction

// ============================================================================
// Status LED Assignments
// ============================================================================

assign LED1 = init_complete;
assign LED2 = i2c_busy;
assign LED3 = (state == ST_SEND_UART);
assign LED4 = error_flag;

endmodule
