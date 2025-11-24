// top_bno055_uart.v
// Top-level module for BNO055 IMU to UART bridge on Nandland Go-Board
// Reads Euler angles (pitch, roll, yaw) and transmits to PC via UART

module top_bno055_uart (
    input wire CLK,           // 25 MHz clock from Go-Board
    input wire SW1,         // Active low reset (SW1)
    
    // I2C connections to BNO055
    inout wire PMOD1,          // Connect to PMOD1 (pin 65)
    output wire PMOD7,         // Connect to PMOD2 (pin 78)
    
    // UART connection to PC
    output wire TX,      // TX to PC
    input wire RX,       // RX from PC (unused in this design)
    
    // Status LEDs
    output wire LED1,         // I2C transaction activecd 
    output wire LED2,         // Data transmission active
    output wire LED3,         // Initialization complete
    output wire LED4          // Error indicator
);

    // BNO055 Register Addresses
    localparam BNO055_ADDR       = 7'h29;           // Default I2C address
    localparam OPR_MODE_REG      = 8'h3D;           // Operation mode register
    localparam PWR_MODE_REG      = 8'h3E;           // Power mode register
    localparam SYS_TRIGGER_REG   = 8'h3F;           // System trigger register
    localparam EULER_H_LSB       = 8'h1A;           // Euler Heading LSB (yaw)
    localparam EULER_R_LSB       = 8'h1C;           // Euler Roll LSB
    localparam EULER_P_LSB       = 8'h1E;           // Euler Pitch LSB
    
    // Operation Modes
    localparam CONFIG_MODE       = 8'h00;           // Configuration mode
    localparam NDOF_MODE         = 8'h0C;           // 9DOF sensor fusion
    
    // State machine for initialization and data reading
    localparam INIT_RESET        = 5'd0;
    localparam INIT_DELAY1       = 5'd1;
    localparam INIT_CONFIG_MODE  = 5'd2;
    localparam INIT_DELAY2       = 5'd3;
    localparam INIT_NDOF_MODE    = 5'd4;
    localparam INIT_DELAY3       = 5'd5;
    localparam INIT_COMPLETE     = 5'd6;
    localparam READ_YAW_LSB      = 5'd7;
    localparam READ_YAW_MSB      = 5'd8;
    localparam READ_ROLL_LSB     = 5'd9;
    localparam READ_ROLL_MSB     = 5'd10;
    localparam READ_PITCH_LSB    = 5'd11;
    localparam READ_PITCH_MSB    = 5'd12;
    localparam TRANSMIT_DATA     = 5'd13;
    localparam WAIT_PERIOD       = 5'd14;
    
    reg [4:0] r_State;
    reg [31:0] r_Delay_Counter;
    reg [3:0] r_TX_Byte_Count;
    
    // I2C Master signals
    reg r_I2C_Enable;
    reg r_I2C_RW;
    reg [7:0] r_I2C_Reg_Addr;
    reg [7:0] r_I2C_Write_Data;
    wire [7:0] w_I2C_Read_Data;
    wire w_I2C_Data_Valid;
    wire w_I2C_Ready;
    
    // BNO055 data registers
    reg [15:0] r_Yaw;           // Heading/Yaw
    reg [15:0] r_Roll;
    reg [15:0] r_Pitch;
    
    // UART signals
    reg [7:0] r_UART_TX_Byte;
    wire w_UART_TX_Active;
    wire w_UART_TX_Done;
    wire [7:0] w_UART_RX_Byte;
    wire w_UART_RX_Valid;
    
    // Data transmission buffer (14 bytes total)
    // Format: [START][YAW_H][YAW_L][ROLL_H][ROLL_L][PITCH_H][PITCH_L][END][NEWLINE]
    reg [7:0] r_TX_Buffer [0:8];
    localparam START_BYTE = 8'h53;  // 'S'
    localparam END_BYTE   = 8'h45;  // 'E'
    localparam NEWLINE    = 8'h0A;  // '\n'
    localparam SPACE      = 8'h20;  // ' '
    
    // Status signals
    reg r_Init_Complete;
    reg r_Error;
    
    // Assign LEDs
    assign LED1 = ~w_I2C_Ready;           // I2C active (inverted for visibility)
    assign LED2 = w_UART_TX_Active;       // UART transmitting
    assign LED3 = r_Init_Complete;        // Initialization done
    assign LED4 = r_Error;                // Error occurred
    
    // Instantiate I2C Master
    i2c_master i2c_master_inst (
        .i_Clk(CLK),
        .i_Rst_L(SW1),
        .i_Enable(r_I2C_Enable),
        .i_RW(r_I2C_RW),
        .i_Slave_Addr(BNO055_ADDR),
        .i_Reg_Addr(r_I2C_Reg_Addr),
        .i_Write_Data(r_I2C_Write_Data),
        .i_Num_Bytes(8'd1),
        .o_Read_Data(w_I2C_Read_Data),
        .o_Data_Valid(w_I2C_Data_Valid),
        .o_Ready(w_I2C_Ready),
        .io_SDA(PMOD1),
        .io_SCL(PMOD7)
    );
    
    // Instantiate UART
    uart #(
        .CLKS_PER_BIT(217)  // 25 MHz / 115200 baud
    ) uart_inst (
        .i_Clk(CLK),
        .i_rx_serial(RX),
        .o_tx_serial(TX),
        .o_rx_dv(w_UART_RX_Valid),
        .o_rx_byte(w_UART_RX_Byte),
        .i_tx_byte(r_UART_TX_Byte),
        .o_tx_active(w_UART_TX_Active),
        .o_tx_done(w_UART_TX_Done)
    );
    
    // Main state machine
    always @(posedge CLK or negedge SW1) begin
        if (!SW1) begin
            r_State <= INIT_RESET;
            r_I2C_Enable <= 1'b0;
            r_I2C_RW <= 1'b0;
            r_I2C_Reg_Addr <= 8'h00;
            r_I2C_Write_Data <= 8'h00;
            r_Delay_Counter <= 0;
            r_Yaw <= 16'h0000;
            r_Roll <= 16'h0000;
            r_Pitch <= 16'h0000;
            r_UART_TX_Byte <= 8'h00;
            r_TX_Byte_Count <= 0;
            r_Init_Complete <= 1'b0;
            r_Error <= 1'b0;
        end
        else begin
            // Default: disable I2C
            if (w_I2C_Ready) begin
                r_I2C_Enable <= 1'b0;
            end
            
            case (r_State)
                // ===== INITIALIZATION SEQUENCE =====
                INIT_RESET: begin
                    // Wait for BNO055 to boot (650ms typical)
                    if (r_Delay_Counter < 25_000_000) begin  // 1 second
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_State <= INIT_DELAY1;
                    end
                end
                
                INIT_DELAY1: begin
                    if (r_Delay_Counter < 1_250_000) begin  // 50ms
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_State <= INIT_CONFIG_MODE;
                    end
                end
                
                INIT_CONFIG_MODE: begin
                    // Set BNO055 to CONFIG mode
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= OPR_MODE_REG;
                        r_I2C_Write_Data <= CONFIG_MODE;
                        r_I2C_RW <= 1'b0;  // Write
                        r_I2C_Enable <= 1'b1;
                        r_State <= INIT_DELAY2;
                    end
                end
                
                INIT_DELAY2: begin
                    // Wait 19ms for mode switch
                    if (!w_I2C_Ready) begin
                        // Wait for I2C to complete
                    end
                    else if (r_Delay_Counter < 500_000) begin  // 20ms
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_State <= INIT_NDOF_MODE;
                    end
                end
                
                INIT_NDOF_MODE: begin
                    // Set BNO055 to NDOF fusion mode
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= OPR_MODE_REG;
                        r_I2C_Write_Data <= NDOF_MODE;
                        r_I2C_RW <= 1'b0;  // Write
                        r_I2C_Enable <= 1'b1;
                        r_State <= INIT_DELAY3;
                    end
                end
                
                INIT_DELAY3: begin
                    // Wait 7ms for mode switch
                    if (!w_I2C_Ready) begin
                        // Wait for I2C to complete
                    end
                    else if (r_Delay_Counter < 250_000) begin  // 10ms
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_Init_Complete <= 1'b1;
                        r_State <= INIT_COMPLETE;
                    end
                end
                
                INIT_COMPLETE: begin
                    // Small delay before starting data reads
                    if (r_Delay_Counter < 25_000_000) begin  // 1 second
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_State <= READ_YAW_LSB;
                    end
                end
                
                // ===== DATA READING SEQUENCE =====
                READ_YAW_LSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_H_LSB;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Yaw[7:0] <= w_I2C_Read_Data;
                        r_State <= READ_YAW_MSB;
                    end
                end
                
                READ_YAW_MSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_H_LSB + 1;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Yaw[15:8] <= w_I2C_Read_Data;
                        r_State <= READ_ROLL_LSB;
                    end
                end
                
                READ_ROLL_LSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_R_LSB;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Roll[7:0] <= w_I2C_Read_Data;
                        r_State <= READ_ROLL_MSB;
                    end
                end
                
                READ_ROLL_MSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_R_LSB + 1;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Roll[15:8] <= w_I2C_Read_Data;
                        r_State <= READ_PITCH_LSB;
                    end
                end
                
                READ_PITCH_LSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_P_LSB;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Pitch[7:0] <= w_I2C_Read_Data;
                        r_State <= READ_PITCH_MSB;
                    end
                end
                
                READ_PITCH_MSB: begin
                    if (w_I2C_Ready && !r_I2C_Enable) begin
                        r_I2C_Reg_Addr <= EULER_P_LSB + 1;
                        r_I2C_RW <= 1'b1;  // Read
                        r_I2C_Enable <= 1'b1;
                    end
                    else if (w_I2C_Data_Valid) begin
                        r_Pitch[15:8] <= w_I2C_Read_Data;
                        // Prepare transmit buffer
                        r_TX_Buffer[0] <= START_BYTE;
                        r_TX_Buffer[1] <= r_Yaw[15:8];
                        r_TX_Buffer[2] <= r_Yaw[7:0];
                        r_TX_Buffer[3] <= SPACE;
                        r_TX_Buffer[4] <= r_Roll[15:8];
                        r_TX_Buffer[5] <= r_Roll[7:0];
                        r_TX_Buffer[6] <= SPACE;
                        r_TX_Buffer[7] <= r_Pitch[15:8];
                        r_TX_Buffer[8] <= r_Pitch[7:0];
                        r_TX_Byte_Count <= 0;
                        r_State <= TRANSMIT_DATA;
                    end
                end
                
                // ===== UART TRANSMISSION =====
                TRANSMIT_DATA: begin
                    if (!w_UART_TX_Active && r_TX_Byte_Count < 9) begin
                        r_UART_TX_Byte <= r_TX_Buffer[r_TX_Byte_Count];
                        r_TX_Byte_Count <= r_TX_Byte_Count + 1;
                    end
                    else if (!w_UART_TX_Active && r_TX_Byte_Count == 9) begin
                        r_UART_TX_Byte <= NEWLINE;
                        r_TX_Byte_Count <= r_TX_Byte_Count + 1;
                    end
                    else if (!w_UART_TX_Active && r_TX_Byte_Count > 9) begin
                        r_State <= WAIT_PERIOD;
                    end
                end
                
                WAIT_PERIOD: begin
                    // Wait ~100ms between readings (for ~10Hz update rate)
                    if (r_Delay_Counter < 2_500_000) begin  // 100ms
                        r_Delay_Counter <= r_Delay_Counter + 1;
                    end
                    else begin
                        r_Delay_Counter <= 0;
                        r_State <= READ_YAW_LSB;
                    end
                end
                
                default: r_State <= INIT_RESET;
            endcase
        end
    end

endmodule
