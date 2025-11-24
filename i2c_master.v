module i2c_master (
    input wire i_Clk,           // 25 MHz Go-Board clock
    input wire i_Rst_L,         // Active low reset
    input wire i_Enable,        // Start transaction
    input wire i_RW,            // 1 = Read, 0 = Write
    input wire [6:0] i_Slave_Addr,  // 7-bit slave address (0x28 or 0x29)
    input wire [7:0] i_Reg_Addr,    // Register address to access
    input wire [7:0] i_Write_Data,  // Data to write
    input wire [7:0] i_Num_Bytes,   // Number of bytes to read
    output reg [7:0] o_Read_Data,   // Data read from slave
    output reg o_Data_Valid,        // Indicates valid read data
    output reg o_Ready,             // Ready for new transaction
    inout wire io_SDA,              // I2C data line
    output wire io_SCL              // I2C clock line
);

    // I2C clock generation - 100 kHz SCL from 25 MHz system clock
    // 25 MHz / 100 kHz = 250 clocks per I2C period
    // Quarter period = 62 (for state transitions at SCL edges)
    localparam CLOCK_DIVIDE = 62;
    
    reg [7:0] r_Clock_Count;
    reg [1:0] r_Clock_Phase;  // 0=SCL low first half, 1=SCL high, 2=SCL low second half, 3=SCL high
    reg r_SCL;
    
    // I2C state machine
    localparam IDLE         = 4'd0;
    localparam START        = 4'd1;
    localparam ADDR_WRITE   = 4'd2;
    localparam ACK_ADDR     = 4'd3;
    localparam REG_ADDR     = 4'd4;
    localparam ACK_REG      = 4'd5;
    localparam RESTART      = 4'd6;
    localparam ADDR_READ    = 4'd7;
    localparam ACK_ADDR_R   = 4'd8;
    localparam READ_DATA    = 4'd9;
    localparam SEND_ACK     = 4'd10;
    localparam WRITE_DATA   = 4'd11;
    localparam ACK_DATA     = 4'd12;
    localparam STOP         = 4'd13;
    
    reg [3:0] r_State;
    reg [3:0] r_Next_State;
    reg [2:0] r_Bit_Count;
    reg [7:0] r_Shift_Reg;
    reg [7:0] r_Byte_Count;
    reg r_SDA_Out;
    reg r_SDA_Enable;  // Controls SDA tri-state (1=drive, 0=high-Z)
    
    // Tri-state control for SDA
    assign io_SDA = r_SDA_Enable ? r_SDA_Out : 1'bz;
    assign io_SCL = r_SCL;
    
    // Clock divider for I2C timing
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (!i_Rst_L) begin
            r_Clock_Count <= 0;
            r_Clock_Phase <= 0;
        end
        else begin
            if (r_State != IDLE) begin
                if (r_Clock_Count == CLOCK_DIVIDE - 1) begin
                    r_Clock_Count <= 0;
                    r_Clock_Phase <= r_Clock_Phase + 1;
                end
                else begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end
            end
            else begin
                r_Clock_Count <= 0;
                r_Clock_Phase <= 0;
            end
        end
    end
    
    // Generate SCL based on phase
    always @(*) begin
        if (r_State == IDLE || r_State == START || r_State == STOP) begin
            r_SCL = 1'b1;
        end
        else begin
            r_SCL = (r_Clock_Phase == 2'd1 || r_Clock_Phase == 2'd3);
        end
    end
    
    // Main state machine
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (!i_Rst_L) begin
            r_State <= IDLE;
            r_Bit_Count <= 0;
            r_Shift_Reg <= 0;
            r_Byte_Count <= 0;
            r_SDA_Out <= 1'b1;
            r_SDA_Enable <= 1'b0;
            o_Ready <= 1'b1;
            o_Data_Valid <= 1'b0;
            o_Read_Data <= 8'h00;
        end
        else begin
            o_Data_Valid <= 1'b0;  // Default, pulse high when data ready
            
            case (r_State)
                IDLE: begin
                    r_SDA_Out <= 1'b1;
                    r_SDA_Enable <= 1'b0;
                    o_Ready <= 1'b1;
                    if (i_Enable) begin
                        r_State <= START;
                        o_Ready <= 1'b0;
                        r_Byte_Count <= 0;
                    end
                end
                
                START: begin
                    // Generate START condition: SDA falls while SCL is high
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b1;
                        r_SDA_Out <= 1'b0;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= ADDR_WRITE;
                        r_Shift_Reg <= {i_Slave_Addr, 1'b0};  // Address + Write bit
                        r_Bit_Count <= 7;
                    end
                end
                
                ADDR_WRITE: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Out <= r_Shift_Reg[7];
                        r_SDA_Enable <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Bit_Count == 0) begin
                            r_State <= ACK_ADDR;
                        end
                        else begin
                            r_Shift_Reg <= {r_Shift_Reg[6:0], 1'b0};
                            r_Bit_Count <= r_Bit_Count - 1;
                        end
                    end
                end
                
                ACK_ADDR: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b0;  // Release SDA for ACK
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= REG_ADDR;
                        r_Shift_Reg <= i_Reg_Addr;
                        r_Bit_Count <= 7;
                    end
                end
                
                REG_ADDR: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Out <= r_Shift_Reg[7];
                        r_SDA_Enable <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Bit_Count == 0) begin
                            r_State <= ACK_REG;
                        end
                        else begin
                            r_Shift_Reg <= {r_Shift_Reg[6:0], 1'b0};
                            r_Bit_Count <= r_Bit_Count - 1;
                        end
                    end
                end
                
                ACK_REG: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b0;  // Release SDA for ACK
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (i_RW) begin
                            r_State <= RESTART;  // Read operation
                        end
                        else begin
                            r_State <= WRITE_DATA;  // Write operation
                            r_Shift_Reg <= i_Write_Data;
                            r_Bit_Count <= 7;
                        end
                    end
                end
                
                RESTART: begin
                    // Generate repeated START
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b1;
                        r_SDA_Out <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd1 && r_Clock_Count == CLOCK_DIVIDE/2) begin
                        r_SDA_Out <= 1'b0;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= ADDR_READ;
                        r_Shift_Reg <= {i_Slave_Addr, 1'b1};  // Address + Read bit
                        r_Bit_Count <= 7;
                    end
                end
                
                ADDR_READ: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Out <= r_Shift_Reg[7];
                        r_SDA_Enable <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Bit_Count == 0) begin
                            r_State <= ACK_ADDR_R;
                        end
                        else begin
                            r_Shift_Reg <= {r_Shift_Reg[6:0], 1'b0};
                            r_Bit_Count <= r_Bit_Count - 1;
                        end
                    end
                end
                
                ACK_ADDR_R: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b0;  // Release SDA for ACK
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= READ_DATA;
                        r_Bit_Count <= 7;
                        r_Shift_Reg <= 8'h00;
                    end
                end
                
                READ_DATA: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b0;  // Release SDA to read
                    end
                    if (r_Clock_Phase == 2'd1 && r_Clock_Count == CLOCK_DIVIDE/2) begin
                        r_Shift_Reg <= {r_Shift_Reg[6:0], io_SDA};
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Bit_Count == 0) begin
                            r_State <= SEND_ACK;
                            o_Read_Data <= {r_Shift_Reg[6:0], io_SDA};
                            o_Data_Valid <= 1'b1;
                            r_Byte_Count <= r_Byte_Count + 1;
                        end
                        else begin
                            r_Bit_Count <= r_Bit_Count - 1;
                        end
                    end
                end
                
                SEND_ACK: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b1;
                        // ACK (0) if more bytes, NACK (1) if last byte
                        r_SDA_Out <= (r_Byte_Count >= i_Num_Bytes) ? 1'b1 : 1'b0;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Byte_Count >= i_Num_Bytes) begin
                            r_State <= STOP;
                        end
                        else begin
                            r_State <= READ_DATA;
                            r_Bit_Count <= 7;
                            r_Shift_Reg <= 8'h00;
                        end
                    end
                end
                
                WRITE_DATA: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Out <= r_Shift_Reg[7];
                        r_SDA_Enable <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        if (r_Bit_Count == 0) begin
                            r_State <= ACK_DATA;
                        end
                        else begin
                            r_Shift_Reg <= {r_Shift_Reg[6:0], 1'b0};
                            r_Bit_Count <= r_Bit_Count - 1;
                        end
                    end
                end
                
                ACK_DATA: begin
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b0;  // Release SDA for ACK
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= STOP;
                    end
                end
                
                STOP: begin
                    // Generate STOP condition: SDA rises while SCL is high
                    if (r_Clock_Phase == 2'd0 && r_Clock_Count == 0) begin
                        r_SDA_Enable <= 1'b1;
                        r_SDA_Out <= 1'b0;
                    end
                    if (r_Clock_Phase == 2'd1 && r_Clock_Count == CLOCK_DIVIDE/2) begin
                        r_SDA_Out <= 1'b1;
                    end
                    if (r_Clock_Phase == 2'd3 && r_Clock_Count == CLOCK_DIVIDE - 1) begin
                        r_State <= IDLE;
                        r_SDA_Enable <= 1'b0;
                    end
                end
                
                default: r_State <= IDLE;
            endcase
        end
    end

endmodule
