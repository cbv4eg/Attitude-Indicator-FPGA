`timescale 1ns / 10ps

module i2c_timing_tb;

// Clock and reset
reg i_clk;
reg i_rst;

// Command interface
reg [7:0] tx_data;
wire [7:0] rx_data;
reg [2:0] i_cmd;
reg i_cmd_valid;
wire o_cmd_done;
wire o_cmd_error;

// I2C physical interface
wire io_sda;
wire io_scl;

// Pull-up resistors (critical for I2C!)
pullup(io_sda);
pullup(io_scl);

// I2C slave emulation signals
reg sda_slave_drive;

// Command encoding
localparam CMD_IDLE      = 3'd0;
localparam CMD_START     = 3'd1;
localparam CMD_WRITE     = 3'd2;
localparam CMD_READ      = 3'd3;
localparam CMD_READ_ACK  = 3'd4;
localparam CMD_READ_NACK = 3'd5;
localparam CMD_STOP      = 3'd6;

// Test parameters - simulating BNO055 sensor
localparam SLAVE_ADDR_W = 8'h52;  // BNO055 write address (0x29 << 1)
localparam SLAVE_ADDR_R = 8'h53;  // BNO055 read address
localparam EULER_H_LSB = 8'h1A;   // Euler heading LSB register
localparam EULER_H_MSB = 8'h1B;   // Euler heading MSB register

// Slave memory to simulate sensor registers
reg [7:0] slave_mem [0:255];

// Instantiate DUT
i2c_master_interface dut (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .tx_data(tx_data),
    .rx_data(rx_data),
    .i_cmd(i_cmd),
    .i_cmd_valid(i_cmd_valid),
    .o_cmd_done(o_cmd_done),
    .o_cmd_error(o_cmd_error),
    .io_sda(io_sda),
    .io_scl(io_scl)
);

// Slave SDA drive (open-drain)
assign io_sda = sda_slave_drive ? 1'b0 : 1'bz;

// Clock generation: 25 MHz
initial begin
    i_clk = 0;
    forever #20 i_clk = ~i_clk;  // 40ns period
end

// Initialize slave memory with test data
initial begin
    integer i;
    for (i = 0; i < 256; i = i + 1) begin
        slave_mem[i] = 8'h00;
    end
    // Simulate BNO055 Euler angle data
    slave_mem[EULER_H_LSB] = 8'hAB;  // Heading LSB
    slave_mem[EULER_H_MSB] = 8'hCD;  // Heading MSB
    slave_mem[8'h1C] = 8'h12;        // Roll LSB
    slave_mem[8'h1D] = 8'h34;        // Roll MSB
    slave_mem[8'h1E] = 8'h56;        // Pitch LSB
    slave_mem[8'h1F] = 8'h78;        // Pitch MSB
end

// Task to send command and wait for completion
task send_command;
    input [2:0] cmd;
    input [7:0] data;
    integer timeout;
    begin
        timeout = 0;
        @(posedge i_clk);
        i_cmd = cmd;
        tx_data = data;
        i_cmd_valid = 1'b1;
        @(posedge i_clk);
        i_cmd_valid = 1'b0;
        
        // Wait for completion with timeout
        while (!o_cmd_done && timeout < 100000) begin
            @(posedge i_clk);
            timeout = timeout + 1;
        end
        
        if (timeout >= 100000) begin
            $display("ERROR: Command timeout at time %0t", $time);
            $finish;
        end
        
        @(posedge i_clk);
    end
endtask

// Slave state machine
reg [2:0] slave_state;
reg [3:0] slave_bit_count;
reg [7:0] slave_shift_reg;
reg [7:0] slave_reg_addr;
reg slave_is_read;

localparam SLAVE_IDLE = 3'd0;
localparam SLAVE_ADDR = 3'd1;
localparam SLAVE_ACK  = 3'd2;
localparam SLAVE_RX_DATA = 3'd3;
localparam SLAVE_TX_DATA = 3'd4;

initial begin
    sda_slave_drive = 0;
    slave_state = SLAVE_IDLE;
    slave_bit_count = 0;
    slave_shift_reg = 0;
    slave_reg_addr = 0;
    slave_is_read = 0;
end

// Detect START condition
always @(negedge io_sda) begin
    if (io_scl) begin
        // START condition detected
        slave_state = SLAVE_ADDR;
        slave_bit_count = 0;
        slave_shift_reg = 0;
        sda_slave_drive = 0;
        $display("Time %0t: Slave detected START", $time);
    end
end

// Detect STOP condition
always @(posedge io_sda) begin
    if (io_scl) begin
        // STOP condition detected
        slave_state = SLAVE_IDLE;
        sda_slave_drive = 0;
        $display("Time %0t: Slave detected STOP", $time);
    end
end

// Slave receiving data on SCL rising edge
always @(posedge io_scl) begin
    case (slave_state)
        SLAVE_ADDR, SLAVE_RX_DATA: begin
            if (slave_bit_count < 8) begin
                slave_shift_reg = {slave_shift_reg[6:0], io_sda};
                slave_bit_count = slave_bit_count + 1;
            end
        end
        
        SLAVE_TX_DATA: begin
            // During transmit, just increment bit counter
            if (slave_bit_count < 8) begin
                slave_bit_count = slave_bit_count + 1;
            end
        end
    endcase
end

// Slave ACK/data output on SCL falling edge
always @(negedge io_scl) begin
    case (slave_state)
        SLAVE_ADDR: begin
            if (slave_bit_count == 8) begin
                // Check if read or write operation
                slave_is_read = slave_shift_reg[0];
                $display("Time %0t: Slave received address 0x%h, R/W=%b", 
                         $time, slave_shift_reg, slave_is_read);
                
                // Send ACK
                sda_slave_drive = 1;
                slave_state = SLAVE_ACK;
                slave_bit_count = 0;
            end
        end
        
        SLAVE_ACK: begin
            sda_slave_drive = 0;  // Release SDA after ACK
            
            if (slave_is_read) begin
                // Master wants to read - prepare first byte
                slave_shift_reg = slave_mem[slave_reg_addr];
                slave_state = SLAVE_TX_DATA;
                slave_bit_count = 0;
                $display("Time %0t: Slave will send data 0x%h from addr 0x%h", 
                         $time, slave_shift_reg, slave_reg_addr);
            end else begin
                // Master wants to write
                slave_state = SLAVE_RX_DATA;
                slave_shift_reg = 0;
            end
        end
        
        SLAVE_RX_DATA: begin
            if (slave_bit_count == 8) begin
                // Received a byte - store as register address
                slave_reg_addr = slave_shift_reg;
                $display("Time %0t: Slave received reg address 0x%h", 
                         $time, slave_reg_addr);
                
                // Send ACK
                sda_slave_drive = 1;
                slave_state = SLAVE_ACK;
                slave_bit_count = 0;
                slave_shift_reg = 0;
            end
        end
        
        SLAVE_TX_DATA: begin
            if (slave_bit_count < 8) begin
                // Drive next data bit (MSB first)
                sda_slave_drive = !slave_shift_reg[7 - slave_bit_count];
            end else begin
                // 8 bits sent, release SDA for master's ACK/NACK
                sda_slave_drive = 0;
                slave_state = SLAVE_ACK;
                slave_bit_count = 0;
                
                // Prepare next byte
                slave_reg_addr = slave_reg_addr + 1;
                slave_shift_reg = slave_mem[slave_reg_addr];
            end
        end
    endcase
end

// Main test sequence - READ operations
initial begin
    // Initialize
    i_rst = 1'b1;
    i_cmd = CMD_IDLE;
    tx_data = 8'h00;
    i_cmd_valid = 1'b0;

    $dumpvars(0, i2c_timing_tb);
    
    #200;
    i_rst = 1'b0;
    #500;
    
    $display("\n=== Test 1: Single Byte Read (with Repeated START) ===");
    
    // START + Write address to set register pointer
    send_command(CMD_START, SLAVE_ADDR_W);
    $display("Sent START + Write Address 0x%h, Error=%b", SLAVE_ADDR_W, o_cmd_error);
    #1000;
    
    // Write register address
    send_command(CMD_WRITE, EULER_H_LSB);
    $display("Wrote register address 0x%h, Error=%b", EULER_H_LSB, o_cmd_error);
    #1000;
    
    // Repeated START + Read address
    send_command(CMD_START, SLAVE_ADDR_R);
    $display("Sent repeated START + Read Address 0x%h, Error=%b", SLAVE_ADDR_R, o_cmd_error);
    #1000;
    
    // Read single byte with NACK (last byte)
    send_command(CMD_READ_NACK, 8'h00);
    $display("Read data: 0x%h, Error=%b", rx_data, o_cmd_error);
    #1000;
    
    // STOP
    send_command(CMD_STOP, 8'h00);
    $display("STOP sent\n");
    #5000;
    
    $display("\n=== Test 2: Multi-Byte Read (6 bytes - all Euler angles) ===");
    
    // START + Write address
    send_command(CMD_START, SLAVE_ADDR_W);
    $display("Sent START + Write Address");
    #1000;
    
    // Write register address
    send_command(CMD_WRITE, EULER_H_LSB);
    $display("Wrote starting register address 0x%h", EULER_H_LSB);
    #1000;
    
    // Repeated START + Read address
    send_command(CMD_START, SLAVE_ADDR_R);
    $display("Sent repeated START + Read Address");
    #1000;
    
    // Read 6 bytes (Heading, Roll, Pitch - LSB and MSB each)
    send_command(CMD_READ_ACK, 8'h00);
    $display("Byte 0 (Heading LSB): 0x%h", rx_data);
    #1000;
    
    send_command(CMD_READ_ACK, 8'h00);
    $display("Byte 1 (Heading MSB): 0x%h", rx_data);
    #1000;
    
    send_command(CMD_READ_ACK, 8'h00);
    $display("Byte 2 (Roll LSB): 0x%h", rx_data);
    #1000;
    
    send_command(CMD_READ_ACK, 8'h00);
    $display("Byte 3 (Roll MSB): 0x%h", rx_data);
    #1000;
    
    send_command(CMD_READ_ACK, 8'h00);
    $display("Byte 4 (Pitch LSB): 0x%h", rx_data);
    #1000;
    
    send_command(CMD_READ_NACK, 8'h00);  // Last byte gets NACK
    $display("Byte 5 (Pitch MSB): 0x%h", rx_data);
    #1000;
    
    // STOP
    send_command(CMD_STOP, 8'h00);
    $display("STOP sent - Read complete\n");
    #5000;
    
    $display("\n=== Test 3: Direct Read (no register address) ===");
    
    // Some I2C devices support direct read from current pointer
    send_command(CMD_START, SLAVE_ADDR_R);
    $display("Sent START + Read Address");
    #1000;
    
    send_command(CMD_READ_NACK, 8'h00);
    $display("Read data: 0x%h", rx_data);
    #1000;
    
    send_command(CMD_STOP, 8'h00);
    $display("STOP sent\n");
    #5000;
    
    $display("\n=== All Read Tests Complete ===");
    $finish;
end

// Timeout watchdog
initial begin
    #1000000;  // 1ms timeout
    $display("ERROR: Testbench timeout!");
    $finish;
end

endmodule
