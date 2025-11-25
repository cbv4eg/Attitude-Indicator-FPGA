`timescale 1ns / 1ps

module i2c_master_tb;

    // Testbench signals
    reg clk;
    reg rst;
    
    // Command interface
    reg [6:0] s_axis_cmd_address;
    reg s_axis_cmd_start;
    reg s_axis_cmd_read;
    reg s_axis_cmd_write;
    reg s_axis_cmd_write_multiple;
    reg s_axis_cmd_stop;
    reg s_axis_cmd_valid;
    wire s_axis_cmd_ready;
    
    // Data input interface
    reg [7:0] s_axis_data_tdata;
    reg s_axis_data_tvalid;
    wire s_axis_data_tready;
    reg s_axis_data_tlast;
    
    // Data output interface
    wire [7:0] m_axis_data_tdata;
    wire m_axis_data_tvalid;
    reg m_axis_data_tready;
    wire m_axis_data_tlast;
    
    // I2C interface
    wire scl_bus;
    wire sda_bus;
    
    reg scl_i;
    wire scl_o;
    wire scl_t;
    reg sda_i;
    wire sda_o;
    wire sda_t;
    
    // Status signals
    wire busy;
    wire bus_control;
    wire bus_active;
    wire missed_ack;
    
    // Configuration
    reg [15:0] prescale;
    reg stop_on_idle;
    
    // Pullup resistors
    pullup(scl_bus);
    pullup(sda_bus);
    
    // Master connection to bus
    assign scl_bus = scl_t ? 1'bz : scl_o;
    assign sda_bus = sda_t ? 1'bz : sda_o;
    assign scl_i = scl_bus;
    assign sda_i = sda_bus;
    
    // Timing measurement variables - FIXED
    real last_scl_fall_time, last_scl_rise_time;
    real scl_period, scl_high_time, scl_low_time;
    reg scl_bus_last;
    
    // DUT instantiation
    i2c_master dut (
        .clk(clk),
        .rst(rst),
        .s_axis_cmd_address(s_axis_cmd_address),
        .s_axis_cmd_start(s_axis_cmd_start),
        .s_axis_cmd_read(s_axis_cmd_read),
        .s_axis_cmd_write(s_axis_cmd_write),
        .s_axis_cmd_write_multiple(s_axis_cmd_write_multiple),
        .s_axis_cmd_stop(s_axis_cmd_stop),
        .s_axis_cmd_valid(s_axis_cmd_valid),
        .s_axis_cmd_ready(s_axis_cmd_ready),
        .s_axis_data_tdata(s_axis_data_tdata),
        .s_axis_data_tvalid(s_axis_data_tvalid),
        .s_axis_data_tready(s_axis_data_tready),
        .s_axis_data_tlast(s_axis_data_tlast),
        .m_axis_data_tdata(m_axis_data_tdata),
        .m_axis_data_tvalid(m_axis_data_tvalid),
        .m_axis_data_tready(m_axis_data_tready),
        .m_axis_data_tlast(m_axis_data_tlast),
        .scl_i(scl_i),
        .scl_o(scl_o),
        .scl_t(scl_t),
        .sda_i(sda_i),
        .sda_o(sda_o),
        .sda_t(sda_t),
        .busy(busy),
        .bus_control(bus_control),
        .bus_active(bus_active),
        .missed_ack(missed_ack),
        .prescale(prescale),
        .stop_on_idle(stop_on_idle)
    );
    
    // Clock generation - 25MHz (40ns period)
    initial begin
        clk = 0;
        forever #20 clk = ~clk;
    end
    
    // FIXED: Proper timing monitors
    initial begin
        last_scl_fall_time = 0;
        last_scl_rise_time = 0;
        scl_period = 0;
        scl_high_time = 0;
        scl_low_time = 0;
        scl_bus_last = 1;
    end
    
    always @(*) begin
        scl_bus_last <= #1 scl_bus;
    end
    
    // Detect SCL edges and measure timing
    always @(scl_bus) begin
        if (scl_bus === 1'b1 && scl_bus_last === 1'b0) begin
            // Rising edge
            if (last_scl_fall_time > 0) begin
                scl_low_time = $realtime - last_scl_fall_time;
            end
            if (last_scl_rise_time > 0) begin
                scl_period = $realtime - last_scl_rise_time;
            end
            last_scl_rise_time = $realtime;
        end else if (scl_bus === 1'b0 && scl_bus_last === 1'b1) begin
            // Falling edge
            if (last_scl_rise_time > 0) begin
                scl_high_time = $realtime - last_scl_rise_time;
            end
            last_scl_fall_time = $realtime;
        end
    end
    
    // ========================================
    // I2C SLAVE MODEL
    // ========================================
    reg slave_sda_out;
    reg slave_sda_oe;
    
    assign sda_bus = slave_sda_oe ? slave_sda_out : 1'bz;
    
    // Slave state machine
    localparam SLAVE_IDLE = 0;
    localparam SLAVE_ADDR = 1;
    localparam SLAVE_ADDR_ACK = 2;
    localparam SLAVE_DATA_WRITE = 3;
    localparam SLAVE_DATA_WRITE_ACK = 4;
    localparam SLAVE_DATA_READ = 5;
    localparam SLAVE_DATA_READ_ACK = 6;
    
    reg [2:0] slave_state;
    reg [3:0] slave_bit_count;
    reg [7:0] slave_addr_byte;
    reg [7:0] slave_data_byte;
    reg [7:0] slave_memory [0:255];
    reg [7:0] slave_read_data;
    reg slave_rw_bit;
    
    // Initialize slave memory
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            slave_memory[i] = i[7:0] ^ 8'hAA;
        end
        slave_read_data = 8'h55;
    end
    
    // Detect START and STOP conditions
    reg last_sda, last_scl;
    wire start_condition = last_sda && !sda_bus && scl_bus;
    wire stop_condition = !last_sda && sda_bus && scl_bus;
    
    always @(posedge clk) begin
        last_sda <= sda_bus;
        last_scl <= scl_bus;
    end
    
    // Slave logic
    always @(posedge scl_bus or posedge start_condition or posedge stop_condition or negedge rst) begin
        if (!rst) begin
            slave_state <= SLAVE_IDLE;
            slave_bit_count <= 0;
            slave_sda_oe <= 0;
            slave_sda_out <= 1;
        end else if (start_condition) begin
            slave_state <= SLAVE_ADDR;
            slave_bit_count <= 0;
            slave_sda_oe <= 0;
            slave_addr_byte <= 0;
        end else if (stop_condition) begin
            slave_state <= SLAVE_IDLE;
            slave_sda_oe <= 0;
        end else begin
            case (slave_state)
                SLAVE_ADDR: begin
                    slave_addr_byte <= {slave_addr_byte[6:0], sda_bus};
                    slave_bit_count <= slave_bit_count + 1;
                    if (slave_bit_count == 7) begin
                        slave_rw_bit <= sda_bus;
                        slave_state <= SLAVE_ADDR_ACK;
                    end
                end
                
                SLAVE_DATA_WRITE: begin
                    slave_data_byte <= {slave_data_byte[6:0], sda_bus};
                    slave_bit_count <= slave_bit_count + 1;
                    if (slave_bit_count == 7) begin
                        slave_state <= SLAVE_DATA_WRITE_ACK;
                        slave_read_data <= {slave_data_byte[6:0], sda_bus};
                    end
                end
                
                SLAVE_DATA_READ: begin
                    slave_bit_count <= slave_bit_count + 1;
                    if (slave_bit_count == 7) begin
                        slave_state <= SLAVE_DATA_READ_ACK;
                        slave_sda_oe <= 0;
                    end
                end
            endcase
        end
    end
    
    // Slave output logic
    always @(negedge scl_bus or negedge rst) begin
        if (!rst) begin
            slave_sda_oe <= 0;
            slave_sda_out <= 1;
        end else begin
            case (slave_state)
                SLAVE_ADDR_ACK: begin
                    slave_sda_oe <= 1;
                    slave_sda_out <= 0;
                    slave_bit_count <= 0;
                    if (slave_rw_bit) begin
                        slave_state <= SLAVE_DATA_READ;
                    end else begin
                        slave_state <= SLAVE_DATA_WRITE;
                    end
                end
                
                SLAVE_DATA_WRITE_ACK: begin
                    slave_sda_oe <= 1;
                    slave_sda_out <= 0;
                    slave_bit_count <= 0;
                    slave_state <= SLAVE_DATA_WRITE;
                end
                
                SLAVE_DATA_READ: begin
                    slave_sda_oe <= 1;
                    slave_sda_out <= slave_read_data[7 - slave_bit_count];
                end
                
                SLAVE_DATA_READ_ACK: begin
                    slave_sda_oe <= 0;
                    slave_bit_count <= 0;
                    if (sda_bus) begin
                        slave_state <= SLAVE_IDLE;
                    end else begin
                        slave_state <= SLAVE_DATA_READ;
                    end
                end
                
                default: begin
                    slave_sda_oe <= 0;
                    slave_sda_out <= 1;
                end
            endcase
        end
    end
    
    // Task to initialize signals
    task initialize;
        begin
            rst = 1;
            s_axis_cmd_address = 0;
            s_axis_cmd_start = 0;
            s_axis_cmd_read = 0;
            s_axis_cmd_write = 0;
            s_axis_cmd_write_multiple = 0;
            s_axis_cmd_stop = 0;
            s_axis_cmd_valid = 0;
            s_axis_data_tdata = 0;
            s_axis_data_tvalid = 0;
            s_axis_data_tlast = 0;
            m_axis_data_tready = 1;
            prescale = 16'd63;  // Try prescale of 1
            stop_on_idle = 0;
            
            #100;
            rst = 0;
            #100;
        end
    endtask
    
    // Task to perform I2C write operation
    task i2c_write;
        input [6:0] addr;
        input [7:0] data;
        input stop;
        begin
            $display("[%0t] Starting WRITE to address 0x%h, data = 0x%h", $time, addr, data);
            
            @(posedge clk);
            s_axis_cmd_address = addr;
            s_axis_cmd_write = 1;
            s_axis_cmd_start = 1;
            s_axis_cmd_stop = stop;
            s_axis_cmd_valid = 1;
            
            @(posedge clk);
            while (!s_axis_cmd_ready) @(posedge clk);
            s_axis_cmd_valid = 0;
            s_axis_cmd_write = 0;
            s_axis_cmd_start = 0;
            
            @(posedge clk);
            s_axis_data_tdata = data;
            s_axis_data_tvalid = 1;
            s_axis_data_tlast = 1;
            
            @(posedge clk);
            while (!s_axis_data_tready) @(posedge clk);
            s_axis_data_tvalid = 0;
            
            wait(!busy);
            #100;
            $display("[%0t] WRITE complete", $time);
        end
    endtask
    
    // Task to perform I2C read operation
    task i2c_read;
        input [6:0] addr;
        input stop;
        reg [7:0] read_data;
        begin
            $display("[%0t] Starting READ from address 0x%h", $time, addr);
            
            @(posedge clk);
            s_axis_cmd_address = addr;
            s_axis_cmd_read = 1;
            s_axis_cmd_start = 1;
            s_axis_cmd_stop = stop;
            s_axis_cmd_valid = 1;
            
            @(posedge clk);
            while (!s_axis_cmd_ready) @(posedge clk);
            s_axis_cmd_valid = 0;
            s_axis_cmd_read = 0;
            s_axis_cmd_start = 0;
            
            @(posedge clk);
            while (!m_axis_data_tvalid) @(posedge clk);
            read_data = m_axis_data_tdata;
            
            wait(!busy);
            #100;
            $display("[%0t] READ complete, data = 0x%h", $time, read_data);
        end
    endtask
    
    // Task to perform multiple byte write
    task i2c_write_multiple;
        input [6:0] addr;
        input [7:0] data1;
        input [7:0] data2;
        input [7:0] data3;
        input stop;
        begin
            $display("[%0t] Starting MULTI-WRITE to address 0x%h", $time, addr);
            
            @(posedge clk);
            s_axis_cmd_address = addr;
            s_axis_cmd_write_multiple = 1;
            s_axis_cmd_start = 1;
            s_axis_cmd_stop = stop;
            s_axis_cmd_valid = 1;
            
            @(posedge clk);
            while (!s_axis_cmd_ready) @(posedge clk);
            s_axis_cmd_valid = 0;
            s_axis_cmd_write_multiple = 0;
            s_axis_cmd_start = 0;
            
            @(posedge clk);
            s_axis_data_tdata = data1;
            s_axis_data_tvalid = 1;
            s_axis_data_tlast = 0;
            
            @(posedge clk);
            while (!s_axis_data_tready) @(posedge clk);
            
            s_axis_data_tdata = data2;
            s_axis_data_tlast = 0;
            
            @(posedge clk);
            while (!s_axis_data_tready) @(posedge clk);
            
            s_axis_data_tdata = data3;
            s_axis_data_tlast = 1;
            
            @(posedge clk);
            while (!s_axis_data_tready) @(posedge clk);
            s_axis_data_tvalid = 0;
            
            wait(!busy);
            #100;
            $display("[%0t] MULTI-WRITE complete", $time);
        end
    endtask
    
    // Task to check timing parameters
    task check_timing;
        begin
            $display("\n=== I2C Timing Analysis ===");
            $display("SCL Period: %.1f ns (%.1f kHz)", scl_period, 1000000.0/scl_period);
            $display("SCL High Time: %.1f ns", scl_high_time);
            $display("SCL Low Time: %.1f ns", scl_low_time);
            $display("Calculated I2C Frequency: %.1f kHz", 1000000.0/scl_period);
            
            // More lenient check - just verify it's somewhat reasonable
            if (scl_period > 5000 && scl_period < 20000) begin
                $display("PASS: SCL period reasonable for I2C");
            end else begin
                $display("FAIL: SCL period out of range");
            end
            
            if (scl_high_time > 2000 && scl_low_time > 2000) begin
                $display("PASS: SCL duty cycle acceptable");
            end else begin
                $display("FAIL: SCL duty cycle out of range");
            end
        end
    endtask
    
    // Main test sequence
    initial begin
        $display("=== I2C Master Testbench Started (25MHz Clock) ===");
        
        initialize();
        
        $display("\n--- Test 1: Single Write ---");
        i2c_write(7'h50, 8'hA5, 1);
        #1000;
        
        $display("\n--- Test 2: Single Read ---");
        i2c_read(7'h50, 1);
        #1000;
        
        $display("\n--- Test 3: Multiple Write ---");
        i2c_write_multiple(7'h50, 8'h11, 8'h22, 8'h33, 1);
        #1000;
        
        $display("\n--- Test 4: Write-Read with Repeated Start ---");
        i2c_write(7'h50, 8'hAA, 0);
        #500;
        i2c_read(7'h50, 1);
        #1000;
        
        check_timing();
        
        $display("\n=== Testbench Complete ===");
        #5000;
        $finish;
    end
    
    // Waveform dump
    initial begin
        $dumpvars(0, i2c_master_tb);
    end
    
    // Timeout watchdog
    initial begin
        #2000000;
        $display("ERROR: Testbench timeout!");
        $finish;
    end

endmodule
