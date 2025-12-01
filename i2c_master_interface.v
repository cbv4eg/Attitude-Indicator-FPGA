/////////////////////////////////////////////////////////////////////
////                                                             ////
////  I2C Interface for Nandland Go Board                        ////
////  Modified from WISHBONE I2C Master controller               ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

// `timescale 1ns / 10ps

module i2c_master_interface (
    input wire i_clk,
    input wire i_rst,
    
    // Simple command interface
    input wire [7:0] tx_data,     // Data to send
    output reg [7:0] rx_data = 8'h00,      // Data received
    input wire [2:0] i_cmd,          // Command type
    input wire i_cmd_valid,          // Start command
    output reg o_cmd_done = 1'b0,           // Command complete
    output reg o_cmd_error = 1'b0,          // NACK or arbitration lost
    
    // I2C physical interface
    inout wire io_sda,
    inout wire io_scl
);

// Command encoding
localparam CMD_IDLE      = 3'd0;
localparam CMD_START     = 3'd1;
localparam CMD_WRITE     = 3'd2;
localparam CMD_READ      = 3'd3;
localparam CMD_READ_ACK  = 3'd4;  // Read and send ACK
localparam CMD_READ_NACK = 3'd5;  // Read and send NACK
localparam CMD_STOP      = 3'd6;

// Internal signals for byte controller
reg byte_start = 1'b0, byte_stop = 1'b0, byte_read = 1'b0, byte_write = 1'b0, byte_ack_in = 1'b0;
reg [7:0] byte_din = 8'h00;
wire byte_cmd_ack, byte_ack_out, byte_busy, byte_al;
wire [7:0] byte_dout;

// I2C bidirectional signals
wire scl_i, scl_o, scl_oen;
wire sda_i, sda_o, sda_oen;

// Clock prescaler for 100 kHz I2C; for 25 MHz: clk_cnt = 50
localparam CLK_PRESCALE = 16'd50;

// Instantiate byte controller
i2c_master_byte_ctrl byte_contoller (
    .clk        (i_clk),              
    .rst        (1'b0),             // use async below instead
    .nReset     (~i_rst),             // active low, invert
    .ena        (1'b1),             // always enable controller
    .clk_cnt    (CLK_PRESCALE),
    .start      (byte_start),       // single clk cycle pulse to start
    .stop       (byte_stop),        // end of command
    .read       (byte_read),        // READ cmd
    .write      (byte_write),       // WRITE cmd
    .ack_in     (byte_ack_in),      // when master is receiving, sends 0=ACK, 1=NACK to slave
    .din        (byte_din),
    .cmd_ack    (byte_cmd_ack),     // from slave
    .ack_out    (byte_ack_out),     // when slave is receiving, sends 0=ACK, 1=NACK to master
    .dout       (byte_dout),
    .i2c_busy   (byte_busy),
    .i2c_al     (byte_al),
    .scl_i      (io_scl),
    .scl_o      (scl_o),            // tied to gnd inside bit_ctrl
    .scl_oen    (scl_oen),
    .sda_i      (io_sda),
    .sda_o      (sda_o),            // tied to gnd inside bit_ctrl
    .sda_oen    (sda_oen)
);

// Bidirectional buffer for SCL, active low
assign io_scl = scl_oen ? 1'bz : scl_o;
assign scl_i = io_scl;

// Bidirectional buffer for SDA, active low
assign io_sda = sda_oen ? 1'bz : sda_o;
assign sda_i = io_sda;

// Control state machine
localparam ST_IDLE = 3'd0;
localparam ST_BUSY = 3'd1;
localparam ST_DONE = 3'd2;
reg [2:0] state = ST_IDLE;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= ST_IDLE;
        o_cmd_done <= 1'b0;
        o_cmd_error <= 1'b0;
        byte_start <= 1'b0;
        byte_stop <= 1'b0;
        byte_read <= 1'b0;
        byte_write <= 1'b0;
        byte_ack_in <= 1'b0;
        rx_data <= 8'h00;
    end else begin
        // Defaults
        byte_start <= 1'b0;
        byte_stop <= 1'b0;
        byte_read <= 1'b0;
        byte_write <= 1'b0;
        o_cmd_done <= 1'b0;
        
        case (state)
            ST_IDLE: begin
                if (i_cmd_valid) begin
                    o_cmd_error <= 1'b0;
                    
                    case (i_cmd)
                        CMD_START: begin
                            byte_start <= 1'b1;
                            byte_write <= 1'b1;
                            byte_din <= tx_data;
                            state <= ST_BUSY;
                        end
                        
                        CMD_WRITE: begin
                            byte_write <= 1'b1;
                            byte_din <= tx_data;
                            state <= ST_BUSY;
                        end
                        
                        CMD_READ_ACK: begin
                            byte_read <= 1'b1;
                            byte_ack_in <= 1'b0;  // Send ACK
                            state <= ST_BUSY;
                        end
                        
                        CMD_READ_NACK: begin
                            byte_read <= 1'b1;
                            byte_ack_in <= 1'b1;  // Send NACK
                            state <= ST_BUSY;
                        end
                        
                        CMD_STOP: begin
                            byte_stop <= 1'b1;
                            byte_read <= 1'b1;  // Dummy read
                            state <= ST_BUSY;
                        end

                        default: begin
                            state <= ST_BUSY;
                        end
                    endcase
                end
            end
            
            ST_BUSY: begin
                if (byte_cmd_ack) begin
                    rx_data <= byte_dout;
                    o_cmd_error <= byte_ack_out | byte_al;  // NACK or arb lost
                    state <= ST_DONE;
                end
            end
            
            ST_DONE: begin
                o_cmd_done <= 1'b1;
                state <= ST_IDLE;
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule