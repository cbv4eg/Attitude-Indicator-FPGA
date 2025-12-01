/////////////////////////////////////////////////////////////////////
////                                                             ////
////  BNO055 I2C Read using Nandland Go Board                    ////
////  Modified from WISHBONE I2C Master controller               ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

`timescale 1ns / 10ps

module bno055_read (
    input wire          i_clk,
    input wire          i_rst,
    input wire [7:0]    i_reg_addr,
    input wire          i_read_chip_id, // start chip id read op
    output reg [7:0]    o_chip_id = 8'h00,
    output reg          o_done = 1'b0,
    inout wire          io_sda,
    inout wire          io_scl
);

// IMU Device Address
localparam BNO055_ADDR_WRITE = 8'h50;  // 0x28 << 1 | 0
localparam BNO055_ADDR_READ  = 8'h51;  // 0x28 << 1 | 1

// Command encoding (from lower level, can we make this in a header files somehow to standardize across and save some GB space?)
localparam CMD_IDLE      = 3'd0;
localparam CMD_START     = 3'd1;
localparam CMD_WRITE     = 3'd2;
localparam CMD_READ      = 3'd3;
localparam CMD_READ_ACK  = 3'd4;  // Read and send ACK
localparam CMD_READ_NACK = 3'd5;  // Read and send NACK
localparam CMD_STOP      = 3'd6;

// I2C interface signals (lower level)
reg [7:0] tx_data = 8'h00;
wire [7:0] rx_data;
reg [2:0] cmd = CMD_IDLE;
reg cmd_valid = 1'b0;
wire cmd_done;
wire cmd_error;

i2c_master_interface i2c (
    .i_clk          (i_clk),
    .i_rst          (i_rst),
    .tx_data        (tx_data),
    .rx_data        (rx_data),
    .i_cmd          (cmd),
    .i_cmd_valid    (cmd_valid),
    .o_cmd_done     (cmd_done),
    .o_cmd_error    (cmd_error),
    .io_sda         (io_sda),
    .io_scl         (io_scl)
);

// Transaction state machine: sets up command, waits until cmd_done, continues
reg [3:0] state = ST_IDLE;
localparam ST_IDLE        = 4'd0; // init wait
localparam ST_START       = 4'd1;
localparam ST_START_WAIT  = 4'd2;
localparam ST_WRITE_REG   = 4'd3;
localparam ST_WRITE_WAIT  = 4'd4;
localparam ST_RESTART     = 4'd5;
localparam ST_RESTART_WAIT = 4'd6;
localparam ST_READ_DATA   = 4'd7;
localparam ST_READ_WAIT   = 4'd8;
localparam ST_STOP        = 4'd9;
localparam ST_STOP_WAIT   = 4'd10;
localparam ST_DONE        = 4'd11;

always @(posedge i_clk) begin
    if (i_rst) begin
        state <= ST_IDLE;
        cmd_valid <= 1'b0;
        o_done <= 1'b0;
    end else begin
        cmd_valid <= 1'b0;  // Default: don't pulse
        o_done <= 1'b0;

        case (state)
            ST_IDLE: begin
                if (i_read_chip_id) begin
                    state <= ST_START;
                end
            end

            ST_START: begin
                cmd <= CMD_START;
                tx_data <= BNO055_ADDR_WRITE;
                cmd_valid <= 1'b1;
                state <= ST_START_WAIT;
            end

            ST_START_WAIT: begin
                if (cmd_done) begin
                    state <= (!cmd_error) ? ST_WRITE_REG : ST_IDLE; // Error, retry
                end
            end

            ST_WRITE_REG: begin
                cmd <= CMD_WRITE;
                tx_data <= i_reg_addr;
                cmd_valid <= 1'b1;
                state <= ST_WRITE_WAIT;
            end

            ST_WRITE_WAIT: begin
                if (cmd_done) begin
                    state <= (!cmd_error) ? ST_RESTART : ST_IDLE;
                end
            end

            ST_RESTART: begin
                cmd <= CMD_START;
                tx_data <= BNO055_ADDR_READ;
                cmd_valid <= 1'b1;
                state <= ST_RESTART_WAIT;
            end

            ST_RESTART_WAIT: begin
                if (cmd_done) begin
                    state <= (!cmd_error) ? ST_READ_DATA : ST_IDLE;
                end
            end

            ST_READ_DATA: begin
                cmd <= CMD_READ_NACK;  // NACK since it's last byte
                cmd_valid <= 1'b1;
                state <= ST_READ_WAIT;
            end

            ST_READ_WAIT: begin
                if (cmd_done) begin
                    o_chip_id <= rx_data;
                    state <= ST_STOP;
                end
            end

            ST_STOP: begin
                cmd <= CMD_STOP;
                cmd_valid <= 1'b1;
                state <= ST_STOP_WAIT;
            end

            ST_STOP_WAIT: begin
                if (cmd_done) begin
                    state <= ST_DONE;
                end
            end

            ST_DONE: begin
                o_done <= 1'b1;
                state <= ST_IDLE;
            end

            default : begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule