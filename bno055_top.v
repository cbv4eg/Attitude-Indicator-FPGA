/////////////////////////////////////////////////////////////////////
////                                                             ////
////  BNO055 I2C Operations using Nandland Go Board              ////
////  Displays CHIP ID reg values using 7-segment display        ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

// `timescale 1ns / 10ps

`define STOP 0
`define READ 1
`define WRITE 2

// EXPECTED VALUES FROM ADDR 0x00 to 0x04: A0, FB, 32, 0F

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
    output S2_G
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
    ST_CONFIG    = 1,
    ST_CONFIG_WAIT     = 2,
    ST_READ_OPRMODE    = 3,
    ST_READ_OPRMODE_WAIT     = 4;

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
            state       <= ST_CONFIG_WAIT;
        end

        ST_CONFIG_WAIT: begin
            opcode <= `STOP; // 0
            if (op_done)
                state <= ST_READ_OPRMODE;   // 7ms delay
        end

        ST_READ_OPRMODE: begin
            // Confirm config change
            reg_addr <= 8'h3D;   // OPR_MODE
            opcode <= `READ; // 1
            state <= ST_READ_OPRMODE_WAIT;
        end

        ST_READ_OPRMODE_WAIT: begin
            opcode <= `STOP; // 0
            if (op_done)
                state <= ST_READ_OPRMODE;  
        end

        endcase
    end
end

// Decode register values to seven segment hex displays
SSD_Bin_Decoder chip_id1(
    .bits(rd_reg_data[7:4]),
    .seg_A(S1_A), .seg_B(S1_B), .seg_C(S1_C), .seg_D(S1_D), .seg_E(S1_E), .seg_F(S1_F), .seg_G(S1_G) 
);

SSD_Bin_Decoder chip_id2(
    .bits(rd_reg_data[3:0]),
    .seg_A(S2_A), .seg_B(S2_B), .seg_C(S2_C), .seg_D(S2_D), .seg_E(S2_E), .seg_F(S2_F), .seg_G(S2_G) 
);

endmodule