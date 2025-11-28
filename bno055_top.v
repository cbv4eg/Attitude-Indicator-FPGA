/////////////////////////////////////////////////////////////////////
////                                                             ////
////  BNO055 I2C Read using Nandland Go Board                    ////
////  Displays CHIP ID reg values using 7-segment display        ////
////                                                             ////
////  Original Author: Richard Herveille                         ////
////  Authors: Jordan Ho & Raymond Wen                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

// `timescale 1ns / 10ps

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
    output S2_G,
    output LED1,
    output LED2,
    output LED3
    //output LED4
);

// Edge detector for SW2
reg Cleaned_SW2;
Debounce_Filter_Edge SW2_debouncer(
    .i_CLK          (CLK),
    .i_Bouncy       (SW2),
    .o_Debounced    (Cleaned_SW2)
);

/*
reg SW2_Reg1 = 1'b0;
reg SW2_Reg2 = 1'b0;
wire SW2_Trigger;
always @(posedge CLK) begin
    SW2_Reg1 <= Cleaned_SW2;
    SW2_Reg2 <= SW2_Reg1;
end
assign SW2_Trigger = SW2_Reg1 & ~SW2_Reg2; // rising edge
*/


// Keep track of which chip id address is being read from
wire [2:0] Chip_Id_Addr;
Mod_N_Counter #(.WIDTH(3), .RESET_VALUE(3'd3)) ID_reg_counter (
    .reset(SW1),
    .clock(Cleaned_SW2),
    .o_Counter(Chip_Id_Addr)
);
assign {LED1, LED2, LED3} = Chip_Id_Addr;

// Reading from registers...
reg [7:0] o_chip_id; // relay register output to SSD Hex Decoder

bno055_read operation (
    .i_clk          (CLK),
    .i_rst          (SW1),
    .i_reg_addr     ({5'h0, Chip_Id_Addr}),
    .i_read_chip_id (Cleaned_SW2),
    .o_chip_id      (o_chip_id),
    .o_done         (   ),
    .io_sda         (PMOD1),
    .io_scl         (PMOD7)
);

// Decode register values to seven segment hex displays
SSD_Bin_Decoder chip_id1(
    .bits(o_chip_id[7:4]),
    .seg_A(S1_A), .seg_B(S1_B), .seg_C(S1_C), .seg_D(S1_D), .seg_E(S1_E), .seg_F(S1_F), .seg_G(S1_G) 
);

SSD_Bin_Decoder chip_id2(
    .bits(o_chip_id[3:0]),
    .seg_A(S2_A), .seg_B(S2_B), .seg_C(S2_C), .seg_D(S2_D), .seg_E(S2_E), .seg_F(S2_F), .seg_G(S2_G) 
);

endmodule