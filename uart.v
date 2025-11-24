// uart.v - Top-level UART (RX + TX) for Nandland Go Board
// 25 MHz clock, 115200 baud, 8N1

module uart
#(
    parameter CLKS_PER_BIT = 217  // 25e6 / 115200 ≈ 217
)
(
    input        i_Clk,       // 25 MHz Go Board clock
    input        i_rx_serial, // From FTDI RX line (PC -> FPGA)
    output       o_tx_serial, // To FTDI TX line (FPGA -> PC)

    // RX interface
    output       o_rx_dv,     // Data valid (1 clk pulse)
    output [7:0] o_rx_byte,   // Received byte

    // TX interface
    input  [7:0] i_tx_byte,   // Byte to transmit
    output       o_tx_active, // High while transmitting
    output       o_tx_done    // 1 clk pulse when done
);

  uart_rx #(
    .CLKS_PER_BIT(CLKS_PER_BIT)
  ) uart_rx_inst (
    .i_Clk(i_Clk),
    .i_rx_serial(i_rx_serial),
    .o_rx_dv(o_rx_dv),
    .o_rx_byte(o_rx_byte)
  );

  uart_tx #(
    .CLKS_PER_BIT(CLKS_PER_BIT)
  ) uart_tx_inst (
    .i_Clk(i_Clk),
    .i_tx_byte(i_tx_byte),
    .o_tx_active(o_tx_active),
    .o_tx_serial(o_tx_serial),
    .o_tx_done(o_tx_done)
  );

endmodule
