// uart_tx.v - UART Transmitter, 8N1 - Continuous Mode

module uart_tx
#(
    parameter CLKS_PER_BIT = 217
)
(
    input        i_Clk,
    input  [7:0] i_tx_byte,
    output reg   o_tx_active = 1'b0,
    output reg   o_tx_serial = 1'b1,
    output reg   o_tx_done   = 1'b0
);

  localparam s_IDLE         = 3'b000;
  localparam s_TX_START_BIT = 3'b001;
  localparam s_TX_DATA_BITS = 3'b010;
  localparam s_TX_STOP_BIT  = 3'b011;
  localparam s_CLEANUP      = 3'b100;

  reg [2:0]  r_state     = s_IDLE;
  reg [7:0]  r_clk_count = 0;
  reg [2:0]  r_bit_index = 0;
  reg [7:0]  r_tx_byte   = 0;

  always @(posedge i_Clk) begin
    o_tx_done <= 1'b0;

    case (r_state)
      s_IDLE: begin
        o_tx_serial <= 1'b1;   // line idle high
        o_tx_active <= 1'b1;   // Always active in continuous mode
        r_clk_count <= 0;
        r_bit_index <= 0;
        r_tx_byte   <= i_tx_byte;  // Sample new data
        r_state     <= s_TX_START_BIT;  // Immediately start transmitting
      end

      s_TX_START_BIT: begin
        o_tx_serial <= 1'b0; // start bit

        if (r_clk_count < CLKS_PER_BIT-1) begin
          r_clk_count <= r_clk_count + 1'b1;
          r_state     <= s_TX_START_BIT;
        end else begin
          r_clk_count <= 0;
          r_state     <= s_TX_DATA_BITS;
        end
      end

      s_TX_DATA_BITS: begin
        o_tx_serial <= r_tx_byte[r_bit_index];

        if (r_clk_count < CLKS_PER_BIT-1) begin
          r_clk_count <= r_clk_count + 1'b1;
        end else begin
          r_clk_count <= 0;

          if (r_bit_index < 3'd7) begin
            r_bit_index <= r_bit_index + 1'b1;
          end else begin
            r_bit_index <= 0;
            r_state     <= s_TX_STOP_BIT;
          end
        end
      end

      s_TX_STOP_BIT: begin
        o_tx_serial <= 1'b1; // stop bit

        if (r_clk_count < CLKS_PER_BIT-1) begin
          r_clk_count <= r_clk_count + 1'b1;
        end else begin
          r_clk_count <= 0;
          o_tx_done   <= 1'b1;
          r_state     <= s_CLEANUP;
        end
      end

      s_CLEANUP: begin
        r_state <= s_IDLE;  // Loop back to idle, which immediately starts next transmission
      end

      default: r_state <= s_IDLE;
    endcase
  end

endmodule
