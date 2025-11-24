// uart_rx.v - UART Receiver, 8N1

module uart_rx
#(
    parameter CLKS_PER_BIT = 217
)
(
    input        i_Clk,
    input        i_rx_serial,
    output reg   o_rx_dv = 1'b0,
    output reg [7:0] o_rx_byte = 8'h00
);

  localparam s_IDLE         = 3'b000;
  localparam s_RX_START_BIT = 3'b001;
  localparam s_RX_DATA_BITS = 3'b010;
  localparam s_RX_STOP_BIT  = 3'b011;
  localparam s_CLEANUP      = 3'b100;

  reg [2:0]  r_state      = s_IDLE;
  reg [7:0]  r_clk_count  = 0;
  reg [2:0]  r_bit_index  = 0; // 0-7
  reg [7:0]  r_rx_byte    = 0;
  reg        r_rx_data    = 1'b1;

  // Optional synchronizer for i_rx_serial
  reg r_rx_meta = 1'b1;
  always @(posedge i_Clk) begin
    r_rx_meta <= i_rx_serial;
    r_rx_data <= r_rx_meta;
  end

  always @(posedge i_Clk) begin
    o_rx_dv <= 1'b0;

    case (r_state)
      s_IDLE: begin
        r_clk_count <= 0;
        r_bit_index <= 0;

        if (r_rx_data == 1'b0)  // start bit detected
          r_state <= s_RX_START_BIT;
        else
          r_state <= s_IDLE;
      end

      s_RX_START_BIT: begin
        if (r_clk_count == (CLKS_PER_BIT-1)/2) begin
          if (r_rx_data == 1'b0) begin
            r_clk_count <= 0;
            r_state     <= s_RX_DATA_BITS;
          end else begin
            r_state <= s_IDLE;  // false start
          end
        end else begin
          r_clk_count <= r_clk_count + 1'b1;
          r_state     <= s_RX_START_BIT;
        end
      end

      s_RX_DATA_BITS: begin
        if (r_clk_count < CLKS_PER_BIT-1) begin
          r_clk_count <= r_clk_count + 1'b1;
        end else begin
          r_clk_count <= 0;
          r_rx_byte[r_bit_index] <= r_rx_data;

          if (r_bit_index < 3'd7)
            r_bit_index <= r_bit_index + 1'b1;
          else begin
            r_bit_index <= 0;
            r_state     <= s_RX_STOP_BIT;
          end
        end
      end

      s_RX_STOP_BIT: begin
        if (r_clk_count < CLKS_PER_BIT-1) begin
          r_clk_count <= r_clk_count + 1'b1;
        end else begin
          o_rx_dv   <= 1'b1;
          o_rx_byte <= r_rx_byte;
          r_clk_count <= 0;
          r_state     <= s_CLEANUP;
        end
      end

      s_CLEANUP: begin
        r_state <= s_IDLE;
      end

      default: r_state <= s_IDLE;
    endcase
  end

endmodule
