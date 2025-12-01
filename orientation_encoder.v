// orientation_encoder.v
// Encodes roll and pitch values into 9 discrete orientations
//
// Orientations:
//   0 = Neutral (center)
//   1 = Up (pitch positive, roll neutral)
//   2 = Up-Right (pitch positive, roll positive)
//   3 = Right (pitch neutral, roll positive)
//   4 = Down-Right (pitch negative, roll positive)
//   5 = Down (pitch negative, roll neutral)
//   6 = Down-Left (pitch negative, roll negative)
//   7 = Left (pitch neutral, roll negative)
//   8 = Up-Left (pitch positive, roll negative)

module orientation_encoder #(
    parameter THRESHOLD = 16'd400  // Raw threshold for tilt detection
)(
    input signed [15:0] roll_raw,   // Signed 16-bit roll value
    input signed [15:0] pitch_raw,  // Signed 16-bit pitch value
    output reg [3:0] orientation    // 4-bit orientation code (0-8)
);

// Internal direction signals
reg signed [1:0] roll_dir;   // -1, 0, or 1
reg signed [1:0] pitch_dir;  // -1, 0, or 1

always @(*) begin
    // Determine roll direction
    // Using $signed() to ensure proper signed comparison
    if ($signed(roll_raw) > $signed(THRESHOLD))
        roll_dir = 2'sd1;       // Right (positive)
    else if ($signed(roll_raw) < $signed(-THRESHOLD))
        roll_dir = -2'sd1;      // Left (negative)
    else
        roll_dir = 2'sd0;       // Neutral

    // Determine pitch direction
    if ($signed(pitch_raw) > $signed(THRESHOLD))
        pitch_dir = 2'sd1;      // Up (positive)
    else if ($signed(pitch_raw) < $signed(-THRESHOLD))
        pitch_dir = -2'sd1;     // Down (negative)
    else
        pitch_dir = 2'sd0;      // Neutral

    // Encode orientation based on combination of roll and pitch
    // Python uses: orientation_map.get((roll_dir, pitch_dir), 0)
    // So we concatenate as {roll_dir[1:0], pitch_dir[1:0]}
    // Signed values: 01 = +1, 11 = -1, 00 = 0
    case ({roll_dir, pitch_dir})
        4'b00_00: orientation = 4'd0;  // Neutral (roll=0, pitch=0)
        4'b00_01: orientation = 4'd1;  // Up (roll=0, pitch=+1)
        4'b01_01: orientation = 4'd2;  // Up-Right (roll=+1, pitch=+1)
        4'b01_00: orientation = 4'd3;  // Right (roll=+1, pitch=0)
        4'b01_11: orientation = 4'd4;  // Down-Right (roll=+1, pitch=-1)
        4'b00_11: orientation = 4'd5;  // Down (roll=0, pitch=-1)
        4'b11_11: orientation = 4'd6;  // Down-Left (roll=-1, pitch=-1)
        4'b11_00: orientation = 4'd7;  // Left (roll=-1, pitch=0)
        4'b11_01: orientation = 4'd8;  // Up-Left (roll=-1, pitch=+1)
        default:  orientation = 4'd0;  // Default to neutral
    endcase
end

endmodule
