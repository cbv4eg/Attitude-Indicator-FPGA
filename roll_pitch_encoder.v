/*******************************************************************************
 * @author:      Jordan Ho (cbv4eg)
 * @date:        November 2025
 * @description: encodes roll & pitch into a 4-bit attitude: 
                    [sgn(roll), sgn(pitch), isZero(roll), isZero(pitch)]
******************************************************************************/


module Roll_Pitch_Encoder (
    input   [15:0]   i_Roll_Raw,
    input   [15:0]   i_Pitch_Raw,
    output  [3:0]    o_Attitude);

    localparam DEG_THRESHOLD = 11'd15;

    /* Calculate absolute values for threshold comparison */
    wire [15:0] roll_abs;
    wire [15:0] pitch_abs;
    
    // Two's complement: if negative (MSB=1), invert and add 1
    assign roll_abs = i_Roll_Raw[15] ? (~i_Roll_Raw + 1'b1) : i_Roll_Raw;
    assign pitch_abs = i_Pitch_Raw[15] ? (~i_Pitch_Raw + 1'b1) : i_Pitch_Raw;

    /* ATTITUDE OUTPUT [sgn(roll), sgn(pitch), isZero(roll), isZero(pitch)] */
    
    // Output signs; 0:pos, 1:neg
    assign o_Attitude[3] = i_Roll_Raw[15]; 
    assign o_Attitude[2] = i_Pitch_Raw[15];
    
    // Converted absolute values in deg (divide by 16)
    // 1 = over threshold, 0 = under threshold
    assign o_Attitude[1] = ((roll_abs >> 4) > DEG_THRESHOLD) ? 1'b1 : 1'b0;
    assign o_Attitude[0] = ((pitch_abs >> 4) > DEG_THRESHOLD) ? 1'b1 : 1'b0;
endmodule
