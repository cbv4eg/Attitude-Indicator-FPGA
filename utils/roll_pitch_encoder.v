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

    localparam DEG_THRESHOLD = 11'd10;
    /*
    // determine sign; 0:pos, 1:neg
    function sgn;
        input [15:0] val;
        begin
            sgn = (val[15] == 1'b0) ? 1'b0 : 1'b1; // checks if positive
        end;
    endfunction
    */

    /*
    // Converted values in deg (divide by 16); 16 LSB = 1 deg
    reg [11:0] roll_deg; // (-180, 180) deg
    reg [11:0] pitch_deg; // (-90, 90) deg
    assign roll_deg = i_Roll_Raw >> 4;
    assign pitch_deg = i_Pitch_Raw >> 4;

    /* ATTITUDE OUTPUT [sgn(roll), sgn(pitch), isZero(roll), isZero(pitch)] */
    
    // Output signs; 0:pos, 1:neg
    assign o_Attitude[3] = roll_deg[15]; 
    assign o_Attitude[2] = pitch_deg[15];
    // Converted values in deg (divide by 16)
    assign o_Attitude[1] = ((i_Roll_Raw >> 4) > DEG_THRESHOLD) ? 1'b0: 1'b1;
    assign o_Attitude[0] = ((i_Pitch_Raw >> 4) > DEG_THRESHOLD) ? 1'b0: 1'b1;

endmodule