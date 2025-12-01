/*******************************************************************************
 * @author:      Jordan Ho (cbv4eg)
 * @date:        November 2025
 * @description: decodes a roll & pitch input into attitude indicator shapes for
    a 7-segment display on the Nandland Go Board
    Display 1: Common Anode (active LOW - 0 = ON)
    Display 2: Common Anode (active LOW - 0 = ON)
    
    7-Segment Layout:
         AAA
        F   B
         GGG
        E   C
         DDD
         
    Attitude encoding: [sgn(roll), sgn(pitch), isZero(roll), isZero(pitch)]
    where isZero = 0 means UNDER threshold, isZero = 1 means OVER threshold
******************************************************************************/


module SSD_Decoder (
    input  [3:0]    i_Attitude, // [sgn(roll), sgn(pitch), isZero(roll), isZero(pitch)]
    output          seg_A1,
    output          seg_B1,
    output          seg_C1,
    output          seg_D1,
    output          seg_E1,
    output          seg_F1,
    output          seg_G1,
    output          seg_A2,
    output          seg_B2,
    output          seg_C2,
    output          seg_D2,
    output          seg_E2,
    output          seg_F2,
    output          seg_G2);


    /*** Decode attitude signals ***/ 
    wire sgn_roll = i_Attitude[3];      // 0:pos, 1:neg
    wire sgn_pitch = i_Attitude[2];     // 0:pos, 1:neg
    wire over_roll = i_Attitude[1];     // 0: under threshold, 1: over threshold
    wire over_pitch = i_Attitude[0];    // 0: under threshold, 1: over threshold


    /*** Determine which segments to light based on roll/pitch conditions ***/
    
    // Both under threshold: center segments (G1, G2)
    wire both_zero = ~over_roll & ~over_pitch;
    
    // Pitch positive over threshold: top segments (A1, A2)
    wire pitch_pos = ~sgn_pitch & over_pitch & ~over_roll;
    
    // Pitch negative over threshold: bottom segments (D1, D2)
    wire pitch_neg = sgn_pitch & over_pitch & ~over_roll;
    
    // Roll positive over threshold: right segments (B2, C2) - RIGHT DISPLAY
    wire roll_pos = ~sgn_roll & over_roll & ~over_pitch;
    
    // Roll negative over threshold: left segments (F1, E1) - LEFT DISPLAY
    wire roll_neg = sgn_roll & over_roll & ~over_pitch;
    
    // Both positive (pitch+, roll+): upper right corner (A2, B2) - RIGHT DISPLAY
    wire both_pos = ~sgn_pitch & ~sgn_roll & over_pitch & over_roll;
    
    // Both negative (pitch-, roll-): lower left corner (D1, E1) - LEFT DISPLAY
    wire both_neg = sgn_pitch & sgn_roll & over_pitch & over_roll;
    
    // Pitch positive, roll negative: upper left corner (A1, F1) - LEFT DISPLAY
    wire pitch_pos_roll_neg = ~sgn_pitch & sgn_roll & over_pitch & over_roll;
    
    // Pitch negative, roll positive: lower right corner (D2, C2) - RIGHT DISPLAY
    wire pitch_neg_roll_pos = sgn_pitch & ~sgn_roll & over_pitch & over_roll;


    /*** Assign segment outputs ***/
    // Both displays: Common Anode (active LOW - invert with ~)
    
    // Segment A (top)
    assign seg_A1 = ~(pitch_pos | pitch_pos_roll_neg);
    assign seg_A2 = ~(pitch_pos | both_pos);
    
    // Segment B (upper right)
    assign seg_B1 = ~(0);
    assign seg_B2 = ~(roll_pos | both_pos);
    
    // Segment C (lower right)
    assign seg_C1 = ~(0);
    assign seg_C2 = ~(roll_pos | pitch_neg_roll_pos);
    
    // Segment D (bottom)
    assign seg_D1 = ~(pitch_neg | both_neg);
    assign seg_D2 = ~(pitch_neg | pitch_neg_roll_pos);
    
    // Segment E (lower left)
    assign seg_E1 = ~(roll_neg | both_neg);
    assign seg_E2 = ~(0);
    
    // Segment F (upper left)
    assign seg_F1 = ~(roll_neg | pitch_pos_roll_neg);
    assign seg_F2 = ~(0);
    
    // Segment G (middle)
    assign seg_G1 = ~(both_zero);
    assign seg_G2 = ~(both_zero);
    
endmodule
