/*******************************************************************************
 * @author:      Jordan Ho (cbv4eg)
 * @date:        November 2025
 * @description: decodes a roll & pitch input into attitude indicator shapes for
    a 7-segment common anode LED display
    https://www.electronics-tutorials.ws/blog/7-segment-display-tutorial.html
    common anode LED segments: 0 = ON (physically, BUT IGNORE IN THE CONTEXT OF CODE)
******************************************************************************/

module SSD_Euler_Decoder (
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

    /*** Logic minimization, for each of (7) LED segments ***/ 
    wire c3 = i_Attitude[3]; // D = isZero(pitch)
    wire c2 = i_Attitude[2]; // C = isZero(roll)
    wire c1 = i_Attitude[1]; // B = sgn(pitch)
    wire c0 = i_Attitude[0]; // A = sgn(roll)

    // Have to add negation to each logic expression to account for common anode tech 
    assign seg_A1 = ~(~c0 & c1 & ~c3);
    assign seg_B1 = ~(0);
    assign seg_C1 = ~(0);
    assign seg_D1 = ~(~c0 & ~c1 & ~c3);
    assign seg_E1 = ~(~c0 & ~c1 & ~c2);
    assign seg_F1 = ~(~c0 & c1 & ~c2);
    assign seg_G1 = ~(c2 & c3);
    assign seg_A2 = ~(c0 & c1 & ~c3);
    assign seg_B2 = ~(c0 & c1 & ~c2);
    assign seg_C2 = ~(c0 & ~c1 & ~c2);
    assign seg_D2 = ~(c0 & ~c1 & ~c3);
    assign seg_E2 = ~(0);
    assign seg_F2 = ~(0);
    assign seg_G2 = ~(c2 & c3);
endmodule