/*******************************************************************************
 * @author:      Jordan Ho (cbv4eg)
 * @date:        September 2025
 * @description: decodes a 4-bit input for a 7-segment common anode LED display
    https://www.electronics-tutorials.ws/blog/7-segment-display-tutorial.html
    common anode LED segments: 0 = ON (physically, BUT IGNORE IN THE CONTEXT OF CODE)
******************************************************************************/

module SSD_Bin_Decoder (
    input  [3:0]    bits,
    output          seg_A,
    output          seg_B,
    output          seg_C,
    output          seg_D,
    output          seg_E,
    output          seg_F,
    output          seg_G);

    /*** Logic minimization, for each of (7) LED segments ***/ 
    wire c3 = bits[3];
    wire c2 = bits[2];
    wire c1 = bits[1];
    wire c0 = bits[0];

    wire nc3 = ~c3;
    wire nc2 = ~c2;
    wire nc1 = ~c1;
    wire nc0 = ~c0;

    assign seg_A = (nc3 & nc2 & nc1 & c0) | (nc3 & c2 & nc1 & nc0) | (c3 & c2 & nc1 & c0) | (c3 & nc2 & c1 & c0); 
    assign seg_B = ~((nc3 & nc2) | (nc3 & nc1 & nc0) | (nc3 & c1 & c0) | (c3 & nc1 & c0) | (nc2 & nc0));
    assign seg_C = ~((nc3 & nc1) | (nc3 & c0) | (nc3 & c2) | (c3 & nc2) | (nc1 & c0));
    assign seg_D = ~((c2 & nc1 & c0) | (c3 & nc1) | (nc3 & nc2 & nc0) | (nc2 & c1 & c0) | (c2 & c1 & nc0));
    assign seg_E = ~((nc2 & nc0) | (c3 & c1) | (c3 & c2) | (c1 & nc0));
    assign seg_F = ~((nc3 & c2 & nc1) | (nc1 & nc0) | (c3 & nc2) | (c2 & nc0) | (c3 & c1));
    assign seg_G = ~((nc3 & c2 & nc1) | (c3 & c0) | (nc2 & c1) | (c3 & nc2) | (c1 & nc0));
endmodule