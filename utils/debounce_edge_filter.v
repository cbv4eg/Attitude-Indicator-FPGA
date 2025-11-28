/*******************************************************************************
 * @author:         Jordan Ho (cbv4eg)
 * @date:           September 2025
 * @description:    submission for Module 4 Challenge Actvity checking for edge detection 
 * @THRESHOLD       debounce threshold (no. clock cycles) until the signal 
                    is stabilized and read
******************************************************************************/

module Debounce_Filter_Edge #(parameter THRESHOLD = 500000) (
    input i_CLK,
    input i_Bouncy, 
    output reg o_Debounced);

    reg [$clog2(THRESHOLD)-1:0] r_Count = 0; // stability counter
    reg [1:0] r_YX = 2'b00; // (Y, X)
    reg r_State = 1'b0;
    reg r_Next_State = 1'b0;

    /** Input state register (sequential, DFFs) **/
    always @(posedge i_CLK) begin
        r_YX[1] <= i_Bouncy;
        r_YX[0] <= r_YX[1];
    end

    // Counter flags
    reg r_Increment = 0;
    reg r_Reset = 0;

    /** Next-state (combinational) operations using my State-Transition Table **/
    always @(*) begin
        // default
        r_Increment = 0;
        r_Reset = 0;
        
        // counter actions based on XOR of the next Y,X state
        if ((r_YX == 2'b00 || r_YX == 2'b10) && (i_Bouncy == 0)) begin
            //r_Next_YX = 2'b00;
            r_Increment = 1;
        end else if ((r_YX == 2'b00 || r_YX == 2'b10) && (i_Bouncy == 1)) begin
            //r_Next_YX = 2'b01;
            r_Reset = 1;
        end else if ((r_YX == 2'b01 || r_YX == 2'b11) && (i_Bouncy == 0)) begin
            //r_Next_YX = 2'b10;
            r_Reset = 1;
        end else if ((r_YX == 2'b01 || r_YX == 2'b11) && (i_Bouncy == 1)) begin
            //r_Next_YX = 2'b11;
            r_Increment = 1;
        end
    end

    /** Counter and output logic **/
    always @(posedge i_CLK) begin
        // count up if not at threshold
        if (r_Reset)
            r_Count <= 0;
        else if (r_Increment) begin
            if (r_Count == THRESHOLD) begin
                r_State <= i_Bouncy;
                r_Count <= 0;
            end else begin
                r_Count <= r_Count + 1;
            end
        end
        o_Debounced <= r_State;
    end
endmodule