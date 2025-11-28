/*******************************************************************************
 * @author:         Jordan Ho (cbv4eg)
 * @date:           September 2025
 * @description:    n-bit counter with modulo functionality (circular)
******************************************************************************/
// TODO: get width just from reset value
module Mod_N_Counter #(parameter WIDTH = 4, RESET_VALUE = 4'd15) (
        // Inputs
        input reset,
        input clock,
        // Outputs
        output [WIDTH-1:0] o_Counter
    );

    reg [WIDTH-1:0] counter_cs; // current
    reg [WIDTH-1:0] counter_ns; // next

    // Counter current state logic
    always @(posedge clock, posedge reset)
        begin
            if (reset == 1'b1)
                counter_cs <= {WIDTH{1'b0}};
            else
                counter_cs <= counter_ns;
        end

    // Counter next state logic
    always @(counter_cs)
        begin
            if (counter_cs == RESET_VALUE)
                counter_ns <= {WIDTH{1'b0}};
            else
                counter_ns <= counter_cs + 1;
        end

    // Counter output logic
    assign o_Counter = counter_cs;
endmodule