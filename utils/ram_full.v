/*******************************************************************************
 * @author:         Jordan Ho (cbv4eg)
 * @date:           October 2025
 * @description:    RAM2 block with separately clocked read and write ports.
******************************************************************************/

module RAM2 #(parameter DATA_WIDTH = 4, ADDR_DEPTH = 32, INIT_FILE = "") ( 
    // Write signals 
    input                     i_Wr_Clk, 
    input [$clog2(ADDR_DEPTH)-1:0] i_Wr_Addr, 
    input                     i_Wr_En, 
    input [DATA_WIDTH-1:0]         i_Wr_Data, 
    // Read signals 
    input                     i_Rd_Clk, 
    input [$clog2(ADDR_DEPTH)-1:0] i_Rd_Addr, 
    // Output signals 
    output reg [DATA_WIDTH-1:0]    o_Rd_Data 
    ); 
    
    // Memory buffer
    reg [DATA_WIDTH-1:0] r_Mem[ADDR_DEPTH-1:0]; 

    // RESET MEMORY FUNCTION
    task reset_memory;
        integer i;
        begin
            for (i = 0; i < ADDR_DEPTH; i = i + 1) begin
                r_Mem[i] = {DATA_WIDTH{1'b0}}; // Set each memory element to zero
            end
        end
    endtask
    
    // Write to mem only if EN = 1, @ posedge
    always @ (posedge i_Wr_Clk) begin 
        if (i_Wr_En)
            r_Mem[i_Wr_Addr] <= i_Wr_Data;
    end

    // Always read from memory @ posedge
    always @ (posedge i_Rd_Clk) begin 
        o_Rd_Data <= r_Mem[i_Rd_Addr]; 
    end

    // Reset memory @ posedge of reset signal
    always @ (posedge i_RST) begin
        reset_memory();
    end

    // If the mem init file is non-empty -> fill; otherwise, fill with zeroes
    initial if (INIT_FILE != "") begin
        $readmemh(INIT_FILE, r_Mem);
    end else begin
        reset_memory();
    end
endmodule