`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.03.2024 15:52:04
// Design Name: 
// Module Name: mips_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
module mips_tb();
    reg clock;

    mips test(clock);

    initial begin
        clock = 1'b0;

        //forever begin
        for(int i=0;i<2;i=i+1)begin
        #10 clock = ~clock;
        end
        $finish;
    end

    initial begin
            $dumpfile("mips_dump.vcd");
            $dumpvars();
    end
endmodule