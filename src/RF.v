`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/11/19 22:19:30
// Design Name: 
// Module Name: RF
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


module RF(
    input Clk_CPU,
    input rstn,
    input RFWr,
    input [5:0] A1,A2,
    input [4:0] A3,
    input [31:0] WD,
    output reg[31:0] RD1,
    output reg[31:0] RD2
        );
        
    reg [31:0] rf[31:0];
   
    integer i;
    
    always@(*)begin
        RD1 = (A1!=0)?rf[A1]:0;
        RD2 = (A2!=0)?rf[A2]:0;
    end
                  
    always@(posedge Clk_CPU or negedge rstn) begin
        if(!rstn) begin
            for(i=0;i<32;i=i+1)
                rf[i] <= i;  
            end
        else
            if(RFWr) begin
                rf[A3] <= WD;
            end
     end     
    
endmodule
