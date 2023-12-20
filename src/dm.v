`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/04 00:05:15
// Design Name: 
// Module Name: dm
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

module dm(
    input Clk_CPU,
    input rstn,
    input DMWr,
    input [5:0] addr,
    input [31:0] din,
    input [2:0] DMType,
    output reg [31:0] dout
    );
    
    reg[7:0] dmem[63:0];
    
    integer i =0;
    
    always@(posedge Clk_CPU or negedge rstn) begin
    if(!rstn) begin
        for(i=0;i<64;i=i+1)
            dmem[i] <= i;
        end
    else if(DMWr == 1) begin
        case(DMType)
            `dm_byte,
            `dm_byte_unsigned:dmem[addr] <= din[7:0];   
            `dm_halfword,
            `dm_halfword_unsigned:begin
                dmem[addr] <= din[7:0];
                dmem[addr+1] <= din[15:8]; 
            end
            `dm_word:begin
                dmem[addr] <= din[7:0];
                dmem[addr+1] <= din[15:8]; 
                dmem[addr+2] <= din[23:16];
                dmem[addr+3] <= din[31:24];
                end
            endcase
        end
        
        case(DMType)
        `dm_byte:dout = {{24{dmem[addr][7]}}, dmem[addr][7:0]};
        `dm_byte_unsigned:dout = {{24{1'b0}}, dmem[addr][7:0]};
        `dm_halfword:dout = {{16{dmem[addr+1][7]}}, dmem[addr+1][7:0], dmem[addr][7:0]};
        `dm_halfword_unsigned:dout = {{16{1'b0}}, dmem[addr+1][7:0], dmem[addr][7:0]}; 
        `dm_word:dout = {dmem[addr+3][7:0],dmem[addr+2][7:0],dmem[addr+1][7:0],dmem[addr][7:0]};
        endcase

    end
        
endmodule
