`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/11/19 22:20:22
// Design Name: 
// Module Name: alu
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

module alu(
    input signed [31:0] A,
    input signed [31:0] B,
    input [3:0] ALUOp,
    output reg signed  [31:0] C,
    output reg [7:0] Zero,
    output reg [7:0] isLess
    );
    always@(*)begin
        case(ALUOp)
            `ALUOp_nop: C = A+B;
            `ALUOp_add: C = A+B;
            `ALUOp_sub: C = A-B;
            `ALUOp_leftShit: C = A<<B;
            `ALUOp_xor: C = A^B;
            `ALUOp_rightShit: C = A>>B;
            `ALUOp_rightShitA: C = A>>>B;
            `ALUOp_or: C = A|B;
            `ALUOp_and: C = A&B;
            `ALUOp_lui: C = B<<12;
            `ALUOp_auipc: C = B<<12 + A;
            `ALUOp_slt: C = (A<B)?{{31{1'B0}},1'b1}:32'b0;
            `ALUOp_subu : C = $unsigned(A)<$unsigned(B);
            `ALUOp_sltu: C = ($unsigned(A)<$unsigned(B))?{{31{1'B0}},1'b1}:32'b0;
   endcase
           Zero = (C==0)?1:0;
           isLess = (C<0)?1:0;
   end
        
endmodule

