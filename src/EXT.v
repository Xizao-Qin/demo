`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/04 11:46:07
// Design Name: 
// Module Name: EXT
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


//EXT CTRL itype, stype, btype, utype, jtype
`define EXT_I   3'b000
`define EXT_IU	3'b001
`define EXT_S	3'b010
`define EXT_SB	3'b011
`define EXT_UJ	3'b100
`define EXT_U	3'b101

module EXT(
    input [31:0] instr,
    input [2:0]	 EXTOp,
    output reg [31:0] immout
    );
     
    always@(*) begin
        case(EXTOp)
            `EXT_I:  immout = {{20{instr[31]}},instr[31:20]};
            `EXT_IU: immout = {{20{1'b0}},instr[31:20]};
            `EXT_S:  immout = {{20{instr[31]}},instr[31:25],instr[11:7]};
            `EXT_SB: immout = {{21{instr[31]}},instr[7],instr[30:25],instr[11:8]};
            `EXT_UJ: immout = {{13{instr[31]}},instr[19:12],instr[20],instr[30:21]};
            `EXT_U:  immout = {{12{1'b0}},instr[31:12]}; 
            default: immout <= 32'b0;
        endcase
    end
endmodule
