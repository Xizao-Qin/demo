`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/04 11:21:30
// Design Name: 
// Module Name: LED
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

`include "macro.vh"
module LED(

    );
    
    reg [63:0] LED_DATA[47:0];
    initial begin
        LED_DATA[0] = 64'hFFFFFFFEFEFEFEFE;
        LED_DATA[1] = 64'hFFFEFEFEFEFEFFFF;
        LED_DATA[2] = 64'hDEFEFEFFFFFFFFFF;
        LED_DATA[3] = 64'hCEFEFEFFFFFFFFFF;
        LED_DATA[4] = 64'hC2FFFFFFFFFFFFFF;
        LED_DATA[5] = 64'hE1FEFFFFFFFFFFFF;
        LED_DATA[6] = 64'hF1FCFFFFFFFFFFFF;
        LED_DATA[7] = 64'hFDF8F7FFFFFFFFFF;
        LED_DATA[8] = 64'hFFF8F3FFFFFFFFFF;
        LED_DATA[9] = 64'hFFFBF1FEFFFFFFFF;
        LED_DATA[10] = 64'hFFFFF9F8FFFFFFFF;
        LED_DATA[11] = 64'hFFFFFDF8F7FFFFFF;
        LED_DATA[12] = 64'hFFFFFFF9F1FFFFFF;
        LED_DATA[13] = 64'hFFFFFFFFF1FCFFFF;
        LED_DATA[14] = 64'hFFFFFFFFF9F8FFFF;
        LED_DATA[15] = 64'hFFFFFFFFFFF8F3FF;
        LED_DATA[16] = 64'hFFFFFFFFFFFBF1FE;
        LED_DATA[17] = 64'hFFFFFFFFFFFFF9BC;
        LED_DATA[18] = 64'hFFFFFFFFFFFFBDBC;
        LED_DATA[19] = 64'hFFFFFFFFBFBFBFBD;
        LED_DATA[20] = 64'hFFFFBFBFBFBFBFFF;
        LED_DATA[21] = 64'hFFBFBFBFBFBFFFFF;
        LED_DATA[22] = 64'hAFBFBFBFFFFFFFFF;
        LED_DATA[23] = 64'h2737FFFFFFFFFFFF;
        LED_DATA[24] = 64'h277777FFFFFFFFFF;
        LED_DATA[25] = 64'h7777777777FFFFFF;
        LED_DATA[26] = 64'hFFFF7777777777FF;
        LED_DATA[27] = 64'hFFFFFF7777777777;
        LED_DATA[28] = 64'hFFFFFFFFFF777771;
        LED_DATA[29] = 64'hFFFFFFFFFFFF7750;
        LED_DATA[30] = 64'hFFFFFFFFFFFFFFC8;
        LED_DATA[31] = 64'hFFFFFFFFFFFFE7CE;
        LED_DATA[32] = 64'hFFFFFFFFFFFFC7CF;
        LED_DATA[33] = 64'hFFFFFFFFFFDEC7FF;    
        LED_DATA[34] = 64'hFFFFFFFFF7CEDFFF;
        LED_DATA[35] = 64'hFFFFFFFFC7CFFFFF;
        LED_DATA[36] = 64'hFFFFFFFEC7EFFFFF;
        LED_DATA[37] = 64'hFFFFFFCECFFFFFFF;
        LED_DATA[38] = 64'hFFFFE7CEFFFFFFFF;
        LED_DATA[39] = 64'hFFFFC7CFFFFFFFFF;
        LED_DATA[40] = 64'hFFDEC7FFFFFFFFFF;
        LED_DATA[41] = 64'hF7CEDFFFFFFFFFFF;
        LED_DATA[42] = 64'hA7CFFFFFFFFFFFFF;
        LED_DATA[43] = 64'hA7AFFFFFFFFFFFFF;
        LED_DATA[44] = 64'hAFBFBFBFFFFFFFFF;
        LED_DATA[45] = 64'hBFBFBFBFBFFFFFFF;
        LED_DATA[46] = 64'hFFFFBFBFBFBFBFFF;
        LED_DATA[47] = 64'hFFFFFFFFBFBFBFBD;
    end
endmodule
