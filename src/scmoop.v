`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/11/19 22:17:54
// Design Name: 
// Module Name: scmoop
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


//`include "seg7x16.v"
//`include "SCPU_TOP.v"
//`include "RF.v"
//`include "alu.v"
module scoomp(clk, rstn, sw_i, disp_seg_o, disp_an_o);
    input clk;
    input rstn;
    input [15:0] sw_i;
    output [7:0] disp_an_o, disp_seg_o;
    
    reg[31:0] clkdiv;
    wire Clk_CPU;
    
    always @(posedge clk or negedge rstn)begin
        if(!rstn) clkdiv <= 0;
        else clkdiv <= clkdiv + 1'b1; end
        
    assign Clk_CPU = (sw_i[15])? clkdiv[27] : clkdiv[25];
    
    reg [63:0] display_data;
    
    
    
    /***********LED**********/
    reg [5:0] led_data_addr;
    reg [63:0] led_disp_data;
    parameter LED_DATA_NUM = 48;
    
    LED U_LED();
    
    always@(posedge Clk_CPU or negedge rstn) begin
    if(!rstn) 
        begin 
            led_data_addr = 6'b0;
            led_disp_data = 64'b0;
        end
    else if(sw_i[0] == 1'b1)
        begin
            if(led_data_addr == LED_DATA_NUM) begin led_data_addr = 6'd0; led_disp_data = 64'b1; end
            led_disp_data = U_LED.LED_DATA[led_data_addr];
            led_data_addr = led_data_addr + 1'b1;
        end
    else 
        led_data_addr = led_data_addr;
    end
        
    wire [31:0] instr;
    reg [31:0] reg_data;
    reg [31:0] alu_disp_data;
    reg [31:0] dmem_data;
    
     /***********ROM*************/
    reg [31:0] rom_addr = 32'b0;
    reg [31:0] next_rom_addr = 32'b0;
    parameter IM_CODE_NUM = 12;
    
    always@(posedge sw_i[1] or negedge rstn) begin
        if(!rstn) 
            begin 
                rom_addr <= 32'b0; 
            end
	    else begin
	       rom_addr = next_rom_addr;
	       if(rom_addr == IM_CODE_NUM) begin rom_addr = 32'b0;end
	    end
    end
    
    SCPU_TOP U_SCPU_TOP(
        .rom_addr(rom_addr),
        .instr(instr)
    );
    /**********Control******/
    wire[6:0] Op = instr[6:0];  // op
    wire[6:0] Funct7 = instr[31:25]; // funct7
    wire[2:0] Funct3 = instr[14:12]; // funct3
    wire[4:0] rs1 = instr[19:15];  // rs1
    wire[4:0] rs2 = instr[24:20];  // rs2
    wire[4:0] rd = instr[11:7];  // rd
        
    ctrl U_CTRL(
        .Op(Op),
        .Funct7(Funct7),
        .Funct3(Funct3)
    );
    
    /***********RF*************/             
    reg [4:0] reg_addr;
    parameter REG_DATA_NUM = 15;
    reg [31:0] WBSrc;
    
    always@(*)begin
        case(U_CTRL.WDSel)
        `WD_ALUout: WBSrc = U_alu_arithmetic.C;
        `WD_PC    : WBSrc = U_PCPLUS1.C;
        `WD_MEM   : WBSrc = U_DM.dout;
        default   : WBSrc = 32'b0;
        endcase
    end
        
     always @(posedge Clk_CPU or negedge rstn) begin
        if(!rstn) begin reg_addr = 5'b0;end
        else if(sw_i[13]==1'b1)begin
            reg_addr = reg_addr + 1'b1;  
            reg_data = U_RF.rf[reg_addr];
            if(reg_addr == REG_DATA_NUM) begin reg_addr = 5'b0;end
        end
        else reg_addr = reg_addr;
    end
    
    RF U_RF(
        .Clk_CPU(sw_i[2]),
        .rstn(rstn),
        .RFWr(U_CTRL.RegWrite),           
        .A1(rs1),
        .A2(rs2),
        .A3(rd),
        .WD(WBSrc)
    );
    /***********ImmGen*************/
    EXT U_EXT(
        .instr(instr),
        .EXTOp(U_CTRL.EXTOp)
    );
    /***********ALUsrc*************/
    wire [31:0] alu_A;
    wire [31:0] alu_B;
    assign alu_A = U_CTRL.ALUSrc_A?U_RF.RD1:rom_addr;
    assign alu_B = U_CTRL.ALUSrc_B?U_EXT.immout:U_RF.RD2;
        
    /***********ALU_Arithmetic*************/       
    alu U_alu_arithmetic(
        .A(alu_A),
        .B(alu_B),
        .ALUOp(U_CTRL.ALUOp)
    );
    
    reg [2:0] alu_addr;
    always @(posedge Clk_CPU or negedge rstn) begin
    if(!rstn) begin alu_addr = 3'b0 ;end
    else if(sw_i[12]==1'b1)begin
            alu_addr = alu_addr + 1'b1;
            case(alu_addr)
            3'b001:alu_disp_data = U_alu_arithmetic.A;
            3'b010:alu_disp_data = U_alu_arithmetic.B;
            3'b011:alu_disp_data = U_alu_arithmetic.C;
            3'b100:alu_disp_data = U_alu_arithmetic.Zero;
            3'b101:alu_disp_data = U_alu_arithmetic.isLess;
            default: alu_disp_data = 32'hFFFFFFFF;
            endcase
        end
    end
    
    /***********ALU_NEXTPC*************/
    alu U_PCPLUS1 (
        .A(rom_addr),
        .B({{31{1'b0}}, 1'b1}),
        .ALUOp(4'b0001)
    ); 
    alu U_PCADDIMM (
        .A(rom_addr),
        .B(U_EXT.immout),
        .ALUOp(4'b0001)
    );
    /***********SELECT_NEXTPC*************/
    always@(*)begin
        case(U_CTRL.NPC)
        `NPC_PCPLUS: next_rom_addr = U_PCPLUS1.C;
        `NPC_JAL   : next_rom_addr = U_PCADDIMM.C;
        `NPC_JALR  : next_rom_addr = U_alu_arithmetic.C;
        `NPC_BGE   : begin
                        if(U_alu_arithmetic.isLess == 0)
                            next_rom_addr = U_PCADDIMM.C;
                        else
                            next_rom_addr = U_PCPLUS1.C;
                    end
        `NPC_BLT   : begin
                        if(U_alu_arithmetic.isLess == 1)
                            next_rom_addr = U_PCADDIMM.C;
                        else
                            next_rom_addr = U_PCPLUS1.C;
                    end
        `NPC_BNE   : begin
                        if(U_alu_arithmetic.Zero == 0)
                            next_rom_addr = U_PCADDIMM.C;
                        else
                            next_rom_addr = U_PCPLUS1.C;
                    end
        
        `NPC_BEQ   : begin
                        if(U_alu_arithmetic.Zero == 1)
                            next_rom_addr = U_PCADDIMM.C;
                        else
                            next_rom_addr = U_PCPLUS1.C;
                    end
        default    : next_rom_addr = U_PCPLUS1.C;
        endcase
    end
    
    /***********DM*************/    
    dm U_DM(
        .Clk_CPU(Clk_CPU),
        .rstn(rstn),
        .DMWr(U_CTRL.MemWrite),
        .addr(U_alu_arithmetic.C[5:0]),
        .din(U_RF.RD2),
        .DMType(U_CTRL.DMType)
        );
        
    reg [5:0] dmem_addr;
    parameter DM_DATA_NUM = 15;
        
    always @(posedge Clk_CPU or negedge rstn) begin
       if(!rstn) begin dmem_addr = 6'b0 ;end
       else if(sw_i[11]==1'b1)begin
            dmem_addr = dmem_addr + 1'b1;
            dmem_data = U_DM.dmem[dmem_addr];
            if(dmem_addr == DM_DATA_NUM) begin dmem_addr = 6'b0;end
        end
        else dmem_addr = dmem_addr;
    end
      
    /***********Display*************/    
    always@(sw_i) begin
        if(sw_i[0] == 0) begin
            case(sw_i[14:11])
                4'b1000: display_data = instr;
                4'b0100: display_data = reg_data;
                4'b0010: display_data = alu_disp_data;
                4'b0001: display_data = dmem_data;
                default: display_data = 64'h0000000000000000;
            endcase end
        else begin display_data = led_disp_data; end
    end
    
    seg7x16 u_seg7x16(
    .clk(clk), 
    .rstn(rstn), 
    .i_data(display_data), 
    .disp_mode(sw_i[0]),
    .o_seg(disp_seg_o), 
    .o_sel(disp_an_o)
    );
endmodule

