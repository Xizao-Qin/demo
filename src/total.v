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

`define ALUOp_nop           4'b0000
`define ALUOp_add           4'b0001   
`define ALUOp_sub           4'b0010
`define ALUOp_leftShit      4'b0011
`define ALUOp_xor           4'b0100
`define ALUOp_rightShit     4'b0101
`define ALUOp_rightShitA    4'b0110
`define ALUOp_or            4'b0111
`define ALUOp_and           4'b1000
`define ALUOp_lui           4'b1001
`define ALUOp_auipc         4'b1010
`define ALUOp_slt           4'b1011
`define ALUOp_subu          4'b1100
`define ALUOp_sltu          4'b1101

`define dm_word              3'b000
`define dm_halfword          3'b001
`define dm_halfword_unsigned 3'b010
`define dm_byte              3'b011
`define dm_byte_unsigned     3'b100

`define EXT_I   3'b000
`define EXT_IU	3'b001
`define EXT_S	3'b010
`define EXT_SB	3'b011
`define EXT_UJ	3'b100
`define EXT_U	3'b101

`define WD_ALUout  2'b01
`define WD_PC      2'b10
`define WD_MEM     2'b11

`define NPC_PCPLUS 3'b000
`define NPC_JAL    3'b001
`define NPC_JALR   3'b010
`define NPC_BGE    3'b011
`define NPC_BLT    3'b100
`define NPC_BNE    3'b101
`define NPC_BEQ    3'b110 

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

module SCPU_TOP(
    input [31:0] rom_addr,
    output [31:0] instr
    );
    
    
    dist_mem_im U_IM(
        .a(rom_addr),
        .spo(instr)
    );
endmodule

module ctrl(
    input [6:0] Op,  //opcode
    input [6:0] Funct7,  //funct7 
    input [2:0] Funct3,    // funct3 
    output RegWrite, // control signal for register write
    output MemWrite, // control signal for memory write
    output [2:0]EXTOp,    // control signal to signed extension
    output [3:0]ALUOp,    // ALU opertion
    output [2:0]NPC,    // next pc operation
    output ALUSrc_A,   // ALU source for a
    output ALUSrc_B,   // ALU source for b
    output [2:0]DMType, //dm r/w type
    output [1:0]WDSel    // (register) write data selection  (MemtoReg) 
    );
    
    wire i_nop = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&~Op[1]&~Op[0]; // 0000000
    
    wire itype_r = ~Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0110011
    wire i_add = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
    wire i_sub = itype_r&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
    wire i_sll = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& ~Funct3[1]& Funct3[0]; // sll 0000000 001
    wire i_slt = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]& ~Funct3[0]; // slt 0000000 010
    wire i_sltu = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]& Funct3[0]; // sltu 0000000 011
    wire i_xor = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]& ~Funct3[1]& ~Funct3[0]; // xor 0000000 100
    wire i_srl = itype_r& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]& ~Funct3[1]& Funct3[0]; // srl 0000000 101
    wire i_sra = itype_r& ~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]& ~Funct3[1]& Funct3[0]; // sra 0000000 101
    wire i_or = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]& Funct3[1]& ~Funct3[0]; // or 0000000 110
    wire i_and = itype_r&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]& Funct3[1]& Funct3[0]; // and 0000000 111    
    
    
    wire itype_load  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
    wire i_lb = itype_load&~Funct3[2]& ~Funct3[1]& ~Funct3[0]; //lb 000
    wire i_lh = itype_load&~Funct3[2]& ~Funct3[1]& Funct3[0];  //lh 001
    wire i_lw = itype_load&~Funct3[2]& Funct3[1]& ~Funct3[0];  //lw 010
    wire i_ld = itype_load&~Funct3[2]& Funct3[1]& Funct3[0];  //ld 011
    wire i_lbu = itype_load&Funct3[2]& ~Funct3[1]& ~Funct3[0]; //lbu 100
    wire i_lhu = itype_load&Funct3[2]& ~Funct3[1]& Funct3[0]; //lhu 101
    wire i_lwu = itype_load&Funct3[2]& Funct3[1]& ~Funct3[0];  //lwu 110
    
    wire itype_imm  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0010011
    wire i_addi  =  itype_imm& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // addi 000 func3
    wire i_slli  =  itype_imm&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& ~Funct3[2]& ~Funct3[1]& Funct3[0]; // slli 001 0000000
    wire i_slti  =  itype_imm& ~Funct3[2]& Funct3[1]& ~Funct3[0]; // slti 010 func3
    wire i_sltiu  =  itype_imm& ~Funct3[2]& Funct3[1]& Funct3[0]; // sltiu 011 func3
    wire i_xori  =  itype_imm& Funct3[2]& ~Funct3[1]& ~Funct3[0]; // xori 100 func3
    wire i_srli  =  itype_imm&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& ~Funct3[1]& Funct3[0]; // srli 101 0000000
    wire i_srai  =  itype_imm&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& ~Funct3[1]& Funct3[0]; // ssrai 101 0100000
    wire i_ori  =  itype_imm& Funct3[2]& Funct3[1]& ~Funct3[0]; // ori 110 func3
    wire i_andi  =  itype_imm& Funct3[2]& Funct3[1]& Funct3[0]; // andi 111 func3
    
    wire itype_s = ~Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//0100011
    wire i_sb = itype_s& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // sb 000
    wire i_sh = itype_s& ~Funct3[2]& ~Funct3[1]&  Funct3[0]; // sh 001
    wire i_sw = itype_s& ~Funct3[2]&  Funct3[1]& ~Funct3[0]; // sw 010
    
    
    wire itype_sb = Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//1100011
    wire i_beq = itype_sb&~Funct3[2]&~Funct3[1]&~Funct3[0]; // beq 000
    wire i_bne = itype_sb&~Funct3[2]&~Funct3[1]& Funct3[0]; // bne 001
    wire i_blt = itype_sb& Funct3[2]&~Funct3[1]&~Funct3[0]; // blt 100
    wire i_bge = itype_sb& Funct3[2]&~Funct3[1]& Funct3[0]; // bge 101
    wire i_bltu = itype_sb& Funct3[2]& Funct3[1]& ~Funct3[0]; // bltu 110
    wire i_bgeu = itype_sb& Funct3[2]& Funct3[1]& Funct3[0]; // bgeu 111
    wire itype_sb_sign = i_beq | i_bne | i_blt | i_bge;
    wire itype_sb_unsign = i_bltu | i_bgeu;
    
    wire i_jal = Op[6]&Op[5]&~Op[4]&Op[3]&Op[2]&Op[1]&Op[0];//1101111
    wire i_jalr = Op[6]&Op[5]&~Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//1100111
    wire i_lui = ~Op[6]&Op[5]&Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//0110111
    wire i_auipc = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&~Op[1]&~Op[0];//0010111
    
   
    assign RegWrite = itype_r | itype_load | itype_imm | i_jal | i_jalr | i_lui | i_auipc;// register write
    assign MemWrite = itype_s;    // memory write
    assign ALUSrc_A = ~i_auipc;
    assign ALUSrc_B = itype_imm | itype_s | itype_load | i_lui | i_auipc | i_jalr;
    
    assign WDSel[0] = itype_load | itype_r | itype_imm | i_lui | i_auipc;   
    assign WDSel[1] = itype_load | i_jal | i_jalr;
    
    assign ALUOp[0] = itype_load | itype_s | i_add | i_addi | i_jal | i_jalr | i_sll | i_slli | i_srl | i_srli | i_or | i_ori | i_lui | i_slt | i_slti | i_sltu | i_sltiu;
    assign ALUOp[1] = itype_sb_sign | i_sub | i_sll | i_slli | i_sra | i_srai | i_or | i_ori | i_auipc | i_slt | i_slti;
    assign ALUOp[2] = i_xor | i_xori | i_srl | i_srli | i_sra | i_srai | i_or | i_ori | itype_sb_unsign | i_sltu | i_sltiu;
    assign ALUOp[3] = i_and | i_andi | i_lui | i_auipc | i_slt | i_sltu | i_slti | i_sltiu | itype_sb_unsign;
   
    assign EXTOp[0] = i_sltiu | itype_sb | i_lui | i_auipc;
    assign EXTOp[1] = itype_s | itype_sb; 
    assign EXTOp[2] = i_jal | | i_lui | i_auipc;
    
    assign DMType[2] = i_lbu;
    assign DMType[1] = i_lb | i_sb | i_lhu;
    assign DMType[0] = i_lh | i_sh | i_lb | i_sb;
    
    assign NPC[0] = i_jal | i_bge | i_bgeu | i_bne;
    assign NPC[1] = i_jalr | i_bge | i_bgeu | i_beq;
    assign NPC[2] = i_blt | i_bltu | i_bne | i_beq;
endmodule

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

module seg7x16(
        input clk, input rstn, input[63:0] i_data,input disp_mode,output [7:0] o_seg, output [7:0] o_sel
    );
    
    reg [14:0] cnt;
    wire seg7_clk;
    always@(posedge clk, negedge rstn)
    if(!rstn) cnt<=0;
    else cnt<=cnt + 1'b1;
    
    assign seg7_clk = cnt[14];
    
    reg [2:0] seg7_addr;
    
    always@(posedge seg7_clk, negedge rstn)
        if(!rstn) seg7_addr <= 0;
        else seg7_addr <= seg7_addr + 1'b1;
        
    reg [7:0] o_sel_r;
    
    always@(*)
        case(seg7_addr)
            7:o_sel_r = 8'b01111111;
            6:o_sel_r = 8'b10111111;
            5:o_sel_r = 8'b11011111;
            4:o_sel_r = 8'b11101111;
            3:o_sel_r = 8'b11110111;
            2:o_sel_r = 8'b11111011;
            1:o_sel_r = 8'b11111101;
            0:o_sel_r = 8'b11111110;
      endcase
      
      reg [63:0] i_data_store;
      always@(posedge clk, negedge rstn)
        if(!rstn) i_data_store <= 0;
        else i_data_store <= i_data;
        
      reg [7:0] seg_data_r;
      always@(*)
      if(disp_mode == 1'b0)begin
        case(seg7_addr)
            0: seg_data_r = i_data_store[3:0];
            1: seg_data_r = i_data_store[7:4];
            2: seg_data_r = i_data_store[11:8];
            3: seg_data_r = i_data_store[15:12];
            4: seg_data_r = i_data_store[19:16];
            5: seg_data_r = i_data_store[23:20];
            6: seg_data_r = i_data_store[27:24];
            7: seg_data_r = i_data_store[31:28];
        endcase end
        else begin
        case(seg7_addr)
            0:seg_data_r = i_data_store[7:0];
            1:seg_data_r = i_data_store[15:8];
            2:seg_data_r = i_data_store[23:16];
            3:seg_data_r = i_data_store[31:24];
            4:seg_data_r = i_data_store[39:32];
            5:seg_data_r = i_data_store[47:40];
            6:seg_data_r = i_data_store[55:48];
            7:seg_data_r = i_data_store[63:56];
         endcase end   
            
        
        reg[7:0] o_seg_r;
        always@(posedge clk, negedge rstn)
            if(!rstn) o_seg_r<=8'hff;
            else if(disp_mode == 1'b0) begin
                case(seg_data_r)
                4'h0: o_seg_r <= 8'hC0;
                4'h1: o_seg_r <= 8'hF9;
                4'h2: o_seg_r <= 8'hA4;
                4'h3: o_seg_r <= 8'hB0;
                4'h4: o_seg_r <= 8'h99;
                4'h5: o_seg_r <= 8'h92;
                4'h6: o_seg_r <= 8'h82;
                4'h7: o_seg_r <= 8'hF8;
                4'h8: o_seg_r <= 8'h80;
                4'h9: o_seg_r <= 8'h90;
                4'hA: o_seg_r <= 8'h88;
                4'hB: o_seg_r <= 8'h83;
                4'hC: o_seg_r <= 8'hC6;
                4'hD: o_seg_r <= 8'hA1;
                4'hE: o_seg_r <= 8'h86;
                4'hF: o_seg_r <= 8'h8E;
                default:o_seg_r <= 8'hFF;
            endcase end
        else begin o_seg_r <= seg_data_r;end
            
        assign o_sel = o_sel_r;
        assign o_seg = o_seg_r;
                          
endmodule
