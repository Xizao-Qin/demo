`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/04 11:35:52
// Design Name: 
// Module Name: CTRL
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
