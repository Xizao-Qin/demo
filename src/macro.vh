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