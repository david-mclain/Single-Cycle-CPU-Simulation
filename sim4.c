/*
    Created by David McLain
    Fall 2022
    sim4.c
*/

#include <stdio.h>
#include "sim4.h"

#define BIT_MASK_5 0x0000001f
#define BIT_MASK_6 0x0000003f
#define BIT_MASK_16 0x0000ffff
#define SIGN_EXTEND_32_BIT 0xffff0000
#define BIT_MASK_26 0x03ffffff
/**
 * @brief Takes instruction and fills in all fields
 * 
 * @param instruction - instruction to decode
 * @param fieldsOut - fields to fill out
 */
void extract_instructionFields(WORD instruction, InstructionFields *fieldsOut) {
    int temp = instruction >> 26;

    fieldsOut->opcode = (temp & BIT_MASK_6);                    // opcode = bits 31-26 of instruction
    temp = instruction >> 21;
    fieldsOut->rs = (temp & BIT_MASK_5);                        // rs = bits 25-21 of instruction
    temp = instruction >> 16;
    fieldsOut->rt = (temp & BIT_MASK_5);                        // rt = bits 20-16 of instruction
    temp = instruction >> 11;
    fieldsOut->rd = (temp & BIT_MASK_5);                        // rd = bits 15-11 of instruction
    temp = instruction >> 6;
    fieldsOut->shamt = (temp & BIT_MASK_5);                     // shamt = bits 10-6 of instruction
    fieldsOut->funct = (instruction & BIT_MASK_6);              // funct = bits 5-0 of instruction
    fieldsOut->imm16 = (instruction & BIT_MASK_16);             // imm16 = bits 15-0 of instruction
    fieldsOut->imm32 = (signExtend16to32(fieldsOut->imm16));    // imm32 = bits 15-0 of instruction sign extended to bit 31
    fieldsOut->address = (instruction & BIT_MASK_26);           // address = bits 25-0 of instruction
}
/**
 * @brief Fills cpu control bits
 * 
 * Instructions implemented outside of required were andi, lui and bne
 * 
 * @param fields - fields containing decoded instruction
 * @param control - control bits to fill in
 * @return int - 1 if valid instruction, 0 otherwise
 */
int fill_CPUControl(InstructionFields *fields, CPUControl *control) {
    int op = fields->opcode;
    control->extra1 = 0;
    control->extra2 = 0;
    control->extra3 = 0;
    if (op == 0) {                                              // If R-Format instruction
        control->ALUsrc = 0;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 1;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        if (fields->funct == 32 || fields->funct == 33) {       // If funct = add or addu instruction
            control->ALU.op = 2;
            control->ALU.bNegate = 0;
        }
        else if (fields->funct == 34 || fields->funct == 35) {  // If funct = sub or subu instruction
            control->ALU.op = 2;
            control->ALU.bNegate = 1;
        }
        else if (fields->funct == 36) {                         // If funct = and instruction
            control->ALU.op = 0;
            control->ALU.bNegate = 0;
        }
        else if (fields->funct == 37) {                         // If funct = or instruction
            control->ALU.op = 1;
            control->ALU.bNegate = 0;
        }
        else if (fields->funct == 38) {                         // If funct = xor instruction
            control->ALU.op = 4;
            control->ALU.bNegate = 0;
        }
        else if (fields->funct == 42) {                         // If funct = slt instruction
            control->ALU.op = 3;
            control->ALU.bNegate = 1;
        }
        else {                                                  // If invalid instruction
            control->regDst = 0;
            control->regWrite = 0;
            control->ALU.op = 0;
            control->ALU.bNegate = 0;
            return 0;
        }
    }
    else if (op == 2) {                                         // If opcode = jump instruction
        control->ALUsrc = 0;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 0;
        control->branch = 0;
        control->jump = 1;
        control->ALU.op = 0;
        control->ALU.bNegate = 0;
    }
    else if (op == 4) {                                          // If opcode = beq instruction
        control->ALUsrc = 0;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 0;
        control->branch = 1;
        control->jump = 0;
        control->ALU.op = 2;
        control->ALU.bNegate = 1;
    }
    else if (op == 5) {                                         // If opcode = bne instruction
        control->ALUsrc = 0;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 0;
        control->branch = 1;
        control->jump = 0;
        control->ALU.op = 2;
        control->ALU.bNegate = 1;
        control->extra2 = 1;
    }
    else if (op == 8 || op == 9) {                              // If opcode = addi or addiu instructions
        control->ALUsrc = 1;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 2;
        control->ALU.bNegate = 0;
    }
    else if (op == 10) {                                        // If opcode = slti instruction
        control->ALUsrc = 1;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 3;
        control->ALU.bNegate = 1;
    }
    else if (op == 12) {                                        // If opcode = andi instruction
        control->ALUsrc = 1;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 0;
        control->ALU.bNegate = 0;
        control->extra3 = 1;
    }
    else if (op == 15) {                                        // If opcode = lui instruction
        control->ALUsrc = 1;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 0;
        control->ALU.bNegate = 0;
        control->extra1 = 1;
    }
    else if (op == 35) {                                        // If opcode = lw instruction
        control->ALUsrc = 1;
        control->memRead = 1;
        control->memToReg = 1;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 1;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 2;
        control->ALU.bNegate = 0;
    }
    else if (op == 43) {                                        // If opcode = sw instruction
        control->ALUsrc = 1;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 1;
        control->regDst = 0;
        control->regWrite = 0;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 2;
        control->ALU.bNegate = 0;
    }
    else {                                                      // If opcode = invalid instruction
        control->ALUsrc = 0;
        control->memRead = 0;
        control->memToReg = 0;
        control->memWrite = 0;
        control->regDst = 0;
        control->regWrite = 0;
        control->branch = 0;
        control->jump = 0;
        control->ALU.op = 0;
        control->ALU.bNegate = 0;
        return 0;
    }
    return 1;
} 
/**
 * @brief Returns next instruction
 * 
 * @param curPC - current PC counter
 * @param instructionMemory - instructions in memory
 * @return WORD - next instruction
 */
WORD getInstruction(WORD curPC, WORD *instructionMemory) {
    return instructionMemory[curPC / 4];
}
/**
 * @brief Returns ALU input a
 * 
 * @param control - control bits
 * @param fields - decoded instruction fields
 * @param rsVal - value in rs section of instruction
 * @param rtVal - value in rt section of instruction
 * @param reg32 - reg33 for hi
 * @param reg33 - reg34 for lo
 * @param oldPC - old program counter
 * @return WORD - ALU input a
 */
WORD getALUinput1(CPUControl *control, InstructionFields *fields, WORD rsVal, WORD rtVal, WORD reg32, WORD reg33, WORD oldPC) {
    return rsVal;
}
/**
 * @brief Returns ALU input b
 * 
 * @param control - control bits
 * @param fields - decoded instruction fields
 * @param rsVal - value in rs section of instruction
 * @param rtVal - value in rt section of instruction
 * @param reg32 - reg33 for hi
 * @param reg33 - reg34 for lo
 * @param oldPC - old program counter
 * @return WORD - ALU input b
 */
WORD getALUinput2(CPUControl *control, InstructionFields *fields, WORD rsVal, WORD rtVal, WORD reg32, WORD reg33, WORD oldPC) {
    if (control->ALUsrc == 1) {
        if (control->extra3 == 0 && control->extra1 == 0)
            return fields->imm32;
        else
            return fields->imm16;
    }
    else {
        return rtVal;
    }
}
/**
 * @brief Executes ALU 
 * 
 * @param control - control bits for instruction
 * @param input1 - ALU input a
 * @param input2 - ALU input b
 * @param aluResult - result of ALU execution
 */
void execute_ALU(CPUControl *control, WORD input1, WORD input2, ALUResult *aluResult) {
    if (control->ALU.op == 0) {                         // if alu operation is &
        aluResult->result = input1 & input2;
    }
    else if (control->ALU.op == 1) {                    // if alu operation is |
        aluResult->result = input1 | input2;
    }
    else if (control->ALU.op == 2) {                    // if alu operation is +
        if (control->ALU.bNegate == 1) {
            aluResult->result = input1 - input2;
        }
        else {
            aluResult->result = input1 + input2;
        }
    }
    else if (control->ALU.op == 3) {                    // if alu operation is <
        aluResult->result = input1 < input2 ? 1 : 0;
    }
    else if (control->ALU.op == 4) {                    // if alu operation is ^
        aluResult->result = input1 ^ input2;
    }
    if (aluResult->result == 0) {
        aluResult->zero = 1;
    }
}
/**
 * @brief Executes memory accessing if necessary
 * 
 * @param control - control bits for instruction
 * @param aluResult - result of ALU execution
 * @param rsVal - value in rs section of instruction
 * @param rtVal - value in rt section of instruction
 * @param memory - array containing words for memory
 * @param resultOut - result for memory accessing
 */
void execute_MEM(CPUControl *control, ALUResult *aluResult, WORD rsVal, WORD rtVal, WORD *memory, MemResult *resultOut) {
    if (control->memRead == 1) {
        resultOut->readVal = memory[aluResult->result / 4];
    }
    else if (control->memWrite == 1) {
        memory[aluResult->result / 4] = rtVal;
        resultOut->readVal = 0;
    }
    else {
        resultOut->readVal = 0;
    }
}
/**
 * @brief Returns next program counter
 * 
 * @param fields - decoded instruction fields
 * @param control - control bits for instruction
 * @param aluZero - if ALU result is 0
 * @param rsVal - value in rs section of instruction
 * @param rtVal - value in rt section of instruction
 * @param oldPC - old program counter
 * @return WORD - new program counter
 */
WORD getNextPC(InstructionFields *fields, CPUControl *control, int aluZero, WORD rsVal, WORD rtVal, WORD oldPC) {
    WORD bAdd = fields->imm32 << 2;
    WORD jAdd = fields->address << 2;
    WORD newPC = oldPC + 4;
    if (control->branch == 1 && aluZero == 1) {
        newPC = newPC + bAdd;
    }
    else if (control->branch == 1 && aluZero == 0 && control->extra2 == 1) {
        newPC = newPC + bAdd;
    }
    else if (control->jump == 1) {
        newPC = newPC & 0xF0000000;
        newPC = newPC + jAdd;
    }
    return newPC;
}
/**
 * @brief Updates registers if changed
 * 
 * @param fields - decoded instruction fields
 * @param control - control bits for instruction
 * @param aluResult - result of ALU execution
 * @param memResult - result of memory execution
 * @param regs - array of WORDs containing registers
 */
void execute_updateRegs(InstructionFields *fields, CPUControl *control, ALUResult *aluResult, MemResult *memResult, WORD *regs) {
    if (control->extra1 == 1) {                     // if control for lui then set register to lui
        regs[fields->rt] = fields->imm16 << 16;
        return;
    }
    if (control->regWrite == 1) {
        if (control->regDst == 1) {
            regs[fields->rd] = aluResult->result;
        }
        else {
            if (control->memToReg == 1) {
                regs[fields->rt] = memResult->readVal;
            }
            else {
                regs[fields->rt] = aluResult->result;
            }
        }
    }
}
