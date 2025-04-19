# Mini MIPS Instruction Set Architecture (ISA) Reference

This document outlines the ISA supported by the Mini MIPS processor for CS220 Assignment 8. It includes instruction types, functionality, opcodes, and function codes (if applicable).

---

## 🔢 Instruction Format

| Format | Description |
|--------|-------------|
| R-type | opcode(6) \| rs(5) \| rt(5) \| rd(5) \| shamt(5) \| funct(6) |
| I-type | opcode(6) \| rs(5) \| rt(5) \| immediate(16) |
| J-type | opcode(6) \| address(26) |
| F-type | opcode(6) \| ftype(5) \| fs(5) \| ft(5) \| fd(5) \| funct(6) |

---

## 🧮 Arithmetic Instructions (R-type unless specified)

| Instruction | Type | Opcode | Funct | Description |
|------------|------|--------|-------|-------------|
| add        | R    | 000000 | 000000 | rd = rs + rt |
| sub        | R    | 000000 | 000010 | rd = rs - rt |
| addu       | R    | 000000 | 100001 | rd = rs + rt (unsigned) |
| subu       | R    | 000000 | 100011 | rd = rs - rt (unsigned) |
| addi       | I    | 100001 | -     | rt = rs + immediate |
| addiu      | I    | 001010 | -     | rt = rs + immediate (unsigned) |
| madd       | R    | 000000 | 101100 | HI, LO += rs * rt |
| maddu      | R    | 000000 | 101101 | HI, LO += rs * rt (unsigned) |
| mul        | R    | 000000 | 011000 | HI, LO = rs * rt |
| and        | R    | 000000 | 100100 | rd = rs & rt |
| or         | R    | 000000 | 100101 | rd = rs \| rt |
| andi       | I    | 001100 | -     | rt = rs & immediate |
| ori        | I    | 001101 | -     | rt = rs \| immediate |
| not        | R    | 000000 | 110000 | rd = ~rs |
| xori       | I    | 001110 | -     | rt = rs ^ immediate |
| xor        | R    | 000000 | 000111 | rd = rs ^ rt |

---

## 🔁 Shift Instructions (R-type)

| Instruction | Type | Opcode | Funct | Description |
|------------|------|--------|-------|-------------|
| sll        | R    | 000000 | 001000 | rd = rt << shamt |
| srl        | R    | 000000 | 001001 | rd = rt >> shamt |
| sla        | R    | 000000 | 001010 | rd = rt <<< shamt (same as sll) |
| sra        | R    | 000000 | 001011 | rd = rt >>> shamt |

---

## 📦 Data Transfer Instructions

| Instruction | Type | Opcode | Funct | Description |
|------------|------|--------|-------|-------------|
| lw         | I    | 100110 | -     | rt = Mem[rs + immediate] |
| sw         | I    | 100111 | -     | Mem[rs + immediate] = rt |
| lui        | I    | 101000 | -     | rt = immediate << 16 |

---

## 🔁 Conditional Branch Instructions (I-type)

| Instruction | Type | Opcode | Description |
|-------------|------|--------|-------------|
| beq         | I    | 101001 | if (rs == rt) branch |
| bne         | I    | 101010 | if (rs != rt) branch |
| bgt         | I    | 101011 | if (rs > rt) branch |
| bgte        | I    | 101100 | if (rs >= rt) branch |
| ble         | I    | 101101 | if (rs < rt) branch |
| bleq        | I    | 101110 | if (rs <= rt) branch |
| bleu        | I    | 101111 | if (rs < rt) branch (unsigned) |
| bgtu        | I    | 110000 | if (rs > rt) branch (unsigned) |

---

## 🔁 Unconditional Branch Instructions

| Instruction | Type | Opcode | Description |
|-------------|------|--------|-------------|
| j           | J    | 010001 | Jump to address |
| jr          | R    | 010010 | funct: 001000 (Jump to rs) |
| jal         | J    | 010011 | Jump and link (ra = PC + 4) |

---

## 🔍 Comparison Instructions

| Instruction | Type | Opcode | Funct | Description |
|-------------|------|--------|-------|-------------|
| slt         | R    | 000000 | 101010 | rd = (rs < rt) ? 1 : 0 |
| slti        | I    | 110001 | -     | rt = (rs < immediate) ? 1 : 0 |
| seq         | I    | 110010 | -     | rt = (rs == immediate) ? 1 : 0 |

---

## 🧊 Floating Point Instructions

| Instruction | Type | Opcode | Funct | Description |
|-------------|------|--------|-------|-------------|
| mfcl        | R    | 010001 | 000000 | rt = f0 |
| mtc1        | R    | 010001 | 000001 | f0 = rt |
| add.s       | R    | 010001 | 000010 | f0 = f1 + f2 |
| sub.s       | R    | 010001 | 000011 | f0 = f1 - f2 |
| c.eq.s      | R    | 010001 | 110010 | cc = (f0 == f1) |
| c.le.s      | R    | 010001 | 110001 | cc = (f0 <= f1) |
| c.lt.s      | R    | 010001 | 110000 | cc = (f0 < f1) |
| c.ge.s      | R    | 010001 | 110011 | cc = (f0 >= f1) |
| c.gt.s      | R    | 010001 | 110100 | cc = (f0 > f1) |
| mov.s       | R    | 010001 | 000100 | f0 = f1 (if cc set) |

---

## Register Mapping

| Register Name | Number | Description         |
|---------------|--------|---------------------|
| $zero         | 0      | Constant 0          |
| $at           | 1      | Assembler temporary |
| $v0–$v1       | 2–3    | Function results    |
| $a0–$a3       | 4–7    | Function arguments  |
| $t0–$t7       | 8–15   | Temporaries         |
| $s0–$s7       | 16–23  | Saved temporaries   |
| $t8–$t9       | 24–25  | More temporaries    |
| $k0–$k1       | 26–27  | OS reserved         |
| $gp           | 28     | Global pointer      |
| $sp           | 29     | Stack pointer       |
| $fp           | 30     | Frame pointer       |
| $ra           | 31     | Return address      |

---

## R-Type Function Codes

| Funct   | Instruction | Description              |
|---------|-------------|--------------------------|
| 100000  | add         | rd = rs + rt             |
| 100010  | sub         | rd = rs - rt             |
| 100001  | addu        | rd = rs + rt (unsigned)  |
| 100011  | subu        | rd = rs - rt (unsigned)  |
| 100100  | and         | rd = rs & rt             |
| 100101  | or          | rd = rs \| rt            |
| 100110  | xor         | rd = rs ^ rt             |
| 110000  | not         | rd = ~rs                 |
| 000000  | sll         | rd = rt << shamt         |
| 000010  | srl         | rd = rt >> shamt         |
| 000011  | sra         | rd = rt >>> shamt        |
| 101010  | slt         | rd = (rs < rt) ? 1 : 0   |
| 101100  | madd        | HI, LO += rs * rt        |
| 011000  | mul         | HI, LO = rs * rt         |
| 001000  | jr          | PC = rs                  |

---

## Control Signal Summary

| Signal        | Meaning                                  |
|---------------|------------------------------------------|
| reg_dst     | 2 = $ra(for JAL), 1 = rd (R-type), 0 = rt (I-type)         |
| alu_src     | 1 = immediate, 0 = register              |
| mem_to_reg  | 1 = data from memory, 0 = ALU result     |
| reg_write   | 1 = enable register write                |
| mem_read    | 1 = enable memory read                   |
| mem_write   | 1 = enable memory write                  |
| branch_type | 3-bit code for type of branching         |
| jump        | 1 = enable jump                          |
| jump_src        | 1 = jump address, 0 = read_data1(for jr)                        |
| alu_op      | ALU control bits (depends on opcode)     |

---

## ALUOp Decoding Table

| ALUOp Code | Operation | Used For                              |
|------------|-----------|----------------------------------------|
| 0          | RTYPE     | R-type instructions (opcode = 0)      |
| 1          | AND       | and                                   |
| 2          | ADDI      | addi                                  |
| 3          | SLT       | slti, bgt, ble, bgte, bleq            |
| 4          | SLTU      | sltu, bgtu, bleu                      |
| 6          | AND       | andi                                  |
| 7          | OR        | ori                                   |
| 8          | XOR       | xori                                  |
| 10         | SEQ       | seq                                   |
| 11         | J         | j                                     |
| 12         | JAL       | jal                                   |
| 15         | RTYPE     | R-type instructions (opcode = 0)      |

---

## ALU Control  Decoding Table

| ALU_control Code | Mnemonic   | Operation                      | Source (ALUOp / funct) |
|------------------|------------|-------------------------------|-------------------------|
| 0                | ALU_ADD    | ADD                           | funct = 0               |
| 1                | ALU_ADDU   | Unsigned Addition             | funct = 1               |
| 2                | ALU_SUB    | Subtraction                   | funct = 2, ALUOp=1      |
| 3                | ALU_SUBU   | Unsigned subtraction          | funct = 3               |
| 4                | ALU_AND    | Bitwise and                   | funct = 7               |
| 5                | ALU_OR     | Bit wise or                   | funct = 8               |
| 6                | ALU_NOT    | Logical not                   | funct = 34, ALUOp=2     |
| 7                | ALU_XOR    | Bitwise xor                   | funct = 42, ALUOp=3     |
| 8                | ALU_SLL    | Logical Shift left            | funct = 12, ALUOp=8     |
| 9                | ALU_SRL    | Logical Shift Right           | ALUOp = 13              |
| 10               | ALU_SRA    | Shift right arithematic       | ALUOp = 14              |
| 11               | ALU_SLT    | Set less than                 | ALUOp = 11              |
| 12               | ALU_SEQ    | Set on equal                  | funct = 48              |
| 13               | ALU_MUL    | multiplication                | funct = 6               |
| 14               | ALU_MADD   | Multiply-Add Signed           | funct = 4               |
| 15               | ALU_MADDU  | Multiply-Add Unsigned         | funct = 5               |

---

## ✅ Notes

- All unused opcodes and funct values should be treated as *NOP or invalid*.
- *Jump target* = {PC+4[31:28], address, 2'b00}
- Immediate values are *sign-extended* where needed.

---
